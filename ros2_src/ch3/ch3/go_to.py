#!/usr/bin/env python3
from math import atan2, sqrt, hypot, radians, degrees

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.exceptions import ROSInterruptException

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from utils.utils.math import normalize, euler_from_quaternion

class P_Controller():
    """A simple proportional controller that takes an error and returns a control output."""
    def __init__(self, k=0.1):
        self.k = k
    
    def compute(self, error):
        return self.k * error


class ClosedLoopPointController(Node):
    def __init__(self) -> None:
        super().__init__('controller')
        client_cb_group = ReentrantCallbackGroup()  # Create a callback group for the subscriber
        timer_cb_group = ReentrantCallbackGroup()  # Create a callback group for the timer

        self.cb_logger = rclpy.logging.get_logger('cb_logger')  # Create a logger for the callback
        self.timer_logger = rclpy.logging.get_logger('timer_logger')  # Create a logger for the timer

        self.x = 0.0  # Current x position
        self.y = 0.0  # Current y position
        self.theta = 0.0  # Current heading angle

        self.goal_dist_tolerance = 0.02  # Tolerance for the distance to the goal
        self.goal_heading_tolerance = 10  # Tolerance for the heading to the goal in degrees

        self.max_vel = 0.22
        self.max_omega = 2.84

        self.declare_parameter('goal_x', 1.0)  # Declare the goal_x parameter
        self.declare_parameter('goal_y', 1.0)  # Declare the goal_y parameter
        self.declare_parameter('goal_th', 90.0)  # Declare the goal_th parameter in degrees

        # Create a geometry_msgs.msg.Point object to store the goal point
        self.goal: Point = Point(x=self.get_parameter('goal_x').value,
                                    y=self.get_parameter('goal_y').value,
                                    z=self.get_parameter('goal_th').value)
        self.get_logger().info(f'Goal: {self.goal.x=}, {self.goal.y=}, {self.goal.z=}')

        self.k_rho = self.declare_parameter('k_rho', 0.3, ParameterDescriptor(name='k_rho',
                                                                              type=ParameterType.PARAMETER_DOUBLE,
                                                                              description='The proportional gain for the distance to the goal.' +
                                                                                'Adjusts the robot\' speed based on the current distance from the goal'))
        self.k_alpha = self.declare_parameter('k_alpha', 0.8, ParameterDescriptor(name='k_alpha',
                                                                                    type=ParameterType.PARAMETER_DOUBLE,
                                                                                    description='The proportional gain for the heading to the goal from the robot\' current position.' +
                                                                                     'Adjusts the robot\' turning action based on it\'s current heading error'))
        self.k_beta = self.declare_parameter('k_beta', 0.25, ParameterDescriptor(name='k_beta',
                                                                                    type=ParameterType.PARAMETER_DOUBLE,
                                                                                    description='The proportional gain for the difference between the robot\' heading and the goal orientation.' +
                                                                                    'Aligns the robot with the goal\' orientation as it approaches'))


        self.hz = 30
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        while self.pub_cmd.get_subscription_count() == 0:
            self.get_logger().info(f"Waiting for sub on {self.pub_cmd.topic} ... ", throttle_duration_sec=1.0)
        self.sub_odom = self.create_subscription(Odometry, '/odom',
                                                 self.odom_callback,
                                                 10, callback_group=client_cb_group)
        self.timer = self.create_timer(1/self.hz, self.go_to, callback_group=timer_cb_group)
        self.get_logger().info('Finished init node')



    def odom_callback(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        quat = msg.pose.pose.orientation
        (_, _, self.theta) = euler_from_quaternion(quaternion=[quat.x, quat.y, quat.z, quat.w])

        self.cb_logger.debug(f'Current Pose: {self.x=}, {self.y=}, {degrees(self.theta)=}', throttle_duration_sec=0.5)

    def go_to(self) -> None:
        '''Calculates the current distance (rho) to the goal_point as well as the angle to the goal.
        Based on the delta angle to the goal this function decides whether to rotate the robot in place or go straight ahead.

        Args:
            goal_point (Point): X and Y position of the desired goal point. Should be in odometry frame
        '''

        def clamp(num, max_vel):
            '''Clamp the input between -max_vel and +max_vel'''
            return max(min(num, max_vel), -max_vel)

        if self.goal is None:
            return

        speed: Twist = Twist()

        # Compute the errors in position and heading
        delta_x = self.goal.x - self.x  # x distance to goal
        delta_y = self.goal.y - self.y  # y distance to goal
        rho = sqrt(delta_x**2 + delta_y**2)  # distance to goal

        # Make sure to use radians consistently
        goal_theta = radians(self.goal.z)  # Goal orientation in radians
        alpha = normalize(atan2(delta_y, delta_x) - self.theta)  # angle difference to goal point (x,y)
        beta = normalize(goal_theta - atan2(delta_y, delta_x))  # heading difference to goal orientation
        heading_error = normalize(goal_theta - self.theta)  # Difference in radians

        self.timer_logger.info(
            f'Distance to goal= {rho:.2f}m\nHeading difference= {heading_error:.3f}', throttle_duration_sec=1)

        v = self.k_rho.value * rho  # Compute the velocity command
        omega = self.k_alpha.value * alpha + self.k_beta.value * beta

        # Clamp the velocities to their maximum values
        v = clamp(v, self.max_vel)
        omega = clamp(omega, self.max_omega)

        speed.linear.x = v
        speed.angular.z = omega
        self.timer_logger.debug(
            f'Publishing {speed=}', throttle_duration_sec=0.5)
        self.pub_cmd.publish(speed)

        if rho < self.goal_dist_tolerance and abs(degrees(heading_error)) < self.goal_heading_tolerance:
            position_error = hypot(delta_x, delta_y)
            self.timer_logger.info(
                f'Final position error: {position_error:.2f}m\nFinal orientation error: {degrees(heading_error):.2f}')
            self.goal = None
            self.stop_robot()
            self.timer.cancel()
            raise SystemExit


    def stop_robot(self) -> None:
        self.get_logger().warn('Stopping robot')
        speed: Twist = Twist()
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        self.pub_cmd.publish(speed)


def main(args=None):
    # Initialize rclpy library
    rclpy.init(args=args)
    node: ClosedLoopPointController = ClosedLoopPointController()
    node.get_logger().set_level(LoggingSeverity.INFO)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ROSInterruptException, SystemExit):
        node.get_logger().info('Exiting')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
