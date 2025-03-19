#!/usr/bin/env python3
from math import atan2, sqrt, radians, degrees

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.exceptions import ROSInterruptException

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from utils.utils.math import normalize, euler_from_quaternion


class P_Controller:
    def __init__(self, k=0.1):
        self.k = k

    def compute(self, error):
        return self.k * error


class ClosedLoopSquareController(Node):
    def __init__(self) -> None:
        super().__init__('square_controller')

        self._init_parameters()
        self._init_variables()

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ensure_publisher_connected()

        self.sub_odom = self.create_subscription(Odometry, '/odom',
                                                 self.odom_callback, 10)
        self.timer = self.create_timer(1/30.0, self.go_to)
        self.get_logger().info(f'Finished initializing node: {self.get_name()}')

    def _init_parameters(self):
        self.declare_parameter('square_side', 2.0, ParameterDescriptor(
            name='square_side', type=ParameterType.PARAMETER_DOUBLE,
            description='Length of the side of the square.'))
        
        self.declare_parameter('times', 1, ParameterDescriptor(
            name='times', type=ParameterType.PARAMETER_INTEGER,
            description='Number of times to loop the square path.'))

        self.declare_parameter('k_rho', 0.3, ParameterDescriptor(
            name='k_rho', type=ParameterType.PARAMETER_DOUBLE,
            description='Proportional gain for distance to the goal (rho).'))
        
        self.declare_parameter('k_alpha', 0.8, ParameterDescriptor(
            name='k_alpha', type=ParameterType.PARAMETER_DOUBLE,
            description="Proportional gain for angle to the goal (alpha)."))
        
        self.declare_parameter('k_beta', 0.3, ParameterDescriptor(
            name='k_beta', type=ParameterType.PARAMETER_DOUBLE,
            description="Proportional gain for orientation alignment (beta)."))

    def _init_variables(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.max_vel = 0.22
        self.max_omega = 2.84

        self.goal_dist_tolerance = 0.02
        self.goal_heading_tolerance = 10

        self.square_side = self.get_parameter('square_side').value
        self.times = self.get_parameter('times').value

        # Define the four corners of the square
        self.goals = [
            Point(x=self.square_side, y=0.0, z=90.0),
            Point(x=self.square_side, y=self.square_side, z=180.0),
            Point(x=0.0, y=self.square_side, z=270.0),
            Point(x=0.0, y=0.0, z=0.0),
        ]

        self.current_goal_index = 0
        self.current_time = 0

        self.goal = self.goals[self.current_goal_index]
        self.get_logger().info(f'Goal: {self.goal.x=}, {self.goal.y=}, {self.goal.z=}')

        self.rho_controller = P_Controller(self.get_parameter('k_rho').value)
        self.alpha_controller = P_Controller(self.get_parameter('k_alpha').value)
        self.beta_controller = P_Controller(self.get_parameter('k_beta').value)

    def ensure_publisher_connected(self):
        while self.pub_cmd.get_subscription_count() == 0:
            self.get_logger().info(f"Waiting for subscriber on {self.pub_cmd.topic} ... ",
                                   throttle_duration_sec=1.0)
            rclpy.spin_once(self, timeout_sec=0.1)

    def odom_callback(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        quat = msg.pose.pose.orientation
        (_, _, self.theta) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

    def go_to(self) -> None:
        def compute_beta_sign(th):
            """All rotations are counter-clockwise."""
            return -1
        if self.goal is None:
            return

        delta_x = self.goal.x - self.x
        delta_y = self.goal.y - self.y
        rho = sqrt(delta_x**2 + delta_y**2)

        goal_theta = radians(self.goal.z)
        alpha = normalize(atan2(delta_y, delta_x) - self.theta)
        beta = normalize(goal_theta - atan2(delta_y, delta_x)) * compute_beta_sign(self.goal.z)
        heading_error = normalize(goal_theta - self.theta)

        self.get_logger().info(f'Distance to goal: {rho:.2f}m, Heading difference: {degrees(heading_error):.2f}°',
                               throttle_duration_sec=1)

        if rho > self.goal_dist_tolerance:
            v = self.rho_controller.compute(rho)
            omega_alpha = self.alpha_controller.compute(alpha)
            omega_beta = self.beta_controller.compute(beta)
            angular_vel = omega_alpha + omega_beta
        else:
            v = 0.0
            angular_vel = self.beta_controller.compute(beta)

        v = self.clamp(v, self.max_vel)
        omega = self.clamp(angular_vel, self.max_omega)

        speed = Twist()
        speed.linear.x = v
        speed.angular.z = omega
        self.pub_cmd.publish(speed)

        if rho < self.goal_dist_tolerance and abs(degrees(heading_error)) < self.goal_heading_tolerance:
            self.finalize_goal()

    def clamp(self, value, max_value):
        return max(min(value, max_value), -max_value)

    def finalize_goal(self):
        self.get_logger().info(f'Goal reached: {self.goal.x=}, {self.goal.y=}, {self.goal.z=}')
        self.current_time += 1
        if self.current_time < self.times * 4:
            self.current_goal_index = (self.current_goal_index + 1) % len(self.goals)
            self.goal = self.goals[self.current_goal_index]
            self.get_logger().info(f'Next goal: {self.goal.x=}, {self.goal.y=}, {self.goal.z=}')
        else:
            self.get_logger().info('All iterations completed. Stopping robot.')
            self.stop_robot()
            self.timer.cancel()
            raise SystemExit

    def stop_robot(self):
        self.get_logger().warn('Stopping robot')
        self.pub_cmd.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopSquareController()
    node.get_logger().set_level(LoggingSeverity.INFO)

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ROSInterruptException, SystemExit):
        node.get_logger().info('Exiting...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()