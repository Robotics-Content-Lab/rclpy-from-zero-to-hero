#!/usr/bin/env python3

from math import hypot, atan2, degrees
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
from geometry_msgs.msg import Twist

class OpenLoopPointController(Node):
    """
    A ROS node that moves a robot to a given point using open-loop control.

    Attributes:
        goal_x (float): The x-coordinate of the goal point.
        goal_y (float): The y-coordinate of the goal point.
        angle_to_goal (float): The angle between the robot's current position and the goal point.
        max_vel (float): The maximum linear velocity of the robot.
        max_omega (float): The maximum angular velocity of the robot.
        hz (int): The frequency at which the node is executed.
        drive_duration (float): The duration for which the robot moves in a straight line towards the goal point.
        turn_duration (float): The duration for which the robot turns to face the goal point.
        state_duration (int): The duration for which the robot is in a particular state.
        state (int): The current state of the robot.
    """
    def __init__(self) -> None:
        super().__init__("point_controller")

        # Declare parameters with default values
        # Default goal point is (2, 3)
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 3.0)
        
        # Retrieve the parameter values
        goal_x = float(self.get_parameter('goal_x').value)
        goal_y = float(self.get_parameter('goal_y').value)

        self.get_logger().info(f"Goal point= x:{goal_x} y:{goal_y}")
        delta_x = goal_x - 0.0
        delta_y = goal_y - 0.0
        self.angle_to_goal = atan2(delta_y, delta_x)  # Compute angle to goal
        self.get_logger().info(f"Angle to goal: {degrees(self.angle_to_goal)}")

        self.max_vel = 0.22  # m/s
        self.max_omega = 2.84 / 4  # rad/s

        self.hz = 30
        self.drive_duration = (hypot(goal_x, goal_y) / self.max_vel) * self.hz  # Calculate duration to drive to goal based on max velocity
        self.turn_duration = (self.angle_to_goal / self.max_omega) * self.hz  # Calculate duration to turn to face goal based on max angular velocity
        self.state_duration = 0  # Initialize state duration variable used to track the duration for which the robot is in a particular state
        self.state = 0  # Initialize state variable to 0 (0, 1, 2) to track the current state of the robot

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        while self.pub_cmd.get_subscription_count() == 0:
            # Wait for subscriber to connect = wait for a robot to subscribe to the cmd_vel topic to be able to move
            self.get_logger().info(f"Waiting for sub on {self.pub_cmd.topic} ... ", throttle_duration_sec=1.0)
        self.timer = self.create_timer(1/self.hz, self.go_to)

    def go_to(self):
        """
        Moves the robot towards the goal point using open-loop control.

        The method sets the linear and angular velocities of the robot based on it's current state. The state variable tracks the current state of the robot, which can be one of three states:
            * state 0: In this state, the robot turns to face the goal point.
            * state 1: In this state, the robot moves in a straight line towards the goal point.
            * state 2: In this state, the robot has reached the goal point and stops.
        """
        speed: Twist = Twist()  # Initialize Twist message to set linear and angular velocities of the robot
        if self.state == 0:
            # Robot is not facing the goal point -> turn to face the goal point
            speed.linear.x = 0.0  # While turning no linear velocity
            speed.angular.z = self.max_omega if self.angle_to_goal >= 0.0 else -self.max_omega  # Turn left or right based on angle to goal
            self.state_duration += 1  # Increment state duration used to track the duration for which the robot is in a particular state

            if self.state_duration >= (self.turn_duration if self.turn_duration >= 0 else -self.turn_duration):
                # If enough time has passed to turn to face the goal point, change state to move towards the goal point
                self.state = 1  # Change state to move towards the goal point
                self.state_duration = 0  # Reset state duration counter
        elif self.state == 1:
            # Robot is facing the goal point -> move towards the goal point
            speed.linear.x = self.max_vel  # Move forward
            speed.angular.z = 0.0  # No angular velocity while moving forward
            self.state_duration += 1

            if self.state_duration >= self.drive_duration:
                # If enough time has passed to reach the goal point, change state to stop
                self.state = 2  # Change state to stop
                self.state_duration = 0  # Reset state duration counter
        elif self.state == 2:
            # Robot has reached the goal point -> stop
            self.get_logger().info("At goal. Shutting down!...")
            self.pub_cmd.publish(Twist())  # Stop the robot by publishing an empty Twist message
            self.timer.cancel()
            raise SystemExit  # Exit the spin loop
        self.pub_cmd.publish(speed)  # Publish the linear and angular velocities to move the robot


def main(args=None):
    rclpy.init(args=args)

    node = OpenLoopPointController()
    executor = SingleThreadedExecutor()
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
