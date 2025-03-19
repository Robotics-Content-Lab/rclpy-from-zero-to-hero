#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ch5_interfaces.action import DriveSquareAction
from utils.utils.math import euler_from_quaternion
import threading
from concurrent.futures import ThreadPoolExecutor


class DriveSquareActionServer(Node):
    def __init__(self):
        super().__init__('drive_square_action_server')  # Initialize the action server node with a specific name
        
        # Variables to keep track of the action server state
        self.square_side = None  # Side length of the square pattern the robot will drive
        self.times = None  # Number of times the robot should repeat the square pattern
        self.current_iteration = 0  # Track which side of the square the robot is currently moving along
        self.current_state = 0  # 0: Moving along x-axis, 1: moving along y-axis
        self.state_complete = False  # Flag to determine if the robot has completed its current state (side of the square)
        self.current_x = None  # Current x-coordinate of the robot, updated from odometry
        self.current_y = None  # Current y-coordinate of the robot, updated from odometry
        self.current_theta = None  # Current orientation of the robot, updated from odometry

        # Action server components
        self.goal_handle = None  # Handle to the current goal being executed
        self.goal_request = None  # Request object to store the goal request
        self.feedback = DriveSquareAction.Feedback()  # Feedback object to be sent to the client to inform about progress

        # Tolerance for stopping movement near the target position
        self.position_tolerance = 0.02  # Acceptable distance to the target position before stopping
        # Control gains for robot movement
        self.k_rho = 0.8  # Proportional gain for controlling speed based on distance to target
        # Velocity limits for the robot
        self.min_vel = 0.1  # Minimum velocity to ensure movement isn't too slow to overcome friction, etc.
        self.max_vel = 0.5  # Maximum velocity to ensure the robot doesn't move too fast and become uncontrollable

        # Timeout and timer management
        self.timeout_timer = None  # TImeout timer to check if the action has taken too long
        self.control_timer = None  # Timer to control the robot's movement
        self.feedback_timer = None  # Timer to publish feedback to the client
        self.time_limit = 0.0  # Maximum allowed time for the action to be completed before timing out
        self.start_time = None  # Start time of the action, used to calculate elapsed time

        # Publishers and subscribers
        self.pub_cmd = self.create_publisher(Twist, '/robotino/cmd_vel', 10)  # Publisher to send velocity commands to the robot
        self.sub_odom = self.create_subscription(Odometry, '/robotino/odom', self.odom_callback, 10,callback_group=ReentrantCallbackGroup())  # Subscriber to receive odometry data from the robot

        # Action server
        self.action_server = ActionServer(
            self,
            DriveSquareAction,  # The action type defined in the .action file
            'drive_square',  # The name of the action
            execute_callback=self.execute_callback,  # Callback to handle the execution of the action
            goal_callback=self.goal_callback,  # Callback to handle new goal requests
            cancel_callback=self.cancel_callback,  # Callback to handle cancellation requests
            callback_group=ReentrantCallbackGroup()  # Allow multiple callbacks to run concurrently
        )

        # Flags and events used to track progress
        self.flag_finish = False  # Flag to indicate if the goal has been completed successfully
        self.flag_timeout = False  # Flag to indicate if the goal has timed out
        self.timeout_event = threading.Event()  # Event to signal that the action has timed out
        self.odometry_received_event = threading.Event()  # Event to signal that odometry data is available
        self.operation_complete_event = threading.Event()  # Event to signal that the operation is complete

        # A lock to protect timer destruction (and a flag to indicate canceled timers)
        self.timer_lock = threading.Lock()
        self.timers_canceled = False

        self.get_logger().info("Drive Square Action Server Initialized")  # Log that the action server has started successfully

    def cleanup(self):
        # Reset state and destroy timers safely.
        self.get_logger().info("Cleaning up")

        self.current_iteration = 0
        self.current_state = 0
        self.state_complete = False
        self.current_x = None
        self.current_y = None
        self.current_theta = None

        self.goal_handle = None
        self.goal_request = None
        self.flag_finish = False
        self.flag_timeout = False
        self.time_limit = 0.0
        self.start_time = None

        # Clear events
        self.odometry_received_event.clear()
        self.operation_complete_event.clear()

        # Destroy timers under lock to avoid race conditions
        with self.timer_lock:
            self.timers_canceled = True
            if self.timeout_timer is not None:
                self.destroy_timer(self.timeout_timer)
                self.timeout_timer = None
            if self.control_timer is not None:
                self.destroy_timer(self.control_timer)
                self.control_timer = None
            if self.feedback_timer is not None:
                self.destroy_timer(self.feedback_timer)
                self.feedback_timer = None

    def odom_callback(self, msg):
        # Callback to update the robot's current position and orientation based on odometry data
        position = msg.pose.pose.position  # Extract the robot's current position
        quat = msg.pose.pose.orientation  # Extract the robot's current orientation in quaternion format
        self.current_x = position.x  # Update the x-coordinate
        self.current_y = position.y  # Update the y-coordinate
        (_, _, self.current_theta) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])  # Convert the quaternion to Euler angles and update theta
        self.odometry_received_event.set()  # Signal that odometry data is available

    def goal_callback(self, goal_request):
        # Callback when a goal request is received
        self.get_logger().info('Received goal request: Drive square')  # Log the reception of a goal request
        self.goal_request = goal_request  # Store the goal request for later use
        return GoalResponse.ACCEPT  # Accept all incoming goals for simplicity

    def cancel_callback(self, goal_handle):
        # Callback when a cancel request is received
        self.get_logger().info('Received cancel request')  # Log the reception of a cancel request
        self.stop_robot()  # Stop the robot
        self.operation_complete_event.set()  # Signal that the operation is complete
        self.flag_finish = False  # Reset the finish flag
        self.flag_timeout = False  # Reset the timeout flag
        return CancelResponse.ACCEPT  # Accept all cancellation requests

    def execute_callback(self, goal_handle):
        # Main entry point for new goals.
        self.cleanup()  # Ensure we start from a clean state
        self.goal_handle = goal_handle  # Store the goal handle for use in feedback publishing
        self.get_logger().info('Executing goal')

        self.start_time = self.get_clock().now()  # Record the start time of the action
        self.square_side = goal_handle.request.side_length  # Retrieve the side length from the goal request
        self.times = goal_handle.request.repetitions  # Retrieve the number of repetitions from the goal request
        self.time_limit = goal_handle.request.time_limit  # Retrieve the time limit from the goal request

        # Wait for odometry data (up to 5s)
        if not self.odometry_received_event.wait(timeout=5.0):
            self.get_logger().error("Timeout waiting for odometry data!")
            goal_handle.abort()
            return DriveSquareAction.Result(success=False)

        self.get_logger().info(f'Starting to drive in square: side={self.square_side}, times={self.times}')

        # Re-enable timers in case they were destroyed previously
        with self.timer_lock:
            self.timers_canceled = False
            self.timeout_timer = self.create_timer(1.0, self.timeout_callback)
            self.control_timer = self.create_timer(1/60.0, self.control_loop)
            self.feedback_timer = self.create_timer(1.0, self.publish_feedback)

        # Offload the blocking wait on operation_complete_event using ThreadPoolExecutor.
        with ThreadPoolExecutor(max_workers=1) as executor:
            future = executor.submit(self.operation_complete_event.wait, self.time_limit + 2.0)
            future.result()

        # Destroy timers once the operation is done or times out
        with self.timer_lock:
            self.timers_canceled = True
            if self.timeout_timer is not None:
                self.destroy_timer(self.timeout_timer)
                self.timeout_timer = None
            if self.control_timer is not None:
                self.destroy_timer(self.control_timer)
                self.control_timer = None
            if self.feedback_timer is not None:
                self.destroy_timer(self.feedback_timer)
                self.feedback_timer = None

        # Decide final action result
        if self.flag_finish:
            goal_handle.succeed()  # Indicate that the goal was successfully completed
            self.get_logger().info('Goal succeeded')
            return DriveSquareAction.Result(success=True)  # Return a successful result
        elif self.flag_timeout:
            self.stop_robot()
            goal_handle.abort()  # Indicate that the goal was aborted due to a timeout
            self.get_logger().info('Goal aborted due to timeout')
            return DriveSquareAction.Result(success=False)  # Return a failed result
        else:
            goal_handle.abort()  # Indicate that the goal was aborted for an unknown reason
            return DriveSquareAction.Result(success=False)  # Return a failed result

    def control_loop(self):
        # Control loop for moving the robot in a square pattern
        # Quick exit if timers are destroyed
        with self.timer_lock:
            if self.timers_canceled:
                return

        speed = Twist()
        direction = 1 if self.current_iteration % 4 < 2 else -1

        if self.current_state == 0:  # Moving along the x-axis
            if not self.state_complete:
                self.start_x = self.current_x
                self.state_complete = True

            rho = abs(self.current_x - self.start_x)
            if rho >= self.square_side - self.position_tolerance:
                self.current_state = 1
                self.state_complete = False
                self.current_iteration += 1
                self.get_logger().info("Switching to moving along y-axis")
            else:
                vel = self.k_rho * (self.square_side - rho)
                speed.linear.x = self.clip_velocity(vel) * direction
                self.get_logger().info(f"Moving along x-axis, forward: {direction == 1}",
                                       throttle_duration_sec=1)

        elif self.current_state == 1:  # Moving along the y-axis
            if not self.state_complete:
                self.start_y = self.current_y
                self.state_complete = True

            rho = abs(self.current_y - self.start_y)
            if rho >= self.square_side - self.position_tolerance:
                self.current_state = 0
                self.state_complete = False
                self.current_iteration += 1
                self.get_logger().info("Switching to moving along x-axis")
            else:
                vel = self.k_rho * (self.square_side - rho)
                speed.linear.y = self.clip_velocity(vel) * direction
                self.get_logger().info(f"Moving along y-axis, left: {direction == 1}",
                                       throttle_duration_sec=1)

        self.pub_cmd.publish(speed)

        # Check for completion
        if self.current_iteration >= self.times * 4:  # Check if the robot has completed all iterations of the square pattern
            self.stop_robot()  # Stop the robot
            self.get_logger().info("Completed all square iterations")  # Log the completion of the action
            self.flag_finish = True  # Set the finish flag to true
            self.flag_timeout = False  # Reset the timeout flag
            self.operation_complete_event.set()  # Signal that the operation is complete

    def publish_feedback(self):
        # Publish feedback to the client
        # Quick exit if timers are destroyed
        with self.timer_lock:
            if self.timers_canceled:
                return

        if self.current_iteration is not None:
            self.feedback.current_side = self.current_iteration % 4
            self.feedback.current_iteration = self.current_iteration
            self.goal_handle.publish_feedback(self.feedback)

    def timeout_callback(self):
        # Callback for handling timeouts
        # Quick exit if timers are destroyed
        with self.timer_lock:
            if self.timers_canceled:
                return

        if self.start_time is not None:
            current_time = self.get_clock().now()
            elapsed_time = (current_time.nanoseconds - self.start_time.nanoseconds) / 1e9
            if not self.flag_finish and elapsed_time > self.time_limit:
                self.get_logger().info("Timeout reached")
                self.flag_finish = False
                self.flag_timeout = True
                self.operation_complete_event.set()

    def clip_velocity(self, velocity):
        # Clip the velocity to be within the defined limits
        return max(self.min_vel, min(velocity, self.max_vel))

    def stop_robot(self):
        # Stop the robot's movement
        speed = Twist()
        self.pub_cmd.publish(speed)
        self.get_logger().info("Stopping robot")
        self.flag_finish = True


def main(args=None):
    rclpy.init(args=args)
    node = DriveSquareActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.get_logger().info("Shutting down")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
