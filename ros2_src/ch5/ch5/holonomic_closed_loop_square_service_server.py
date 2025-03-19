import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup  # Allows multiple callbacks to be executed concurrently
from utils.utils.math import euler_from_quaternion  # Utility to convert quaternion to Euler angles
from ch5_interfaces.srv import DriveInSquare  # Import the custom service type
import threading  # Import threading for synchronization

class HolonomicClosedLoopSquareServer(Node):
    def __init__(self):
        super().__init__('holonomic_closed_loop_square_service_server')

        self.square_side = 2.0  # Default square side length
        self.times = 2  # Default number of repetitions for the square pattern
        self.current_iteration = 0  # Counter for how many sides of the square have been completed
        self.current_state = 0  # 0: Moving along x, 1: moving along y
        self.state_complete = False  # Flag to indicate if the current side movement is complete
        self.current_x = None  # Placeholder for current x position from odometry
        self.current_y = None  # Placeholder for current y position from odometry
        self.current_theta = None  # Placeholder for current orientation (theta) from odometry
        
        # Tolerance for deciding when to stop moving (distance from the target point)
        self.position_tolerance = 0.02

        # Controller gain for controlling speed proportional to distance to target
        self.k_rho = 0.8

        # Velocity limits to ensure smooth and safe operation
        self.min_vel = 0.1  # Minimum velocity to avoid stalling
        self.max_vel = 0.5  # Maximum velocity to prevent excessive speed

        # Initialize the publisher for velocity commands
        self.pub_cmd = self.create_publisher(Twist, '/robotino/cmd_vel', 10)
        
        # Initialize the subscriber for odometry data with a reentrant callback group
        self.sub_odom = self.create_subscription(Odometry, '/robotino/odom', self.odom_callback, 10, callback_group=ReentrantCallbackGroup())

        # Create the service server to handle "start_square" requests
        self.srv = self.create_service(DriveInSquare, 'start_square', self.drive_in_square, callback_group=ReentrantCallbackGroup())
        
        self.timer = None  # Timer will be initialized later for the control loop

        # Create threading events for synchronization
        self.odometry_received_event = threading.Event()  # Event to signal that odometry data is available
        self.operation_complete_event = threading.Event()  # Event to signal that the operation is complete

        self.get_logger().info("Holonomic closed-loop square service initialized")

    def odom_callback(self, msg):
        # Callback function to process incoming odometry data and update current position and orientation
        position = msg.pose.pose.position  # Extract position from the message
        quat = msg.pose.pose.orientation  # Extract orientation (quaternion) from the message
        self.current_x = position.x  # Update current x position
        self.current_y = position.y  # Update current y position
        (_, _, self.current_theta) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])  # Convert quaternion to Euler angles and update theta
        self.odometry_received_event.set()  # Signal that odometry data is available

    def drive_in_square(self, request, response):
        # Service callback that handles the client's request to start driving in a square
        self.square_side = request.length  # Update square side length based on request
        self.times = request.times  # Update the number of repetitions based on request
        self.current_iteration = 0  # Reset the iteration counter
        self.current_state = 0  # Reset to the initial state (moving along x-axis)
        self.state_complete = False  # Reset the state complete flag

        # Wait until odometry data is available without blocking executor threads
        while not self.odometry_received_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)  # Avoid blocking executor threads

        # Start the control loop timer if not already started
        if self.timer is None:
            self.timer = self.create_timer(1/60.0, self.control_loop)

        # Wait for the operation to complete without blocking executor threads
        while not self.operation_complete_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)  # Avoid blocking executor threads

        response.success = True
        return response

    def control_loop(self):
        # Main control loop that adjusts the robot's movement based on it's current position and state
        if self.current_iteration >= self.times * 4:
            self.stop_robot()
            self.get_logger().info("Completed all square iterations")
            self.operation_complete_event.set()  # Signal that operation is complete
            return

        speed = Twist()  # Initialize a Twist message to hold velocity commands
        
        # Determine direction based on the current iteration
        # Every two iterations form a complete square segment; alternate direction each pair of segments
        direction = 1 if self.current_iteration % 4 < 2 else -1

        self.get_logger().info(f"Iteration {self.current_iteration + 1}", throttle_duration_sec=1)
        if self.current_state == 0:  # Moving along x-axis
            if not self.state_complete:
                self.start_x = self.current_x  # Record the starting x position
                self.state_complete = True  # Mark the state as in-progress

            rho = abs(self.current_x - self.start_x)  # Calculate distance traveled along x-axis

            if rho >= self.square_side - self.position_tolerance:
                self.current_state = 1  # Switch to moving along y-axis
                self.state_complete = False  # Mark the state as complete
                self.current_iteration += 1  # Increment the iteration counter
                self.get_logger().info("Switching to moving along y-axis")
            else:
                vel = self.k_rho * (self.square_side - rho)  # Calculate velocity based on remaining distance
                speed.linear.x = self.clip_velocity(vel) * direction  # Set linear velocity along x-axis with direction
                self.get_logger().info(f"Moving along x-axis, straight: {direction == 1}", throttle_duration_sec=1)

        elif self.current_state == 1:  # Moving along y-axis
            if not self.state_complete:
                self.start_y = self.current_y  # Record the starting y position
                self.state_complete = True  # Mark the state as in-progress

            rho = abs(self.current_y - self.start_y)  # Calculate distance traveled along y-axis

            if rho >= self.square_side - self.position_tolerance:
                self.current_state = 0  # Switch back to moving along x-axis
                self.state_complete = False  # Mark the state as complete
                self.current_iteration += 1  # Increment the iteration counter
                self.get_logger().info("Switching to moving along x-axis")
            else:
                vel = self.k_rho * (self.square_side - rho)  # Calculate velocity based on remaining distance
                speed.linear.y = self.clip_velocity(vel) * direction  # Set linear velocity along y-axis with direction
                self.get_logger().info(f"Moving along y-axis, left: {direction == 1}", throttle_duration_sec=1)

        self.pub_cmd.publish(speed)  # Publish the velocity command to move the robot

    def clip_velocity(self, velocity):
        """Clips the velocity to be within the min and max limits."""
        return max(self.min_vel, min(velocity, self.max_vel))  # Ensure velocity stays within specified limits

    def stop_robot(self):
        # Method to stop the robot's movement
        speed = Twist()  # Initialize a zero-velocity Twist message
        self.pub_cmd.publish(speed)  # Publish the zero-velocity command
        if self.timer is not None:
            self.timer.cancel()  # Cancel the control loop timer
        self.get_logger().info("Stopping robot")  # Log that the robot is stopping
        self.flag_finish = True  # Set the finished flag to true

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    node = HolonomicClosedLoopSquareServer()  # Instantiate the service server node
    executor = MultiThreadedExecutor(num_threads=4)  # Create a multi-threaded executor with 3 threads
    executor.add_node(node)  # Add the node to the executor
    try:
        executor.spin()  # Spin the executor to process callbacks
    except KeyboardInterrupt:
        pass  # Handle keyboard interrupt gracefully

    node.get_logger().info("Shutting down")  # Log that the node is shutting down
    node.destroy_node()  # Destroy the node to free resources
    rclpy.shutdown()  # Shutdown the ROS 2 client library

if __name__ == '__main__':
    main()  # Entry point of the script
