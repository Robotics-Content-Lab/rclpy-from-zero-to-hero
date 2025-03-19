from math import atan2, sqrt, radians, degrees, hypot

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
    """A simple proportional controller that computes a control output based on the error."""
    def __init__(self, k=0.1):
        self.k = k

    def compute(self, error):
        return self.k * error


class ClosedLoopPointController(Node):
    def __init__(self) -> None:
        super().__init__('controller')

        # Declare parameters for goal position and controller gains
        self._init_parameters()
        self._init_variables()

        # Publisher, Subscriber, and Timer
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ensure_publisher_connected()

        self.sub_odom = self.create_subscription(Odometry, '/odom',
                                                    self.odom_callback, 10)
        self.timer = self.create_timer(1/30.0, self.go_to)
        self.get_logger().info(f'Finished initializing node: {self.get_name()}')

    def _init_parameters(self):
        """Declare all required ROS parameters."""
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 1.0)
        self.declare_parameter('goal_th', 90.0)  # Goal in degrees

        self.k_rho = self.declare_parameter('k_rho', 0.3, ParameterDescriptor(
            name='k_rho', type=ParameterType.PARAMETER_DOUBLE,
            description='Proportional gain for distance to the goal (rho).'))
        
        self.k_alpha = self.declare_parameter('k_alpha', 0.8, ParameterDescriptor(
            name='k_alpha', type=ParameterType.PARAMETER_DOUBLE,
            description="Proportional gain for angle to the goal (alpha)."))
        
        self.k_beta = self.declare_parameter('k_beta', 0.3, ParameterDescriptor(
            name='k_beta', type=ParameterType.PARAMETER_DOUBLE,
            description="Proportional gain for orientation alignment (beta)."))

    def _init_variables(self):
        """Initialize variables for the controller."""
        # Initialize loggers
        self.cb_logger = rclpy.logging.get_logger('cb_logger')
        self.timer_logger = rclpy.logging.get_logger('timer_logger')

        # Initialize robot state variables
        self.x = 0.0  # Current x position
        self.y = 0.0  # Current y position
        self.theta = 0.0  # Current heading angle (yaw)

        self.max_vel = 0.22
        self.max_omega = 2.84

        # Goal tolerance and maximum velocity/omega settings
        self.goal_dist_tolerance = 0.02  # Distance tolerance for reaching goal
        self.goal_heading_tolerance = 10  # Heading tolerance in degrees

        # Create the goal point from the parameters
        self.goal = Point(x=self.get_parameter('goal_x').value,
                            y=self.get_parameter('goal_y').value,
                            z=self.get_parameter('goal_th').value)
        self.get_logger().info(f'Goal: {self.goal.x=}, {self.goal.y=}, {self.goal.z=}')

        # Initialize proportional controllers for distance and angle control
        self.rho_controller = P_Controller(self.k_rho.value)
        self.alpha_controller = P_Controller(self.k_alpha.value)
        self.beta_controller = P_Controller(self.k_beta.value)


    def ensure_publisher_connected(self):
        """Ensure the cmd_vel publisher is connected."""
        while self.pub_cmd.get_subscription_count() == 0:
            self.get_logger().info(f"Waiting for subscriber on {self.pub_cmd.topic} ... ",
                                    throttle_duration_sec=1.0)
            rclpy.spin_once(self, timeout_sec=0.1)

    def odom_callback(self, msg: Odometry) -> None:
        """Callback to update the robot's position and orientation."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        quat = msg.pose.pose.orientation
        (_, _, self.theta) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        self.cb_logger.debug(f'Current Pose: {self.x=}, {self.y=}, {degrees(self.theta)=}', throttle_duration_sec=0.5)

    def go_to(self) -> None:
        """Main loop function for controlling the robot to reach the goal."""
        def compute_beta_sign(th):
            """Compute the sign of the beta gain based on the goal angle."""
            if 0 < th <= (180):
                return -1
            else:
                return 1
        if self.goal is None:
            return

        # Calculate distance (rho) and angle errors (alpha, beta)
        delta_x, delta_y = self.goal.x - self.x, self.goal.y - self.y
        rho = sqrt(delta_x**2 + delta_y**2)  # Distance to goal

        goal_theta = radians(self.goal.z)  # Convert goal angle to radians
        alpha = normalize(atan2(delta_y, delta_x) - self.theta)  # Angle to goal point
        beta = normalize(goal_theta - atan2(delta_y, delta_x)) * compute_beta_sign(goal_theta)  # Heading error with correct sign
        heading_error = normalize(goal_theta - self.theta)

        self.timer_logger.info(f'Distance to goal: {rho:.2f}m, Heading difference: {degrees(heading_error):.2f} degrees', throttle_duration_sec=1)

        # Control logic
        if rho > self.goal_dist_tolerance:
            # Compute control commands using the proportional controllers
            v = self.rho_controller.compute(rho)
            omega_aplpha = self.alpha_controller.compute(alpha)
            omega_beta = self.beta_controller.compute(beta)
            angular_vel = omega_aplpha + omega_beta
        else:
            v = 0.0
            angular_vel = self.beta_controller.compute(beta)

        # Clamp velocities to their respective maximum values
        v = self.clamp(v, self.max_vel)
        omega = self.clamp(angular_vel, self.max_omega)

        # Publish command velocity to move the robot
        self.publish_velocity(v, omega)

        # Stop robot if it is within the distance and heading tolerances
        if rho < self.goal_dist_tolerance and abs(degrees(heading_error)) < self.goal_heading_tolerance:
            self.finalize_goal()

    @staticmethod
    def clamp(value, max_value):
        """Clamp the value between -max_value and +max_value."""
        return max(min(value, max_value), -max_value)

    def publish_velocity(self, linear, angular):
        """Publish velocity commands."""
        speed = Twist()
        speed.linear.x = linear
        speed.angular.z = angular
        self.timer_logger.debug(f'Publishing speed: {speed}', throttle_duration_sec=0.5)
        self.pub_cmd.publish(speed)

    def finalize_goal(self):
        """Handle the final step when the robot reaches the goal."""
        position_error = hypot(self.goal.x - self.x, self.goal.y - self.y)
        heading_error = abs(self.goal.z - degrees(self.theta))

        self.timer_logger.info(f'Final position error: {position_error:.2f}m\nFinal heading error: {heading_error:.2f}')
        self.goal = None
        self.stop_robot()
        self.timer.cancel()
        raise SystemExit

    def stop_robot(self):
        """Stop the robot by publishing zero velocity."""
        self.get_logger().warn('Stopping robot')
        self.publish_velocity(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopPointController()
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
