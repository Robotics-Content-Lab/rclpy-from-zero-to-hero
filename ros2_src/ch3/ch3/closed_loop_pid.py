import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.exceptions import ROSInterruptException
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, radians, degrees, hypot

from utils.utils.math import euler_from_quaternion, normalize

class PID_Controller:
    def __init__(self, kp, ki, kd):
        """A simple PID controller"""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.previous_error = 0

    def compute(self, error, dt):
        """Compute the PID output
        Args:
            error (float): Error between the desired and actual value
            dt (float): Time difference between two consecutive calls
        Returns:
            float: PID output = kp * error + ki * integral + kd * derivative
        """
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class PIDControlNode(Node):
    def __init__(self):
        super().__init__('pid_control_node')
        self._init_variables()

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ensure_publisher_connected()

        self.timer = self.create_timer(0.1, self.go_to)

    def ensure_publisher_connected(self):
        """Ensure the cmd_vel publisher is connected."""
        while self.pub_cmd.get_subscription_count() == 0:
            self.get_logger().info(f"Waiting for subscriber on {self.pub_cmd.topic} ... ",
                                    throttle_duration_sec=1.0)
            rclpy.spin_once(self, timeout_sec=0.1)

    def _init_variables(self):
        # Initialize loggers
        self.cb_logger = rclpy.logging.get_logger('cb_logger')
        self.timer_logger = rclpy.logging.get_logger('timer_logger')

        # Initialize the robot's pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Initialize the goal point and tolerances
        self.goal_dist_tolerance = 0.1  # meters
        self.goal_heading_tolerance = 10  # degrees

        # Create the goal point from the parameters
        self.goal = Point(x=self.declare_parameter('goal_x', 2.0).value,
                            y=self.declare_parameter('goal_y', 3.0).value,
                            z=self.declare_parameter('goal_th', 90.0).value)
        self.get_logger().info(f'Coordinates of the goal: x={self.goal.x}, y={self.goal.y}, theta={self.goal.z}')
        self.max_vel = self.declare_parameter('max_vel', 0.22).value
        self.max_omega = self.declare_parameter('max_omega', 2.84).value

        # Create PID controllers for the distance, angle, and heading
        kp_rho = self.declare_parameter('kp_rho', 0.3).value
        ki_rho = self.declare_parameter('ki_rho', 0.01).value
        kd_rho = self.declare_parameter('kd_rho', 0.5).value
        self.rho_pid = PID_Controller(kp_rho, ki_rho, kd_rho)

        kp_alpha = self.declare_parameter('kp_alpha', 0.8).value
        ki_alpha = self.declare_parameter('ki_alpha', 0.01).value
        kd_alpha = self.declare_parameter('kd_alpha', 0.5).value
        self.alpha_pid = PID_Controller(kp_alpha, ki_alpha, kd_alpha)

        kp_beta = self.declare_parameter('kp_beta', 0.05).value
        ki_beta = self.declare_parameter('ki_beta', 0.01).value
        kd_beta = self.declare_parameter('kd_beta', 0.5).value
        self.beta_pid = PID_Controller(kp_beta, ki_beta, kd_beta)

        # Initialize the previous clock for time difference calculation for the integral and derivative terms
        self.prev_clock = None

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def go_to(self):
        """Main loop function for controlling the robot to reach the goal."""
        def compute_beta_sign(th):
            """Compute the sign of the beta gain based on the goal angle."""
            if 0 < th <= (180):
                return -1
            else:
                return 1
        if self.goal is None:
            return
        if self.prev_clock is None:
            self.prev_clock = self.get_clock().now()
            return
        dt = (self.get_clock().now() - self.prev_clock).nanoseconds / 1e9

        # Calculate distance (rho) and angle errors (alpha, beta)
        delta_x, delta_y = self.goal.x - self.x, self.goal.y - self.y
        rho = sqrt(delta_x**2 + delta_y**2)
        if rho < self.goal_dist_tolerance:
            rho = 0.0

        goal_theta = radians(self.goal.z)
        alpha = normalize(atan2(delta_y, delta_x) - self.theta)
        beta = normalize(goal_theta - atan2(delta_y, delta_x)) * compute_beta_sign(goal_theta)
        heading_error = normalize(goal_theta - self.theta)
        if abs(heading_error) < radians(self.goal_heading_tolerance):
            heading_error = 0.0

        self.timer_logger.info(f'Distance to goal: {rho:.2f}m, Heading difference: {degrees(heading_error):.3f} degrees',
                                throttle_duration_sec=1)

        # Control logic
        if rho > self.goal_dist_tolerance:
            # If we are still far from the goal, control both position and heading
            v = self.rho_pid.compute(rho, dt)
            omega_alpha = self.alpha_pid.compute(alpha, dt)
            omega_beta = self.beta_pid.compute(beta, dt)
            angular_vel = omega_alpha + omega_beta
        else:
            # We are close enough to the goal, focus only on heading control
            v = 0.0
            angular_vel = self.beta_pid.compute(heading_error, dt)

        # Clamp the velocities to the maximum values
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
    node = PIDControlNode()

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
