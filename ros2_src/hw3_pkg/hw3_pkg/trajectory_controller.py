import rclpy
from rclpy.exceptions import ROSInterruptException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt
from typing import Tuple
import time


from utils.utils.math import euler_from_quaternion, normalize

import numpy as np

def create_star() -> Tuple[np.ndarray, np.ndarray]:
    num_vertices = 5
    
    angles = np.linspace(0, 2 * np.pi, num_vertices, endpoint=False)
    
    x = np.cos(angles)
    y = np.sin(angles)
    
    indices = [0, 2, 4, 1, 3, 0]
    star_x = x[indices]
    star_y = y[indices]
    
    return star_x, star_y

class PIController:
    def __init__(self, kp, ki):
        self.kp = kp
        self.ki = ki
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * (dt.nanoseconds * 1e-9)
        return self.kp * error + self.ki * self.integral

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.last_error = 0
    
    def compute(self, error, dt):
        self.integral += error * (dt.nanoseconds * 1e-9)
        derivative = (error - self.last_error) / (dt.nanoseconds * 1e-9)
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_controller')
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.goal_x = None
        self.goal_y = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None

        self.kp_linear = self.declare_parameter('kp_linear', 1.0).value
        self.ki_linear = self.declare_parameter('ki_linear', 0.1).value
        self.kp_angular = self.declare_parameter('kp_angular', 1.6).value
        self.ki_angular = self.declare_parameter('ki_angular', 0.05).value
        self.kd_angular = self.declare_parameter('kd_angular', 0.3).value

        self.linear_controller = PIController(self.kp_linear, self.ki_linear)
        self.angular_controller = PIDController(self.kp_angular, self.ki_angular, self.kd_angular)

        self.star_x, self.star_y = create_star()
        self.trajectory_index = 0

        self.state = 'rotate'  # Start with rotating towards the first goal

        while self.pub_cmd.get_subscription_count() == 0:
            self.get_logger().info(f"Waiting for sub on {self.pub_cmd.topic} ... ", throttle_duration_sec=1.0)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Trajectory Controller Node has been started')

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        (_, _, self.theta) = euler_from_quaternion(quaternion=[quat.x, quat.y, quat.z, quat.w])

    def compute_angle_to_target(self, target_x, target_y):
        delta_x = target_x - self.x
        delta_y = target_y - self.y
        return atan2(delta_y, delta_x)

    def timer_callback(self):
        if self.last_time is None:
            self.last_time = self.get_clock().now()
            return
        current_time = self.get_clock().now()

        def clamp(num, max_vel):
            '''Clamp the input between -max_vel and +max_vel'''
            return max(min(num, max_vel), -max_vel)
        
        if self.goal_x is None and self.trajectory_index < len(self.star_x):
            self.goal_x = self.star_x[self.trajectory_index]
            self.goal_y = self.star_y[self.trajectory_index]

        if self.goal_x is not None:
            delta_x = self.goal_x - self.x
            delta_y = self.goal_y - self.y
            distance_to_target = sqrt(delta_x ** 2 + delta_y ** 2)
            target_theta = atan2(delta_y, delta_x)
            angular_error = normalize(target_theta - self.theta)

            twist_msg = Twist()

            if self.state == 'rotate':
                angular_velocity = clamp(self.angular_controller.compute(angular_error, (current_time - self.last_time)), 2.88)
                twist_msg.angular.z = angular_velocity
                self.pub_cmd.publish(twist_msg)

                if abs(angular_error) < 0.02:  # If the robot is facing the goal, switch to move state
                    self.state = 'move'
                    self.stop_robot()
                    self.get_logger().info(f'Rotated to target {self.trajectory_index}, moving towards it')

            elif self.state == 'move':
                linear_velocity = clamp(self.linear_controller.compute(distance_to_target, (current_time - self.last_time)), 0.22)
                angular_velocity = clamp(self.angular_controller.compute(angular_error, (current_time - self.last_time)), 2.88)
                twist_msg.linear.x = linear_velocity
                twist_msg.angular.z = angular_velocity
                self.pub_cmd.publish(twist_msg)

                if distance_to_target < 0.1:  # If the robot reached the goal, switch to rotate state for next goal
                    self.goal_x = None
                    self.goal_y = None
                    self.trajectory_index += 1
                    self.state = 'rotate'
                    self.stop_robot()
                    self.get_logger().info(f'Reached target {self.trajectory_index - 1}, rotating to next target')
                    
                    if self.trajectory_index == len(self.star_x):
                        self.get_logger().info('Finished trajectory, exiting')
                        self.timer.cancel()
                        raise SystemExit

            self.last_time = current_time
                        
    def stop_robot(self) -> None:
        self.get_logger().warn('Stopping robot')
        speed: Twist = Twist()
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        self.pub_cmd.publish(speed)

def main(args=None):
    time.sleep(5)
    rclpy.init(args=args)
    node = TrajectoryController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ROSInterruptException, SystemExit):
        node.get_logger().info('Exiting')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
