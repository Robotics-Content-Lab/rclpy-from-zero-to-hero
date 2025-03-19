"""This module implements a Kalman filter for position estimation in 2D using cmd_vel (control) and joint_states (measurement) data."""
from threading import Lock
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from utils.utils.math import quaternion_from_euler

# Dimensions for the state, control, and measurement vectors
STATE_DIM = 3  # [x, y, theta]
CONTROL_DIM = 2  # [v, omega]
MEASUREMENT_DIM = 3  # [x, y, theta]

class KalmanFilter:
    @staticmethod
    def predict(x, P, u, A, B, Q):
        """
        Predict the state and covariance of the system.
        args:
            x: state estimate (position and orientation)
            P: state covariance
            u: control input (linear and angular velocity)
            A: state transition matrix
            B: control matrix
            Q: process noise covariance
        returns:
            x_pred: predicted state
            P_pred: predicted covariance
        """
        x_pred = A @ x + B @ u
        P_pred = A @ P @ A.T + Q
        return x_pred, P_pred

    @staticmethod
    def update(x_pred, P_pred, z, C, R):
        """
        Update the state and covariance of the system.
        args:
            x_pred: predicted state
            P_pred: predicted covariance
            z: measurement (position and orientation)
            C: measurement matrix
            R: measurement noise covariance
        returns:
            x_upd: updated state
            P_upd: updated covariance
        """
        K = P_pred @ C.T @ np.linalg.inv(C @ P_pred @ C.T + R)
        x_upd = x_pred + K @ (z - C @ x_pred)
        P_upd = (np.eye(STATE_DIM) - K @ C) @ P_pred
        return x_upd, P_upd


class KalmanFilterNode(Node):
    """ROS 2 node to implement a Kalman filter for position estimation in 2D using cmd_vel (control) and joint_states (measurement) data."""
    def __init__(self):
        super().__init__('kalman_filter_2d_node')
        self.init_variables()

        cb_group = ReentrantCallbackGroup()
        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10,
            callback_group=cb_group)
        self.subscription_joint_states = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10,
            callback_group=cb_group)

        self.publisher_odom_kf = self.create_publisher(Odometry, 'odom_kf', 10)
        
        self.timer = self.create_timer(1/self.rate, self.run_kalman_filter)

    def init_variables(self):
        self.lock = Lock()  # Lock to protect shared variables between threads
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])
        self.Q_noise = self.declare_parameter('Q_noise', [0.1, 0.1, 0.01]).value
        self.R_noise = self.declare_parameter('R_noise', [0.05, 0.05, 0.01]).value
        self.rate = 30

        # Initialize Kalman filter variables
        self.x = np.zeros((STATE_DIM, 1))  # State: [x, y, theta]
        self.P = np.eye(STATE_DIM)  # Covariance matrix
        self.u = np.zeros((CONTROL_DIM, 1))  # Control input: [v, omega]
        self.z = np.zeros((MEASUREMENT_DIM, 1))  # Measurement

        # State transition and control matrices
        self.A = np.eye(STATE_DIM)  # State transition matrix (identity for 2D)
        self.B = np.zeros((STATE_DIM, CONTROL_DIM))  # Control input matrix
        self.C = np.eye(MEASUREMENT_DIM)  # Measurement matrix

        # Process and measurement noise
        self.Q = np.diag(self.Q_noise)
        self.R = np.diag(self.R_noise)

        self.predicted = False  # Flag to indicate if prediction step has been completed
        self.cmd_vel_received = False  # Flag to indicate if cmd_vel message has been received
        self.joint_states_received = False  # Flag to indicate if joint_states have been received
        self.last_time = None  # To track time for dt calculation
        self.prev_joint_states = None  # To track previous joint states

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback function to receive cmd_vel messages.
        Update the control input (linear and angular velocity) and compute the control matrix B.
        """
        current_time = self.get_clock().now().nanoseconds * 1e-9
        with self.lock:
            if self.last_time is None:
                self.last_time = current_time
                return
            dt = current_time - self.last_time
            self.last_time = current_time

            # Update control input [v, omega]
            self.u[0, 0] = msg.linear.x  # Linear velocity
            self.u[1, 0] = msg.angular.z  # Angular velocity

            # Update the control matrix B based on the time step and robot's orientation
            theta = self.x[2, 0]
            self.B = np.array([[np.cos(theta) * dt, 0],  # Effect of v on x
                            [np.sin(theta) * dt, 0],  # Effect of v on y
                            [0, dt]])  # Effect of omega on theta

            self.cmd_vel_received = True

    def joint_states_callback(self, msg: JointState):
        """
        Callback function to receive joint_states messages.
        Compute the robot's position (x, y) and orientation (theta) based on the wheel joint states.
        """
        WHEEL_RADIUS = 0.033  # Wheel radius in meters
        WHEEL_BASE = 0.16     # Distance between the two wheels (wheelbase) in meters

        with self.lock:
            if self.prev_joint_states is None:
                # Store initial joint states (wheel angles)
                self.prev_joint_states = np.array([msg.position[0], msg.position[1]])  # Left and right wheel angles
                return

            # Calculate the change in wheel angles (radians)
            delta_left = msg.position[0] - self.prev_joint_states[0]  # Change in left wheel angle
            delta_right = msg.position[1] - self.prev_joint_states[1]  # Change in right wheel angle

            # Update previous joint states
            self.prev_joint_states = np.array([msg.position[0], msg.position[1]])

            # Compute the distance traveled by each wheel
            distance_left = delta_left * WHEEL_RADIUS
            distance_right = delta_right * WHEEL_RADIUS

            # Compute the average distance traveled and the change in orientation (theta)
            distance_center = (distance_left + distance_right) / 2.0
            delta_theta = (distance_right - distance_left) / WHEEL_BASE

            # Update the robot's position (x, y) and orientation (theta) using the odometry model
            theta = self.x[2, 0]  # Current orientation (theta)
            self.x[0, 0] += distance_center * np.cos(theta)  # Update x position
            self.x[1, 0] += distance_center * np.sin(theta)  # Update y position
            self.x[2, 0] += delta_theta  # Update orientation (theta)

            # Normalize theta to be within [-pi, pi]
            self.x[2, 0] = np.arctan2(np.sin(self.x[2, 0]), np.cos(self.x[2, 0]))

            # Set the measurement z to the updated position and orientation
            self.z = self.x.copy()
            self.joint_states_received = True

    def run_kalman_filter(self):
        with self.lock:
            if self.cmd_vel_received:
                self.cmd_vel_received = False
                self.x, self.P = KalmanFilter.predict(self.x, self.P, self.u, self.A, self.B, self.Q)
                self.predicted = True
                self.publish_odometry(self.x, self.P)

            if self.joint_states_received and self.predicted:
                self.joint_states_received = False
                self.x, self.P = KalmanFilter.update(self.x, self.P, self.z, self.C, self.R)
                self.publish_odometry(self.x, self.P)

    def publish_odometry(self, x, P):
        """
        Publish the estimated state as a Odometry message.
        """
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'

        odom_msg.pose.pose.position.x = x[0, 0]
        odom_msg.pose.pose.position.y = x[1, 0]

        # Represent orientation as quaternion (for simplicity, assume no roll/pitch)
        quat = quaternion_from_euler(0, 0, x[2, 0])
        odom_msg.pose.pose.position.x = x[0, 0]
        odom_msg.pose.pose.position.y = x[1, 0]
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        covariance_6x6 = np.zeros((3, 3))
        covariance_6x6[0:3, 0:3] = P
        odom_msg.pose.covariance = covariance_6x6.flatten().tolist()

        self.publisher_odom_kf.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    kalman_filter_node = KalmanFilterNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(kalman_filter_node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        kalman_filter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
