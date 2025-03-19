"""This module implements a Kalman filter for position estimation in 1D using cmd_vel (control) and joint_states (measurement) data."""
from threading import Lock, Event
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor, Parameter
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

STATE_DIM = 1 # [x]
CONTROL_DIM = 1 # [v]
MEASUREMENT_DIM = 1 # [x]

class KalmanFilter:
    @staticmethod
    def predict(mu, P, u, A, B, Q, dt):
        """
        Predict the state and covariance of the system.
        args:
            mu: state estimate
            P: state covariance
            u: control input
            A: state transition matrix
            B: control matrix
            Q: process noise covariance
            dt: time step
        returns:
            mu_pred: predicted state
            P_pred: predicted covariance
        """
        assert mu.shape == (STATE_DIM, 1), "mu must be a 1x1 matrix"
        assert P.shape == (STATE_DIM, STATE_DIM), f"P must be a {STATE_DIM}x{STATE_DIM} matrix"
        assert u.shape == (CONTROL_DIM, 1), "u must be a {CONTROL_DIM}x1 matrix"
        assert A.shape == (STATE_DIM, STATE_DIM), f"A must be a {STATE_DIM}x{STATE_DIM} matrix"
        assert B.shape == (STATE_DIM, CONTROL_DIM), f"B must be a {STATE_DIM}x{CONTROL_DIM} matrix"
        assert Q.shape == (STATE_DIM, STATE_DIM), f"Q must be a {STATE_DIM}x{STATE_DIM} matrix"
        A = np.eye(STATE_DIM) + A * dt  # Discretize the state transition matrix
        B = B * dt  # Discretize the control matrix
        mu_pred = A @ mu + B @ u  # Predict the state
        P_pred = A @ P @ A.T + Q  # Predict the state covariance
        return mu_pred, P_pred

    @staticmethod
    def update(mu_pred, P_pred, z, C, R):
        """
        Update the state and covariance of the system.
        args:
            mu_pred: predicted state
            P_pred: predicted covariance
            z: measurement
            C: measurement matrix
            R: measurement noise covariance
        returns:
            mu_upd: updated state
            P_upd: updated covariance
        """
        assert mu_pred.shape == (STATE_DIM, 1), "x_pred must be a 1x1 matrix"
        assert P_pred.shape == (STATE_DIM, STATE_DIM), f"P_pred must be a {STATE_DIM}x{STATE_DIM} matrix"
        assert z.shape == (MEASUREMENT_DIM, 1), "z must be a {MEASUREMENT_DIM}x1 matrix"
        assert C.shape == (MEASUREMENT_DIM, STATE_DIM), f"C must be a {MEASUREMENT_DIM}x{STATE_DIM} matrix"
        assert R.shape == (MEASUREMENT_DIM, MEASUREMENT_DIM), "R must be a {MEASUREMENT_DIM}x{MEASUREMENT_DIM} matrix"

        S = C @ P_pred @ C.T + R  # Innovation, calculates how much the measurement differs from the prediction
        K = P_pred @ C.T @ np.linalg.inv(S)  # Kalman gain, determines how much the measurement should be trusted
        y = (z - C @ mu_pred)

        # Check if the measurement is an outlier
        if KalmanFilter.is_outlier(y, S, 8):
            # Reject the measurement as it is an outlier
            mahalanobis_dist = KalmanFilter.mahalanobis_distance(y, S)
            print(f"Measurement rejected due to high Mahalanobis distance: {mahalanobis_dist}")
            return mu_pred, P_pred

        mu_upd = mu_pred + K @ (z - C @ mu_pred)  # Update the state estimate
        P_upd = (np.eye(STATE_DIM) - K @ C) @ P_pred  # Update the state covariance
        return mu_upd, P_upd

    @staticmethod
    def mahalanobis_distance(y, S):
        return np.sqrt(y.T @ np.linalg.inv(S) @ y)

    @staticmethod
    def is_outlier(y, S, threshold):
        mahalanobis_dist = KalmanFilter.mahalanobis_distance(y, S)
        return mahalanobis_dist > threshold

class KalmanFilterNode(Node):
    """ROS 2 node to implement a Kalman filter for position estimation in 1D using cmd_vel (control) and joint_states (measurement) data."""
    def __init__(self):
        super().__init__('kalman_filter_node')
        self.init_variables()
        self.add_on_set_parameters_callback(self.parameter_change_cb)

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
        self.Q_noise = self.declare_parameter('Q_noise', [1.0], ParameterDescriptor(description="Process noise")).value
        self.R_noise = self.declare_parameter('R_noise', [0.1], ParameterDescriptor(description="Measurement noise")).value
        self.rate = 50  # [Hz]
       
        self.WHEEL_RADIUS = 0.033  # Wheel radius in meters
        self.predicted = Event()  # Event to indicate if the prediction step has been completed
        self.last_time = None  # Last time the cmd_vel message was received
        self.cmd_vel_received = Event()  # Flag to indicate if a cmd_vel message has been received
        self.prev_joint_states = None  # Previous joint states
        self.joint_states_received = Event()  # Flag to indicate if joint states have been received
        
        self.mu = np.array([[0.0]])  # State vector [x]
        self.P = np.eye(STATE_DIM)  # Covariance matrix
        self.u = np.array([[0.0]])  # Control input [v]
        self.A = np.array([[1]])  # State transition matrix, describes how the state evolves over time
        self.B = np.array([[1]])  # Control matrix, describes how the control input affects the state
        self.C = np.array([[1]])  # Measurement matrix, maps the state to the measurement space
        self.Q = np.diag(self.Q_noise)  # Process noise covariance
        self.R = np.diag(self.R_noise)  # Measurement noise covariance

    def parameter_change_cb(self, params: list[Parameter]):
        for param in params:
            if param.name == 'Q_noise':
                self.Q = np.diag(param.value)
            if param.name == 'R_noise':
                self.R = np.diag(param.value)

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback function to receive cmd_vel messages.
        Calculate the distance traveled by the robot and update the control input.
        Integrates the linear velocity over time to get the distance traveled.
        Arrives with a frequency of ~ 10 Hz.
        """
        with self.lock:  # Lock to protect shared variables (self.u)
            self.u[0, 0] = msg.linear.x
            self.cmd_vel_received.set()

    def joint_states_callback(self, msg: JointState):
        """
        Callback function to receive joint_states messages.
        Calculate the distance traveled by the robot and update the measurement.
        Computes the arc length traveled by the wheel using the current and previous joint states.
        Arrives with a frequency of ~ 25 Hz.
        """
        if self.prev_joint_states is None:
            self.prev_joint_states = np.array([[msg.position[0]]])
            return
        with self.lock:  # Lock to protect shared variables (self.z, self.prev_joint_states)
            arc = (msg.position[0] - self.prev_joint_states) * self.WHEEL_RADIUS
            self.z = arc
            self.prev_joint_states = self.z
            self.joint_states_received.set()

    def run_kalman_filter(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        with self.lock:  # Lock to protect shared variables (self.u, self.z, self.x, self.P)
            dt = current_time - self.last_time if self.last_time is not None else 0
            if self.cmd_vel_received.is_set():
                self.mu, self.P = KalmanFilter.predict(self.mu, self.P, self.u, self.A, self.B, self.Q, dt)
                self.predicted.set()
                self.cmd_vel_received.clear()

            if self.joint_states_received.is_set() and self.predicted.is_set():
                self.mu, self.P = KalmanFilter.update(self.mu, self.P, self.z, self.C, self.R)
                self.joint_states_received.clear()
                self.predicted.clear()

            self.publish_odometry(self.mu, self.P)
            self.last_time = self.get_clock().now().nanoseconds / 1e9

    def publish_odometry(self, mu, P):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = mu[0, 0]
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0

        covariance = np.zeros((6, 6))  # Odometry msgs have a 6x6 covariance matrix
        covariance[0, 0] = P[0, 0]  # Covariance of the estimated position (x)
        covariance[1,1] = 0.1  # Just to make the covariance an ellipse
        odom_msg.pose.covariance = covariance.flatten().tolist()  # Flatten the covariance matrix to a 1D list

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
