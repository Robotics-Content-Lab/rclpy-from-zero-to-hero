import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from utils.utils.math import quaternion_from_euler, euler_from_quaternion, normalize
from threading import Lock, Event
import numpy as np

STATE_DIM = 6 # [x, y, theta, \dot{x}, \dot{y}, \dot{\theta}]
CONTROL_DIM = 2 # [v, w]
MEASUREMENT_DIM = 4 # [linear_acceleration_x, angular_velocity_z]

class ExtendedKalmanFilter:
    @staticmethod
    def predict(mu, P, u, Q, dt):
        assert mu.shape == (STATE_DIM, 1), f"x must be a {STATE_DIM}x1 matrix"
        assert P.shape == (STATE_DIM, STATE_DIM), f"P must be a {STATE_DIM}x{STATE_DIM} matrix"
        assert u.shape == (CONTROL_DIM, 1), "u must be a 2x1 matrix"
        assert Q.shape == (STATE_DIM, STATE_DIM), f"Q must be a {STATE_DIM}x{STATE_DIM} matrix"

        x_pred = ExtendedKalmanFilter.g(mu, u, dt)
        G = ExtendedKalmanFilter.G(mu, u, dt)
        P_pred = G @ P @ G.T + Q
        return x_pred, P_pred

    @staticmethod
    def update(mu_pred, P_pred, z, z_, R, mu_, dt):
        assert mu_pred.shape == (STATE_DIM, 1), f"x_pred must be a {STATE_DIM}x1 matrix"
        assert P_pred.shape == (STATE_DIM, STATE_DIM), f"P_pred must be a {STATE_DIM}x{STATE_DIM} matrix"
        assert z.shape == (MEASUREMENT_DIM, 1), "z must be a 4x1 matrix"
        assert z_.shape == (MEASUREMENT_DIM, 1), "z_ must be a 4x1 matrix"
        assert R.shape == (MEASUREMENT_DIM, MEASUREMENT_DIM), "R must be a 4x4 matrix"

        H = ExtendedKalmanFilter.H()
        z_pred = ExtendedKalmanFilter.h(mu_pred, mu_, dt)

        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)

        mu_upd = mu_pred + K @ (z - z_pred)
        mu_upd[2, 0] = normalize(mu_upd[2, 0])
        
        P_upd = (np.eye(mu_pred.shape[0]) - K @ H) @ P_pred
        return mu_upd, P_upd

    @staticmethod
    def g(mu, u, dt):
        """
        Compute the predicted state.
        args:
            mu: state estimate
            u: control input
            dt: time step
        returns:
            mu_pred: predicted state
        Implements the motion model: mu_{t+1} = mu_t + f(mu_t, u_t) * dt
        """
        theta = mu[2, 0]
        v = u[0, 0]
        w = u[1, 0]

        mu_dot = np.array([
            [v * np.cos(theta)],
            [v * np.sin(theta)],
            [w],
            [v],
            [0],
            [w]
        ])  # State derivative

        mu_pred = mu + mu_dot * dt
        mu_pred[2, 0] = normalize(mu_pred[2, 0])
        return mu_pred

    @staticmethod
    def h(mu, mu_, dt):
        """
        Compute the predicted measurement.
        args:
            x: state estimate
            x_: previous state estimate
            dt: time step
        returns:
            z_pred: predicted measurement
        """
        theta = mu[2, 0]  # Yaw angle
        omega = mu[5, 0]  # Angular velocity (yaw rate)
        a_x = (mu[2][0] - mu_[2][0]) / (dt + 1e-6)
        a_y = (mu[3][0] - mu_[3][0]) / (dt + 1e-6)

        return np.array([[theta], [omega], [a_x], [a_y]])

    
    @staticmethod
    def G(mu, u, dt):
        """
        Compute the Jacobian of the state transition matrix.
        args:
            mu: state estimate
            u: control input
            dt: time step
        returns:
            G: Jacobian of the state transition matrix
        """
        theta = mu[2, 0]
        v = u[0, 0]
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        G = np.array([
            [1, 0, -dt * v * sin_theta, dt * cos_theta, 0, 0],
            [0, 1, dt * v * cos_theta, dt * sin_theta, 0, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        return G

    @staticmethod
    def H():
        """
        Compute the Jacobian of the measurement matrix.
        returns:
            H: Jacobian of the measurement matrix
        """
        H = np.zeros((MEASUREMENT_DIM, STATE_DIM))  # 4 measurements (theta, omega, a_x, a_y), STATE_DIM state variables
        H[0, 2] = 1  # ∂theta/∂theta
        H[1, 5] = 1  # ∂omega/∂omega
        H[2, 3] = 1  # ∂a_x/∂v_x
        H[3, 4] = 1  # ∂a_y/∂v_y
        return H

class ExtendedKalmanFilterNode(Node):
    def __init__(self):
        super().__init__('extended_kalman_filter_node')
        self.init_variables()
        self.add_on_set_parameters_callback(self.parameter_change_cb)
        cb_group = ReentrantCallbackGroup()
        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10,
            callback_group=cb_group)
        self.subscription_imu = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10,
            callback_group=cb_group)

        self.publisher_odom_ekf = self.create_publisher(Odometry, 'odom_ekf', 10)
        self.timer = self.create_timer(1/self.rate, self.run_ekf)

    def init_variables(self):
        self.lock = Lock()
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])
        self.Q_noise = self.declare_parameter('Q_noise', [0.01, 0.01, 0.01, 0.01, 0.01, 0.01], ParameterDescriptor(description="Process noise")).value
        self.R_noise = self.declare_parameter('R_noise', [0.001, 4.0e-08, 0.000289, 0.000289], ParameterDescriptor(description="Measurement noise")).value
        self.rate = 30  # [Hz]

        self.last_time = None
        self.last_imu = None
        
        self.predicted = Event()
        self.cmd_vel_received = Event()
        self.imu_received = Event()

        self.mu = np.zeros((STATE_DIM, 1))  # State vector [x, y, theta, \dot{x}, \dot{y}, \dot{\theta}]
        self.last_mu = np.zeros((STATE_DIM, 1))  # Previous state vector, used to compute acceleration from change in velocity in h(mu, mu_, dt)
        self.P = np.eye(STATE_DIM)  # Covariance matrix
        self.u = np.zeros((CONTROL_DIM, 1))  # Control input [v, w]
        self.z = np.zeros((MEASUREMENT_DIM, 1))  # Measurement [linear_acceleration_x, angular_velocity_z]
        self.Q = np.diag(self.Q_noise)  # Process noise covariance
        self.R = np.diag(self.R_noise)  # Measurement noise covariance

    def parameter_change_cb(self, params):
        for param in params:
            if param.name == 'Q_noise':
                assert np.diag(param.value).shape == (STATE_DIM,), f"Q_noise must be a {STATE_DIM}x1 matrix"
                assert all([val >= 0 for val in param.value]), "Q_noise must be positive"
                self.Q = np.diag(param.value)
            if param.name == 'R_noise':
                assert np.diag(param.value).shape == (MEASUREMENT_DIM,), "R_noise must be a 2x1 matrix"
                assert all([val >= 0 for val in param.value]), "R_noise must be positive"
                self.R = np.diag(param.value)

    def cmd_vel_callback(self, msg):
        """Callback function to receive cmd_vel (control input) messages."""
        with self.lock:
            self.u = np.array([[msg.linear.x], [msg.angular.z]])
            self.cmd_vel_received.set()

    def imu_callback(self, msg: Imu):
        """Callback function to receive IMU (measurement) messages."""
        (_, _, theta) = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        with self.lock:
            self.z = np.array([[theta], [msg.angular_velocity.z], [msg.linear_acceleration.x], [msg.linear_acceleration.y]])
            if self.last_imu is None:
                self.last_imu = self.z
                return
            self.imu_received.set()

    def run_ekf(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time if self.last_time is not None else 1/self.rate
        with self.lock:
            if self.cmd_vel_received.is_set():
                self.mu, self.P = ExtendedKalmanFilter.predict(self.mu, self.P, self.u, self.Q, dt)
                self.predicted.set()
                self.cmd_vel_received.clear()

            if self.imu_received.is_set() and self.predicted.is_set():
                self.mu, self.P = ExtendedKalmanFilter.update(self.mu, self.P, self.z, self.last_imu, self.R, self.last_mu, dt)
                self.predicted.clear()
                self.imu_received.clear()

            self.last_mu = self.mu.copy()
            self.last_time = self.get_clock().now().nanoseconds / 1e9
            self.publish_odometry(self.mu, self.P)

    def publish_odometry(self, mu, P):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = mu[0, 0]
        odom_msg.pose.pose.position.y = mu[1, 0]
        quat = quaternion_from_euler(0, 0, mu[2, 0])
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        covariance_6x6 = np.zeros((6, 6))
        covariance_6x6[0:6, 0:6] = P
        odom_msg.pose.covariance = covariance_6x6.flatten().tolist()

        self.publisher_odom_ekf.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    extended_kalman_filter_node = ExtendedKalmanFilterNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(extended_kalman_filter_node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        extended_kalman_filter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
