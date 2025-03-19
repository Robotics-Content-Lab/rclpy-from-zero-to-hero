import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Imu
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from threading import Lock

STATE_DIM = 6  # [x, y, theta, vx, vy, omega]
CONTROL_DIM = 3  # [v_x_cmd, v_y_cmd, omega_cmd]
MEASUREMENT_DIM = 4  # [theta, omega]

class ExtendedKalmanFilter:
    current_state = None  # Class variable to store current state
    
    @staticmethod
    def set_current_state(x):
        """
        Update the current state estimate
        args:
            x: current state estimate
        """
        ExtendedKalmanFilter.current_state = x

    @staticmethod
    def predict(x, P, u, Q, dt):
        assert x.shape == (STATE_DIM, 1), f"x must be a {STATE_DIM}x1 matrix"
        assert P.shape == (STATE_DIM, STATE_DIM), f"P must be a {STATE_DIM}x{STATE_DIM} matrix"
        assert u.shape == (CONTROL_DIM, 1), "u must be a 2x1 matrix"
        assert Q.shape == (STATE_DIM, STATE_DIM), f"Q must be a {STATE_DIM}x{STATE_DIM} matrix"

        x_pred = ExtendedKalmanFilter.g(x, u, dt)
        G = ExtendedKalmanFilter.G(x, u, dt)
        P_pred = G @ P @ G.T + Q
        return x_pred, P_pred

    @staticmethod
    def update(x_pred, P_pred, z, z_, R, x_, dt):
        assert x_pred.shape == (STATE_DIM, 1), f"x_pred must be a {STATE_DIM}x1 matrix"
        assert P_pred.shape == (STATE_DIM, STATE_DIM), f"P_pred must be a {STATE_DIM}x{STATE_DIM} matrix"
        assert z.shape == (MEASUREMENT_DIM, 1), "z must be a 4x1 matrix"
        assert z_.shape == (MEASUREMENT_DIM, 1), "z_ must be a 4x1 matrix"
        assert R.shape == (MEASUREMENT_DIM, MEASUREMENT_DIM), "R must be a 4x4 matrix"

        H = ExtendedKalmanFilter.H()
        z_pred = ExtendedKalmanFilter.h(x_pred, x_, z, dt)

        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)
        # Measurement residual
        y = z - z_pred

        if ExtendedKalmanFilter.is_outlier(y, S, 8) or ExtendedKalmanFilter.is_unreasonable_measurement(z):
            # Reject the measurement as it is an outlier
            return x_pred, P_pred
        
        x_upd = x_pred + K @ y
        P_upd = (np.eye(x_pred.shape[0]) - K @ H) @ P_pred
        return x_upd, P_upd
    
    @staticmethod
    def mahalanobis_distance(y, S):
        return np.sqrt(y.T @ np.linalg.inv(S) @ y)

    @staticmethod
    def is_outlier(y, S, threshold):
        mahalanobis_dist = ExtendedKalmanFilter.mahalanobis_distance(y, S)
        return mahalanobis_dist > threshold

    @staticmethod
    def is_unreasonable_measurement(z):
        # Define thresholds for unreasonable measurements
        MAX_ACCELERATION = 0.80  # m/s^2
        MAX_ANGULAR_VELOCITY = 2.84  # rad/s

        linear_acceleration_x = z[2, 0]
        angular_velocity_z = z[1, 0]

        if abs(linear_acceleration_x) > MAX_ACCELERATION:
            return True
        elif abs(angular_velocity_z) > MAX_ANGULAR_VELOCITY:
            return True
        else:
            return False
    
    @staticmethod
    def g(x, u, dt):
        """
        Compute the predicted state.
        args:
            x: state estimate
            u: control input
            dt: time step
        returns:
            x_pred: predicted state
        Implements the motion model: x_{t+1} = x_t + f(x_t, u_t) * dt
        """
        if dt < 1e-6:  # Skip update for negligible time steps
            return x
            
        theta = x[2, 0]
        vx = u[0, 0]  # Directly use commanded velocity
        vy = u[1, 0]
        omega = u[2, 0]
        
        # Calculate velocity components in world frame
        vx_world = vx * np.cos(theta) - vy * np.sin(theta)
        vy_world = vx * np.sin(theta) + vy * np.cos(theta)
        
        # Update state with velocity
        x_pred = np.array([
            [x[0, 0] + vx_world * dt],
            [x[1, 0] + vy_world * dt],
            [x[2, 0] + omega * dt],
            [vx],  # Set velocity to commanded value
            [vy],
            [omega]
        ])
        
        # Normalize orientation
        x_pred[2, 0] = np.arctan2(np.sin(x_pred[2, 0]), np.cos(x_pred[2, 0]))
        
        return x_pred
    
    @staticmethod
    def G(x, u, dt):
        """
        Compute the Jacobian of the motion model.
        args:
            x: state estimate
            u: control input
            dt: time step
        returns:
            G: Jacobian matrix
        """
        if dt < 1e-6:  # Skip update for negligible time steps
            return np.eye(STATE_DIM)
            
        theta = x[2, 0]
        vx = u[0, 0]  # Commanded velocity
        vy = u[1, 0]
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        
        # Partial derivatives of world frame velocities
        dvx_world_dtheta = -vx * sin_theta - vy * cos_theta
        dvy_world_dtheta = vx * cos_theta - vy * sin_theta
        
        G = np.eye(STATE_DIM)
        
        # Position partials
        G[0, 2] = dvx_world_dtheta * dt  # d(x)/d(theta)
        G[0, 3] = cos_theta * dt         # d(x)/d(vx)
        G[0, 4] = -sin_theta * dt        # d(x)/d(vy)
        
        G[1, 2] = dvy_world_dtheta * dt  # d(y)/d(theta)
        G[1, 3] = sin_theta * dt         # d(y)/d(vx)
        G[1, 4] = cos_theta * dt         # d(y)/d(vy)
        
        # Orientation partials
        G[2, 5] = dt  # d(theta)/d(omega)
        
        # Velocity partials are zero since we directly set velocities
        G[3, 3] = 0.0  # d(vx)/d(vx)
        G[4, 4] = 0.0  # d(vy)/d(vy)
        G[5, 5] = 0.0  # d(omega)/d(omega)
        
        return G


    @staticmethod
    def h(x, x_, z, dt):
        """
        Compute the predicted measurement.
        args:
            x: state estimate
            x_: previous state estimate
            z: measurement vector [theta, omega, a_x_body, a_y_body]
            dt: time step
        returns:
            z_pred: predicted measurement
        """
        theta = x[2, 0]  # Yaw angle
        omega = x[5, 0]  # Angular velocity (yaw rate)
        
        # Get IMU measurements
        imu_theta = z[0, 0]  # IMU orientation
        a_x_body = z[2, 0]  # Linear acceleration x from IMU
        a_y_body = z[3, 0]  # Linear acceleration y from IMU
        
        # Remove gravity component using IMU orientation
        g = 9.81  # Gravity constant
        a_x_body -= g * np.sin(imu_theta)
        a_y_body += g * np.cos(imu_theta)
        
        # Transform to world frame using current orientation
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        
        # Linear accelerations in world frame
        a_x_world = a_x_body * cos_theta - a_y_body * sin_theta
        a_y_world = a_x_body * sin_theta + a_y_body * cos_theta
        
        return np.array([
            [theta],
            [omega],
            [a_x_world],  # Linear acceleration in world x
            [a_y_world]   # Linear acceleration in world y
        ])
    
    @staticmethod
    def H():
        """
        Compute the Jacobian of the measurement matrix.
        returns:
            H: Jacobian of the measurement matrix
        """
        H = np.zeros((MEASUREMENT_DIM, STATE_DIM))
        
        # Orientation measurement
        H[0, 2] = 1  # theta
        
        # Angular velocity measurement
        H[1, 5] = 1  # omega
        
        # Linear acceleration measurements
        # Get current state from class variable
        if ExtendedKalmanFilter.current_state is None:
            return np.zeros((MEASUREMENT_DIM, STATE_DIM))
            
        x = ExtendedKalmanFilter.current_state
        theta = x[2, 0]
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        
        # World x acceleration partials
        H[2, 2] = -x[3, 0] * sin_theta - x[4, 0] * cos_theta  # d(a_x)/dtheta
        H[2, 3] = cos_theta  # d(a_x)/dvx
        H[2, 4] = -sin_theta  # d(a_x)/dvy
        
        # World y acceleration partials
        H[3, 2] = x[3, 0] * cos_theta - x[4, 0] * sin_theta  # d(a_y)/dtheta
        H[3, 3] = sin_theta  # d(a_y)/dvx
        H[3, 4] = cos_theta  # d(a_y)/dvy
        
        return H
    
class EKFHolonomicDrive(Node):
    def __init__(self):
        super().__init__('ekf_holonomic_drive')
        self.get_logger().info("EKF Holonomic Drive Node Started")
        self.init_variables()
        self.callback_group = ReentrantCallbackGroup()
        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            '/robotino/cmd_vel',
            self.cmd_vel_callback,
            10,
            callback_group=self.callback_group)
        self.subscription_imu = self.create_subscription(
            Imu,
            '/robotino/imu/out',
            self.imu_callback,
            10,
            callback_group=self.callback_group)
        self.publisher_odom_ekf = self.create_publisher(Odometry, 'odom_ekf', 10)
        
        self.timer = self.create_timer(1/self.rate, self.run_ekf)
        self.get_logger().info("EKF Holonomic Drive Node initialized")

    def init_variables(self):
        self.lock = Lock()
        self.rate = 30
        self.last_time = None
        self.last_imu = None
        
        self.predicted = False
        self.cmd_vel_received = False
        self.imu_received = False

        self.Q_noise = self.declare_parameter('Q_noise', [0.001, 0.001, 0.001, 0.001, 0.001, 0.001], ParameterDescriptor(description="Process noise")).value
        self.R_noise = self.declare_parameter('R_noise', [1.0, 1.0, 100.0, 100.0], ParameterDescriptor(description="Measurement noise")).value
        self.Q = np.diag(self.Q_noise)
        self.R = np.diag(self.R_noise)

        self.x = np.zeros((STATE_DIM, 1))  # State vector [x, y, theta, \dot{x}, \dot{y}, \dot{\theta}]
        self.last_x = np.zeros((STATE_DIM, 1))  # Previous state vector, used to compute acceleration from change in velocity in h(x, x_, dt)
        self.P = np.eye(STATE_DIM)  # Covariance matrix
        self.u = np.zeros((CONTROL_DIM, 1))  # Control input [v, w]
        self.z = np.zeros((MEASUREMENT_DIM, 1))  # Measurement [linear_acceleration_x, angular_velocity_z]
        self.filter = ExtendedKalmanFilter()
        

    def cmd_vel_callback(self, msg):
        """Callback function to receive cmd_vel (control input) messages."""
        # self.get_logger().info("Received cmd_vel message")
        with self.lock:
            self.u = np.array([[msg.linear.x], [msg.linear.y], [msg.angular.z]])
            self.cmd_vel_received = True

    def imu_callback(self, msg: Imu):
        """Callback function to receive IMU (measurement) messages."""
        # self.get_logger().info("Received IMU message")
        (_, _, theta) = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        with self.lock:
            self.z = np.array([[theta], [msg.angular_velocity.z], [msg.linear_acceleration.x], [msg.linear_acceleration.y]])
            if self.last_imu is None:
                self.last_imu = self.z
                return
            self.imu_received = True

    def run_ekf(self):
        # self.get_logger().info("Running EKF")
        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        dt = current_time - self.last_time if self.last_time is not None else 0
        with self.lock:
            if self.cmd_vel_received:
                self.cmd_vel_received = False
                self.x, self.P = self.filter.predict(self.x, self.P, self.u, self.Q, dt)
                self.predicted = True
                self.cmd_vel_received = False

            if self.imu_received and self.predicted:
                self.imu_received = False
                self.x, self.P = self.filter.update(self.x, self.P, self.z, self.last_imu, self.R, self.last_x, dt)
                self.imu_received = False
                self.predicted = False

            self.last_x = self.x.copy()
            self.last_time = self.get_clock().now().nanoseconds / 1e9
            self.publish_odometry(self.x, self.P)


    def publish_odometry(self, x, P):
        """Publish the estimated state as an Odometry message"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set the pose (x, y, theta)
        odom_msg.pose.pose.position.x = x[0, 0]
        odom_msg.pose.pose.position.y = x[1, 0]
        quat = quaternion_from_euler(0, 0, x[2, 0])
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Set the velocity (vx, vy, omega)
        odom_msg.twist.twist.linear.x = x[3, 0]
        odom_msg.twist.twist.linear.y = x[4, 0]
        odom_msg.twist.twist.angular.z = x[5, 0]
        
        covariance_6x6 = np.zeros((6, 6))
        covariance_6x6[0:6, 0:6] = P
        odom_msg.pose.covariance = covariance_6x6.flatten().tolist()

        # Publish the odometry message
        self.publisher_odom_ekf.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EKFHolonomicDrive()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
