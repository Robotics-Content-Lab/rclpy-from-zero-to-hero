import os
import numpy as np
from typing import List, Dict, Tuple, Optional
from sklearn.cluster import DBSCAN
# import matplotlib
# matplotlib.use('module://mplcairo.tk')
from scipy.optimize import minimize
from scipy.spatial.distance import euclidean
from scipy.stats import chi2
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from threading import Lock
import yaml
from multiprocessing import Pool, Manager

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
import tf2_ros


def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

def load_landmarks(filename: str) -> List[Dict[str, float]]:
    with open(filename, 'r') as f:
        landmarks = yaml.safe_load(f)
    return landmarks

def normalize_angle(angle: float) -> float:
    return (angle + np.pi) % (2 * np.pi) - np.pi

class Plotter():
    """ 
    A class for plotting the EKF localization results.
    """
    # TODO add subplot to compare theta
    def __init__(self, Map) -> None:
        self.map = Map
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Robot Odometry")
        self.ax.set_aspect('equal')
        self.plot_landmarks()

    def update_plot(self, states: List[np.ndarray], covariance: List[np.ndarray], observed_landmarks, correspondences, cluster: np.ndarray, GTpose: List[List], save_fig=False):
        """ Update all plots """
        states = np.asarray(states)
        state_x = states[:, 0]
        state_y = states[:, 1]
        odom_x, odom_y, odom_th = GTpose

        self.ax.clear()
        # Plot starting positions
        if len(state_x) != 0 and len(state_y) != 0:
            self.ax.scatter(state_x[0], state_x[0], marker="o", color="red", s=50, linewidths=1.5, edgecolors="k", label="EKF Start")
        if len(odom_x) != 0 and len(odom_y) != 0:
            self.ax.scatter(odom_x[0], odom_y[0], marker="o", color="green", s=50, linewidths=1.5, edgecolors="k", label="GT Start")
        self.plot_landmarks()
        self.plot_clustered_observations(clustered_observations=cluster, GTpose=np.asarray([odom_x[-1], odom_y[-1], odom_th[-1]]))
        if correspondences != None:
            self.plot_correspondences(correspondences=correspondences)
        if observed_landmarks != None:
            self.plot_observed_landmarks(observed_landmarks)
        self.plot_state_and_covariance(state=states[-1], covariance=covariance[-1])
        self.plot_odometries(odom_x=odom_x, odom_y=odom_y, state_x=state_x, state_y=state_y)
        self.ax.set_xlabel('x[m]')
        self.ax.set_ylabel('y[m]')

        plt.subplots_adjust(bottom=0.2)
        # Put a legend below current axis
        self.ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15),
                fancybox=True, shadow=True, ncol=3)
        plt.draw()
        plt.pause(0.01)

    def plot_clustered_observations(self, clustered_observations: List[Dict['str', float]], GTpose: np.ndarray, color='magenta', label='Clustered Observations'):
        if clustered_observations.shape[0] == 0:
            return
        
        # Convert observations to Cartesian coordinates
        X = np.array([[r * np.cos(b), r * np.sin(b)] for r, b in clustered_observations])

        # Extract ground truth pose information
        x_gt, y_gt, theta_gt = GTpose
        t = np.array([x_gt, y_gt])

        # Create rotation matrix and translation vector
        R = np.array([[np.cos(theta_gt), -np.sin(theta_gt)],
                    [np.sin(theta_gt), np.cos(theta_gt)]])

        X_transformed = X @ R.T + t # Transform observations into GT frame
        self.ax.scatter(X_transformed[:, 0], X_transformed[:, 1], c=color, marker='x', label=label, alpha=0.5) # Plot the transformed points

    def plot_landmarks(self):
        """ Plot the GT landmarks. """
        x_values = []
        y_values = []
        for dict in self.map:
            x_values.append(dict['x'])
            y_values.append(dict['y'])
        self.ax.scatter(x_values, y_values, marker="*", color="yellow",
                        s=50, linewidths=1.5, edgecolors="k", label="Landmark GT")

    def plot_observed_landmarks(self, observed_landmarks: List[Dict[str, float]]):
        """ Plot the observed landmarks. """
        x_values = []
        y_values = []
        for index in observed_landmarks:
            if index is not None:
                x_values.append(self.map[index]['x'])
                y_values.append(self.map[index]['y'])
        if len(x_values) == 0 or len(y_values) == 0:
            return
        self.ax.scatter(x_values, y_values, marker="o", color="red", s=50, linewidths=1.5, alpha=0.1, linestyle="--", edgecolors="r", label="Observed Landmarks")

    def plot_correspondences(self, correspondences):
        """ Plot the correspondences between the current state and the observed landmarks. """
        for index in correspondences:
            if index is not None:
                self.ax.plot([self.mu[0], self.map[index]['x']], [self.mu[1], self.map[index]['y']], color='b', linestyle="--", alpha=0.5)

    def plot_odometries(self, odom_x: List[float], odom_y: List[float], state_x: List[float], state_y: List[float]):
        """ Plot the odometry data of the GT and EKF state. """
        self.ax.plot(odom_x, odom_y, color='green', label='Odometry GT')
        self.ax.plot(state_x, state_y, color='red', label='EKF State')

    def plot_state_and_covariance(self, state: np.ndarray, covariance: np.ndarray):
        """ Plot the current state and covariance. """
        mu = state[0:2]
        P = covariance[0:2, 0:2]
        # Ensure P is symmetric
        P = 0.5 * (P + P.T)
        # Add a small constant for numerical stability
        P = P + 1e-10 * np.eye(P.shape[0])
        
        self.ax.plot(mu[0], mu[1], 'x', color='red') # Plot the current state

        # Plot the covariance as a blue ellipse
        eigenvalues, eigenvectors = np.linalg.eig(P[0:2, 0:2])
        eigenvalues = np.real(eigenvalues)
        eigenvectors = np.real(eigenvectors)
        angle = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))
        ellipse = Ellipse((mu[0], mu[1]), 2*np.sqrt(max(5.991*eigenvalues[0], 1e-10)),
                            2*np.sqrt(max(5.991*eigenvalues[1], 1e-10)), angle=angle, fill=False, color='blue')
        self.ax.add_artist(ellipse)

    def save_config(self, mode: str, node_name: str):
        """ Destructor to save the plot window and config. """
        if not os.getenv("ROS2_PACKAGE_PATH"):
            print("ROS2_PACKAGE_PATH not set, cannot save images")
            return
        img_path = os.path.join(os.getenv("ROS2_PACKAGE_PATH"), 'projects', 'ekf_localization', 'images')
        print("--------------------"*10)
        print(img_path)
        print("--------------------"*10)
        if not os.path.exists(img_path):
            os.makedirs(img_path)
        if not os.path.exists(os.path.join(img_path, '.gitignore')):
            command = f"echo '*' > {os.path.join(img_path, '.gitignore')}"
            os.system(command)
        identifier = self.get_clock().now().to_msg().sec
        img_name = f"{node_name}_{mode}_{identifier}"
        with open(os.path.join(img_path, f"{img_name}_config.yaml"), 'w') as f:
            config = {
                "max_association_distance": self.get_parameter('max_association_distance').value,
                "process_noise": {
                    "initial": np.diag(self.get_parameter('process_noise').value).tolist(),
                    "final": self.Q.tolist()
                },
                "measurement_noise": {
                    "initial": np.diag(self.get_parameter('measurement_noise').value).tolist(),
                    "final": self.R.tolist()
                },
                "covariance": {
                    "initial": np.diag(self.get_parameter('initial_covariance').value).tolist(),
                    "final": self.P.tolist()
                },
                "state": {
                    "initial_ekf": self.get_parameter('initial_state').value,
                    "initial_odom": [self.odom_x[0], self.odom_y[0], 0],
                    "final_ekf": self.mu.tolist(),
                    "final_odom": [self.odom_x[-1], self.odom_y[-1], 0]
                }
            }
            yaml.dump(config, f)
        plt.savefig(os.path.join(img_path, f"{img_name}.png"))
        plt.close()


class ROSNode(Node):
    """
    A ROS node for the EKF localization project.
    """
    def __init__(self, node_name: str = 'ekf_node', *args, **kwargs):
        super().__init__(node_name, *args)
        self.declare_parameter('process_noise', kwargs.get('process_noise', [0.1, 0.1, 0.1]))
        self.declare_parameter('measurement_noise', kwargs.get('measurement_noise', [5.0, 5.0]))
        self.declare_parameter('initial_covariance', kwargs.get('initial_covariance', [5.0, 5.0, 3.0]))
        self.declare_parameter('initial_state', kwargs.get('initial_state', [0.0, 0.0, 0.0]))
        self.declare_parameter('save_fig', kwargs.get('save_fig', False))

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/ekf_pose', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 1)  # For motion model and prediction
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 1)  # For measurement model and update
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 1)  # For GT comparison

        self._u = np.array([0.0, 0.0])
        self._measurements = np.array([])
        self._clustered_measurements = np.array([])

        self._last_time = None
        self._dt = 0.0

        self._odom = []

    @property
    def u(self) -> np.ndarray:
        return self._u
    @property
    def measurements(self) -> np.ndarray:
        return self._measurements
    @measurements.setter
    def measurements(self, value: np.ndarray):
        self._measurements = value
    @property
    def clustered_measurements(self) -> np.ndarray:
        return self._clustered_measurements
    @clustered_measurements.setter
    def clustered_measurements(self, value: np.ndarray):
        self._clustered_measurements = value
    @property
    def last_time(self) -> float:
        return self._last_time
    @last_time.setter
    def last_time(self, value: float):
        self._last_time = value
    @property
    def dt(self) -> float:
        return self._dt
    @dt.setter
    def dt(self, value: float):
        self._dt = value
    @property
    def odom_x(self) -> List[float]:
        return [x for x, _, _ in self._odom]
    @property
    def odom_y(self) -> List[float]:
        return [y for _, y, _ in self._odom]
    @property
    def odom_th(self) -> List[float]:
        return [th for _, _, th in self._odom]
    @property
    def odom(self) -> List[List[float]]:
        return self._odom
    @odom.setter
    def odom(self, value: List[List[float]]):
        self._odom = value
    
    def publish_pose_and_tf(self):
        def expand_covariance(P):
            expanded_P = np.zeros((6, 6))
            expanded_P[0:2, 0:2] = P[0:2, 0:2]  # x, y
            expanded_P[5, 5] = P[2, 2]  # theta
            return expanded_P
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = self.mu[0]
        pose_msg.pose.pose.position.y = self.mu[1]
        pose_msg.pose.pose.orientation.z = np.sin(self.mu[2]/2.0)
        pose_msg.pose.pose.orientation.w = np.cos(self.mu[2]/2.0)
        pose_msg.pose.covariance = expand_covariance(self.P).flatten().tolist()
        self.pose_pub.publish(pose_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = self.mu[0]
        tf_msg.transform.translation.y = self.mu[1]
        tf_msg.transform.rotation.z = np.sin(self.mu[2]/2.0)
        tf_msg.transform.rotation.w = np.cos(self.mu[2]/2.0)
        self.tf_broadcaster.sendTransform(tf_msg)

    def odom_callback(self, msg):
        """ Callback function for odometry/GT messages. """
        # Extract x, y from odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        th = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)[2]
        self.odom.append([x, y, th])

    def cmd_vel_callback(self, msg: Twist):
        time = self.get_clock().now().to_msg()
        current_time = time.sec + time.nanosec * 1e-9

        if self.last_time == None:
            self.dt = 0
        else:
            self.dt = current_time - self.last_time
        self.last_time = current_time

        self.v = msg.linear.x
        self.omega = msg.angular.z
        u = [self.v, self.omega]
        self._u = u

    def scan_callback(self, msg: LaserScan) -> np.ndarray:
        self._measurements = self.process_laser_scan(msg)
        self._clustered_measurements = self.cluster_observations(self._measurements)

    @staticmethod
    def process_laser_scan(msg: LaserScan) -> np.ndarray:
        angle_increment = msg.angle_increment  # Compute the angle increment between each laser scan ray
        measurements_list = []  # Initialize the measurements list
        # Loop over all ranges and get the corresponding angles
        for i, range_val in enumerate(msg.ranges):
            # Check if the range measurement is valid
            if msg.range_min < range_val < msg.range_max:  # filtering out invalid and inf measurements
                bearing = msg.angle_min + i * angle_increment  # counting counter clock-wise
                measurements_list.append([range_val, bearing])  # Store valid measurements in the list
        measurements = np.array(measurements_list, dtype=np.float32)
        return measurements

    @staticmethod
    def cluster_observations(observations: np.ndarray, eps: float = 0.5, min_samples: int = 2) -> np.ndarray:
        if len(observations) == 0:
            return []
        
        # Convert observations to Cartesian coordinates
        ranges = observations[:, 0]
        bearings = observations[:, 1]
        X_x = ranges * np.cos(bearings)
        X_y = ranges * np.sin(bearings)
        X = np.column_stack((X_x, X_y))


        clustering: DBSCAN = DBSCAN(eps=eps, min_samples=min_samples).fit(X)
        labels = clustering.labels_
        
        clustered_observations = np.ndarray(shape=(0, 2), dtype=np.float32)
        
        unique_labels = np.unique(labels)
        for label in unique_labels:
            if label == -1:
                continue
            indices = np.where(labels == label)[0]
            centroid_x = np.mean(X[indices, 0])
            centroid_y = np.mean(X[indices, 1])
            centroid_range = np.sqrt(centroid_x**2 + centroid_y**2)
            centroid_bearing = np.arctan2(centroid_y, centroid_x)
            clustered_observations = np.vstack((clustered_observations, np.array([centroid_range, centroid_bearing])))

        return clustered_observations

class EKF():
    """
    An EKF localization node with unkown correspondences.
    """
    def __init__(self, landmark_file: str=os.path.join(get_package_share_directory('ekf_localization'), 'config', 'landmarks.yaml'), 
                 logger = None, **kwargs):

        self.logger = logger
        self._mu = kwargs.get('initial_state')  # mu: State [x, y, theta]
        self._P = kwargs.get('initial_covariance')  # P: Covariance
        self._Q = kwargs.get('process_noise')  # Q : Process Noise
        self._Q_base = self.Q.copy()  # Used for updating the process noise
        self._R = kwargs.get('measurement_noise')  # R: Measurement Noise
        self._map = load_landmarks(landmark_file)
        if logger != None:
            self.logger = logger
            self.logger.info("Finished init ekf")

    @property
    def mu(self) -> np.ndarray:
        return self._mu
    @mu.setter
    def mu(self, value: np.ndarray):
        self._mu = value
    @property
    def P(self) -> np.ndarray:
        return self._P
    @P.setter
    def P(self, value: np.ndarray):
        self._P = value
    @property
    def Q(self) -> np.ndarray:
        return self._Q
    @Q.setter
    def Q(self, value: np.ndarray):
        self._Q = value
    @property
    def R(self) -> np.ndarray:
        return self._R
    @R.setter
    def R(self, value: np.ndarray):
        self._R = value
    @property
    def map(self) -> List[Dict[str, float]]:
        return self._map
    @map.setter
    def map(self, value: List[Dict[str, float]]):
        self._map = value

    @staticmethod
    def f(state: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
        x, y, theta = state[0], state[1], state[2]
        v, omega = u[0], u[1]
        if np.isclose(np.abs(omega), 0.0):  # Driving straight
            x = x + v*dt*np.cos(theta).item()
            y = y + v*dt*np.sin(theta).item()
            theta = theta
        else:
            x = x - (v/omega)*np.sin(theta).item() + \
                (v/omega)*np.sin(theta + omega*dt).item()
            y = y + (v/omega)*np.cos(theta).item() - \
                (v/omega)*np.cos(theta + omega*dt).item()
            theta = normalize_angle(theta + omega*dt)
        return np.array([x, y, theta])

    @staticmethod
    def jacobian_of_f(state: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
        # unpack the state
        x, y, theta = state[0], state[1], state[2]
        v, omega = u[0], u[1]
        # Compute the jacobian of the motion model
        if np.isclose(np.abs(omega), 0.0):  # Driving straight
            F = np.array([
                [1, 0, -v*dt*np.sin(theta)],
                [0, 1, v*dt*np.cos(theta)],
                [0, 0, 1]
            ])
        else:  # Turning motion
            F = np.array([
                [1, 0, v/omega*(-np.cos(theta) + np.cos(theta+omega*dt))],
                [0, 1, v/omega*(-np.sin(theta) + np.sin(theta+omega*dt))],
                [0, 0, 1]
            ])
        return F

    @staticmethod
    def h(state: np.ndarray, landmark_pos: Dict[str, float]) -> np.ndarray:
        dx = landmark_pos['x'] - state[0]
        dy = landmark_pos['y'] - state[1]
        q = dx**2 + dy**2
        z_hat = np.array(
            [np.sqrt(q), normalize_angle(np.arctan2(dy, dx) - state[2])])
        return z_hat

    @staticmethod
    def jacobian_of_h(state: np.ndarray, landmark_pos: Dict[str, float]) -> np.ndarray:
        # Get the x and y differences between the state and the landmark position
        dx = landmark_pos['x'] - state[0]
        dy = landmark_pos['y'] - state[1]
        # Calculate the squared distance between the state and the landmark position
        q = dx**2 + dy**2
        # Calculate the distance between the state and the landmark position
        sqrt_q = np.sqrt(q)
        # Calculate the Jacobian of the measurement
        H = np.array([[-sqrt_q * dx, -sqrt_q * dy, 0],
                    [dy, -dx, -q]])
        H /= q
        return H

    @staticmethod
    def update_process_noise(u: np.ndarray, Q_old: np.ndarray) -> np.ndarray:
        # Adjust the process noise based on the control inputs
        v, omega = u[0], u[1]
        Q = Q_old.copy()
        Q[0, 0] = Q_old[0, 0] * np.abs(v)
        Q[1, 1] = Q_old[1, 1] * np.abs(v)
        Q[2, 2] = Q_old[2, 2] * np.abs(omega**2)
        return Q

    def predict(self, mu: np.ndarray, P: np.ndarray, u: np.ndarray, dt: float, Q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        # Update the process noise
        Q = self.update_process_noise(u, Q)
        # Compute the Jacobian of the motion model
        F = self.jacobian_of_f(mu, u, dt)
        # Compute the predicted state
        mu = self.f(mu, u, dt)
        # Compute the predicted covariance
        P = F @ P @ F.T + Q
        return mu, P

    def update(self, mu: np.ndarray, P: np.ndarray, z: dict, measurement_noise: np.ndarray, landmark_pos: np.ndarray, epsilon: float = 1.1) -> Tuple[np.ndarray, np.ndarray]:
        # Calculate the expected measurement
        expected_measurement = self.h(mu, landmark_pos)
        # Calculate the measurement residual (innovation)
        measurement_residual = np.array([z[0] - expected_measurement[0],
                                        normalize_angle(z[1] - expected_measurement[1])])
        H = self.jacobian_of_h(mu, landmark_pos)
        # Calculate S = H @ P @ H^T + R and check if it's near singular
        S = H @ P @ H.T + measurement_noise  # Innovation covariance
        K = P @ H.T @ np.linalg.inv(S)
        # Update the state and covariance
        mu = mu + K @ measurement_residual
        mu[2] = normalize_angle(mu[2])
        I_KH = np.eye(len(mu)) - K @ H
        # Joseph's form is numerically more stable
        P = I_KH @ P @ I_KH.T + K @ measurement_noise @ K.T
        return mu, P * epsilon

class EKFNode(ROSNode, EKF):
    def __init__(self, node_name: str = 'ekf_node', *args, **kwargs):
        ROSNode.__init__(self, node_name=node_name, *args, **kwargs)
        
        # Extract or set default values for parameters needed by EKF
        landmark_file = os.path.join(get_package_share_directory('ekf_localization'), 'config', 'landmarks.yaml')
        initial_state = np.asarray(self.get_parameter('initial_state').value)
        initial_covariance = np.asarray(self.get_parameter('initial_covariance').value)
        process_noise = np.diag(self.get_parameter('process_noise').value)
        measurement_noise = np.diag(self.get_parameter('measurement_noise').value)

        # Initialize EKF
        EKF.__init__(self, logger=self.get_logger(), landmark_file=landmark_file, initial_state=initial_state, 
                     initial_covariance=initial_covariance, process_noise=process_noise, 
                     measurement_noise=measurement_noise)
        
        self.declare_parameter('data_association_method', kwargs.get('data_association_method', 'BSC'))
        self.declare_parameter('max_association_distance', kwargs.get('max_association_distance', 1.0))
        self.method = self.declare_parameter('method', kwargs.get('method', 'MHT')).value
        self.SIMILARITY_THRESHOLD = self.declare_parameter('similarity_threshold', kwargs.get('similarity_threshold', 0.1)).value

        self.max_hypotheses = self.declare_parameter('max_hypotheses', kwargs.get('max_hypotheses', 10))
        self.THRESHOLD= 0.1
        self.hypotheses = self.initialize_multiple_states(num_hypotheses=self.max_hypotheses)

        plot_fq = 2
        localization_fq = 30
        self.lock = Lock()
        self.create_timer(1/localization_fq, self.localize_callback)
        self.create_timer(1/plot_fq, self.plot_callback)

        # Initialize the state for plotting
        self.Plotter = Plotter(Map=self._map)
        self.states = [self.mu]
        self.covariances = [self.P]
        self.get_logger().info("Finished init ekf plot node")

    def initialize_multiple_states(self, num_hypotheses):
        hypotheses = []
        for _ in range(num_hypotheses):
            # Randomize the initial state around the original initial state
            randomized_state = self.mu + np.random.normal(loc=0, scale=[5, 5, 0.1], size=self.mu.shape)
            
            # Initialize the covariance matrix (P) for each hypothesis.
            P = self.P.copy()
            
            # Add the new hypothesis
            hypotheses.append({
                'state': randomized_state,
                'covariance': P,
                'likelihood': 1.0  # Initialize likelihood
            })
        return hypotheses

    def localize_callback(self):
        if self.last_time == None:
            return
        with self.lock:
            # Prediction Phase
            self.mu, self.P = self.predict(self.mu, self.P, self._u, self.dt, self.Q)

            # If clustered measurements are available, use them for the update
            if len(self._clustered_measurements) > 0:
                # Find correspondences using data Association
                correspondence_indices = self.data_association(
                    state = self.mu, measurements = self._clustered_measurements, known_landmarks=self.map, covariance=self.P, 
                    measurement_noise=self.R, max_distance=self.get_parameter('max_association_distance').value,
                    method=self.get_parameter('data_association_method').value  # Choose either 'NN' or 'MLDA'
                )

                # Update State
                self.update_state_with_correspondences(self._clustered_measurements, correspondence_indices)

            self.states.append(self.mu)
            self.covariances.append(self.P)
            self.publish_pose_and_tf()

    def plot_callback(self):
        if self.last_time == None:
            return
        with self.lock:
            self.Plotter.update_plot(
                states=self.states,
                covariance=self.covariances,
                observed_landmarks=None,
                correspondences=None,
                cluster=self._clustered_measurements,
                GTpose=[self.odom_x, self.odom_y, self.odom_th],
                save_fig=self.get_parameter('save_fig').value
            )

    def update_state_with_correspondences(self, observations: np.ndarray, correspondences: List[Optional[int]]) -> None:
        """ Update state and covariance using known correspondences """
        for i, z in enumerate(observations):
            if correspondences[i] is not None:
                self.mu, self.P = self.update(self.mu, self.P, z, self.R, self.map[correspondences[i]])
            else:
                # We assume that the map is complete, so we should never get here (only used in SLAM later)
                pass

    def data_association(self, state: np.ndarray, measurements: List[Dict['str', float]], known_landmarks: List[Dict['str', float]], covariance, measurement_noise, max_distance: float = np.inf, method: str = 'NN'):
        if method == 'NN':
            return self.nearest_neighbor_data_association(state=state, measurements=measurements, known_landmarks=known_landmarks, max_distance=max_distance)
        elif method == 'MLDA':
            return self.maximum_likelihood_data_association(state=state, measurements=measurements, known_landmarks=known_landmarks, covariance=covariance, measurement_noise=measurement_noise, max_distance=max_distance)
        elif method == "BSC":
            bsc_correspondences = self.bsc_data_association(state=state, measurements=measurements, known_landmarks=known_landmarks, covariance=covariance, measurement_noise=measurement_noise)
            single_correspondences = []
            for potential in bsc_correspondences:
                if len(potential) == 0:
                    single_correspondences.append(None)
                else:
                    single_correspondences.append(int(potential[0]))
            return single_correspondences
        elif method == 'JCBB':
            return self.jcbb_data_association(state=state, measurements=measurements, known_landmarks=known_landmarks, covariance=covariance, measurement_noise=measurement_noise, alpha=0.05)[0]
        else:
            self.get_logger().error('Invalid data association method.')
            raise ValueError('Invalid data association method.')

    def filter_nearby_landmarks(self, state: np.ndarray, known_landmarks: List[Dict['str', float]], radius: float):
        """Filter landmarks that are within a certain radius of the current state."""
        nearby_landmarks = []
        for i, landmark in enumerate(known_landmarks):
            distance = euclidean([state[0], state[1]], [landmark['x'], landmark['y']])
            if distance < radius:
                nearby_landmarks.append((i, landmark))
        return nearby_landmarks

    def compute_mahalanobis_distance(self, state: np.ndarray, measurement:np.ndarray, landmark: Dict['str', float], covariance: np.ndarray, measurement_noise: np.ndarray):
        expected_measurement = self.h(state, landmark)
        measurement_residual = np.array(
            [measurement[0] - expected_measurement[0], normalize_angle(measurement[1] - expected_measurement[1])])
        H = self.jacobian_of_h(state, landmark)
        S = H @ covariance @ H.T + measurement_noise
        mahalanobis_distance = measurement_residual.T @ np.linalg.inv(S) @ measurement_residual
        return mahalanobis_distance

    def nearest_neighbor_data_association(self, state: np.ndarray, measurements: List[Dict['str', float]], known_landmarks: List[Dict['str', float]], max_distance: float = np.inf):
        correspondence_indices = []
        for measurement in measurements:
            min_distance = np.inf
            closest_index = -1
            for i, landmark in enumerate(known_landmarks):
                distance = np.sqrt(
                    (state[0] - landmark['x'])**2 + (state[1] - landmark['y'])**2)
                if distance < min_distance and distance < max_distance:
                    min_distance = distance
                    closest_index = i
            correspondence_indices.append(
                closest_index if closest_index != -1 else None)
        return correspondence_indices

    def maximum_likelihood_data_association(self, state: np.ndarray, measurements: np.ndarray, known_landmarks: List[Dict['str', float]], covariance: np.ndarray, measurement_noise: np.ndarray, max_distance: float = np.inf):
        correspondence_indices = []
        for measurement in measurements:
            min_mahalanobis_distance = np.inf
            closest_index = -1
            for i, landmark in enumerate(known_landmarks):
                mahalanobis_distance = self.compute_mahalanobis_distance(
                    state=state, measurement=measurement, landmark=landmark, covariance=covariance, measurement_noise=measurement_noise)
                if mahalanobis_distance < min_mahalanobis_distance and mahalanobis_distance < max_distance:
                    min_mahalanobis_distance = mahalanobis_distance
                    closest_index = i
            correspondence_indices.append(
                closest_index if closest_index != -1 else None)
        return correspondence_indices

    def predict_measurement(self, state: np.ndarray, best_landmark_id: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        # Retrieve the coordinates of the best landmark
        landmark_pos = self.map[best_landmark_id]

        # Predict the measurement using the current state and landmark
        predicted_measurement = self.h(state, landmark_pos)

        # Calculate the Jacobian of the measurement model
        H = self.jacobian_of_h(state, landmark_pos)

        # Compute the innovation covariance
        S = H @ self.P @ H.T + self.R

        return predicted_measurement, H, S

    def check_known_landmark(self, measurement):
        current_position = self.mu[:2]  # [x, y, theta]
        for landmark in self.map:
            dx = landmark['x'] - current_position[0]
            dy = landmark['y'] - current_position[1]
            
            expected_range = np.sqrt(dx**2 + dy**2)
            expected_bearing = np.arctan2(dy, dx) - self.mu[2]  # Subtracting the robot's orientation
            expected_bearing = normalize_angle(expected_bearing)
            
            # Compute the difference between the expected and actual measurements
            range_diff = np.abs(expected_range - measurement['range'])
            bearing_diff = np.abs(normalize_angle(expected_bearing - measurement['bearing']))
            
            # If the difference is below a certain threshold, we consider it a match
            if range_diff < self.THRESHOLD and bearing_diff < 0.05:
                return True
        return False

    def extract_best_global_hypothesis(self, measurements: List[Dict[str, float]]):
        likelihoods = [self.evaluate_likelihood(hypothesis, measurements) for hypothesis in self.hypotheses]
        max_likelihood_index = np.argmax(likelihoods)
        best_hypothesis = self.hypotheses[max_likelihood_index]
        return best_hypothesis

    def evaluate_likelihood(self, hypothesis: Dict[str, np.ndarray], measurements: List[Dict[str, float]]) -> float:
        total_likelihood = 1.0
        for measurement in measurements:
            predicted_measurements = [self.h(hypothesis['state'], {'x': landmark['x'], 'y': landmark['y']}) for landmark in self.map]
                    
            # Find the landmark with the predicted measurement closest to the actual measurement
            diffs = [np.linalg.norm(np.array([measurement['range'] - pred[0], normalize_angle(measurement['bearing'] - pred[1])])) for pred in predicted_measurements]
            closest_measurement = predicted_measurements[np.argmin(diffs)]
            
            # Calculate the measurement residual (innovation)
            innovation = np.array([measurement['range'] - closest_measurement[0], 
                                normalize_angle(measurement['bearing'] - closest_measurement[1])])

            # Compute the likelihood for this measurement using the Gaussian likelihood formula
            S = self.R  # Here, we use the measurement noise as the covariance
            likelihood = (1.0 / (2 * np.pi * np.linalg.det(S)**0.5)) * np.exp(-0.5 * innovation.T @ np.linalg.inv(S) @ innovation)
            
            total_likelihood *= likelihood  # Combine likelihoods for all measurements
                
        if total_likelihood is None:
            return -np.inf
        return total_likelihood

    def best_correspondence(self, state: np.ndarray, covariance: np.ndarray, measurement: Dict[str, float]) -> Tuple[np.ndarray, np.ndarray]:
        best_landmark_id = None
        S = None
        if self.method == "MHT":  # Multi-Hypothesis Testing
            best_landmark_id, S = self.mht(state, covariance, measurement)
        elif self.method == "MLDA":  # Maximum Likelihood Data Association
            best_landmark_id, S = self.mlda(state, covariance, measurement)
        else:
            raise ValueError("Invalid method specified!")

        return best_landmark_id, S

    def nn(self, state, covariance, measurement):
        # Using the nearest neighbor method
        correspondence_indices = self._nearest_neighbor_data_association(
            state=state, 
            measurements=[measurement], 
            known_landmarks=self.map, 
            max_distance=np.inf
        )
        best_landmark_id = correspondence_indices[0]
        if best_landmark_id is not None:
            predicted_measurement, H, S = self.predict_measurement(state, best_landmark_id)
            return best_landmark_id, S
        else:
            return None, None

    def mht(self, state, covariance, measurement):
        best_landmark_id = None
        best_S = None
        best_likelihood = -np.inf
        
        for hypothesis in self.hypotheses:
            local_state = hypothesis['state']
            local_covariance = hypothesis['covariance']
            
            # Run MLDA or any other data association method
            correspondence_indices = self._maximum_likelihood_data_association(
                state=local_state, 
                measurements=[measurement], 
                known_landmarks=self.map, 
                covariance=local_covariance, 
                measurement_noise=self.R, 
                max_distance=np.inf
            )
            
            landmark_id = correspondence_indices[0]
            if landmark_id is not None:
                predicted_measurement, H, S = self.predict_measurement(local_state, landmark_id)
                
                # Compute the likelihood for this hypothesis and measurement
                likelihood = self.evaluate_likelihood(hypothesis, [measurement])
                
                if likelihood > best_likelihood:
                    best_likelihood = likelihood
                    best_landmark_id = landmark_id
                    best_S = S
                    
        return best_landmark_id, best_S

    def mlda(self, state, covariance, measurement):
        correspondence_indices = self.maximum_likelihood_data_association(
            state=state, 
            measurements=[measurement], 
            known_landmarks=self.map, 
            covariance=covariance, 
            measurement_noise=self.R, 
            max_distance=np.inf
        )
        best_landmark_id = correspondence_indices[0]
        if best_landmark_id is not None:
            predicted_measurement, H, S = self.predict_measurement(state, best_landmark_id)
            return best_landmark_id, S
        else:
            # Fallback to nearest neighbor or other method
            best_landmark_id, best_S = self.nn(state, covariance, measurement)
            return best_landmark_id, best_S

    # TODO speed up this function, this shit never converges
    def jcbb_data_association(self, state: np.ndarray, measurements: np.ndarray, known_landmarks: List[Dict['str', float]], covariance: np.ndarray, measurement_noise: np.ndarray, alpha: float = 0.05):
        """ Joint Compatibility Branch and Bound Data Association (Parallelized) """
        n = len(measurements)
        # Filter landmarks within a certain radius to reduce the search space
        nearby_landmarks = self.filter_nearby_landmarks(state, known_landmarks, 5.0)
        m = len(nearby_landmarks)

        manager = Manager()
        best_cost = manager.Value('d', np.inf)
        best_hypothesis = manager.list([None] * n)

        def search(hypothesis, cost, i):
            if cost.value >= best_cost.value:
                return

            if i == n:
                if cost.value < best_cost.value:
                    best_cost.value = cost.value
                    best_hypothesis[:] = [None if h is None else h[1] for h in hypothesis]
                return

            # No association case
            search(hypothesis + [(i, None)], cost, i + 1)
            
            for j in range(m):
                new_hypothesis = hypothesis + [(i, j)]
                new_cost = cost.value + self.compute_mahalanobis_distance(state, measurements[i], nearby_landmarks[j], covariance, measurement_noise)
                if new_cost < chi2.ppf(1 - alpha, 2 * (i + 1)):
                    search(new_hypothesis, manager.Value('d', new_cost), i + 1)

        with Pool(processes=12) as pool:
            pool.apply_async(search, ([], manager.Value('d', 0.0), 0))

        pool.close()
        pool.join()

        return list(best_hypothesis), best_cost.value
        
    def bsc_data_association(self, state, measurements, known_landmarks, covariance, measurement_noise, alpha=0.05, bound=1):
        n = len(measurements)
        m = len(known_landmarks)
        bsc_correspondences = []

        for i in range(n):
            potential_correspondences = []
            for j in range(m):
                mahalanobis_distance = self.compute_mahalanobis_distance(
                    state, measurements[i], known_landmarks[j], covariance, measurement_noise)
                if mahalanobis_distance < chi2.ppf(1-alpha, 2):
                    potential_correspondences.append((j, mahalanobis_distance))
            
            # Sort potential correspondences based on Mahalanobis distance and keep up to 'bound' closest ones
            potential_correspondences.sort(key=lambda x: x[1])
            bsc_correspondences.append([float(idx) for idx, _ in potential_correspondences[:bound]])

        return bsc_correspondences


def main(args=None):
    rclpy.init(args=args)
    ekf_node = EKFNode(initial_state=[1.0, 1.0, 0.0],
                           initial_covariance=[10.0, 10.0, 10.0],
                           process_noise=[0.5, 0.5, 1.0],
                           measurement_noise=[5.0, 5.0]
                           )
    # executor = MultiThreadedExecutor(num_threads=4)
    executor = SingleThreadedExecutor()
    executor.add_node(ekf_node)
    while  rclpy.ok():
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
    executor.shutdown()
    ekf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()