from typing import Tuple, Dict, List, Callable
import numpy as np
from scipy.stats import multivariate_normal
import matplotlib.pyplot as plt

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from ekf_localization.utils.helpers import normalize_angle, Plotter
from ekf_localization.ekf_localization_with_unknown_correspondences import EKFNode

from ekf_localization.utils.helpers import normalize_angle, plot_landmarks, plot_observed_landmarks, plot_correspondences, plot_odometries, save_config, plot_state_and_covariance
from ekf_localization.utils.ekf import EKF

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
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

import os
from typing import Tuple, Dict, List
import numpy as np
from sklearn.cluster import DBSCAN

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from ekf_localization.utils.helpers import load_landmarks, normalize_angle

class ROSNode(Node):
    """
    A ROS node for the EKF localization project.
    """
    def __init__(self, node_name: str = 'ekf_node', *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        super().__init__(node_name)
        self.declare_parameter('process_noise', kwargs.get('process_noise', [0.1, 0.1, 0.1]))
        self.declare_parameter('measurement_noise', kwargs.get('measurement_noise', [5.0, 5.0]))
        self.declare_parameter('initial_covariance', kwargs.get('initial_covariance', [10.0, 10.0, 3.0]))
        self.declare_parameter('initial_state', kwargs.get('initial_state', [0.0, 0.0, 0.0]))
        self.declare_parameter('save_fig', kwargs.get('save_fig', False))

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 1)  # For motion model and prediction
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 1)  # For measurement model and update
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 1)  # For GT comparison


        self._measurements = np.array([])
        self._clustered_measurements = np.array([])

        self.odom_x = []
        self.odom_y = []
        self.odom_th = []

    def odom_callback(self, msg):
        """ Callback function for odometry/GT messages. """
        # Extract x, y from odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Store in lists
        self.odom_x.append(x)
        self.odom_y.append(y)
        self.odom_th.append(euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)[2])

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
        total_angles = len(msg.ranges) # Get the total number of laser scan rays
        measurements = np.ndarray(shape=(total_angles, 2), dtype=np.float32) # Initialize the measurements array
        angle_increment = msg.angle_increment # Compute the angle increment between each laser scan ray

        # Loop over all ranges and get the corresponding angles
        for i in range(total_angles):
            # Check if the range measurement is valid
            if msg.ranges[i] < msg.range_max:
                # Note: The angle increases counter-clockwise, starting from X (forward)
                bearing = msg.angle_min + i * angle_increment
                # If we need the angle to increase clockwise, starting from X (forward), we can use:
                # bearing = -bearing

                measurements[0][i] = msg.ranges[i] # Store the range measurement
                measurements[1][i] = bearing # Store the bearing measurement
        self._measurements = measurements

    def cluster_observations(self, observations: np.ndarray, eps: float = 0.5, min_samples: int = 2) -> List[Dict['str', float]]:
        if len(observations) == 0:
            return []
        
        # Convert observations to Cartesian coordinates
        X = np.array([[r * np.cos(b), r * np.sin(b)] for r, b in observations])
        
        # Perform DBSCAN clustering
        clustering: DBSCAN = DBSCAN(eps=eps, min_samples=min_samples).fit(X)
        labels = clustering.labels_
        
        # Initialize clustered observations
        clustered_observations = np.ndarray(shape=(0, 2), dtype=np.float32)
        
        # Loop through each cluster label
        unique_labels = np.unique(labels)
        for label in unique_labels:
            if label == -1:
                # Skip the noise points
                continue
            
            # Find the indices of points in this cluster
            indices = np.where(labels == label)[0]
            
            # Compute the centroid of these points
            centroid_x = np.mean(X[indices, 0])
            centroid_y = np.mean(X[indices, 1])
            
            # Convert the centroid back to range-bearing form
            centroid_range = np.sqrt(centroid_x**2 + centroid_y**2)
            centroid_bearing = np.arctan2(centroid_y, centroid_x)
            
            clustered_observations = np.vstack((clustered_observations, np.array([centroid_range, centroid_bearing])))

        self._clustered_measurements = clustered_observations

class EKF():
    """
    An EKF localization node with unkown correspondences.
    """
    def __init__(self, node_name: str = 'ekf_node',
                 landmark_file: str=os.path.join(get_package_share_directory('ekf_localization'), 'config', 'landmarks.yaml'), 
                 logger, **kwargs):

        self.logger = logger
        self._mu = np.asarray(kwargs.get('initial_state') , [0.0, 0.0, 0.0])  # mu [x, y, theta]
        self._P = np.diag(kwargs.get('initial_covariance'), [3.0, 3.0, 3.0] )  # Sigma
        self._Q = np.diag(kwargs.get('process_noise'), [5.0, 5.0])  # Q
        self._Q_base = self.Q.copy()  # Used for updating the process noise
        self._R = np.diag(kwargs.get('measurement_noise').value)  # Measurement Noise
        self._map = load_landmarks(landmark_file)

        self.logger().info("Finished init ekf parent class")
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


class EKFplotNode(ROSNode, EKF):
    def __init__(self, node_name: str = 'ekf_node', *args, **kwargs):
        ROSNode.__init__(node_name, *args, **kwargs)
        EKF.__init__(node_name, *args, **kwargs)

        self.Plotter = Plotter()
        
        plot_fq = 10
        localization_fq = 10

        self.create_timer(1/localization_fq, self.localize_callback)
        self.create_timer(1/plot_fq, self.plot_callback)

        self.logger().info("Finished init ekf plot node")

    def localize_callback(self):
        pass

    def plot_callback(self):
        pass

class MHTEKFNode(EKF):
    """
    An EKF localization node with Multi-Hypothesis Testing (MHT) or Maximum Likelihood Data Association (MLDA).
    """

    def __init__(self, max_hypotheses=3, **kwargs):
        super().__init__(**kwargs)
        self.get_logger().info("Initializing EKF Node")
        self.declare_parameter('max_association_distance', kwargs.get('max_association_distance', 2.0))
        self.declare_parameter('data_association_method', 'MLDA')  # MLDA or NN

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 1)  # For motion model and prediction
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 1)  # For measurement model and update
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 1)  # For GT comparison

        self.v = 0.0
        self.omega = 0.0
        self.dt = 0.0

        self.method = self.declare_parameter('method', kwargs.get('method', 'MHT')).value
        self.max_hypotheses = max_hypotheses
        self.THRESHOLD= 0.1
        self.SIMILARITY_THRESHOLD = self.declare_parameter('similarity_threshold', kwargs.get('similarity_threshold', 0.1)).value
        # Initialize multiple hypotheses
        self.hypotheses = self.initialize_multiple_states(num_hypotheses=max_hypotheses)

        self.state_x = []  # For plotting the state
        self.state_y = []  # For plotting the state

        self.odom_x = []  # For plotting the GT
        self.odom_y = []  # For plotting the GT
        self.odom_th = []  # For plotting the GT

        self.last_time = None
        self.cnt = 0

        # Set up pyplot for visualization
        plt.ion()
        self.fig, self.ax = plt.subplots()

        self.ax.set_title("Robot Odometry")

        self.ax.set_aspect('equal')
        self.plot_landmarks()

    def initialize_multiple_states(self, num_hypotheses=10):
        hypotheses = []
        for _ in range(num_hypotheses):
            # Randomize the initial state around some mean (your original initial state)
            randomized_state = self.mu + np.random.normal(loc=0, scale=[5, 5, 0.1], size=self.mu.shape)
            
            # Initialize the covariance matrix (P) for each hypothesis. This can be the same for all hypotheses or varied.
            P = self.P.copy()
            
            # Add the new hypothesis
            hypotheses.append({
                'state': randomized_state,
                'covariance': P,
                'likelihood': 1.0  # Initialize likelihood
            })
        return hypotheses

    def odom_callback(self, msg):
        """ Callback function for odometry/GT messages. """
        # Extract x, y from odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Store in lists
        self.odom_x.append(x)
        self.odom_y.append(y)
        self.odom_th.append(euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)[2])
        self.plot_odometries()


    def cmd_vel_callback(self, msg: Twist):
        # Get the current time
        time = self.get_clock().now().to_msg()
        current_time = time.sec + time.nanosec * 1e-9

        # Compute the time since the last time the callback was called
        if self.last_time == None:
            self.dt = 0
        else:
            self.dt = current_time - self.last_time

        # Update the last time
        self.last_time = current_time

        # Get the current control inputs
        self.v = msg.linear.x
        self.omega = msg.angular.z
        u = [self.v, self.omega]

        self.Q = self.update_process_noise(u, self.Q)
        mu_, P_ = self.predict(mu=self.mu, P=self.P, Q=self.Q, f=self.f, jacobian_of_f=self.jacobian_of_f, u=u, dt=self.dt)
        
        self.mu = mu_
        self.P = P_

    @staticmethod
    def predict(mu: np.ndarray, P: np.ndarray, Q: np.ndarray, f: Callable, jacobian_of_f:Callable, u: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        # 1. Compute the predicted state using the motion model
        predicted_state = f(mu, u, dt)
        
        # 2. Compute the Jacobian of the motion model
        G = jacobian_of_f(mu, u, dt)
        
        # 3. Update the covariance
        predicted_covariance = G @ P @ G.T + Q
        
        return predicted_state, predicted_covariance

    def laser_callback(self, msg: LaserScan):
        measurements = self.process_laser_scan(msg)
        measurements = self.cluster_observations(measurements)
        known_measurements = []
        unknown_measurements = []

        # Separate measurements into known and unknown
        for measurement in measurements:
            if self.check_known_landmark(measurement):
                known_measurements.append(measurement)
            else:
                unknown_measurements.append(measurement)

        new_hypotheses = []

        for hypothesis_dict in self.hypotheses:
            # new_hypotheses.extend(self.update_hypothesis(hypothesis_dict, known_measurements, unknown_measurements))
            new_hyp = self.update_hypothesis(hypothesis_dict, {'known': known_measurements, 'unknown': unknown_measurements})
            new_hypotheses.append(new_hyp)
        # Evaluate likelihood of each hypothesis
        likelihoods = [self.evaluate_likelihood(hypothesis, measurements) for hypothesis in new_hypotheses]

        # Sort hypotheses based on likelihood and retain the top ones
        sorted_indices = sorted(range(len(likelihoods)), key=lambda k: likelihoods[k], reverse=True)
        self.hypotheses = [new_hypotheses[i] for i in sorted_indices[:self.max_hypotheses]]

        # Set the current state and covariance to the most likely hypothesis
        best_hypothesis = self.extract_best_global_hypothesis(measurements)
        self.mu = self.hypotheses[0]['state']
        self.P = self.hypotheses[0]['covariance']

        # Visualize and log as before
        if self.cnt % 2 == 0:
            self.plot_most_likely_observations(self.mu, measurements, self.map)

            observed_landmarks = known_measurements + unknown_measurements
            correspondences = [(i, self.check_known_landmark(measurement)) for i, measurement in enumerate(observed_landmarks)]
            self.update_plot(observed_landmarks=observed_landmarks, correspondences=correspondences, clustered_observations=measurements)
            
        self.state_x.append(self.mu[0])
        self.state_y.append(self.mu[1])
        self.cnt += 1

    def update_hypothesis(self, hypothesis: Dict[str, float], measurements: Dict) -> Dict:
        state = hypothesis['state']
        covariance = hypothesis['covariance']
        likelihood = hypothesis.get('likelihood', 1.0)  # get likelihood if exists, else initialize as 1

        for key in ['known', 'unknown']:
            for measurement in measurements[key]:
                if key == 'known':
                    predicted_measurement, H, S = self.predict_measurement(state, measurement['landmark_id'])
                else:  # 'unknown'
                    best_landmark_id, best_S = self.best_correspondence(state, covariance, measurement)
                    predicted_measurement, H, S = self.predict_measurement(state, best_landmark_id)
                                
                # Update state and covariance
                innovation = np.array([measurement['range'] - predicted_measurement[0],
                                    normalize_angle(measurement['bearing'] - predicted_measurement[1])])
                # Calculate the adaptive scaling factor
                lambda_factor = innovation.T @ np.linalg.inv(S) @ innovation / S.shape[0]
                
                R_adaptive = lambda_factor * self.R  # Compute the adaptive measurement noise covariance

                # # Compute Kalman gain
                # K = covariance @ H.T @ np.linalg.inv(S)
                # Compute the Kalman gain using the updated R
                S_adaptive = H @ covariance @ H.T + R_adaptive  # Compute the innovation covariance using the updated R
                K = covariance @ H.T @ np.linalg.inv(S_adaptive)  # Compute the Kalman gain using the updated R
                                    
                state += K @ innovation
                covariance -= K @ S @ K.T

                # Update likelihood of this hypothesis
                likelihood *= multivariate_normal.pdf(innovation, mean=np.zeros(2), cov=S)

        return {'state': state, 'covariance': covariance, 'likelihood': likelihood}

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
        if self.method == "NN":  # Nearest Neighbor
            best_landmark_id, S = self.nn(state, covariance, measurement)
        elif self.method == "MHT":  # Multi-Hypothesis Testing
            best_landmark_id, S = self.mht(state, covariance, measurement)
        elif self.method == "MLDA":  # Maximum Likelihood Data Association
            best_landmark_id, S = self.mlda(state, covariance, measurement)
        elif self.method == "BSC": # Best Single Correspondence
            best_landmark_id, S = self.bsc(state, covariance, measurement)
        else:
            raise ValueError("Invalid method specified!")

        return best_landmark_id, S

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
        correspondence_indices = self._maximum_likelihood_data_association(
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
            best_landmark_id, best_S = self.nearest_neighbor(state, covariance, measurement)
            return best_landmark_id, best_S

    def bsc(self, state: np.ndarray, covariance: np.ndarray, measurement: Dict[str, float]) -> Tuple[np.ndarray, np.ndarray]:
        best_landmark_id = None
        best_S = None
        min_mahalanobis_distance = np.inf  # Initialize with a large value
        
        for landmark_id in range(len(self.map)):
            # Predict the measurement for each landmark
            predicted_measurement, H, S = self.predict_measurement(state, landmark_id)
            
            # Compute the innovation
            innovation = np.array([measurement['range'] - predicted_measurement[0],
                                normalize_angle(measurement['bearing'] - predicted_measurement[1])])
            
            # Compute the innovation covariance
            S = H @ covariance @ H.T + self.R

            # Compute Mahalanobis distance
            mahalanobis_distance = innovation.T @ np.linalg.inv(S) @ innovation
            
            # Keep track of the best landmark ID and innovation covariance based on Mahalanobis distance
            if mahalanobis_distance < min_mahalanobis_distance:
                min_mahalanobis_distance = mahalanobis_distance
                best_landmark_id = landmark_id
                best_S = S
                
        return best_landmark_id, best_S

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


    def _nearest_neighbor_data_association(self, state: np.ndarray, measurements: List[Dict['str', float]], known_landmarks: List[Dict['str', float]], max_distance: float = np.inf):
        # For each measurement, find the index of the landmark that is closest
        correspondence_indices = []
        for measurement in measurements:
            min_distance = np.inf
            closest_index = -1
            for i, landmark in enumerate(known_landmarks):
                distance = np.sqrt(
                    (state['x'] - landmark['x'])**2 + (state['y'] - landmark['y'])**2)
                if distance < min_distance and distance < max_distance:
                    min_distance = distance
                    closest_index = i
            correspondence_indices.append(
                closest_index if closest_index != -1 else None)
        return correspondence_indices

    def _maximum_likelihood_data_association(self, state: np.ndarray, measurements: List[Dict['str', float]], known_landmarks: List[Dict['str', float]], covariance: np.ndarray, measurement_noise: np.ndarray, max_distance: float = np.inf):
        # For each measurement, find the index of the landmark that maximizes the likelihood
        correspondence_indices = []
        for measurement in measurements:
            # Initialize the minimum Mahalanobis distance to infinity
            min_mahalanobis_distance = np.inf
            # Initialize the closest index to -1
            closest_index = -1
            # For each known landmark
            for i, landmark in enumerate(known_landmarks):
                # Compute the expected measurement using the measurement model
                expected_measurement = self.h(state, landmark)
                # Compute the measurement residual (innovation). The difference between the actual and expected measurements
                # Normalize the bearing difference to be between -pi and pi
                measurement_residual = np.array(
                    [measurement['range'] - expected_measurement[0], normalize_angle(measurement['bearing'] - expected_measurement[1])])
                # Compute the Jacobian of the measurement function with respect to the state at the current state and landmark
                H = self.jacobian_of_h(state, landmark)
                # Compute the measurement covariance (S) using the formula S = H*Sigma*H^T + R
                S = H @ covariance @ H.T + measurement_noise
                # Compute the Mahalanobis distance. It is a measure of the distance between a point P and a distribution D,
                # as: D_M(P) = sqrt( (P-M)^T * Sigma^-1 * (P-M) )
                mahalanobis_distance = measurement_residual.T @ np.linalg.inv(S) @ measurement_residual
                # If the Mahalanobis distance is less than the minimum so far and less than the maximum allowed distance
                if mahalanobis_distance < min_mahalanobis_distance and mahalanobis_distance < max_distance:
                    # Update the minimum Mahalanobis distance and the closest index
                    min_mahalanobis_distance = mahalanobis_distance
                    closest_index = i
            # Append the closest landmark index to the list of correspondences. If no association is found, append None.
            correspondence_indices.append(
                closest_index if closest_index != -1 else None)
        # Return the list of correspondences
        return correspondence_indices



    def plot_most_likely_observations(self, state: np.ndarray, measurements: List[Dict[str, float]], landmarks: List[Dict[str, float]]) -> None:
        # Predict the measurement for each landmark using the provided state
        predicted_measurements = [self.h(state, landmark) for landmark in landmarks]
        
        # Compare the predicted measurements with actual measurements to find the most likely observed landmarks
        observed_landmarks = []
        for measurement in measurements:
            diffs = [np.linalg.norm(np.array([measurement['range'] - pred[0], normalize_angle(measurement['bearing'] - pred[1])])) for pred in predicted_measurements]
            observed_landmarks.append(landmarks[np.argmin(diffs)])
               
        # Plot the most likely observed landmarks as red stars
        x_values = []
        y_values = []
        print("---------------------"*10)
        print("Obesrverd Landmarks#: ", len(observed_landmarks))
        for landmark in observed_landmarks:
            x_values.append(landmark['x'])
            y_values.append(landmark['y'])
        self.ax.scatter(x_values, y_values, marker="o", color="r", alpha=0.5, s=50, label="Most likely observations")

    def update_plot(self, observed_landmarks, correspondences, clustered_observations):
        """ Update all plots """
        self.ax.clear()
        # Plot starting positions
        if len(self.state_x) != 0 and len(self.state_y) != 0:
            self.ax.scatter(self.state_x[0], self.state_x[0], marker="o", color="red", s=50, linewidths=1.5, edgecolors="k", label="EKF Start")
        if len(self.odom_x) != 0 and len(self.odom_y) != 0:
            self.ax.scatter(self.odom_x[0], self.odom_y[0], marker="o", color="green", s=50, linewidths=1.5, edgecolors="k", label="GT Start")
        self.plot_landmarks()
        self.plot_clustered_observations(ax=self.ax, clustered_observations=clustered_observations, GTpose=np.asarray([self.odom_x[-1], self.odom_y[-1], self.odom_th[-1]]))
        if correspondences != None:
            self.plot_correspondences(correspondences=correspondences)
        if observed_landmarks != None:
            self.plot_observed_landmarks(observed_landmarks)
        self.plot_state_and_covariance()
        self.plot_odometries()
        self.ax.set_xlabel('x[m]')
        self.ax.set_ylabel('y[m]')

        plt.subplots_adjust(bottom=0.2)
        # Put a legend below current axis
        self.ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15),
                fancybox=True, shadow=True, ncol=3)
        plt.draw()
        plt.pause(0.01)

    def plot_landmarks(self):
        """ Plot the GT landmarks. """
        plot_landmarks(self)

    def plot_observed_landmarks(self, observed_landmarks):
        """ Plot the observed landmarks. """
        plot_observed_landmarks(self, observed_landmarks)

    def plot_correspondences(self, correspondences):
        """ Plot the correspondences between the current state and the observed landmarks. """
        plot_correspondences(self, correspondences)

    def plot_odometries(self):
        """ Plot the odometry data of the GT and EKF state. """
        plot_odometries(self)

    def plot_state_and_covariance(self):
        """ Plot the current state and covariance. """
        plot_state_and_covariance(self)

    def save_config(self):
        """ Save the plot window and config. """    
        if self.get_parameter('save_fig'):
            save_config(self, "ekf_localiation_with_unknown_correspondences")

def main(args=None):
    rclpy.init(args=args)
    node = MHTEKFNode(measurement_noise=[1.4, 50.0], initial_state=[3.0, 1.0, 0.0], max_hypotheses=100)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt, shutting down node")
        # node.save_config()
        node.destroy_node()


if __name__ == '__main__':
    main()
