import os
from typing import Tuple, Dict, List
import numpy as np
from sklearn.cluster import DBSCAN

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from ekf_localization.utils.helpers import load_landmarks, normalize_angle



class EKF(Node):
    """
    An EKF localization node with unkown correspondences.
    """
    def __init__(self, node_name: str = 'ekf_node',
                 landmark_file: str=os.path.join(get_package_share_directory('ekf_localization'), 'config', 'landmarks.yaml'), 
                 *args, **kwargs):

        super().__init__(node_name)
        self.declare_parameter('process_noise', kwargs.get('process_noise', [0.1, 0.1, 0.1]))
        self.declare_parameter('measurement_noise', kwargs.get('measurement_noise', [5.0, 5.0]))
        self.declare_parameter('initial_covariance', kwargs.get('initial_covariance', [3.0, 3.0, 3.0]))
        self.declare_parameter('initial_state', kwargs.get('initial_state', [0.10, 0.50, 0.0]))
        self.declare_parameter('save_fig', kwargs.get('save_fig', False))


        self.mu = np.asarray(self.get_parameter(
            'initial_state').value)  # mu [x, y, theta]
        self.P = np.diag(self.get_parameter(
            'initial_covariance').value)  # Sigma
        self.Q = np.diag(
            self.get_parameter('process_noise').value)  # Q
        self.Q_base = self.Q.copy()  # Used for updating the process noise
        self.R = np.diag(
            self.get_parameter('measurement_noise').value)  # Measurement Noise
        self.map = load_landmarks(landmark_file)

        self.get_logger().info("Finished init ekf parent class")


    @staticmethod
    def process_laser_scan(msg: LaserScan) -> List[Dict['str', float]]:
        """
        Process the laser scan message and return a list of range-bearing measurements
        :param msg: Laser scan message
        :type msg: LaserScan
        :return: List of range-bearing measurements {'range': float, 'bearing': float}
        :rtype: List[Dict['str', float]]
        """
        # List to hold the measurements
        measurements: List[Dict['str', float]] = []
        # Get the total number of laser scan rays
        total_angles = len(msg.ranges)
        # Compute the angle increment between each laser scan ray
        angle_increment = msg.angle_increment

        # Loop over all ranges and get the corresponding angles
        for i in range(total_angles):
            # Check if the range measurement is valid
            if msg.ranges[i] < msg.range_max:
                # Calculate the bearing (angle) of the measurement
                # Note: The angle increases counter-clockwise, starting from X (forward)
                bearing = msg.angle_min + i * angle_increment
                # If we need the angle to increase clockwise, starting from X (forward), we can use:
                # bearing = -bearing

                # Add the range and bearing to the list of measurements
                measurements.append({
                    'range': msg.ranges[i],
                    'bearing': bearing
                })
        return measurements


    @staticmethod
    def cluster_observations(observations: List[Dict['str', float]], eps: float = 0.5, min_samples: int = 2) -> List[Dict['str', float]]:
        """
        Cluster the laser scan observations using DBSCAN algorithm.
        
        :param observations: List of range-bearing measurements {'range': float, 'bearing': float}
        :param eps: The maximum distance between two samples for them to be considered as in the same cluster.
        :param min_samples: The number of samples (or total weight) in a neighborhood for a point to be considered as a core point.
        
        :return: List of clustered range-bearing measurements {'range': float, 'bearing': float}
        """
        if len(observations) == 0:
            return []
        
        # Convert observations to Cartesian coordinates
        X = np.array([[obs['range'] * np.cos(obs['bearing']), obs['range'] * np.sin(obs['bearing'])] for obs in observations])
        
        # Perform DBSCAN clustering
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(X)
        labels = clustering.labels_
        
        # Initialize clustered observations
        clustered_observations = []
        
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
            
            clustered_observations.append({
                'range': centroid_range,
                'bearing': centroid_bearing
            })
        
        return clustered_observations

    @staticmethod
    def plot_clustered_observations(ax, clustered_observations: List[Dict['str', float]], GTpose: np.ndarray, color='magenta', label='Clustered Observations'):
        """
        Plot the clustered laser scan observations on a given Matplotlib axis.
        
        :param ax: Matplotlib axis to plot on
        :param clustered_observations: List of clustered range-bearing measurements {'range': float, 'bearing': float}
        :param color: Color to use for the clustered points
        :param label: Label for the clustered points in the legend
        """
        
        if len(clustered_observations) == 0:
            return
        
        # Convert observations to Cartesian coordinates
        X = np.array([[obs['range'] * np.cos(obs['bearing']), obs['range'] * np.sin(obs['bearing'])] for obs in clustered_observations])

        # Extract ground truth pose information
        x_gt, y_gt, theta_gt = GTpose

        # Create rotation matrix and translation vector
        R = np.array([[np.cos(theta_gt), -np.sin(theta_gt)],
                    [np.sin(theta_gt), np.cos(theta_gt)]])

        t = np.array([x_gt, y_gt])

        # Transform observations into GT frame
        X_transformed = X @ R.T + t

        # Plot the transformed points
        ax.scatter(X_transformed[:, 0], X_transformed[:, 1], c=color, marker='x', label=label, alpha=0.5)


    @staticmethod
    def f(state: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
        """
        Motion model f(.) for a differential drive robot.
        Predict the next state of a 2D differential drive robot.

        :param state: Current state [x, y, theta] (3x1 numpy array)
        :param v: Linear velocity (m/s)
        :param omega: Angular velocity (rad/s)
        :param dt: Time step (s)
        :return: Predicted next state (3x1 numpy array)
        """
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
        """
        Compute G, the jacobian of the motion model f(.)

        :param state: The current state
        :type state: numpy.ndarray
        :param v: The linear velocity
        :type v: float
        :param omega: The angular velocity
        :type omega: float
        :param dt: The time step
        :type dt: float
        :returns The jacobian of the motion model
        :rtype: numpy.ndarray
        """
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
        """
        Compute the expected measurement given the current state and landmark position.
        Calculation is based on euclidean distance and bearing angle.

        :param state: The current state
        :type state: numpy.ndarray
        :param landmark_pos: The position of the landmark
        :type landmark_pos: dict
        :returns The expected measurement
        :rtype: numpy.ndarray
        """
        dx = landmark_pos['x'] - state[0]
        dy = landmark_pos['y'] - state[1]
        q = dx**2 + dy**2
        z_hat = np.array(
            [np.sqrt(q), normalize_angle(np.arctan2(dy, dx) - state[2])])
        return z_hat

    @staticmethod
    def jacobian_of_h(state: np.ndarray, landmark_pos: Dict[str, float]) -> np.ndarray:
        """
        Compute the Jacobian of the measurement model with respect to the state.

        :param state: The current state.
        :type state: numpy.ndarray
        :param landmark_pos: The position of the measured landmark.
        :type landmark_pos: dict
        :returns The Jacobian of the measurement model with respect to the state.
        :rtype: numpy.ndarray
        """
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
        """
        Update the process noise matrix Q based on the control inputs.
        """
        # Adjust the process noise based on the control inputs
        v, omega = u[0], u[1]
        Q = Q_old.copy()
        Q[0, 0] = Q_old[0, 0] * np.abs(v)
        Q[1, 1] = Q_old[1, 1] * np.abs(v)
        Q[2, 2] = Q_old[2, 2] * np.abs(omega**2)
        return Q


    def update(self, mu: np.ndarray, P: np.ndarray, z: dict, measurement_noise: np.ndarray, landmark_pos: np.ndarray, epsilon: float = 1.1) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update the state and covariance given a new measurement.

        :param mu (state) The robot's current state.
        :param P (covariance) The robot's current covariance.
        :param z (measurement) The measurement as an (x, y, theta) dict. # TODO Check this
        :param measurement_noise The measurement noise matrix.
        :param landmark_pos The position of the landmark.
        :param epsilon Inflation factor for the covariance.
        :return The updated state and covariance.
        """
        # Calculate the expected measurement
        expected_measurement = self.h(mu, landmark_pos)

        # Calculate the measurement residual (innovation)
        measurement_residual = np.array([z['range'] - expected_measurement[0],
                                        normalize_angle(z['bearing'] - expected_measurement[1])])

        H = self.jacobian_of_h(mu, landmark_pos)

        # Calculate S = H @ P @ H^T + R and check if it's near singular
        S = H @ P @ H.T + measurement_noise
        K = P @ H.T @ np.linalg.inv(S)

        # Update the state and covariance
        mu = mu + K @ measurement_residual
        mu[2] = normalize_angle(mu[2])
        I_KH = np.eye(len(mu)) - K @ H
        # Joseph's form is numerically more stable
        P = I_KH @ P @ I_KH.T + K @ measurement_noise @ K.T
        return mu, P * epsilon

