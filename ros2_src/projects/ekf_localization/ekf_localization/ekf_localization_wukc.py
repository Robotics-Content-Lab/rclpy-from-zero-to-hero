import os
import numpy as np
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from scipy.stats import chi2

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from ament_index_python.packages import get_package_share_directory

from ekf_localization.utils.helpers import normalize_angle, plot_landmarks, plot_observed_landmarks, plot_correspondences, plot_odometries, save_config, plot_state_and_covariance, load_landmarks


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
        self.declare_parameter('mahalanobis_gate_threshold', kwargs.get('mahalanobis_gate_threshold', 0.95))  # 95% confidence

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

class EKFNode(EKF):
    def __init__(self, node_name: str = 'ekf_node',
                 *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        self.get_logger().info("Initializing EKF Node")
        self.state_x = []  # For plotting the state
        self.state_y = []  # For plotting the state

        self.odom_x = []  # For plotting the GT
        self.odom_y = []  # For plotting the GT
        self.odom_th = []  # For plotting the GT

        self.last_time = None
        self.cnt = 0

        self.declare_parameter('max_association_distance', kwargs.get('max_association_distance', 2.0))
        self.declare_parameter('data_association_method', 'MLDA')  # MLDA or NN

        self.init_plot()

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 1)  # For motion model and prediction
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 1)  # For measurement model and update
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 1)  # For GT comparison



    def init_plot(self):
        # Set up pyplot for visualization
        plt.ion()
        self.fig, self.ax = plt.subplots()

        self.ax.set_title("Robot Odometry")

        self.ax.set_aspect('equal')
        self.plot_landmarks()


    def cmd_vel_callback(self, msg: Twist):
        """
        Callback function for cmd_vel messages.
        Updates the robot's state based on the motion model.
        """
        # Get the current time
        time = self.get_clock().now().to_msg()
        current_time = time.sec + time.nanosec * 1e-9

        # Compute the time since the last time the callback was called
        if self.last_time == None:
            dt = 0
        else:
            dt = current_time - self.last_time

        # Update the last time
        self.last_time = current_time

        # Get the current control inputs
        v = msg.linear.x
        omega = msg.angular.z
        u = np.array([v, omega])
        self.Q = self.update_process_noise(u, self.Q)
        
        # Predict the current state
        self.mu = self.f(self.mu, u, dt)
        self.get_logger().info("Predicted state: \n\tx: {}\n\t y: {}\n\t theta: {}".format(
            self.mu[0], self.mu[1], self.mu[2]), throttle_duration_sec=0.5)

        # Update the covariance
        F = self.jacobian_of_f(self.mu, u, dt)
        self.P = F @ self.P @ F.T + self.Q

    def laser_callback(self, msg: LaserScan) -> None:
        """
        Process the laser scan message and update the state estimate
        :param msg: Laser scan message
        """
        measurements = self.process_laser_scan(msg)
        measurements = self.cluster_observations(measurements)
        correspondences = self.data_association(state=self.mu, measurements=measurements, known_landmarks=self.map, covariance=self.P,
                                        measurement_noise=self.R, max_distance=self.get_parameter('max_association_distance').value,
                                        method=self.get_parameter('data_association_method').value)
        
        observed_landmarks = []
        for i, measurement in enumerate(measurements):
            # Check if the landmark is observed
            if correspondences[i] is not None:
                observed_landmarks.append(correspondences[i])
        
        for i, measurement in enumerate(measurements):
            # Only update with the observed landmarks
            if correspondences[i] in observed_landmarks:
                self.mu, self.P = self.update(self.mu, self.P, measurement, self.R, self.map[correspondences[i]])

        if self.cnt % 2 != 0:
            self.update_plot(observed_landmarks=observed_landmarks, correspondences=correspondences, clustered_observations=measurements)
        self.state_x.append(self.mu[0])
        self.state_y.append(self.mu[1])
        self.cnt += 1

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


    def data_association(self, state: np.ndarray, measurements: List[Dict['str', float]], known_landmarks: List[Dict['str', float]], covariance, measurement_noise, max_distance: float = np.inf, method: str = 'NN'):
        if method == 'NN':
            return self.nearest_neighbor_data_association(state=state, measurements=measurements, known_landmarks=known_landmarks, max_distance=max_distance)
        elif method == 'MLDA':
            return self.maximum_likelihood_data_association(state=state, measurements=measurements, known_landmarks=known_landmarks, covariance=covariance, measurement_noise=measurement_noise, max_distance=max_distance)
        else:
            self.get_logger().error('Invalid data association method.')
            raise ValueError('Invalid data association method.')

    def nearest_neighbor_data_association(self, state: np.ndarray, measurements: List[Dict['str', float]], known_landmarks: List[Dict['str', float]], max_distance: float = np.inf):
        """ 
        Implements nearest neighbor data association with Mahalanobis gating.
        It finds the known landmark closest to each measurement using the Mahalanobis distance.

        :param state: The current state.
        :type state: numpy.ndarray
        :param measurements: The measurements as a list, where each measurement is a dictionary containing 'range' and 'bearing'.
        :type measurements: list
        :param known_landmarks: The known landmarks as a list, where each landmark is a dictionary containing 'x' and 'y'.
        :type known_landmarks: list
        :param max_distance: The maximum distance to associate a measurement with a landmark.
        :type max_distance: float
        :return: A list of indices of the associated landmarks.
        :rtype: list
        """
        # Define the Mahalanobis gating threshold (e.g., 95% confidence level for a 2D measurement -> threshold ≈ 5.99)
        threshold = chi2.ppf(0.95, df=2)

        # List to store the indices of corresponding landmarks
        correspondence_indices = []

        # Iterate over all measurements
        for measurement in measurements:
            min_mahalanobis_distance = np.inf
            closest_index = -1
            
            # Iterate over all known landmarks
            for i, landmark in enumerate(known_landmarks):
                # Compute the expected measurement given the current state and the landmark position
                expected_measurement = self.h(state, landmark)
                
                # Compute the measurement residual (difference between actual and expected measurement)
                measurement_residual = np.array([
                    measurement['range'] - expected_measurement[0],
                    normalize_angle(measurement['bearing'] - expected_measurement[1])
                ])
                
                # Compute the Jacobian of the measurement model with respect to the state
                H = self.jacobian_of_h(state, landmark)
                
                # Compute the measurement covariance (S) using the formula S = H * P * H^T + R
                S = H @ self.P @ H.T + self.R
                
                # Compute the Mahalanobis distance
                mahalanobis_distance = measurement_residual.T @ np.linalg.inv(S) @ measurement_residual

                # Apply Mahalanobis gating
                if mahalanobis_distance < min_mahalanobis_distance and mahalanobis_distance < threshold:
                    min_mahalanobis_distance = mahalanobis_distance
                    closest_index = i

            # Append the index of the closest landmark (or None if no valid association was found)
            correspondence_indices.append(closest_index if closest_index != -1 else None)
        
        return correspondence_indices

    def maximum_likelihood_data_association(self, state: np.ndarray, measurements: List[Dict['str', float]], known_landmarks: List[Dict['str', float]], covariance: np.ndarray, measurement_noise: np.ndarray, max_distance: float = np.inf):
        """
        Implements maximum likelihood data association. It finds the known landmark that maximizes the likelihood for each measurement.
        :param state: The current state.
        :type state: numpy.ndarray
        :param measurements: The measurements as a list, where each measurement is a dictionary containing 'range' and 'bearing'.
        :type measurements: list
        :param known_landmarks: The known landmarks as a list, where each landmark is a dictionary containing 'x' and 'y'.
        :type known_landmarks: list
        :param covariance: The covariance matrix.
        :type covariance: numpy.ndarray
        :param measurement_noise: The measurement noise matrix.
        :type measurement_noise: numpy.ndarray
        :param max_distance: The maximum distance to associate a measurement with a landmark.
        :type max_distance: float
        :return: A list of indices of the associated landmarks.
        :rtype: list
        """
        threshold = chi2.ppf(self.get_parameter('mahalanobis_gate_threshold').value, 2)  # 2 degrees of freedom
        
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

                # Apply Mahalanobis gating
                if mahalanobis_distance < min_mahalanobis_distance and mahalanobis_distance < threshold:
                    min_mahalanobis_distance = mahalanobis_distance
                    closest_index = i      

                # # If the Mahalanobis distance is less than the minimum so far and less than the maximum allowed distance
                # if mahalanobis_distance < min_mahalanobis_distance and mahalanobis_distance < max_distance:
                #     # Update the minimum Mahalanobis distance and the closest index
                #     min_mahalanobis_distance = mahalanobis_distance
                #     closest_index = i
            # Append the closest landmark index to the list of correspondences. If no association is found, append None.
            correspondence_indices.append(
                closest_index if closest_index != -1 else None)
        # Return the list of correspondences
        return correspondence_indices


    def update_plot(self, observed_landmarks, correspondences, clustered_observations):
        """ Update all plots """
        self.ax.clear()
        # Plot starting positions
        self.ax.scatter(self.state_x[0], self.state_y[0], marker="o", color="red", s=50, linewidths=1.5, edgecolors="k", label="EKF Start")
        if len(self.odom_x) != 0 and len(self.odom_y) != 0:
            self.ax.scatter(self.odom_x[0], self.odom_y[0], marker="o", color="green", s=50, linewidths=1.5, edgecolors="k", label="GT Start")
        self.plot_landmarks()
        if len(clustered_observations) != 0:
            self.plot_clustered_observations(ax=self.ax, clustered_observations=clustered_observations, GTpose=np.asarray([self.odom_x[-1], self.odom_y[-1], self.odom_th[-1]]))
        if len(correspondences) != 0:
            self.plot_correspondences(correspondences=correspondences)
        if len(observed_landmarks) != 0:
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
    node = EKFNode(initial_state=[3.0, -2.0, 0.0], measurement_noise=[10.0, 10.0])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt, shutting down node")
        node.save_config()
        node.destroy_node()


if __name__ == '__main__':
    main()
