import os
import numpy as np
from typing import Dict, List

import matplotlib.pyplot as plt

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

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



class EKFNode(EKF):
    def __init__(self, node_name: str = 'ekf_node',
                 *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        self.get_logger().info("Initializing EKF Node")
        self.initial_state = self.get_parameter('initial_state').value
        self.state_x = [self.initial_state[0]]  # For plotting the state
        self.state_y = [self.initial_state[1]]  # For plotting the state

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
        self.update_plot([], [], [])


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
        Implements nearest neighbor data association. It finds the known landmark closest to each measurement.

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
    node = EKFNode(initial_state=[10.0, -8.0, 0.0], measurement_noise=[10.0, 10.0])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt, shutting down node")
        node.save_config()
        node.destroy_node()


if __name__ == '__main__':
    main()
