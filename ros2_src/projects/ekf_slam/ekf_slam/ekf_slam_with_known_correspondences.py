import os
from typing import Dict, List, Tuple
import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory



from ekf_slam.utils.helpers import load_landmarks, normalize_angle, plot_landmarks, plot_observed_landmarks, plot_correspondences, plot_odometries, save_config, plot_state_and_covariance


def predict_diff_drive(state: np.ndarray, v: float, omega: float, dt: float) -> np.ndarray:
    """ Predict the next state given the current state and control inputs. """
    x, y, theta = state
    if np.abs(omega) < 1e-10:  # Driving straight
        x_new = x + v * dt * np.cos(theta)
        y_new = y + v * dt * np.sin(theta)
        theta_new = theta
    else:  # Turning motion
        x_new = x - v/omega * np.sin(theta) + v/omega * np.sin(theta + omega * dt)
        y_new = y + v/omega * np.cos(theta) - v/omega * np.cos(theta + omega * dt)
        theta_new = theta + omega * dt
    theta_new = normalize_angle(theta_new)
    return np.array([x_new, y_new, theta_new])

def jacobian_of_predict_diff_drive(state: np.ndarray, v: float, omega: float, dt: float) -> np.ndarray:
    """ Compute the Jacobian of the prediction model with respect to the state. """
    x, y, theta = state
    if np.abs(omega) < 1e-10:  # Driving straight
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

def measurement_function(state: np.ndarray, landmark_pos: Dict[str, float]) -> np.ndarray:
    """ Compute the expected measurement given the current state and landmark position. """
    dx = landmark_pos['x'] - state[0]
    dy = landmark_pos['y'] - state[1]
    q = dx**2 + dy**2
    z_hat = np.array([np.sqrt(q), normalize_angle(np.arctan2(dy, dx) - state[2])])
    return z_hat

def jacobian_of_measurement(state: np.ndarray, landmark_pos: Dict[str, float]) -> np.ndarray:
    """ Compute the Jacobian of the measurement model with respect to the state. """
    dx = landmark_pos['x'] - state[0]
    dy = landmark_pos['y'] - state[1]
    q = dx**2 + dy**2
    sqrt_q = np.sqrt(q)
    H = np.array([[-sqrt_q * dx, -sqrt_q * dy, 0],
                  [dy, -dx, -q]])
    H /= q
    return H

def update(state: np.ndarray, covariance: np.ndarray, measurement: dict, measurement_noise: np.ndarray, landmark_pos: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """ Update the state and covariance given a new measurement. """
    expected_measurement = measurement_function(state, landmark_pos)
    measurement_residual = np.array([measurement['range'] - expected_measurement[0],
                                    normalize_angle(measurement['bearing'] - expected_measurement[1])])
    H = jacobian_of_measurement(state, landmark_pos)
    S = H @ covariance @ H.T + measurement_noise
    K = covariance @ H.T @ np.linalg.inv(S)
    state = state + K @ measurement_residual
    state[2] = normalize_angle(state[2])
    I_KH = np.eye(len(state)) - K @ H
    covariance = I_KH @ covariance @ I_KH.T + K @ measurement_noise @ K.T
    return state, covariance

class EKFNode(Node):
    """ An EKF localization node with known correspondences. """

    def __init__(self):
        super().__init__('ekf_node')
        self.declare_parameter('max_association_distance', 5.0)
        self.declare_parameter('process_noise', [0.01, 0.01, 0.01])
        self.declare_parameter('measurement_noise', [0.01, 0.01])
        self.declare_parameter('initial_covariance', [3.0, 3.0, 0.01])
        self.declare_parameter('initial_state', [3.0, 1.0, 0.0])

        self.state = np.asarray(self.get_parameter('initial_state').value)  # mu [x, y, theta]
        self.covariance = np.diag(self.get_parameter('initial_covariance').value)  # Sigma
        self.process_noise = np.diag(self.get_parameter('process_noise').value)  # Q
        self.base_process_noise = self.process_noise.copy()  # Used for updating the process noise
        self.measurement_noise = np.diag(self.get_parameter('measurement_noise').value)  # R
        self.landmarks = load_landmarks(os.path.join(
            get_package_share_directory('ekf_localization'), 'config', 'landmarks.yaml'))

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 1)  # For motion model and prediction
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 1)  # For measurement model and update
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 1)  # For GT comparison

        self.state_x = []  # For plotting the state
        self.state_y = []  # For plotting the state

        self.odom_x = []  # For plotting the GT
        self.odom_y = []  # For plotting the GT

        self.last_time = None
        self.cnt = 0

        # Set up pyplot for visualization
        plt.ion()
        self.fig, self.ax = plt.subplots()

        self.ax.set_title("Robot Odometry")

        self.ax.set_aspect('equal')
        self.plot_landmarks()
        self.get_logger().info("Finished init class")

    def cmd_vel_callback(self, msg: Twist):
        """ Callback function for cmd_vel messages. """
        time = self.get_clock().now().to_msg()
        current_time = time.sec + time.nanosec * 1e-9
        if self.last_time == None:
            dt = 0
        else:
            dt = current_time - self.last_time
        self.last_time = current_time
        v = msg.linear.x
        omega = msg.angular.z
        self.process_noise = self.update_process_noise(v, omega)
        self.state = predict_diff_drive(self.state, v, omega, dt)
        self.get_logger().info("Predicted state: \n\tx: {}\n\t y: {}\n\t theta: {}".format(
            self.state[0], self.state[1], self.state[2]), throttle_duration_sec=0.5)
        F = jacobian_of_predict_diff_drive(self.state, v, omega, dt)
        self.covariance = F @ self.covariance @ F.T + self.process_noise

    def laser_callback(self, msg: LaserScan) -> None:
        """ Process the laser scan message and update the state estimate """
        measurements = self.process_laser_scan(msg)
        known_correspondences = []
        for measurement in measurements:
            # Convert range and bearing to global frame using odometry
            dx = measurement['range'] * np.cos(measurement['bearing'])
            dy = measurement['range'] * np.sin(measurement['bearing'])
            landmark_global_x = self.state[0] + dx * np.cos(self.state[2]) - dy * np.sin(self.state[2])
            landmark_global_y = self.state[1] + dx * np.sin(self.state[2]) + dy * np.cos(self.state[2])
            
            # Find the closest known landmark
            min_dist = float('inf')
            closest_landmark_idx = None
            for i, landmark in enumerate(self.landmarks):
                dist = np.sqrt((landmark['x'] - landmark_global_x)**2 + (landmark['y'] - landmark_global_y)**2)
                if dist < min_dist:
                    min_dist = dist
                    closest_landmark_idx = i
            
            # Check if the closest landmark is within the association threshold
            if min_dist < self.get_parameter('max_association_distance').value:
                known_correspondences.append(closest_landmark_idx)
            else:
                known_correspondences.append(None)

        # Now, use these correspondences to update the state and covariance
        for i, measurement in enumerate(measurements):
            if known_correspondences[i] is not None:
                self.state, self.covariance = update(self.state, self.covariance, measurement, self.measurement_noise, self.landmarks[known_correspondences[i]])
        if self.cnt % 2 == 0:
            self.update_plot(observed_landmarks=known_correspondences, correspondences=known_correspondences)
        self.state_x.append(self.state[0])
        self.state_y.append(self.state[1])
        self.cnt += 1


    def odom_callback(self, msg):
        """ Callback function for odometry/GT messages. """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.odom_x.append(x)
        self.odom_y.append(y)
        self.plot_odometries()

    def process_laser_scan(self, msg: LaserScan) -> List[Dict['str', float]]:
        """ Process the laser scan message and return a list of range-bearing measurements """
        measurements: List[Dict['str', float]] = []
        total_angles = len(msg.ranges)
        angle_increment = msg.angle_increment
        for i in range(total_angles):
            if msg.ranges[i] < msg.range_max:
                bearing = msg.angle_min + i * angle_increment
                measurements.append({
                    'range': msg.ranges[i],
                    'bearing': bearing
                })
        return measurements


    def update_plot(self, observed_landmarks, correspondences):
        """ Update all plots """
        self.ax.clear()
        # Plot starting positions
        if len(self.state_x) != 0 and len(self.state_y) != 0:
            self.ax.scatter(self.state_x[0], self.state_x[0], marker="o", color="red", s=50, linewidths=1.5, edgecolors="k", label="EKF Start")
        if len(self.odom_x) != 0 and len(self.odom_y) != 0:
            self.ax.scatter(self.odom_x[0], self.odom_y[0], marker="o", color="green", s=50, linewidths=1.5, edgecolors="k", label="GT Start")
        self.plot_landmarks()
        self.plot_correspondences(correspondences=correspondences)
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
        """ Destructor to save the plot window and config. """
        save_config(self, "ekf_localization_with_known_correspondences")
        
def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt, shutting down node")
        node.save_config()
        node.destroy_node()

if __name__ == '__main__':
    main()
