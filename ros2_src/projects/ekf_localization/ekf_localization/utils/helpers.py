from typing import Dict, List
import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np

def load_landmarks(filename: str) -> List[Dict[str, float]]:
    """ Load landmark positions from a yaml file. """
    with open(filename, 'r') as f:
        landmarks = yaml.safe_load(f)
    return landmarks


def normalize_angle(angle: float) -> float:
    """
    Normalize an angle to [-pi, pi]

    :param angle: The angle to normalize
    :type angle: float
    :returns The angle normalized to [-pi, pi]
    :rtype: float
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi


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

def plot_odometries(self):
    """ Plot the odometry data of the GT and EKF state. """
    self.ax.plot(self.odom_x, self.odom_y,
                    color='green', label='Odometry GT')
    self.ax.plot(self.state_x, self.state_y,
                    color='red', label='EKF State')

def plot_state_and_covariance(self):
    """ Plot the current state and covariance. """
    # Plot the state as a red 'x'
    self.ax.plot(self.mu[0], self.mu[1], 'x', color='red')

    # Plot the covariance as a blue ellipse
    eigenvalues, eigenvectors = np.linalg.eig(self.P[0:2, 0:2])
    angle = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))
    ellipse = Ellipse((self.mu[0], self.mu[1]), 2*np.sqrt(5.991*eigenvalues[0]),
                        2*np.sqrt(5.991*eigenvalues[1]), angle=angle, fill=False, color='blue')
    self.ax.add_artist(ellipse)


def save_config(self, node_name: str):
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
    img_name = f"{node_name}_{identifier}"
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

    def plot(self, x, y, th, mu, P, measurements, clustered_observations, correspondences, observed_landmarks, save_fig=False):
        pass

    def update_plot(self, states: List[List], observed_landmarks, correspondences, cluster: np.ndarray, GTpose: List[List], save_fig=False):
        """ Update all plots """
        state_x, state_y, state_th = states
        odom_x, odom_y, odom_th = GTpose

        self.ax.clear()
        # Plot starting positions
        if len(state_x) != 0 and len(state_y) != 0:
            self.ax.scatter(state_x[0], state_y[0], marker="o", color="red", s=50, linewidths=1.5, edgecolors="k", label="EKF Start")
        if len(odom_x) != 0 and len(odom_y) != 0:
            self.ax.scatter(odom_x[0], odom_y[0], marker="o", color="green", s=50, linewidths=1.5, edgecolors="k", label="GT Start")
        self.plot_landmarks()
        self.plot_clustered_observations(ax=self.ax, clustered_observations=cluster, GTpose=np.asarray([odom_x[-1], odom_y[-1], odom_th[-1]]))
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

    def plot_clustered_observations(self, clustered_observations: List[Dict['str', float]], GTpose: np.ndarray, color='magenta', label='Clustered Observations'):
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
        self.ax.scatter(X_transformed[:, 0], X_transformed[:, 1], c=color, marker='x', label=label, alpha=0.5)

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

    def plot_odometries(self):
        """ Plot the odometry data of the GT and EKF state. """
        self.ax.plot(self.odom_x, self.odom_y,
                        color='green', label='Odometry GT')
        self.ax.plot(self.state_x, self.state_y,
                        color='red', label='EKF State')

    def plot_state_and_covariance(self, state: np.ndarray, covariance: np.ndarray):
        """ Plot the current state and covariance. """
        mu = state[0:2]
        P = covariance[0:2, 0:2]
        # Plot the state as a red 'x'
        self.ax.plot(mu[0], mu[1], 'x', color='red')

        # Plot the covariance as a blue ellipse
        eigenvalues, eigenvectors = np.linalg.eig(P[0:2, 0:2])
        angle = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))
        ellipse = Ellipse((mu[0], mu[1]), 2*np.sqrt(5.991*eigenvalues[0]),
                            2*np.sqrt(5.991*eigenvalues[1]), angle=angle, fill=False, color='blue')
        self.ax.add_artist(ellipse)


    def save_config(self, node_name: str):
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
        img_name = f"{node_name}_{identifier}"
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