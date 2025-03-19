from typing import Dict, List
import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

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
    for dict in self.landmarks:
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
            x_values.append(self.landmarks[index]['x'])
            y_values.append(self.landmarks[index]['y'])
    if len(x_values) == 0 or len(y_values) == 0:
        return
    self.ax.scatter(x_values, y_values, marker="o", color="red", s=50, linewidths=1.5, alpha=0.6, linestyle="--", edgecolors="k", label="Observed Landmarks")

def plot_correspondences(self, correspondences):
    """ Plot the correspondences between the current state and the observed landmarks. """
    for index in correspondences:
        if index is not None:
            self.ax.plot([self.state[0], self.landmarks[index]['x']], [self.state[1], self.landmarks[index]['y']], color='b', linestyle="--", alpha=0.5)
    # self.ax.plot(Line2D([0], [0], color='blue', linewidth=2), label='Correspondences')

def plot_odometries(self):
    """ Plot the odometry data of the GT and EKF state. """
    self.ax.plot(self.odom_x, self.odom_y,
                    color='green', label='Odometry GT')
    self.ax.plot(self.state_x, self.state_y,
                    color='red', label='EKF State')

def plot_state_and_covariance(self):
    """ Plot the current state and covariance. """
    # Plot the state as a red 'x'
    self.ax.plot(self.state[0], self.state[1], 'x', color='red')

    # Plot the covariance as a blue ellipse
    eigenvalues, eigenvectors = np.linalg.eig(self.covariance[0:2, 0:2])
    angle = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))
    ellipse = Ellipse((self.state[0], self.state[1]), 2*np.sqrt(5.991*eigenvalues[0]),
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
                "final": self.process_noise.tolist()
            },
            "measurement_noise": {
                "initial": np.diag(self.get_parameter('measurement_noise').value).tolist(),
                "final": self.measurement_noise.tolist()
            },
            "covariance": {
                "initial": np.diag(self.get_parameter('initial_covariance').value).tolist(),
                "final": self.covariance.tolist()
            },
            "state": {
                "initial_ekf": self.get_parameter('initial_state').value,
                "initial_odom": [self.odom_x[0], self.odom_y[0], 0],
                "final_ekf": self.state.tolist(),
                "final_odom": [self.odom_x[-1], self.odom_y[-1], 0]
            }
        }
        yaml.dump(config, f)
    plt.savefig(os.path.join(img_path, f"{img_name}.png"))
    plt.close()
