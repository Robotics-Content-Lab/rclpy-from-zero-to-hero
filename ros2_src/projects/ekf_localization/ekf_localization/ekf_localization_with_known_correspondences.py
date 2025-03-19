import numpy as np
import matplotlib.pyplot as plt

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


from ekf_localization.utils.helpers import plot_landmarks, plot_observed_landmarks, plot_correspondences, plot_odometries, save_config, plot_state_and_covariance
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
    """ An EKF localization node with known correspondences. """

    def __init__(self, node_name='ekf_node', initial_state=[6., 10., 0.], measurement_noise=[2., 2.], *args, **kwargs):
        super().__init__(node_name=node_name, initial_state=initial_state, measurement_noise=measurement_noise)
        self.initial_state = self.get_parameter('initial_state').value
        self.state_x = [self.initial_state[0]]  # For plotting the state
        self.state_y = [self.initial_state[1]]  # For plotting the state

        self.odom_x = []  # For plotting the GT
        self.odom_y = []  # For plotting the GT
        self.odom_th = []  # For plotting the GT

        self.last_time = None
        self.cnt = 0

        self.init_figure()

        self.declare_parameter('max_association_distance', kwargs.get('max_association_distance', 2.0))
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 1)  # For motion model and prediction
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 1)  # For measurement model and update
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 1)  # We take odom as GT for comparison
        self.get_logger().info("Finished init class")
    
    def init_figure(self):
        # Set up pyplot for visualization
        plt.ion()
        self.fig, self.ax = plt.subplots()

        self.ax.set_title("Robot Odometry")

        self.ax.set_aspect('equal')
        # self.plot_landmarks()
        self.update_plot([], [], [])

    def cmd_vel_callback(self, msg: Twist):
        """ Callback function for cmd_vel messages. """
        time = self.get_clock().now().to_msg()
        current_time = time.sec + time.nanosec * 1e-9
        if self.last_time == None:
            dt = 0
        else:
            dt = current_time - self.last_time
        self.last_time = current_time
        u = np.array([msg.linear.x, msg.angular.z])  # control input
        self.Q = self.update_process_noise(u, self.Q)  # Update process noise
        self.mu = self.f(self.mu, u, dt)  # Predict state based on motion model
        self.get_logger().info("Predicted state: \n\tx: {}\n\t y: {}\n\t theta: {}".format(
            self.mu[0], self.mu[1], self.mu[2]), throttle_duration_sec=0.5)
        G = self.jacobian_of_f(self.mu, u, dt)  # Compute Jacobian of motion model
        self.P = G @ self.P @ G.T + self.Q  # Update covariance based on motion model

    def laser_callback(self, msg: LaserScan) -> None:
        if len(self.odom_x) == 0:
            return
        """ Process the laser scan message and update the state estimate """

        observations = self.process_laser_scan(msg)
        clustered_observations = self.cluster_observations(observations)
        observations = clustered_observations
        known_correspondences = []

        for observation in observations:
            # Convert range and bearing to global frame using GT (known correspondences)
            dx = observation['range'] * np.cos(observation['bearing'])
            dy = observation['range'] * np.sin(observation['bearing'])
            landmark_global_x = self.odom_x[-1] + dx * np.cos(self.odom_th[-1]) - dy * np.sin(self.odom_th[-1])
            landmark_global_y = self.odom_y[-1] + dx * np.sin(self.odom_th[-1]) + dy * np.cos(self.odom_th[-1])
            
            # Find the closest known landmark
            min_dist = float('inf')
            closest_landmark_idx = None
            for i, landmark in enumerate(self.map):
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
        for i, z in enumerate(observations):
            if known_correspondences[i] is not None:
                self.mu, self.P = self.update(self.mu, self.P, z, self.R, self.map[known_correspondences[i]])
        if self.cnt % 2 != 0:
            self.update_plot(observed_landmarks=known_correspondences, correspondences=known_correspondences, clustered_observations=clustered_observations)
            
        self.state_x.append(self.mu[0])
        self.state_y.append(self.mu[1])
        self.cnt += 1

    def odom_callback(self, msg: Odometry):
        """ Callback function for odometry/GT messages. """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        th = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)[2]
        self.odom_x.append(x)
        self.odom_y.append(y)
        self.odom_th.append(th)
        self.plot_odometries()

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
        """ Destructor to save the plot window and config. """
        print("Saving config: ", self.get_parameter('save_fig'))
        if self.get_parameter_or('save_fig', False):
            save_config(self, "ekf_localization_with_known_correspondences")




def main(args=None):
    rclpy.init(args=args)
    node = EKFNode(initial_state=[-8.0, 8.0, 0.0], measurement_noise=[4.0, 4.0])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt, shutting down node")
        node.save_config()
        node.destroy_node()

if __name__ == '__main__':
    main()
