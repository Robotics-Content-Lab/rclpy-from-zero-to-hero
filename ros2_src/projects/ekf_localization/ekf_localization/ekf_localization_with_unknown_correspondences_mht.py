from typing import Tuple, Dict, List
import numpy as np
import matplotlib.pyplot as plt

import rclpy
from geometry_msgs.msg import Twist

from ekf_localization.utils.helpers import normalize_angle
from ekf_localization.ekf_localization_with_unknown_correspondences import EKFNode

class MHTEKFNode(EKFNode):
    """
    An EKF localization node with Multi-Hypothesis Testing (MHT).
    """

    def __init__(self, max_hypotheses=10, **kwargs):
        super().__init__(**kwargs)

        self.v = 0.0
        self.omega = 0.0
        self.dt = 0.0

        # Initialize the list of hypotheses
        self.hypotheses: List[Dict['str', np.ndarray]] = [{'state': self.state, 'covariance': self.covariance}]
        self.max_hypotheses = max_hypotheses  # Maximum number of hypotheses to retain
        self.plot_landmarks()  # plot initial markers

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
            self.dt = 0
        else:
            self.dt = current_time - self.last_time

        # Update the last time
        self.last_time = current_time

        # Get the current control inputs
        self.v = msg.linear.x
        self.omega = msg.angular.z
        self.process_noise = self.update_process_noise(self, self.v, self.omega)
        
        # Predict the current state
        self.state = self.predict_diff_drive(self.state, self.v, self.omega, self.dt)
        self.get_logger().info("Predicted state: \n\tx: {}\n\t y: {}\n\t theta: {}".format(
            self.state[0], self.state[1], self.state[2]), throttle_duration_sec=0.5)

        # Update the covariance
        F = self.jacobian_of_predict_diff_drive(self.state, self.v, self.omega, self.dt)
        self.covariance = F @ self.covariance @ F.T + self.process_noise

    def laser_callback(self, msg: List[Dict['str', float]]):
        """
        Handle multiple hypotheses based on a list of measurements.
        """
        measurements = self.process_laser_scan(msg)
        new_measurements = self.cluster_measurements(measurements)
        self.compare_clusters(original=measurements, clustered=new_measurements)
        measurements = new_measurements
        new_hypotheses = []

        for hypothesis_dict in self.hypotheses:
            # Predict and update for each hypothesis
            predicted_state, predicted_covariance = self.predict(hypothesis_dict['state'], hypothesis_dict['covariance'])  # Assuming motion inputs
            updated_state, updated_covariance = self.update(predicted_state, predicted_covariance, measurements)
            new_hypothesis = {'state': updated_state, 'covariance': updated_covariance}
            new_hypotheses.append(new_hypothesis)

        # Evaluate likelihood of each hypothesis
        likelihoods = [self.evaluate_likelihood(hypothesis, measurements) for hypothesis in new_hypotheses]

        # Sort hypotheses based on likelihood and retain the top ones
        sorted_indices = sorted(range(len(likelihoods)), key=lambda k: likelihoods[k], reverse=True)
        self.hypotheses = [new_hypotheses[i] for i in sorted_indices[:self.max_hypotheses]]

        # Set the current state and covariance to the most likely hypothesis
        self.state = self.hypotheses[0]['state']
        self.covariance = self.hypotheses[0]['covariance']
        
        if self.cnt % 2 == 0:
            self.update_plot(observed_landmarks=None, correspondences=None, measurements=measurements)
        self.state_x.append(self.state[0])
        self.state_y.append(self.state[1])
        self.cnt += 1

    def predict(self, state: np.ndarray, covariance: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Predict the next state and covariance based on the motion command.
        
        :param state: Current state [x, y, theta]. [3, 1]
        :param covariance: Current covariance matrix. [3, 3]
        :param v: Linear velocity. [1]
        :param omega: Angular velocity. [1]
        :param dt: Time step. [1]
        :return: Predicted state and covariance. [3, 1], [3, 3]
        """
        
        # 1. Compute the predicted state using the motion model
        predicted_state = self.predict_diff_drive(state, self.v, self.omega, self.dt)
        
        # 2. Compute the Jacobian of the motion model
        G = self.jacobian_of_predict_diff_drive(state, self.v, self.omega, self.dt)
        
        # 3. Update the covariance
        predicted_covariance = G @ covariance @ G.T + self.process_noise
        
        return predicted_state, predicted_covariance

    def update(self, state: np.ndarray, covariance: np.ndarray, measurements: List[Dict['str', float]]) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update the state and covariance given a list of measurements.

        :param state: The robot's current state.
        :param covariance: The robot's current covariance.
        :param measurements: The list of measurements (range and bearing).
        :return: The updated state and covariance.
        """
        for measurement in measurements:
            # Step 1: Predict the measurement for each landmark
            predicted_measurements = [self.measurement_function(state, landmark) for landmark in self.landmarks]

            # Step 2: Associate the measurement with a landmark
            # This could be done using a nearest-neighbor approach based on the difference between predicted and actual measurements
            diffs = [np.linalg.norm(np.array([measurement['range'] - pred[0], normalize_angle(measurement['bearing'] - pred[1])])) for pred in predicted_measurements]
            associated_landmark_index = np.argmin(diffs)
            associated_landmark = self.landmarks[associated_landmark_index]

            # Step 3: Update state and covariance based on associated measurement
            state, covariance = super().update(state, covariance, measurement, self.measurement_noise, associated_landmark)

        return state, covariance

    def evaluate_likelihood(self, hypothesis, measurements):
        """
        Evaluate the likelihood of a hypothesis given a list of laser measurements.
        
        :param hypothesis: The current hypothesis with state and covariance.
        :param measurements: A list of actual measurements.
        :return: The combined likelihood of the hypothesis given all measurements.
        """
        total_likelihood = 1.0
        
        for measurement in measurements:
            # Predict the measurement for each landmark
            predicted_measurements = [self.measurement_function(hypothesis['state'], landmark) for landmark in self.landmarks]
            
            # Find the landmark with the predicted measurement closest to the actual measurement
            diffs = [np.linalg.norm(np.array([measurement['range'] - pred[0], normalize_angle(measurement['bearing'] - pred[1])])) for pred in predicted_measurements]
            closest_measurement = predicted_measurements[np.argmin(diffs)]
            
            # Calculate the measurement residual (innovation)
            innovation = np.array([measurement['range'] - closest_measurement[0], 
                                normalize_angle(measurement['bearing'] - closest_measurement[1])])

            # Compute the likelihood for this measurement using the Gaussian likelihood formula
            S = self.measurement_noise  # Here, we use the measurement noise as the covariance
            likelihood = (1.0 / (2 * np.pi * np.linalg.det(S)**0.5)) * np.exp(-0.5 * innovation.T @ np.linalg.inv(S) @ innovation)
            
            total_likelihood *= likelihood  # Combine likelihoods for all measurements
        
        return total_likelihood


    def plot_most_likely_observations(self, state, measurements, landmarks):
        """
        Plot the most likely observations based on the provided state and measurements.
        
        :param state: The robot's state from the most likely hypothesis.
        :param measurements: The actual measurements.
        :param landmarks: The list of all landmarks.
        """
        # Predict the measurement for each landmark using the provided state
        predicted_measurements = [self.measurement_function(state, landmark) for landmark in landmarks]
        
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
        self.ax.scatter(x_values, y_values, marker="o", color="r", alpha=0.3, s=55, label="Most likely observations")

    def update_plot(self, observed_landmarks, correspondences, measurements=None):
        """ Update all plots """
        self.ax.clear()
        self.plot_most_likely_observations(self.state, measurements, self.landmarks)
        # Plot starting positions
        if len(self.state_x) != 0 and len(self.state_y) != 0:
            self.ax.scatter(self.state_x[0], self.state_y[0], marker="o", color="red", s=50, linewidths=1.5, edgecolors="k", label="EKF Start")
        if len(self.odom_x) != 0 and len(self.odom_y) != 0:
            self.ax.scatter(self.odom_x[0], self.odom_y[0], marker="o", color="green", s=50, linewidths=1.5, edgecolors="k", label="GT Start")
        self.plot_landmarks()
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

def main(args=None):
    rclpy.init(args=args)
    node = MHTEKFNode(measurement_noise=[1.4, 50.0], initial_state=[3.0, 1.0, 0.0], max_hypotheses=100)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt, shutting down node")
        node.save_config()
        node.destroy_node()


if __name__ == '__main__':
    main()
