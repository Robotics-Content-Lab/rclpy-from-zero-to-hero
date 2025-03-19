#!/usr/bin/env python3
from typing import List, Optional, Dict, Callable, Tuple
from time import time
import numpy as np
from scipy.optimize import minimize
from scipy.stats import chi2
from scipy.linalg import block_diag
from multiprocessing import Pool, Manager
from ekf_localization.utils.helpers import normalize_angle

# Helper Functions #
def filter_nearby_landmarks(state: np.ndarray, known_landmarks: List[Dict['str', float]], radius: float):
    """
    Filter landmarks within a certain radius of the robot.

    Args:
    ========
    * `state`: The current state of the robot in the format (x, y, theta).
    * `known_landmarks`: A list of landmarks in the format {'x': x, 'y': y}.
    * `radius`: The radius within which to filter landmarks.

    Returns:
    ========
    * `nearby_landmarks`: A list of tuples containing the index and landmark of each landmark within the radius.
    """
    
    nearby_landmarks = []
    for i, landmark in enumerate(known_landmarks):
        distance = np.sqrt((state[0] - landmark['x'])**2 + (state[1] - landmark['y'])**2)
        if distance < radius:
            nearby_landmarks.append((i, landmark))
    return nearby_landmarks

def compute_mahalanobis_distance(state: np.ndarray, measurement:np.ndarray, landmark: Dict['str', float], covariance: np.ndarray, measurement_noise: np.ndarray, h: Callable, jacobian_of_h: Callable):
    """
    Compute the Mahalanobis distance between a measurement and a landmark.

    Args:
    ========
    * `state`: The current state of the robot in the format (x, y, theta).
    * `measurement`: The measurement in the format (range, bearing).
    * `landmark`: The landmark in the format {'x': x, 'y': y}.
    * `covariance`: The covariance matrix of the robot state.
    * `measurement_noise`: The covariance matrix of the measurement noise.
    * `h`: The measurement model.
    * `jacobian_of_h`: The Jacobian of the measurement model.

    Returns:
    ========
    * `mahalanobis_distance`: The Mahalanobis distance between the measurement and the landmark.
    """

    expected_measurement = h(state, landmark)
    measurement_residual = np.array(
        [measurement[0] - expected_measurement[0], normalize_angle(measurement[1] - expected_measurement[1])])
    H = jacobian_of_h(state, landmark)
    S = H @ covariance @ H.T + measurement_noise
    mahalanobis_distance = measurement_residual.T @ np.linalg.inv(S) @ measurement_residual
    return mahalanobis_distance


# EKF Localization With Known Correspondences #
def get_known_correspondences(observations: np.ndarray, GTPose: np.ndarray, Map: List[Dict], max_association_distance: float = 0.5) -> List[Optional[int]]:
    """
    Given a list of observations, a ground truth pose, and a map, find the closest landmark for each observation.
    If the closest landmark is within the association threshold, return the index of the closest landmark.
    Otherwise, return None.
    
    Args:
    ========
    * `observations`: A list of observations in the format (range, bearing).
    * `GTPose`: The ground truth pose of the robot in the format (x, y, theta).
    * `Map`: A list of landmarks in the format {'x': x, 'y': y}.
    * `max_association_distance`: The maximum distance for a landmark to be associated with an observation.

    Returns:
    ========
    * `known_correspondences`: A list of indices of the closest landmarks for each observation. If the closest
    landmark is not within the association threshold, the index is None.
    """
    odom_x = GTPose[0]
    odom_y = GTPose[1]
    odom_th = GTPose[2]
    known_correspondences = []
    for observation in observations:
        # Convert range and bearing to global frame using GT
        dx = observation[0] * np.cos(observation[1])
        dy = observation[0] * np.sin(observation[1])
        landmark_global_x = odom_x + dx * np.cos(odom_th) - dy * np.sin(odom_th)
        landmark_global_y = odom_y + dx * np.sin(odom_th) + dy * np.cos(odom_th)
        # Find the closest known landmark
        min_dist = float('inf')
        closest_landmark_idx = None
        for i, landmark in enumerate(Map):
            dist = np.sqrt((landmark['x'] - landmark_global_x)**2 + (landmark['y'] - landmark_global_y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_landmark_idx = i
        # Check if the closest landmark is within the association threshold
        if min_dist < max_association_distance:
            known_correspondences.append(closest_landmark_idx)
        else:
            known_correspondences.append(None)
    return known_correspondences

# EKF Localization With Unknown Correspondences #
def nearest_neighbor_data_association(state: np.ndarray, measurements: np.ndarray, known_landmarks: List[Dict['str', float]], covariance: np.ndarray, measurement_noise: np.ndarray, h: Callable, jacobian_of_h: Callable, max_distance: float = np.inf):
    """
    Given a list of measurements, a list of known landmarks, and a maximum distance, find the closest landmark for each measurement.
    This is implemented using the nearest neighbor data association algorithm.
    If the closest landmark is within the maximum distance, return the index of the closest landmark.
    Otherwise, return None.

    Args:
    ========
    * `state`: The current state of the robot in the format (x, y, theta).
    * `measurements`: A list of measurements in the format {'range': range, 'bearing': bearing}.
    * `known_landmarks`: A list of landmarks in the format {'x': x, 'y': y}.
    * `max_distance`: The maximum distance for a landmark to be associated with a measurement.

    Returns:
    ========
    * `correspondence_indices`: A list of indices of the closest landmarks for each measurement. If the closest
    landmark is not within the maximum distance, the index is None.
    """
    
    correspondence_indices = []
    for measurement in measurements:
        min_distance = np.inf
        closest_index = -1
        for i, landmark in enumerate(known_landmarks):
            # Convert measurement into global coordinates
            dx = measurement[0] * np.cos(measurement[1] + state[2])
            dy = measurement[0] * np.sin(measurement[1] + state[2])
            mx, my = state[0] + dx, state[1] + dy
            
            # Compute the Euclidean distance
            euclidean_distance = np.sqrt((mx - landmark['x'])**2 + (my - landmark['y'])**2)
            
            if euclidean_distance < min_distance and euclidean_distance < max_distance:
                min_distance = euclidean_distance
                closest_index = i
        correspondence_indices.append(closest_index if closest_index != -1 else None)
    return correspondence_indices

def maximum_likelihood_data_association(state: np.ndarray, measurements: np.ndarray, known_landmarks: List[Dict['str', float]], covariance: np.ndarray, measurement_noise: np.ndarray, h: Callable, jacobian_of_h: Callable, max_distance: float = np.inf):
    """
    Given a list of measurements, a list of known landmarks, a covariance matrix, a measurement noise matrix, and a maximum distance,
    find the closest landmark for each measurement. This is implemented using the maximum likelihood data association algorithm.
    If the closest landmark is within the maximum distance, return the index of the closest landmark.
    Otherwise, return None.

    Args:
    ========
    * `state`: The current state of the robot in the format (x, y, theta).
    * `measurements`: A list of measurements in the format (range, bearing).
    * `known_landmarks`: A list of landmarks in the format {'x': x, 'y': y}.
    * `covariance`: The covariance matrix of the robot state.
    * `measurement_noise`: The covariance matrix of the measurement noise.
    * `max_distance`: The maximum distance for a landmark to be associated with a measurement.

    Returns:
    ========
    * `correspondence_indices`: A list of indices of the closest landmarks for each measurement. If the closest
    landmark is not within the maximum distance, the index is None.
    """
    correspondence_indices = []
    for measurement in measurements:
        min_distance = np.inf
        closest_index = -1
        for i, landmark in enumerate(known_landmarks):
            mahalanobis_distance = compute_mahalanobis_distance(
                state=state, measurement=measurement, landmark=landmark, covariance=covariance,
                measurement_noise=measurement_noise, h=h, jacobian_of_h=jacobian_of_h)
            if mahalanobis_distance < min_distance and mahalanobis_distance < max_distance:
                min_distance = mahalanobis_distance
                closest_index = i
        correspondence_indices.append(closest_index if closest_index != -1 else None)
    return correspondence_indices

# TODO speed up this function, this shit never converges
def jcbb_data_association(state: np.ndarray, measurements: np.ndarray, known_landmarks: List[Dict['str', float]], covariance: np.ndarray, measurement_noise: np.ndarray, h: Callable, jacobian_of_h: Callable, alpha: float = 0.05):
    """
    Given a list of measurements, a list of known landmarks, a covariance matrix, a measurement noise matrix, and a maximum distance,
    find the closest landmark for each measurement. This is implemented using the joint compatibility branch and bound data association algorithm.
    If the closest landmark is within the maximum distance, return the index of the closest landmark.
    Otherwise, return None.

    Args:
    ========
    * `state`: The current state of the robot in the format (x, y, theta).
    * `measurements`: A list of measurements in the format (range, bearing).
    * `known_landmarks`: A list of landmarks in the format {'x': x, 'y': y}.
    * `covariance`: The covariance matrix of the robot state.
    * `measurement_noise`: The covariance matrix of the measurement noise.
    * `alpha`: The probability of a false positive.

    Returns:
    ========
    * `correspondence_indices`: A list of indices of the closest landmarks for each measurement. If the closest
    landmark is not within the maximum distance, the index is None.
    """

    def joint_compatibility_test(hypothesis):
        innovations = []
        joint_covariance = []
        for i, j in hypothesis:
            if j is not None:
                expected_measurement = h(state, known_landmarks[j])
                innovation = measurements[i] - expected_measurement
                innovations.extend(innovation)
                H = jacobian_of_h(state, known_landmarks[j])
                cov = H @ covariance @ H.T + measurement_noise
                joint_covariance.append(cov)
        if innovations:
            S = block_diag(*joint_covariance)
            chi2_val = np.array(innovations).T @ np.linalg.inv(S) @ np.array(innovations)
            threshold = chi2.ppf(1 - alpha, 2 * len(innovations))
            return chi2_val < threshold
        return True

    n = len(measurements)
    # Filter landmarks within a certain radius to reduce the search space
    nearby_landmarks = filter_nearby_landmarks(state, known_landmarks, 5.0)
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
            new_cost = cost.value + compute_mahalanobis_distance(
                state=state, measurement=measurements[i], known_landmarks=known_landmarks[j],covariance=covariance,
                measurement_noise=measurement_noise, h=h, jacobian_of_h=jacobian_of_h)
            if new_cost < chi2.ppf(1 - alpha, 2 * (i + 1)):
                search(new_hypothesis, manager.Value('d', new_cost), i + 1)

    with Pool(processes=12) as pool:
        pool.apply_async(search, ([], manager.Value('d', 0.0), 0))

    pool.close()
    pool.join()

    return list(best_hypothesis), best_cost.value
    
def bsc_data_association(state: np.ndarray, measurements: np.ndarray, known_landmarks: List[Dict['str', float]], covariance: np.ndarray, measurement_noise: np.ndarray, h: Callable, jacobian_of_h: Callable, alpha=0.05, time_limit=5.0):
    """
    Given a list of measurements, a list of known landmarks, a covariance matrix, a measurement noise matrix, and a maximum distance,
    find the closest landmark for each measurement. This is implemented using the branch and bound data association algorithm.
    If the closest landmark is within the maximum distance, return the index of the closest landmark.
    Otherwise, return None.

    Args:
    ========
    * `state`: The current state of the robot in the format (x, y, theta).
    * `measurements`: A list of measurements in the format (range, bearing).
    * `known_landmarks`: A list of landmarks in the format {'x': x, 'y': y}.
    * `covariance`: The covariance matrix of the robot state.
    * `measurement_noise`: The covariance matrix of the measurement noise.
    * `alpha`: The probability of a false positive.
    * `bound`: The maximum number of potential correspondences to consider for each measurement.

    Returns:
    ========
    * `correspondence_indices`: A list of indices of the closest landmarks for each measurement. If the closest
    landmark is not within the maximum distance, the index is None.
    """

    start_time = time()
    n = len(measurements)
    m = len(known_landmarks)
    best_cost = float('inf')
    best_hypotheses = [None] * n

    chi2_threshold = chi2.ppf(1-alpha, 2)

    def search(hypothesis: List[Tuple[int, int]], cost: float, i: int):
        nonlocal best_cost, best_hypotheses
        elapsed_time = time() - start_time
        if elapsed_time > time_limit:
            return
        if cost >= best_cost:
            return
        if i == n:
            if cost < best_cost:
                best_cost = cost
                best_hypotheses[:] = [h[1] for h in hypothesis]
            return

        # No association case
        search(hypothesis + [(i, None)], cost, i + 1)

        # Association cases
        for j in range(m):
            mahalanobis_distance = compute_mahalanobis_distance(
                state, measurements[i], known_landmarks[j], covariance, measurement_noise, h, jacobian_of_h)
            
            if mahalanobis_distance < chi2_threshold:
                new_cost = cost + mahalanobis_distance
                search(hypothesis + [(i, j)], new_cost, i + 1)

    # Start the recursive search
    search([], 0.0, 0)

    return best_hypotheses

# TODO This could be used in SLAM later on. When unknown correspondences are present,
# we can use triangulation to estimate the landmark positions.
@staticmethod
def triangulate(observations, correspondences, landmarks, state, covariance):
    def objective(params):
        x, y, theta = params
        error = 0.0
        for i, obs in enumerate(observations):
            dx = obs[0] * np.cos(obs[1])
            dy = obs[0] * np.sin(obs[1])
            landmark_global_x = x + dx * np.cos(theta) - dy * np.sin(theta)
            landmark_global_y = y + dx * np.sin(theta) + dy * np.cos(theta)

            landmark = landmarks[correspondences[i]]
            dist = np.sqrt((landmark['x'] - landmark_global_x)**2 + (landmark['y'] - landmark_global_y)**2)
            error += dist**2
        return error

    initial_guess = [state[0], state[1], state[2]]
    result = minimize(objective, initial_guess, method='SLSQP', options={'maxiter': 30})
    if result.success:
        updated_state = np.array(result.x)
        updated_covariance = covariance  
        return updated_state, updated_covariance
    else:
        return state, covariance
