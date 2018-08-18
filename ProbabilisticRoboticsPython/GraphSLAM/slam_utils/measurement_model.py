from slam_utils.angle_utils import normalize_angle_pi_minus_pi

import numpy as np

import math
import random as rnd


def get_landmarks_and_distances_in_range(ground_truth_state, landmarks, max_sensing_range):
    landmark_distances = [np.linalg.norm(landmark[:2] - ground_truth_state[:2]) for landmark in landmarks]
    return [(landmark, landmark_distances[index]) for index, landmark in
            enumerate(landmarks) if landmark_distances[index]
            <= max_sensing_range]


def calculate_measurement_vector_for_detection(ground_truth_state, detected_landmark_and_distance):
    x_landmark = detected_landmark_and_distance[0][0]
    y_landmark = detected_landmark_and_distance[0][1]
    x_state = ground_truth_state[0]
    y_state = ground_truth_state[1]

    phi = math.atan2(y_landmark - y_state, x_landmark - x_state)
    phi = phi - ground_truth_state[2]

    return np.array([[detected_landmark_and_distance[1],
                    phi,
                    detected_landmark_and_distance[0][2]]]).T


def generate_measurements(ground_truth_states, landmarks, max_sensing_range, sensing_range_deviation,
                          distance_deviation, heading_deviation):
    """
    Generates a list of measurements for every state in ground_truth_states. Measurements are numpy arrays of
    [r, phi, s].T, r being the distance to landmark, phi is heading (positive X direction is 0 rad, grows
    counter-clockwise), s is the landmark descriptor - an integer. Measurements contain additional zero-mean gaussian
    noise for the distance and heading, as specified by distance_deviation and heading_deviation.

    For each state, only landmarks within max_sensing_range are considered. From this group of landmarks, detections
    are sampled using the absolute values of random samples from a zero mean, sensing_range_deviation normal
    distribution as distance threshold.
    """

    measurements = []

    for ground_truth_state in ground_truth_states:
        landmarks_and_distances_in_range = get_landmarks_and_distances_in_range(ground_truth_state, landmarks,
                                                                                max_sensing_range)

        # Sample obstacles from the in-range ones, using the absolute values of samples from a zero-mean normal
        # distribution as distance thresholds
        detected_landmarks_and_distances = [landmark_and_distance for landmark_and_distance in
                                            landmarks_and_distances_in_range if
                                            abs(rnd.normalvariate(0, sensing_range_deviation))
                                            >= landmark_and_distance[1]]

        measurements_for_state = [
            calculate_measurement_vector_for_detection(ground_truth_state, detected_landmark_and_distance) for
            detected_landmark_and_distance in detected_landmarks_and_distances]

        for measurement in measurements_for_state:
            measurement[0] = measurement[0] + rnd.normalvariate(0, distance_deviation)
            measurement[1] = measurement[1] + rnd.normalvariate(0, heading_deviation)
            measurement[1] = normalize_angle_pi_minus_pi(measurement[1])

        measurements.append(measurements_for_state)

    return measurements
