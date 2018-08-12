from slam_parameters import *
from slam_utils.map import generate_ground_truth_map
from slam_utils.measurement_model import generate_measurements
from slam_utils.path_generator import generate_ground_truth_path

import matplotlib.pyplot as plt
import numpy as np

import random as rnd


class GraphSlamState(object):
    def __init__(self):
        self.ground_truth_map = np.empty((0, 0))
        self.landmarks = []

        self.ground_truth_states = []
        self.measurements = []

        self.true_random_gen = rnd.SystemRandom()


if __name__ == "__main__":
    ground_truth_map, landmarks = generate_ground_truth_map(MAP_HEIGHT, MAP_WIDTH, LANDMARK_COUNT)

    # Set up truly random number generation for creating the ground truth path (if the system supports it)
    true_random_gen = rnd.SystemRandom()
    rnd.seed(true_random_gen.random())

    ground_truth_states = generate_ground_truth_path(ground_truth_map, max_velocity=MAX_VELOCITY,
                                                     velocity_variance=VELOCITY_VARIANCE, max_turn_rate=MAX_TURN_RATE,
                                                     turn_rate_variance=TURN_RATE_VARIANCE, step_count=STEP_COUNT)

    measurements = generate_measurements(ground_truth_states, landmarks, max_sensing_range=MAX_SENSING_RANGE,
                                         sensing_range_variance=SENSING_RANGE_VARIANCE)

    plt.plot()
    plt.title("Ground truth map")
    plt.imshow(ground_truth_map, origin='lower')

    path_x = []
    path_y = []
    for state in ground_truth_states:
        path_x.append(state[0])
        path_y.append(state[1])

    plt.plot(path_x, path_y, marker='o')

    current_state = 1

    measurements_for_state = measurements[current_state]
    x_measurements = []
    y_measurements = []

    for measurement in measurements_for_state:
        x_measurements.append(ground_truth_states[current_state][0] + math.cos(measurement[1]) * measurement[0])
        y_measurements.append(ground_truth_states[current_state][1] + math.sin(measurement[1]) * measurement[0])

    plt.scatter(x_measurements, y_measurements, c="red")
    plt.scatter(ground_truth_states[current_state][0], ground_truth_states[current_state][1], s=100, c='green')

    plt.show()
