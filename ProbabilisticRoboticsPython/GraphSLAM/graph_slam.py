from slam_parameters import *
from slam_utils.graph_slam_initialize import graph_slam_initialize
from slam_utils.map_generator import generate_ground_truth_map
from slam_utils.measurement_model import generate_measurements
from slam_utils.path_generator import generate_ground_truth_path
from slam_utils.plot_utils import plot_path, plot_measurements_for_state

from slam_utils.ctrv_motion_model import calculate_odometry_from_controls, calculate_jacobian_from_controls

import matplotlib.pyplot as plt
import numpy as np

import random as rnd


class GraphSlamState(object):
    def __init__(self):
        self.ground_truth_map = np.empty((0, 0))
        self.landmarks = []

        self.ground_truth_states = []
        self.controls = []

        self.measurements = []

        self.initial_state_estimates = []

        self.true_random_gen = rnd.SystemRandom()


if __name__ == "__main__":
    ground_truth_map, landmarks = generate_ground_truth_map(MAP_HEIGHT, MAP_WIDTH, LANDMARK_COUNT)

    # Set up truly random number generation for creating the ground truth path (if the system supports it)
    true_random_gen = rnd.SystemRandom()
    rnd.seed(true_random_gen.random())

    ground_truth_states, controls = \
        generate_ground_truth_path(ground_truth_map, max_velocity=MAX_VELOCITY,
                                   velocity_deviation=VELOCITY_DEVIATION, max_turn_rate=MAX_TURN_RATE,
                                   turn_rate_deviation=TURN_RATE_DEVIATION, step_count=STEP_COUNT,
                                   velocity_control_deviation=VELOCITY_CONTROL_DEVIATION,
                                   turn_rate_control_deviation=TURN_RATE_CONTROL_DEVIATION)

    measurements = generate_measurements(ground_truth_states, landmarks, max_sensing_range=MAX_SENSING_RANGE,
                                         sensing_range_deviation=SENSING_RANGE_DEVIATION,
                                         distance_deviation=DISTANCE_DEVIATION, heading_deviation=HEADING_DEVIATION)

    state_estimates = graph_slam_initialize(controls, state_t0=ground_truth_states[0])

    # Perform GraphSLAM linearize
    omega = np.identity(3, dtype="float") * 100000
    xi = np.empty((1, 1))

    R = np.identity(3)

    for index, control in enumerate(controls[1:]):
        v = control.item(0)
        om = control.item(1)
        previous_state = state_estimates[index]
        current_state_estimate = state_estimates[index] + calculate_odometry_from_controls(v, om, previous_state)

        # Calculate the 6-by-6 update for the information matrix Omega
        Gt = np.identity(3)
        Gt[:, -1] = np.ravel(calculate_jacobian_from_controls(v, om, previous_state))
        omega_xt_xt_1 = np.dot(np.concatenate((-Gt.T, np.identity(3))), np.linalg.inv(R))
        omega_xt_xt_1 = np.dot(omega_xt_xt_1, np.concatenate((-Gt, np.identity(3)), axis=1))

        upper_left_index = index * 3
        lower_right_index = index * 3 + 6

        # Expand Omega to accommodate the new information from the motion
        while lower_right_index > omega.shape[0]:
            omega = np.concatenate((omega, np.zeros((omega.shape[0], 1))), axis=1)
            omega = np.concatenate((omega, np.zeros((1, omega.shape[1]))), axis=0)

        omega[upper_left_index:lower_right_index, upper_left_index:lower_right_index] += omega_xt_xt_1

    plt.plot()
    plt.title("Ground truth map")
    plt.imshow(ground_truth_map, origin='lower')

    plot_path(ground_truth_states, 'C0')
    plot_path(state_estimates, 'C1')

    current_state = 1
    plot_measurements_for_state(ground_truth_states[current_state], measurements[current_state])

    plt.show()
