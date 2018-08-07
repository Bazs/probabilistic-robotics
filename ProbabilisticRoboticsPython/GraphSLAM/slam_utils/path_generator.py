from slam_utils.map import generate_random_free_coordinate

import numpy as np

import math
from math import sin, cos
import random as rnd


def clip(value, minimum, maximum):
    return min(max(value, minimum), maximum)


def normalize_angle_pi_minus_pi(angle):
    while -math.pi > angle:
        angle = angle + math.pi * 2
    while math.pi < angle:
        angle = angle - math.pi * 2

    return angle


def generate_ground_truth_path(ground_truth_map, max_velocity, velocity_variance, max_turn_rate, turn_rate_variance,
                               step_count):
    """Generates a random path using a constant velocity and turn rate motion model.

    Generates a sequence of step_count consecutive [x, y, theta].T states. The velocity and turn rate is piecewise
    constant between two states, and they change by a random value at each state, which is sampled from a zero mean
    gaussian distribution, whose variance is given by the parameters velocity_variance and turn_rate_variance. The
    velocity and turn rate are also limited to the range of [0, max_velocity] and [-max_turn_rate, max_turn_rate],
    respectively.
    Additionally, each generated node is ensured to be collision free with respect to the specified map.
    """
    start_x, start_y = generate_random_free_coordinate(ground_truth_map)
    start_x = start_x + rnd.random() - 1
    start_y = start_y + rnd.random() - 1
    start_th = math.pi * (2 * rnd.random() - 1)

    current_state = np.array([[start_x, start_y, start_th]]).T
    ground_truth_states = [current_state]

    # velocity and turn rate
    v, omega = 0, 0

    for step in range(step_count):
        proposal_state_is_valid = False

        while proposal_state_is_valid is False:
            v_proposal = v + rnd.normalvariate(0, velocity_variance)
            v_proposal = clip(v_proposal, 0, max_velocity)

            omega_proposal = omega + rnd.normalvariate(0, turn_rate_variance)
            omega_proposal = clip(omega_proposal, -max_turn_rate, max_turn_rate)

            # CTRV motion model
            delta_pos = np.array([[v_proposal/omega_proposal * (sin(omega_proposal + current_state[2])
                                                                - sin(current_state[2])),
                                   v_proposal/omega_proposal * (-cos(omega_proposal + current_state[2])
                                                                + cos(current_state[2])),
                                   omega_proposal]]).T
            proposal_state = current_state + delta_pos

            proposal_x = round(proposal_state.item(0))
            proposal_y = round(proposal_state.item(1))

            proposal_state_is_within_map = (0 <= proposal_y < ground_truth_map.shape[0] and
                                            0 <= proposal_x < ground_truth_map.shape[1])
            proposal_state_is_valid = (proposal_state_is_within_map and
                                       0 == ground_truth_map.item(proposal_y, proposal_x))

        v = v_proposal
        omega = omega_proposal

        proposal_state[2] = normalize_angle_pi_minus_pi(proposal_state[2])

        ground_truth_states.append(proposal_state)
        current_state = proposal_state

    return ground_truth_states
