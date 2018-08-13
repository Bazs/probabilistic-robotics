from slam_utils.ctrv_motion_model import calculate_odometry_from_controls

import numpy as np


def graph_slam_initialize(controls):
    """Initializes the mean pose vector by forward-propagating the controls using the motion model. The initial state
    is at [0, 0, 0].
    """
    state_t0 = np.array([[0, 0, 0]]).T

    initialized_states = [state_t0]

    for index, control in enumerate(controls[1:]):
        initialized_states.append(initialized_states[index - 1] +
                                  calculate_odometry_from_controls(control[0], control[1],
                                                                   initialized_states[index - 1]))

    return initialized_states
