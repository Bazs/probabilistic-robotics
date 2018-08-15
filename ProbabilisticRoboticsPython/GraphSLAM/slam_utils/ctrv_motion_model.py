from math import sin, cos

import numpy as np


def calculate_odometry_from_controls(v, omega, previous_state):
    """Calculates a pose delta using a constant turn rate and velocity motion model.
    """
    return np.array([[v / omega * (sin(omega + previous_state[2]) - sin(previous_state[2])),
                    v / omega * (-cos(omega + previous_state[2]) + cos(previous_state[2])),
                    omega]]).T


def calculate_jacobian_from_controls(v, omega, previous_state):
    return np.array([[v / omega * (cos(omega + previous_state[2]) - cos(previous_state[2])),
                     v / omega * (sin(omega + previous_state[2] - sin(previous_state[2]))),
                     1]]).T
