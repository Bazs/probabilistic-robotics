from slam_utils.ctrv_motion_model import calculate_odometry_from_controls, calculate_jacobian_from_controls

import numpy as np


def ensure_square_matrix_size(matrix, size):
    if size > matrix.shape[0]:
        matrix = np.concatenate((matrix, np.zeros((matrix.shape[0], size - matrix.shape[1]))), axis=1)
        matrix = np.concatenate((matrix, np.zeros((size - matrix.shape[0], matrix.shape[1]))), axis=0)

    return matrix


def ensure_column_vector_length(vector, length):
    if length > vector.shape[0]:
        vector = np.concatenate((vector, np.zeros((length - vector.shape[0], 1))))

    return vector


def linearize_controls(xi, omega, R, controls, state_estimates):
    for index, control in enumerate(controls[1:]):
        v = control.item(0)
        om = control.item(1)
        previous_state = state_estimates[index]
        current_state_estimate = state_estimates[index] + calculate_odometry_from_controls(v, om, previous_state)

        # Calculate the 6-by-6 update for the information matrix omega
        Gt = np.identity(3)
        Gt[:, -1] = np.ravel(calculate_jacobian_from_controls(v, om, previous_state))
        Gt_exp_dot_R_inv = np.dot(np.concatenate((-Gt.T, np.identity(3))), np.linalg.inv(R))
        omega_xt_xt_1 = np.dot(Gt_exp_dot_R_inv, np.concatenate((-Gt, np.identity(3)), axis=1))
        xi_xt_xt_1 = np.dot(Gt_exp_dot_R_inv, current_state_estimate - np.dot(Gt, previous_state))

        start_index = index * 3
        end_index = index * 3 + 6

        omega = ensure_square_matrix_size(omega, end_index)
        omega[start_index:end_index, start_index:end_index] += omega_xt_xt_1

        xi = ensure_column_vector_length(xi, end_index)
        xi[start_index:end_index] += xi_xt_xt_1

    return xi, omega


def linearize_measurements(xi, omega, H, measurements, state_estimates):
    return


def graph_slam_linearize(controls, measurements, state_estimates, motion_error_covariance,
                         measurement_error_covariance):
    # The initial state is regarded as extremely reliable, i.e. high values in the information matrix omega
    omega = np.identity(3, dtype="float") * 100000
    xi = np.zeros((1, 1))

    # The motion noise covariance
    R = motion_error_covariance
    H = measurement_error_covariance

    xi, omega = linearize_controls(xi, omega, R, controls, state_estimates)
    linearize_measurements(xi, omega, H, measurements, state_estimates)
