import numpy as np


def graph_slam_reduce(xi, omega, landmark_estimates):
    num_landmarks = len(landmark_estimates)
    state_information_size = xi.shape[0] - 3 * num_landmarks

    omega_x_x = omega[:state_information_size, :state_information_size]
    omega_x_m = omega[:state_information_size, state_information_size:]
    omega_m_m_inv = np.linalg.inv(omega[state_information_size:, state_information_size:])
    omega_m_x = omega[state_information_size:, :state_information_size]

    omega_xm_dot_mm_inv = np.dot(omega_x_m, omega_m_m_inv)
    omega_reduced = omega_x_x - np.dot(omega_xm_dot_mm_inv, omega_m_x)

    xi_x = xi[:state_information_size]
    xi_m = xi[state_information_size:]

    xi_reduced = xi_x - np.dot(omega_xm_dot_mm_inv, xi_m)

    # TODO fix implementation below
    # omega_reduced = omega[:state_information_size, :state_information_size]
    # xi_reduced = xi[:state_information_size]
    # landmark_indices = sorted(landmark_estimates.keys(), reverse=True)
    #
    # for landmark_index in landmark_indices:
    #     landmark_start_index = omega.shape[0] - (num_landmarks + landmark_index) * 3
    #     landmark_end_index = landmark_start_index + 3
    #
    #     omega_j_j_inv = np.linalg.inv(omega[landmark_start_index:landmark_end_index,
    #                                   landmark_start_index:landmark_end_index])
    #
    #     omega_t_j = omega[:state_information_size, landmark_start_index:landmark_end_index]
    #
    #     omega_t_j_dot_j_j_inv = np.dot(omega_t_j, omega_j_j_inv)
    #
    #     omega_j_t = omega[landmark_start_index:landmark_end_index, :state_information_size]
    #     omega_t_t_reduction = np.dot(omega_t_j_dot_j_j_inv, omega_j_t)
    #
    #     omega_reduced -= omega_t_t_reduction
    #
    #     xi_t_reduction = np.dot(omega_t_j_dot_j_j_inv, xi[landmark_start_index:landmark_end_index])
    #     xi_reduced -= xi_t_reduction

    return xi_reduced, omega_reduced
