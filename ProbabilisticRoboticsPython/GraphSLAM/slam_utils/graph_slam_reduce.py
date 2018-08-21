import numpy as np


def graph_slam_reduce(xi, omega, landmark_estimates, correspondences):
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

    # TODO exploit the block-diagonality of omega_m_m when inverting
    # landmark_indices = sorted(landmark_estimates.keys(), reverse=True)

    # for landmark_index in landmark_indices:
    #     corresponding_state_indices = []
    #
    #     landmark_start_index = omega.shape[0] - (num_landmarks + landmark_index) * 3
    #     landmark_end_index = landmark_start_index + 3
    #
    #     for state_index, correspondences_for_state in enumerate(correspondences):
    #         if landmark_index in correspondences_for_state:
    #             corresponding_state_indices.append(state_index)
    #             break
    #
    #     for corresponding_state_index in corresponding_state_indices:
    #         state_start_index = corresponding_state_index * 3
    #         state_end_index = state_start_index + 3
    #
    #         omega_reduced_t_j = omega_reduced[state_start_index:state_end_index,
    #                                           landmark_start_index:landmark_end_index]
    #         omega_reduced_j_j_inv = np.linalg.inv(omega_reduced[landmark_start_index:landmark_end_index,
    #                                               landmark_start_index:landmark_end_index])
    #         omega_reduced_tj_dot_jj_inv = np.dot(omega_reduced_t_j, omega_reduced_j_j_inv)
    #
    #         xi_t_j = np.dot(omega_reduced_tj_dot_jj_inv, xi[landmark_start_index:landmark_end_index])
    #         xi_reduced[state_start_index:state_end_index] -= xi_t_j
    #         xi_reduced[landmark_start_index:landmark_end_index] -= xi_t_j
    #
    #         omega_reduced_j_t = \
    #               omega_reduced[landmark_start_index:landmark_end_index, state_end_index:state_end_index]
    #         omega_jj = np.dot(omega_reduced_tj_dot_jj_inv, omega_reduced_j_t)

    return xi_reduced, omega_reduced
