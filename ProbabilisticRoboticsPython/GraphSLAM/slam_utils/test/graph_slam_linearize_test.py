from slam_utils.graph_slam_linearize import initialize_xi_omega, linearize_controls, linearize_measurements

from slam_utils.ctrv_motion_model import calculate_odometry_from_controls

import numpy as np

import math
import unittest


class TestGraphSlamLinearize(unittest.TestCase):

    @staticmethod
    def calculate_states_from_controls(initial_state, controls):
        state_estimates = [initial_state]

        for index, control in enumerate(controls[1:]):
            state_estimates.append(state_estimates[index]
                                   + calculate_odometry_from_controls(control.item(0), control.item(1),
                                                                      state_estimates[index]))

        return state_estimates

    @staticmethod
    def solve_from_full_information(xi, omega):
        sigma = np.linalg.inv(omega)
        mu = np.dot(sigma, xi)

        return mu, sigma

    def assert_right_information_size(self, expected_information_size, xi, omega):
        self.assertEqual((expected_information_size, 1), xi.shape)
        self.assertEqual((expected_information_size, expected_information_size), omega.shape)

    def test_linearize_controls_round_trip(self):
        # Test a straight movement, and circular movement
        test_controls = [
            [np.array([[1, 0]]).T] * 5,
            [np.array([[1, 0.1]]).T] * 5
        ]

        for controls in test_controls:
            xi, omega = initialize_xi_omega()

            # Perfect estimates, according to controls
            state_estimates = self.calculate_states_from_controls(np.array([[0, 0, 0]]).T, controls)

            # Negligible motion error covariance
            R = np.identity(3) * 0.00001

            # Calculate the information matrix and vector from linearized controls
            xi, omega = linearize_controls(xi, omega, R, state_estimates, controls)

            expected_information_size = len(state_estimates) * 3
            self.assert_right_information_size(expected_information_size, xi, omega)

            # Recover the moments representation of belief, i.e. the covariance and mean
            mu, sigma = self.solve_from_full_information(xi, omega)

            # Assert that the recovered mean is indistinguishable from the input state estimates
            for index, state_estimate in enumerate(state_estimates):
                mu_base_index = index * 3
                recovered_estimate = np.array([[mu[mu_base_index], mu[mu_base_index + 1], mu[mu_base_index + 2]]]).T
                self.assertTrue(np.allclose(recovered_estimate, state_estimate))

    def test_linearize_measurements_no_correspondence_round_trip(self):
        controls = [np.array([[1, 0]]).T] * 4
        state_estimates = self.calculate_states_from_controls(np.array([[0, 0, 0]]).T, controls)

        measurements = []
        correspondences = []

        for index in range(len(controls)):
            measurements.append([np.array([[1, math.pi / 2, 0]]).T])
            correspondences.append([index])

        # Negligible motion error covariance
        R = np.identity(3) * 0.00001
        # Negligible measurement noise covariance
        Q = np.identity(3) * 0.00001

        xi, omega = initialize_xi_omega()

        landmark_estimates = dict()

        xi, omega = linearize_controls(xi, omega, R, state_estimates, controls)
        xi, omega, landmark_estimates = linearize_measurements(xi, omega, Q, state_estimates, landmark_estimates,
                                                               measurements, correspondences)

        num_landmarks = len(state_estimates)
        expected_information_size = (len(state_estimates) + num_landmarks) * 3
        self.assert_right_information_size(expected_information_size, xi, omega)

        self.assertEqual(num_landmarks, len(landmark_estimates))

        # TODO finish test after GraphSLAM_reduce and GraphSLAM_solve have been implemented
        pass

    def test_linearize_measurements_with_correspondence_round_trip(self):
        controls = [np.array([[1, 0]]).T] * 4
        state_estimates = self.calculate_states_from_controls(np.array([[0, 0, 0]]).T, controls)

        measurements = []
        correspondences = []

        measurement_distance = 10

        for index in range(len(controls)):
            measurements.append([np.array([[measurement_distance, 0, 0]]).T])
            correspondences.append([0])
            measurement_distance = measurement_distance - controls[index][0]

        # Negligible motion error covariance
        R = np.identity(3) * 0.00001
        # Negligible measurement noise covariance
        Q = np.identity(3) * 0.00001

        xi, omega = initialize_xi_omega()

        landmark_estimates = dict()

        xi, omega = linearize_controls(xi, omega, R, state_estimates, controls)
        xi, omega, landmark_estimates = linearize_measurements(xi, omega, Q, state_estimates, landmark_estimates,
                                                               measurements, correspondences)

        num_landmarks = 1
        expected_information_size = (len(state_estimates) + num_landmarks) * 3
        self.assert_right_information_size(expected_information_size, xi, omega)

        self.assertEqual(num_landmarks, len(landmark_estimates))

        # TODO finish test after GraphSLAM_reduce and GraphSLAM_solve have been implemented
        pass


if __name__ == "__main__":
    unittest.main()
