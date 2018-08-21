from slam_utils.ctrv_motion_model import calculate_odometry_from_controls
from slam_utils.graph_slam_linearize import initialize_xi_omega, graph_slam_linearize, linearize_controls
from slam_utils.graph_slam_reduce import graph_slam_reduce

import numpy as np

import math
import unittest


class TestGraphSlamLinearizeReduce(unittest.TestCase):

    @staticmethod
    def calculate_states_from_controls(initial_state, controls):
        state_estimates = [initial_state]

        for index, control in enumerate(controls[1:]):
            state_estimates.append(state_estimates[index]
                                   + calculate_odometry_from_controls(control.item(0), control.item(1),
                                                                      state_estimates[index]))

        return state_estimates

    @staticmethod
    def solve_from_reduced_information(xi, omega):
        sigma = np.linalg.inv(omega)
        mu = np.dot(sigma, xi)

        return mu, sigma

    def assert_right_information_size(self, expected_information_size, xi, omega):
        self.assertEqual((expected_information_size, 1), xi.shape)
        self.assertEqual((expected_information_size, expected_information_size), omega.shape)

    def assert_mu_close_to_ground_truth_states(self, mu, ground_truth_states):
        for index, state in enumerate(ground_truth_states):
            mu_base_index = index * 3
            recovered_estimate = np.array([[mu[mu_base_index], mu[mu_base_index + 1], mu[mu_base_index + 2]]]).T
            self.assertTrue(np.allclose(recovered_estimate, state))

    def test_linearize_controls_round_trip(self):
        """
        Linearizes controls and accumulates them into the information representation. Tests that the mean states
        recovered from the information representation match the expected states, created by just applying the controls
        in a noise-free world.
        """
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
            mu, sigma = self.solve_from_reduced_information(xi, omega)

            # Assert that the recovered mean is indistinguishable from the input state estimates
            self.assert_mu_close_to_ground_truth_states(mu, state_estimates)

    def test_linearize_reduce_no_correspondence_round_trip(self):
        """
        Performs GraphSLAM linearize on measurements and controls, then reduces the dimensionality of the information
        representation via GraphSLAM reduce. Finally, recovers the state means, and checks that they are according to
        the expected values calculated from the controls in a noise-free world. Each input measurement corresponds to
        a different landmark.
        """
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

        landmark_estimates = dict()

        xi, omega, landmark_estimates = graph_slam_linearize(state_estimates, landmark_estimates, controls,
                                                             measurements, correspondences, R, Q)

        xi_reduced, omega_reduced = graph_slam_reduce(xi, omega, landmark_estimates, correspondences)

        num_landmarks = len(state_estimates)
        expected_information_size = (len(state_estimates) + num_landmarks) * 3
        self.assert_right_information_size(expected_information_size, xi, omega)

        self.assertEqual(num_landmarks, len(landmark_estimates))

        mu, sigma = self.solve_from_reduced_information(xi_reduced, omega_reduced)

        # Assert that the recovered mean is indistinguishable from the input state estimates
        self.assert_mu_close_to_ground_truth_states(mu, state_estimates)

    def test_linearize_reduce_with_correspondence_round_trip(self):
        """
        The test execution follows the description of
        "meth:`slam_utils.test.TestGraphSlamLinearizeReduce.test_linearize_reduce_no_correspondence_round_trip`,
        with the exception, that now every input measurement corresponds to the same landmark.
        """
        controls = [np.array([[1, 0]]).T] * 4
        state_estimates = self.calculate_states_from_controls(np.array([[0, 0, 0]]).T, controls)

        measurements = []
        correspondences = []

        measurement_distance = 10

        for index in range(len(controls)):
            measurements.append([np.array([[measurement_distance, 0, 0]]).T])
            correspondences.append([0])
            measurement_distance = measurement_distance - controls[index].item(0)

        # Negligible motion error covariance
        R = np.identity(3) * 0.00001
        # Negligible measurement noise covariance
        Q = np.identity(3) * 0.00001

        landmark_estimates = dict()

        xi, omega, landmark_estimates = graph_slam_linearize(state_estimates, landmark_estimates, controls,
                                                             measurements, correspondences, R, Q)
        xi_reduced, omega_reduced = graph_slam_reduce(xi, omega, landmark_estimates, correspondences)

        num_landmarks = 1
        expected_information_size = (len(state_estimates) + num_landmarks) * 3
        self.assert_right_information_size(expected_information_size, xi, omega)

        self.assertEqual(num_landmarks, len(landmark_estimates))

        mu, sigma = self.solve_from_reduced_information(xi_reduced, omega_reduced)

        self.assert_mu_close_to_ground_truth_states(mu, state_estimates)


if __name__ == "__main__":
    unittest.main()
