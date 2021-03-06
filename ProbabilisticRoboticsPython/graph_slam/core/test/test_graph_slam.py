from utils.ctrv_motion_model import calculate_odometry_from_controls
from core.initialize import graph_slam_initialize
from core.linearize import initialize_xi_omega, graph_slam_linearize, linearize_controls
from core.reduce import graph_slam_reduce
from core.solve import graph_slam_solve, recover_state_estimates
from core.landmark_correspondence import calculate_correspondence_probability
from utils.measurement_model import add_measurement_to_pose, calculate_landmark_distance, calculate_landmark_heading, \
    add_noise_to_measurements_for_state
from utils.path_generator import add_noise_to_control

import numpy as np

import math
from typing import List
import unittest


class TestGraphSlam(unittest.TestCase):

    def setUp(self):
        self.test_controls = [
            [np.array([[1, 0]]).T] * 5,
            [np.array([[1, 0.1]]).T] * 5
        ]

        self.test_ground_truth_states = []

        for controls in self.test_controls:
            state_estimates = self.calculate_states_from_controls(np.array([[0, 0, 0]]).T, controls)
            self.test_ground_truth_states.append(state_estimates)

        self.R = np.identity(3) * 0.00001
        self.Q = np.identity(3) * 0.00001

    @staticmethod
    def calculate_states_from_controls(initial_state, controls):
        state_estimates = [initial_state]

        for index, control in enumerate(controls[1:]):
            state_estimates.append(state_estimates[index]
                                   + calculate_odometry_from_controls(control.item(0), control.item(1),
                                                                      state_estimates[index]))

        return state_estimates

    @staticmethod
    def solve_from_information(xi, omega):
        sigma = np.linalg.inv(omega)
        mu = np.dot(sigma, xi)

        return mu, sigma

    def assert_right_information_size(self, expected_information_size, xi, omega):
        self.assertEqual((expected_information_size, 1), xi.shape)
        self.assertEqual((expected_information_size, expected_information_size), omega.shape)

    def assert_state_estimates_close(self, estimates_a, estimates_b):
        for estimate_a, estimate_b in zip(estimates_a, estimates_b):
            self.assertTrue(np.allclose(estimate_a, estimate_b))

    def assert_mu_close_to_ground_truth_states(self, mu, ground_truth_states):
        for index, state in enumerate(ground_truth_states):
            mu_base_index = index * 3
            recovered_estimate = np.array([[mu[mu_base_index], mu[mu_base_index + 1], mu[mu_base_index + 2]]]).T
            self.assertTrue(np.allclose(recovered_estimate, state))

    def assert_mu_close_to_ground_truth_landmarks(self, mu, ground_truth_landmarks):
        num_landmarks = len(ground_truth_landmarks)
        for landmark_index, landmark in ground_truth_landmarks.items():
            landmark_start_index = mu.shape[0] + (landmark_index - num_landmarks) * 3
            landmark_end_index = landmark_start_index + 3
            recovered_landmark = mu[landmark_start_index:landmark_end_index]

            self.assertTrue(np.allclose(landmark, recovered_landmark))

    def assert_expected_landmark_estimates(self, ground_truth_landmarks, landmark_estimates):
        for landmark_index, ground_truth_landmark in ground_truth_landmarks.items():
            self.assertTrue(np.allclose(ground_truth_landmark, landmark_estimates[landmark_index]))

    @staticmethod
    def generate_unique_landmark_measurements(state_estimates):
        measurements = []
        correspondences = []

        expected_landmarks = dict()

        landmark_index = 0

        for index, state_estimate in enumerate(state_estimates):
            measurements_for_state = [np.array([[1, math.pi / 2, 0]]).T,
                                      np.array([[1, -math.pi / 2, 0]]).T]
            measurements.append(measurements_for_state)

            correspondences_for_state = []

            for measurement in measurements_for_state:
                correspondences_for_state.append(landmark_index)

                x, y = add_measurement_to_pose(state_estimate, measurement)
                expected_landmark = np.array([[x, y, measurement.item(2)]]).T
                expected_landmarks[landmark_index] = expected_landmark

                landmark_index = landmark_index + 1

            correspondences.append(correspondences_for_state)

        return measurements, correspondences, expected_landmarks

    @staticmethod
    def generate_measurements_of_same_landmarks(state_estimates: List[np.ndarray]) -> \
            (List[List[np.ndarray]], List[np.ndarray]):
        measurements = []

        landmarks = [np.array([[10, 0, 0]]).T,
                     np.array([[10, math.pi / 2, 0]]).T]

        for state_index, state_estimate in enumerate(state_estimates):
            measurements_for_state = []

            for landmark in landmarks:
                distance = calculate_landmark_distance(state_estimate, landmark)
                heading = calculate_landmark_heading(state_estimate, landmark)
                measurements_for_state.append(np.array([[distance, heading, 0]]).T)

            measurements.append(measurements_for_state)

        return measurements, landmarks

    @staticmethod
    def generate_corresponding_measurements_of_same_landmarks(state_estimates):
        measurements, landmarks = TestGraphSlam.generate_measurements_of_same_landmarks(state_estimates)

        correspondences = [[index for index in range(len(landmarks))] for _ in measurements]
        expected_landmarks = {index: landmark for index, landmark in enumerate(landmarks)}

        return measurements, correspondences, expected_landmarks

    @staticmethod
    def generate_non_corresponding_measurements_of_same_landmarks(state_estimates):
        measurements, landmarks = TestGraphSlam.generate_measurements_of_same_landmarks(state_estimates)

        correspondences = [[state_index * len(landmarks) + landmark_index for landmark_index in range(len(landmarks))]
                           for state_index in range(len(measurements))]
        expected_landmarks = {state_index * len(landmarks) + landmark_index: landmark for landmark_index, landmark in
                              enumerate(landmarks) for state_index in range(len(state_estimates))}

        return measurements, correspondences, expected_landmarks

    @staticmethod
    def calculate_rms_state_error(output_state_estimates, ground_truth_state_estimates):
        state_errors = np.zeros((len(output_state_estimates)))

        for state_index, ground_truth_state in enumerate(ground_truth_state_estimates):
            output_state = output_state_estimates[state_index]
            state_errors[state_index] = np.linalg.norm(output_state - ground_truth_state)

        rms_state_error = np.sqrt((state_errors ** 2).mean())
        return rms_state_error

    def test_initialize(self):
        for test_index, controls in enumerate(self.test_controls):
            state_t0 = np.array([[0, 0, 0]]).T
            initialized_states = graph_slam_initialize(controls, state_t0)

            expected_state_estimates = self.test_ground_truth_states[test_index]

            for index, expected_state_estimate in enumerate(expected_state_estimates):
                self.assertTrue(np.allclose(expected_state_estimate, initialized_states[index]))

    def test_linearize_controls_round_trip(self):
        """
        Linearizes controls and accumulates them into the information representation. Tests that the mean states
        recovered from the information representation match the expected states, created by just applying the controls
        in a noise-free world.
        """
        for test_index, controls in enumerate(self.test_controls):
            xi, omega = initialize_xi_omega()

            # Perfect estimates, according to controls
            ground_truth_states = self.test_ground_truth_states[test_index]

            # Calculate the information matrix and vector from linearized controls
            xi, omega = linearize_controls(xi, omega, self.R, ground_truth_states, controls)

            expected_information_size = len(ground_truth_states) * 3
            self.assert_right_information_size(expected_information_size, xi, omega)

            # Recover the moments representation of belief, i.e. the covariance and mean
            mu, sigma = self.solve_from_information(xi, omega)

            # Assert that the recovered mean is indistinguishable from the input state estimates
            self.assert_mu_close_to_ground_truth_states(mu, ground_truth_states)

    def test_linearize_no_correspondence_round_trip(self):
        """
        Performs GraphSLAM linearize on measurements and controls, then recovers the state means, and checks that they
        are according to the expected values calculated from the controls in a noise-free world. Each input measurement
        corresponds to a different landmark.
        """
        for test_index, controls in enumerate(self.test_controls):
            ground_truth_states = self.test_ground_truth_states[test_index]

            measurements, correspondences, ground_truth_landmarks = \
                self.generate_unique_landmark_measurements(ground_truth_states)

            landmark_estimates = dict()

            xi, omega, landmark_estimates = graph_slam_linearize(ground_truth_states, landmark_estimates, controls,
                                                                 measurements, correspondences, self.R, self.Q)

            num_landmarks = len(ground_truth_landmarks)
            expected_information_size = (len(ground_truth_states) + num_landmarks) * 3
            self.assert_right_information_size(expected_information_size, xi, omega)
            self.assertEqual(num_landmarks, len(landmark_estimates))

            mu, sigma = self.solve_from_information(xi, omega)

            self.assert_mu_close_to_ground_truth_states(mu, ground_truth_states)
            self.assert_mu_close_to_ground_truth_landmarks(mu, ground_truth_landmarks)

    def test_linearize_with_correspondence_round_trip(self):
        """
        The test execution follows the description of
        "meth:`core.test.TestGraphSlam.test_linearize_no_correspondence_round_trip`,
        with the exception, that now every input measurement corresponds to the same landmark.
        """
        for test_index, controls in enumerate(self.test_controls):
            ground_truth_states = self.test_ground_truth_states[test_index]

            measurements, correspondences, expected_landmarks \
                = self.generate_corresponding_measurements_of_same_landmarks(ground_truth_states)

            landmark_estimates = dict()

            xi, omega, landmark_estimates = graph_slam_linearize(ground_truth_states, landmark_estimates, controls,
                                                                 measurements, correspondences, self.R, self.Q)

            num_landmarks = len(expected_landmarks)
            expected_information_size = (len(ground_truth_states) + num_landmarks) * 3
            self.assert_right_information_size(expected_information_size, xi, omega)
            self.assertEqual(num_landmarks, len(landmark_estimates))

            mu, sigma = self.solve_from_information(xi, omega)

            self.assert_mu_close_to_ground_truth_states(mu, ground_truth_states)

    def test_linearize_reduce_no_correspondence_round_trip(self):
        """
        Performs GraphSLAM linearize on measurements and controls, then reduces the dimensionality of the information
        representation via GraphSLAM reduce. Finally, recovers the state means, and checks that they are according to
        the expected values calculated from the controls in a noise-free world. Each input measurement corresponds to
        a different landmark.
        """
        for test_index, controls in enumerate(self.test_controls):
            ground_truth_states = self.test_ground_truth_states[test_index]

            measurements, correspondences, ground_truth_landmarks = \
                self.generate_unique_landmark_measurements(ground_truth_states)

            landmark_estimates = dict()

            xi, omega, landmark_estimates = graph_slam_linearize(ground_truth_states, landmark_estimates, controls,
                                                                 measurements, correspondences, self.R, self.Q)
            xi_reduced, omega_reduced = graph_slam_reduce(xi, omega, landmark_estimates)

            expected_information_size = len(ground_truth_states) * 3
            self.assert_right_information_size(expected_information_size, xi_reduced, omega_reduced)

            mu, sigma = self.solve_from_information(xi_reduced, omega_reduced)

            self.assert_mu_close_to_ground_truth_states(mu, ground_truth_states)

    def test_linearize_reduce_with_correspondence_round_trip(self):
        """
        The test execution follows the description of
        "meth:`core.test.TestGraphSlam.test_linearize_reduce_no_correspondence_round_trip`,
        with the exception, that now every input measurement corresponds to the same landmark.
        """
        for test_index, controls in enumerate(self.test_controls):
            ground_truth_states = self.test_ground_truth_states[test_index]

            measurements, correspondences, expected_landmarks \
                = self.generate_corresponding_measurements_of_same_landmarks(ground_truth_states)

            landmark_estimates = dict()

            xi, omega, landmark_estimates = graph_slam_linearize(ground_truth_states, landmark_estimates, controls,
                                                                 measurements, correspondences, self.R, self.Q)
            xi_reduced, omega_reduced = graph_slam_reduce(xi, omega, landmark_estimates)

            expected_information_size = len(ground_truth_states) * 3
            self.assert_right_information_size(expected_information_size, xi_reduced, omega_reduced)

            mu, sigma = self.solve_from_information(xi_reduced, omega_reduced)

            self.assert_mu_close_to_ground_truth_states(mu, ground_truth_states)

    def test_graph_slam_full_no_correspondence_round_trip(self):
        """
        Performs GraphSLAM a round of linearize, reduce and solve, and checks that the estimated states and landmarks
        are correct. Each input measurement corresponds to a different landmark.
        """
        for test_index, controls in enumerate(self.test_controls):
            ground_truth_states = self.test_ground_truth_states[test_index]

            measurements, correspondences, ground_truth_landmarks = \
                self.generate_unique_landmark_measurements(ground_truth_states)

            landmark_estimates = dict()

            xi, omega, landmark_estimates = graph_slam_linearize(ground_truth_states, landmark_estimates, controls,
                                                                 measurements, correspondences, self.R, self.Q)
            xi_reduced, omega_reduced = graph_slam_reduce(xi, omega, landmark_estimates)
            output_state_estimates, sigma, landmark_estimates = graph_slam_solve(xi_reduced, omega_reduced, xi, omega)

            self.assert_state_estimates_close(output_state_estimates, ground_truth_states)
            self.assert_expected_landmark_estimates(ground_truth_landmarks, landmark_estimates)

    def test_graph_slam_full_with_correspondence_round_trip(self):
        """
        The test execution follows the description of
        "meth:`core.test.TestGraphSlam.test_graph_slam_full_no_correspondence_round_trip`,
        with the exception, that now every input measurement corresponds to the same landmark.
        """
        for test_index, controls in enumerate(self.test_controls):
            ground_truth_states = self.test_ground_truth_states[test_index]

            measurements, correspondences, ground_truth_landmarks \
                = self.generate_corresponding_measurements_of_same_landmarks(ground_truth_states)

            landmark_estimates = dict()

            xi, omega, landmark_estimates = graph_slam_linearize(ground_truth_states, landmark_estimates, controls,
                                                                 measurements, correspondences, self.R, self.Q)
            xi_reduced, omega_reduced = graph_slam_reduce(xi, omega, landmark_estimates)
            output_state_estimates, sigma, landmark_estimates = graph_slam_solve(xi_reduced, omega_reduced, xi, omega)

            self.assert_state_estimates_close(output_state_estimates, ground_truth_states)
            self.assert_expected_landmark_estimates(ground_truth_landmarks, landmark_estimates)

    def test_graph_slam_full_with_correspondence_improvement(self):
        """
        Tests that from noisy controls and measurements, the full algorithm reduces the RMS error of state estimates
        over multiple iterations, if the correspondences are correctly given.
        """
        for test_index, controls in enumerate(self.test_controls):
            ground_truth_states = self.test_ground_truth_states[test_index]

            measurements, correspondences, ground_truth_landmarks \
                = self.generate_corresponding_measurements_of_same_landmarks(ground_truth_states)

            noisy_controls = [add_noise_to_control(control, 0.4, math.pi * 5 / 180) for control in controls]
            noisy_measurements = [add_noise_to_measurements_for_state(measurements_for_state, 0.01, math.pi * 0.1 / 180)
                                  for measurements_for_state in measurements]
            landmark_estimates = dict()

            output_state_estimates = graph_slam_initialize(noisy_controls, ground_truth_states[0])
            initial_rms_state_error = self.calculate_rms_state_error(output_state_estimates, ground_truth_states)

            for iteration_index in range(100):
                xi, omega, landmark_estimates = graph_slam_linearize(output_state_estimates, landmark_estimates,
                                                                     noisy_controls, noisy_measurements,
                                                                     correspondences, self.R, self.Q)
                xi_reduced, omega_reduced = graph_slam_reduce(xi, omega, landmark_estimates)
                output_state_estimates, sigma, landmark_estimates = graph_slam_solve(xi_reduced, omega_reduced, xi,
                                                                                     omega)

            rms_state_error = self.calculate_rms_state_error(output_state_estimates, ground_truth_states)

            # Assert that incorporating associated measurements will always improve upon the odometry-only estimate
            self.assertLess(rms_state_error, initial_rms_state_error)

    def test_correspondence_probability_with_correspondence_round_trip(self):
        """
        In this test, perfect measurements of the same landmark from different poses are fed into GraphSLAM, all
        measurements with unique correspondence values. Then, the correspondence probabilities are calculated, and
        verified.
        """
        for test_index, controls in enumerate(self.test_controls):
            ground_truth_states = self.test_ground_truth_states[test_index]

            measurements, correspondences, ground_truth_landmarks \
                = self.generate_non_corresponding_measurements_of_same_landmarks(ground_truth_states)

            landmark_estimates = dict()

            xi, omega, landmark_estimates = graph_slam_linearize(ground_truth_states, landmark_estimates, controls,
                                                                 measurements, correspondences, self.R, self.Q)
            xi_reduced, omega_reduced = graph_slam_reduce(xi, omega, landmark_estimates)
            output_state_estimates, sigma, landmark_estimates = graph_slam_solve(xi_reduced, omega_reduced, xi, omega)

            for j in range(len(landmark_estimates)):
                for k in range(j + 1, len(landmark_estimates)):
                    p = calculate_correspondence_probability(omega, sigma, landmark_estimates, j, k)
                    pass
            # TODO check for all pairs of landmarks, that p is ~1.


if __name__ == "__main__":
    unittest.main()
