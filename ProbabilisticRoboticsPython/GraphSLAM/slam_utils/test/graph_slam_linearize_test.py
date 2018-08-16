from slam_utils.graph_slam_linearize import linearize_controls

import numpy as np

import unittest


class TestGraphSlamLinearize(unittest.TestCase):

    def test_linearize_controls_round_trip(self):
        # Initial state is extremely reliable, i.e. high values in the information matrix
        omega = np.identity(3, dtype="float") * 100000
        xi = np.zeros((1, 1))

        # Perfect controls, without noise
        controls = [np.array([[1, 0]]).T] * 5

        # Perfect estimates, according to controls
        state_estimates = [
            np.array([[0, 0, 0]]).T,
            np.array([[1, 0, 0]]).T,
            np.array([[2, 0, 0]]).T,
            np.array([[3, 0, 0]]).T,
            np.array([[4, 0, 0]]).T,
            ]

        # Negligible motion error covariance
        R = np.identity(3) * 0.00001

        # Calculate the information matrix and vector from linearized controls
        xi, omega = linearize_controls(xi, omega, R, controls, state_estimates)

        # Recover the moments representation of belief, i.e. the covariance and mean
        sigma = np.linalg.inv(omega)
        mu = np.dot(sigma, xi)

        # Assert that the recovered mean is indistinguishable from the input state estimates
        for index, state_estimate in enumerate(state_estimates):
            mu_base_index = index * 3
            recovered_estimate = np.array([[mu[mu_base_index], mu[mu_base_index + 1], mu[mu_base_index + 2]]]).T
            self.assertTrue(np.allclose(recovered_estimate, state_estimate))

        return


if __name__ == "__main__":
    unittest.main()
