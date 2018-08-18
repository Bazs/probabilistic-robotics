from slam_utils.graph_slam_linearize import linearize_controls

from slam_utils.ctrv_motion_model import calculate_odometry_from_controls

import numpy as np

import unittest


class TestGraphSlamLinearize(unittest.TestCase):

    def test_linearize_controls_round_trip(self):

        # Test a straight movement, and circular movement
        test_controls = [
            [np.array([[1, 0]]).T] * 5,
            [np.array([[1, 0.1]]).T] * 5
        ]

        for controls in test_controls:
            # Initial state is extremely reliable, i.e. high values in the information matrix
            omega = np.identity(3, dtype="float") * 100000
            xi = np.zeros((1, 1))

            # Perfect estimates, according to controls
            state_estimates = [np.array([[0, 0, 0]]).T]

            for index, control in enumerate(controls[1:]):
                state_estimates.append(state_estimates[index]
                                       + calculate_odometry_from_controls(control.item(0), control.item(1),
                                                                          state_estimates[index]))

            # Negligible motion error covariance
            R = np.identity(3) * 0.00001

            # Calculate the information matrix and vector from linearized controls
            xi, omega = linearize_controls(xi, omega, R, state_estimates, controls)

            # Recover the moments representation of belief, i.e. the covariance and mean
            sigma = np.linalg.inv(omega)
            mu = np.dot(sigma, xi)

            # Assert that the recovered mean is indistinguishable from the input state estimates
            for index, state_estimate in enumerate(state_estimates):
                mu_base_index = index * 3
                recovered_estimate = np.array([[mu[mu_base_index], mu[mu_base_index + 1], mu[mu_base_index + 2]]]).T
                self.assertTrue(np.allclose(recovered_estimate, state_estimate))


if __name__ == "__main__":
    unittest.main()
