import numpy as np


def graph_slam_solve(xi_reduced, omega_reduced, xi, omega):
    sigma = np.linalg.inv(omega)
    mu = np.dot(sigma, xi)

    state_information_size = xi_reduced.shape[0]

    return mu, sigma[:state_information_size, :state_information_size]
