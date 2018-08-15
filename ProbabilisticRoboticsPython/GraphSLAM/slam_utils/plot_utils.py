import matplotlib.pyplot as plt

import math


def plot_path(path_states, color):
    path_x = []
    path_y = []
    for state in path_states:
        path_x.append(state[0])
        path_y.append(state[1])

    return plt.plot(path_x, path_y, marker='o', c=color)


def plot_measurements_for_state(state, measurements):
    x_measurements = []
    y_measurements = []

    for measurement in measurements:
        x_measurements.append(state[0] + math.cos(measurement[1]) * measurement[0])
        y_measurements.append(state[1] + math.sin(measurement[1]) * measurement[0])

    state_scatter = plt.scatter(state[0], state[1], s=100, c='green')
    measurements_scatter = plt.scatter(x_measurements, y_measurements, c="red")

    return state_scatter, measurements_scatter
