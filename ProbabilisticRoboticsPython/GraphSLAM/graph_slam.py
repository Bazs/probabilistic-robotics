from slam_utils.map import generate_ground_truth_map
from slam_utils.path_generator import generate_ground_truth_path

import matplotlib.pyplot as plt
import numpy as np

import math
import random as rnd

MAP_WIDTH = 50
MAP_HEIGHT = 50
OBSTACLE_COUNT = 20

# TODO rename obstacles to landmarks
ground_truth_map, obstacles = generate_ground_truth_map(MAP_HEIGHT, MAP_WIDTH, OBSTACLE_COUNT)

# Set up truly random number generation for creating the ground truth path (if the system supports it)
true_random_gen = rnd.SystemRandom()
rnd.seed(true_random_gen.random())

max_velocity = 1
velocity_variance = math.pow(1.5, 2)
max_turn_rate = math.pi / 4
turn_rate_variance = math.pow(math.pi / 180 * 6, 2)
step_count = 200

ground_truth_states = generate_ground_truth_path(ground_truth_map, max_velocity=max_velocity,
                                                 velocity_variance=velocity_variance, max_turn_rate=max_turn_rate,
                                                 turn_rate_variance=turn_rate_variance, step_count=step_count)

MAX_SENSING_RANGE = 15
SENSING_RANGE_VARIANCE = 13

for ground_truth_state in ground_truth_states:
    # Select obstacles and their distances to current state, if the distance  is lower than the threshold
    obstacle_distances = [np.linalg.norm(obstacle[:2] - ground_truth_state[:2]) for obstacle in obstacles]
    obstacles_in_range, in_range_distances = zip(*[(obstacle, obstacle_distances[index]) for index, obstacle in
                                                   enumerate(obstacles) if obstacle_distances[index]
                                                   <= MAX_SENSING_RANGE])

    # Sample obstacles from the in-range ones, using the absolute values of samples from a zero-mean normal
    # distribution as distance thresholds
    detected_obstacles, detected_distances = zip(*[(obstacle, in_range_distances[index]) for index, obstacle in
                                                   enumerate(obstacles_in_range) if
                                                   abs(rnd.normalvariate(0, SENSING_RANGE_VARIANCE))
                                                   >= in_range_distances[index]])

    # TODO calculate measurement vector (r, phi, s) for detected obstacles

plt.plot()
plt.title("Ground truth map")
plt.imshow(ground_truth_map)

path_x = []
path_y = []
for state in ground_truth_states:
    path_x.append(state[0])
    path_y.append(state[1])

plt.plot(path_x, path_y, marker='o')

plt.show()
