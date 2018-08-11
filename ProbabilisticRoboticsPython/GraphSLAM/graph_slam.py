from slam_utils.map import generate_ground_truth_map
from slam_utils.measurement_model import generate_measurements
from slam_utils.path_generator import generate_ground_truth_path

import matplotlib.pyplot as plt

import math
import random as rnd

MAP_WIDTH = 50
MAP_HEIGHT = 50
LANDMARK_COUNT = 20

ground_truth_map, landmarks = generate_ground_truth_map(MAP_HEIGHT, MAP_WIDTH, LANDMARK_COUNT)

# Set up truly random number generation for creating the ground truth path (if the system supports it)
true_random_gen = rnd.SystemRandom()
rnd.seed(true_random_gen.random())

MAX_VELOCITY = 1
VELOCITY_VARIANCE = math.pow(1.5, 2)
MAX_TURN_RATE = math.pi / 4
TURN_RATE_VARIANCE = math.pow(math.pi / 180 * 6, 2)
STEP_COUNT = 200

ground_truth_states = generate_ground_truth_path(ground_truth_map, max_velocity=MAX_VELOCITY,
                                                 velocity_variance=VELOCITY_VARIANCE, max_turn_rate=MAX_TURN_RATE,
                                                 turn_rate_variance=TURN_RATE_VARIANCE, step_count=STEP_COUNT)

MAX_SENSING_RANGE = 15
SENSING_RANGE_VARIANCE = 13

measurements = generate_measurements(ground_truth_states, landmarks, max_sensing_range=MAX_SENSING_RANGE,
                                     sensing_range_variance=SENSING_RANGE_VARIANCE)

plt.plot()
plt.title("Ground truth map")
plt.imshow(ground_truth_map, origin='lower')

path_x = []
path_y = []
for state in ground_truth_states:
    path_x.append(state[0])
    path_y.append(state[1])

plt.plot(path_x, path_y, marker='o')

plt.show()
