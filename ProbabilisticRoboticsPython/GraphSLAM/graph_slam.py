from slam_utils.map import generate_ground_truth_map, generate_random_free_coordinate
from slam_utils.path_generator import generate_ground_truth_path

import matplotlib.pyplot as plt

import math
import random as rnd

MAP_WIDTH = 50
MAP_HEIGHT = 50
OBSTACLE_COUNT = 20

ground_truth_map = generate_ground_truth_map(MAP_HEIGHT, MAP_WIDTH, OBSTACLE_COUNT)

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

plt.plot()
plt.title("Ground truth map")
plt.imshow(ground_truth_map)

path_x = []
path_y = []
for state in ground_truth_states:
    path_x.append(state[0])
    path_y.append(state[1])

print(path_x)
print(path_y)

plt.plot(path_x, path_y, marker='o')

plt.show()
