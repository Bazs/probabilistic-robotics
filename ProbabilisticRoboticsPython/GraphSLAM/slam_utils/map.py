import numpy as np

import random


def generate_random_coordinate(map_matrix):
    return random.randint(0, map_matrix.shape[1]-1), random.randint(0, map_matrix.shape[0]-1)


def generate_obstacles(map_matrix, obstacle_count):
    random.seed(3)
    generated_count = 0

    while generated_count < obstacle_count:

        location_is_free = False
        while location_is_free is False:
            x, y = generate_random_coordinate(map_matrix)
            location_is_free = 0 == map_matrix[y, x]

        map_matrix[y, x] = 1

        generated_count = generated_count + 1


def generate_ground_truth_map(map_height, map_width, obstacle_count):
    """
    Returns an integer numpy matrix of shape (map_height, map_width). Zero values represent free space,
    one values represent obstacles. The borders of the map are always set as obstacles. An
    obstacle_count amount of uniformly randomly sampled point-like obstacles are also generated.
    """
    # The ground truth map is discretized. Zero values mean free space, one values mean obstacles.
    ground_truth_map = np.zeros((map_height, map_width), dtype='int')

    # The edges of the map are untraversable.
    ground_truth_map[0, 0:] = 1
    ground_truth_map[map_height - 1, 0:] = 1
    ground_truth_map[0:, 0] = 1
    ground_truth_map[0:, map_width - 1] = 1

    generate_obstacles(ground_truth_map, obstacle_count)

    return ground_truth_map
