from slam_utils.map import generate_ground_truth_map, generate_random_free_coordinate

from slam_utils.state import *
import matplotlib.pyplot as plt

import math
from math import sin, cos
import random as rnd

MAP_WIDTH = 50
MAP_HEIGHT = 50
OBSTACLE_COUNT = 20

ground_truth_map = generate_ground_truth_map(MAP_HEIGHT, MAP_WIDTH, OBSTACLE_COUNT)

true_random_gen = rnd.SystemRandom()

rnd.seed(true_random_gen.random())

start_x, start_y = generate_random_free_coordinate(ground_truth_map)
start_x = start_x + rnd.random() - 1
start_y = start_y + rnd.random() - 1
start_th = math.pi * (2 * rnd.random() - 1)

current_state = np.array([[start_x, start_y, start_th]]).T
ground_truth_states = [current_state]

MAX_VELOCITY = 1
VELOCITY_VARIANCE = math.pow(1.5, 2)
MAX_TURN_RATE = math.pi / 4
TURN_RATE_VARIANCE = math.pow(math.pi / 180 * 15, 2)

# velocity and turn rate
v, omega = 0, 0

ROBOT_STEP_COUNT = 200


def clip(value, minimum, maximum):
    return min(max(value, minimum), maximum)


for step in range(ROBOT_STEP_COUNT):
    proposal_state_is_valid = False

    while proposal_state_is_valid is False:
        v_proposal = v + rnd.normalvariate(0, VELOCITY_VARIANCE)
        v_proposal = clip(v_proposal, 0, MAX_VELOCITY)

        omega_proposal = omega + rnd.normalvariate(0, TURN_RATE_VARIANCE)
        omega_proposal = clip(omega_proposal, -MAX_TURN_RATE, MAX_TURN_RATE)

        # CTRV moditon model
        delta_pos = np.array([[v_proposal/omega_proposal * (sin(omega_proposal + current_state[2])
                                                            - sin(current_state[2])),
                               v_proposal/omega_proposal * (-cos(omega_proposal + current_state[2])
                                                            + cos(current_state[2])),
                               omega_proposal]]).T
        proposal_state = current_state + delta_pos

        proposal_x = round(proposal_state.item(0))
        proposal_y = round(proposal_state.item(1))

        proposal_state_is_within_map = 0 <= proposal_y < MAP_WIDTH and 0 <= proposal_x < MAP_HEIGHT
        # TODO: fix the validity check: obstacles go from x.5 to x.5+1 ! Still doesn't work...
        proposal_state_is_valid = proposal_state_is_within_map and (0 == ground_truth_map[proposal_y, proposal_x])

    v = v_proposal
    omega = omega_proposal
    print(ground_truth_map[proposal_y, proposal_x])

    proposal_theta = proposal_state[2]

    while -math.pi > proposal_theta:
        proposal_theta = proposal_theta + math.pi * 2
    while math.pi < proposal_theta:
        proposal_theta = proposal_theta - math.pi * 2

    proposal_state[2] = proposal_theta

    ground_truth_states.append(proposal_state)
    current_state = proposal_state

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
