from slam_utils.map import generate_ground_truth_map

from slam_utils.state import *
import matplotlib.pyplot as plt

MAP_WIDTH = 50
MAP_HEIGHT = 50
OBSTACLE_COUNT = 20

ground_truth_map = generate_ground_truth_map(MAP_HEIGHT, MAP_WIDTH, OBSTACLE_COUNT)

plt.plot()
plt.title("Ground truth map")
plt.imshow(ground_truth_map)
plt.colorbar()
plt.show()

mu0 = State()

print(mu0.array)
