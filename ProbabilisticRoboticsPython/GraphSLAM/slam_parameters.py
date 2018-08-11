import math

# Map parameters
MAP_WIDTH = 50
MAP_HEIGHT = 50
LANDMARK_COUNT = 20

# Path parameters
MAX_VELOCITY = 1
VELOCITY_VARIANCE = math.pow(1.5, 2)
MAX_TURN_RATE = math.pi / 4
TURN_RATE_VARIANCE = math.pow(math.pi / 180 * 6, 2)
STEP_COUNT = 200

# Sensor model parameters
MAX_SENSING_RANGE = 15
SENSING_RANGE_VARIANCE = 13
