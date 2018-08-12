import math

# Map parameters
MAP_WIDTH = 50
MAP_HEIGHT = 50
LANDMARK_COUNT = 20

# Path parameters
MAX_VELOCITY = 1
VELOCITY_DEVIATION = math.pow(1.5, 2)
MAX_TURN_RATE = math.pi / 4
TURN_RATE_DEVIATION = math.pow(math.pi / 180 * 6, 2)
STEP_COUNT = 200

# Sensor model parameters
MAX_SENSING_RANGE = 15
SENSING_RANGE_DEVIATION = 13
DISTANCE_DEVIATION = 1
HEADING_DEVIATION = 1.5 * math.pi / 180.0
