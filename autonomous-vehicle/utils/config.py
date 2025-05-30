"""
Configuration file for CARLA simulation environment.
All simulation parameters are centralized here for easy modification.
"""

# Connection settings
HOST = "localhost"
PORT = 2000
TIMEOUT = 30.0

# Vehicle settings
CAR_NAME = 'vehicle.mini.cooper'
EPISODE_LENGTH = 120
MAX_TIMESTEPS = 7500

# Traffic settings
NUMBER_OF_VEHICLES = 10
NUMBER_OF_PEDESTRIANS = 30

# Action space settings
CONTINUOUS_ACTION = True

# Display settings
VISUAL_DISPLAY = True

# Camera settings
RGB_CAMERA = 'sensor.camera.rgb'
SEMANTIC_CAMERA = 'sensor.camera.semantic_segmentation'

# Environment settings
IMAGE_WIDTH = 160
IMAGE_HEIGHT = 80
CAMERA_FOV = 125

# Reward parameters
TARGET_SPEED = 22.0
MAX_SPEED = 25.0
MIN_SPEED = 15.0
MAX_DISTANCE_FROM_CENTER = 3.0
COLLISION_REWARD = -100
LANE_DEVIATION_REWARD = -50
SLOW_SPEED_REWARD = -50
SPEED_PENALTY = -1
