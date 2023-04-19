import math

# Sprite Settings
SPRITE_SIZE = 20
PIX_PER_M = 100
PARTICLE_SIZE = 1

# Screen Configurations
SCREEN_WIDTH = 400
SCREEN_HEIGHT = 300

# ROS Configurations
ROS_RATE = 20
FORWARD_SPEED = 0.2 # m/s
ANGULAR_SPEED = math.pi/6 # rad/s
DT = 1.0/ROS_RATE # s

# Color Definitions
YELLOW = (255, 255, 0)
BLACK = (0, 0, 0)

# Rotation Matrix
ZAXIS = (0, 0, 1)

# Collision Warning Settings 
COLLISION_DISTANCE = 0.2 # m