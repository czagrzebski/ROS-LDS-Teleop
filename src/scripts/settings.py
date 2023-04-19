import math

# Sprite Settings
SPRITE_SIZE = 20
PIX_PER_M = 100
PARTICLE_SIZE = 1.5

# Screen Configurations
SCREEN_WIDTH = 600
SCREEN_HEIGHT = 500

# ROS Configurations
ROS_RATE = 10
FORWARD_SPEED = 0.2 # m/s
ANGULAR_SPEED = math.pi/6 # rad/s
DT = 1.0/ROS_RATE # s

# Color Definitions
YELLOW = (255, 255, 0)
BLACK = (0, 0, 0)

# Rotation Matrix Definition
XAXIS = (1, 0, 0)
YAXIS = (0, 1, 0)
ZAXIS = (0, 0, 1)

# Collision Warning Sensitivity 
COLLISION_DISTANCE = 0.2 # m