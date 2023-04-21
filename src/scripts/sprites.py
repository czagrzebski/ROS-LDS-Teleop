import pygame as pg
from settings import *
import rospkg
import tf
import numpy as np

class RobotModel(pg.sprite.Sprite):
    def __init__(self, x, y, group):
        super().__init__(group)
        
        # Load image
        ros_pack = rospkg.RosPack()
        self.image = pg.image.load(ros_pack.get_path('cs431-lds') + '/src/scripts/tank.png')
        self.image = pg.transform.scale(self.image, (self.image.get_rect().width / SPRITE_SIZE, self.image.get_rect().height / SPRITE_SIZE))
        
        self.rect = self.image.get_rect()
        self.rect.center = (x, y)
        
        self.x = x
        self.y = y
        
    def update(self):
        # Translate robot to new position in global frame
        self.rect.centerx = self.x
        self.rect.centery = self.y

class LDS(pg.sprite.Sprite):
    def __init__(self, group, leader, deg, r):
        super().__init__(group)
        self.image = pg.surface.Surface((PARTICLE_SIZE, PARTICLE_SIZE))
        self.image.fill((255, 0, 0))
        self.rect = self.image.get_rect()
        self.leader = leader
        
        self.x = 0
        self.y = 0
        
        self.refresh(deg, r)
        
    def refresh(self, deg, r):
        radius = np.array([r, 0, 0]) 
        
        # Get the  rotation matrix with respect to the robot
        rot_z = tf.transformations.rotation_matrix(np.deg2rad(deg), ZAXIS) [:3, :3]
        
        # Get the relative position of the LDS particle with respect to the robot local reference frame
        lds_loc = np.dot(rot_z, radius)
        self.x = -lds_loc[1] * PIX_PER_M + self.leader.rect.centerx
        self.y = -lds_loc[0] * PIX_PER_M + self.leader.rect.centery
        
    def update(self):
        self.rect.centerx = self.x
        self.rect.centery = self.y
        
class Text(pg.sprite.Sprite): 
    def __init__(self, text):
        super().__init__()
        font = pg.font.SysFont('Arial', 30)
        self.image = font.render(text, False, YELLOW)
        self.rect = self.image.get_rect()
