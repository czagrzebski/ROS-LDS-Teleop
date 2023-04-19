import rospy
import pygame as pg
import threading
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sprites import LDS, RobotModel, Text
from settings import *
from pygame.locals import * 
import sys
import numpy as np

scan_data_lock = threading.Condition()

class LDSTeleop():
    def __init__(self):
        self.lds_particles = []
        self.all_sprites = pg.sprite.Group()
        self.robot = RobotModel((SCREEN_WIDTH/2), (SCREEN_HEIGHT/2), self.all_sprites)
        for x in range(360):
            self.lds_particles.append(LDS(self.all_sprites, self.robot, x, 0))
        
        # Initialize ROS
        rospy.init_node('lds_teleop', anonymous=False) 
        self.sub = rospy.Subscriber('/scan', LaserScan, self.ros_lds_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.collision = False
        
        # Differential Drive Controller Twist Protocol
        self.tw = Twist()
        self.tw.linear.x = 0
        self.tw.linear.y = 0  
        self.ros_clock = rospy.Rate(ROS_RATE)

        pg.init()
        pg.display.set_caption("Laser Scan Display Teleoperation")
        self.screen = pg.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.text = Text('Collision Warning') 
        self.running = True

        # Start ROS subscriber thread
        self.sub_thread = threading.Thread(target=self.subscriber_thread)
        self.sub_thread.daemon = True
        self.sub_thread.start()
        
    def ros_lds_callback(self, msg):
        with scan_data_lock:
            self.ranges = np.array(msg.ranges)
            self.ranges[np.isinf(self.ranges)] = 0
            for d in self.ranges:
                if(d <= COLLISION_DISTANCE and d != 0):
                    self.collision = True
                    break 
                else:
                    self.collision = False
            scan_data_lock.notify()
    
    def subscriber_thread(self):
        rospy.spin()
    
    def run(self):
        while self.running:
            self.ros_clock.sleep()
            self.events()
            self.update()
            self.draw()
            
    def events(self):
        for event in pg.event.get():
            if event.type == QUIT:
                self.running = False
                pg.quit()
                sys.exit()
            
            if event.type == KEYDOWN:
                if event.key == K_a:
                    self.tw.angular.z = ANGULAR_SPEED
                if event.key == K_d:
                    self.tw.angular.z = -ANGULAR_SPEED
                if event.key == K_w:
                    self.tw.linear.x = FORWARD_SPEED
                if event.key == K_s:
                    self.tw.linear.x = -FORWARD_SPEED
                    
            if event.type == KEYUP:
                if event.key == K_a or event.key == K_d:
                    self.tw.angular.z = 0
                if event.key == K_w or event.key == K_s:
                    self.tw.linear.x = 0
                    
    def update(self):
        with scan_data_lock:
            scan_data_lock.wait()
            for x in range(360):
                self.lds_particles[x].refresh(x, self.ranges[x])    
            if self.collision:
                pg.display.set_caption('Collision Warning!')
                self.all_sprites.add(self.text)
            else:
                pg.display.set_caption('Laser Scan Display Teleoperation')
                self.all_sprites.remove(self.text)
        self.all_sprites.update()
        self.pub.publish(self.tw)

    def draw(self):
        self.screen.fill(BLACK)
        self.all_sprites.draw(self.screen)
        pg.display.flip()

if __name__ == '__main__':
    lds_teleop = LDSTeleop()
    lds_teleop.run()