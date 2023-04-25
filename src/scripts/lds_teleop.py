#!/usr/bin/env python3
"""
Laser Scan Display Teleoperation for Differential Drive Robots
CS431 - Introduction to Robotics

Author: Creed Zagrzebski (zagrzebski1516@uwlax.edu)    
Last Modified: 04/20/2023
    
"""
import rospy
import pygame as pg
import threading
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sprites import LDS, LocalRefFrame, Text
from settings import *
from pygame.locals import * 
import sys
import signal
import numpy as np

class LDSTeleop():
    def __init__(self):
        self.lds_particles = []
        self.all_sprites = pg.sprite.Group()
        self.robot = LocalRefFrame((SCREEN_WIDTH/2), (SCREEN_HEIGHT/2), self.all_sprites)
        self.scan_data_lock = threading.Condition()
        
        # Initialize LDS particles in coordinate system
        for x in range(360):
            self.lds_particles.append(LDS(self.all_sprites, self.robot, x, 0))
            
        # Initialize LDS ranges
        self.ranges = np.zeros(360)
        
        # Initialize ROS
        rospy.init_node('lds_teleop', anonymous=False) 
        self.sub = rospy.Subscriber('/scan', LaserScan, self.ros_lds_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.ros_clock = rospy.Rate(ROS_RATE)
        
        # Initialize Collision Warning
        self.collision = False
        
        # Differential Drive Controller Twist Message
        self.tw = Twist()
        self.tw.linear.x = 0
        self.tw.linear.y = 0  

        # Initialize Pygame
        pg.init()
        pg.display.set_caption("Laser Scan Display Teleoperation")
        self.screen = pg.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.text = Text('Collision Warning') 
        self.running = True

        # Start ROS subscriber thread as daemon to allow for clean shutdown
        self.sub_thread = threading.Thread(target=self.subscriber_thread)
        self.sub_thread.daemon = True
        self.sub_thread.start()
        
        # Start Pygame thread 
        self.pygame_thread = threading.Thread(target=self.run)
        self.pygame_thread.daemon = True
        self.pygame_thread.start()
        
        # Register signal handler for clean shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # Join pygame thread for graceful shutdown. Subscriber thread will terminate with main thread.
        self.pygame_thread.join()
        
    def signal_handler(self, sig, frame):
        sys.exit()
             
    def ros_lds_callback(self, msg):
        with self.scan_data_lock:
            self.ranges = np.array(msg.ranges)
            self.ranges[np.isinf(self.ranges)] = 0
            # Check for potential collisions
            for d in self.ranges:
                if(d <= COLLISION_DISTANCE and d != 0):
                    self.collision = True
                    break 
                else:
                    self.collision = False
            self.scan_data_lock.notify()
    
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
        with self.scan_data_lock:
            self.scan_data_lock.wait()
            for x in range(360):
                self.lds_particles[x].refresh(x, self.ranges[x])    
        self.all_sprites.update()
        self.pub.publish(self.tw)

    def draw(self):
        self.screen.fill(BLACK)
        self.all_sprites.draw(self.screen)
        if self.collision:
            pg.display.set_caption('Collision Warning!')
            self.screen.blit(self.text.image, self.text.rect)
        else:
            pg.display.set_caption('Laser Scan Display Teleoperation')
        pg.display.flip()

if __name__ == '__main__':
    lds_teleop = LDSTeleop()