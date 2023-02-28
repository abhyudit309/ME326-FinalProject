#!/usr/bin/env python3

import time
import rospy
import tf
import numpy as np
import cv2
import scipy as sp
from scipy import linalg

from occupancy_grid import OccupancyGrid
from path_planner import PathPlanner
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import threading

class DriveController:

    def __init__(self):
        self.mobile_base_vel_publisher = rospy.Publisher("/locobot/mobile_base/commands/velocity", Twist, queue_size=1)

        self.L = 0.2
        self.target = np.zeros(2)
        self.p = np.zeros(2)
        self.thread_lock = threading.Lock()

        self.maxSpeed = 0.3 # m/s
        self.maxRotation = 1 # rad/s

        self.go = False

        rospy.Subscriber("/locobot/mobile_base/odom", Odometry, self.OdometryCallback)
        rospy.Subscriber("position", Float32MultiArray, self.traj_callback)
        
    def traj_callback(self, msg):
        self.target = np.array([msg.data[0], msg.data[1]])     
    
    def set_target(self, target):
        self.go = True
        self.target = target

    def get_P_pos(self):
        self.thread_lock.acquire()
        p = self.p
        self.thread_lock.release()
        return p

    def OdometryCallback(self, data):
        qw = data.pose.pose.orientation.w
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        R11 = qw**2 + qx**2 - qy**2 -qz**2
        R12 = 2*qx*qz - 2*qw*qz
        R21 = 2*qx*qz + 2*qw*qz
        R22 = qw**2 - qx**2 + qy**2 -qz**2

        M = np.matrix([[R11,self.L*R12],[R21,self.L*R22]])

        px = data.pose.pose.position.x + self.L*R11
        py = data.pose.pose.position.y + self.L*R21
        p = np.array([px,py])

        self.thread_lock.acquire()
        self.p = p
        self.thread_lock.release()

        k = 0.5
        u = np.ravel((k*np.linalg.inv(M)) @ (self.target - p).reshape((2,1)))

        v = u[0]
        v = np.clip(v, -self.maxSpeed, self.maxSpeed) # clip speed
        w = u[1]
        w = np.clip(w, -self.maxRotation, self.maxRotation) # clip rotation
        control_msg = Twist()
        control_msg.linear.x = float(v) #forward velocity
        control_msg.angular.z = float(w) #angular velocity

        #now publish the control output:
        if(self.go):
            self.mobile_base_vel_publisher.publish(control_msg)     
            
    def manual(self, v, w):
        self.go = False
        control_msg = Twist()
        control_msg.linear.x = float(v) #forward velocity
        control_msg.angular.z = float(w) #angular velocity
        self.mobile_base_vel_publisher.publish(control_msg)   
