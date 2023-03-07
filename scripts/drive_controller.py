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
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from std_msgs.msg import Float32MultiArray
import threading

class DriveController:

    def __init__(self, run_on_robot):
        print("Drive Controller Starting")
        self.run_on_robot = run_on_robot

        self.mobile_base_vel_publisher = rospy.Publisher("/locobot/mobile_base/commands/velocity", Twist, queue_size=1)

        self.L = 0.2
        self.path = np.zeros((1,2))
        self.p = np.zeros(2)
        self.thread_lock = threading.Lock()

        self.maxSpeed = 0.3 # m/s
        self.maxRotation = 1 # rad/s

        self.go = False

        self.path = None

        if(self.run_on_robot):
            rospy.Subscriber("/camera_frame/mavros/vision_pose/pose", PoseStamped, self.OdometryCallback)
        else:
            rospy.Subscriber("/locobot/mobile_base/odom", Odometry, self.OdometryCallback)
        rospy.Subscriber("path_publisher", Float32MultiArray, self.traj_callback)
        
    def traj_callback(self, msg):
        self.go = True
        self.thread_lock.acquire()
        self.path = np.array(msg.data).reshape((-1,2))
        self.thread_lock.release()
    
    '''def set_target(self, target):
        self.go = True
        self.target = target'''

    def get_P_pos(self):
        self.thread_lock.acquire()
        p = self.p
        self.thread_lock.release()
        return p

    def OdometryCallback(self, data):
        if self.run_on_robot:
            orientation = data.pose.orientation
            position = data.pose.position
        
        else:   
            orientation = data.pose.pose.orientation
            position = data.pose.pose.position
        
        qw = orientation.w
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        R11 = qw**2 + qx**2 - qy**2 -qz**2
        R12 = 2*qx*qz - 2*qw*qz
        R21 = 2*qx*qz + 2*qw*qz
        R22 = qw**2 - qx**2 + qy**2 -qz**2

        M = np.matrix([[R11,self.L*R12],[R21,self.L*R22]])

        px = position.x + self.L*R11
        py = position.y + self.L*R21
        p = np.array([px,py])

        #print("Drive Controller Recieved Robot Position:\n", position)
        #print("Drive Controller Recieved Robot Orientation:\n", orientation)

        self.thread_lock.acquire()
        self.p = p
        self.thread_lock.release()

        if self.path is None:
            return
        #print("current_pos", self.p)
        dist_to_path = np.linalg.norm(self.p[np.newaxis,:] - self.path, axis = 1)
        #print("dist_to_path", dist_to_path)
        path_index = int(np.minimum(np.argmin(dist_to_path) + 1, self.path.shape[0] - 1))
        #print("path_index", path_index)
        target = self.path[path_index]

        k = 13
        u = np.ravel((k*np.linalg.inv(M)) @ (target - p).reshape((2,1)))

        v = u[0]
        v = np.clip(v, -self.maxSpeed, self.maxSpeed) # clip speed
        w = u[1]
        w = np.clip(w, -self.maxRotation, self.maxRotation) # clip rotation
        control_msg = Twist()
        control_msg.linear = Vector3(x=v) #forward velocity
        control_msg.angular = Vector3(z=w) #angular velocity

        #now publish the control output:
        if(self.go):
            self.mobile_base_vel_publisher.publish(control_msg)
            #print("Drive Controller Publishing:\n", control_msg)     
            
    def manual(self, v, w):
        self.go = False
        control_msg = Twist()
        control_msg.linear.x = float(v) #forward velocity
        control_msg.angular.z = float(w) #angular velocity
        self.mobile_base_vel_publisher.publish(control_msg)
        #print("Drive Controller Publishing:\n", control_msg)      
