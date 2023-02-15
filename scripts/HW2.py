#!/usr/bin/env python3

import rospy
import numpy as np
import scipy as sp
from scipy import linalg
import geometry_msgs

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
#You will also later in your code, need to define the publisher (example name):




class LocobotTrajTracker(object):
    def __init__(self):
        rospy.init_node('LocobotTrajTracker')
        self.mobile_base_vel_publisher = rospy.Publisher("/locobot/mobile_base/commands/velocity", Twist, queue_size=1)
        
        self.t_init = rospy.get_time()
        self.L = 0.025

        rospy.Subscriber("/locobot/mobile_base/odom", Odometry, self.OdometryCallback) #this says: listen to the odom message, of type odometry, and send that to the callback function specified


    def OdometryCallback(self, data):
        # Step 1: Calculate the point P location (distance L on the x-axis), and publish the marker so it can be seen in Rviz
        #first determine the relative angle of the mobile base in the world xy-plane, this angle is needed to determine where to put the point P
        #the rotation will be about the world/body z-axis, so we will only need the qw, and qz quaternion components. We can then use knoweldge of the 
        #relationship between quaternions and rotation matricies to see how we must rotate the Lx vector into the world (odom) frame and add it to the base position
        #to obtain the point P (for more info on quaterions, see a primer at the bottom of this page: https://arm.stanford.edu/resources/armlab-references)
        qw = data.pose.pose.orientation.w
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        R11 = qw**2 + qx**2 - qy**2 -qz**2
        R12 = 2*qx*qz - 2*qw*qz
        R21 = 2*qx*qz + 2*qw*qz
        R22 = qw**2 - qx**2 + qy**2 -qz**2

        M = np.matrix([[R11,self.L*R12],[R21,self.L*R22]])
        
        t = rospy.get_time() - self.t_init
        
        phase = t * 2/10 *np.pi
        rd = 0.5 * np.array([np.cos(phase),np.sin(phase)])

        px = data.pose.pose.position.x + self.L*R11
        py = data.pose.pose.position.y + self.L*R21
        p = np.array([px,py])

        k = 0.5
        u = np.ravel((k*np.linalg.inv(M)) @ (rd - p).reshape((2,1)))

        v = u[0]#fix this control input
        w = u[1] #fix this control input
        control_msg = Twist()
        control_msg.linear.x = float(v) #forward velocity
        control_msg.angular.z = float(w) #angular velocity
        #now publish the control output:
        self.mobile_base_vel_publisher.publish(control_msg)
        # Now store these for plotting later:
        #control_msg_plot = TwistStamped()

        
   
        

if __name__ == '__main__':
    LTT = LocobotTrajTracker()
    rospy.spin() #This is ros python's way of 'always listening' for the subscriber messages
    


