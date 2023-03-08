#!/usr/bin/env python3

# Make sure a locobot works by testing every essential maneuver

import time
import rospy
import sys
import traceback
from drive_controller import DriveController
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from locobot_motion_example import OrientCamera, MoveLocobotArm
import moveit_commander
#from interbotix_xs_modules.locobot import InterbotixLocobotXS

class Motion(object):
    def __init__(self, run_on_robot = False):
        self.x = 0
        self.y = 0
        self.max_ang_vel = 0.3
        self.max_vel = 0.5
        self.motion_time = 2 #seconds

        self.drive_controller = DriveController(run_on_robot = run_on_robot)
        self.orient_camera = OrientCamera()
        moveit_commander.roscpp_initialize(sys.argv)
        self.move_locobot_arm = MoveLocobotArm(moveit_commander)
        
        if(self.run_on_robot):
            rospy.Subscriber("/camera_frame/mavros/vision_pose/pose", PoseStamped, self.OdometryCallback)
        else:
            rospy.Subscriber("/locobot/mobile_base/odom", Odometry, self.OdometryCallback)
    
    def OdometryCallback(self, data):
        if self.run_on_robot:
            self.x = data.pose.position.x
            self.y = data.pose.position.y
        else:
            self.x = data.pose.pose.position.x
            self.y = data.pose.pose.position.y
        self.x_init = (self.x, self.y)
    
    def stop(self):
        print("Stopping")
        timer_end = rospy.Time.now().to_sec() + self.motion_time
        while(rospy.Time.now().to_sec() < timer_end):
            self.drive_controller.manual(0,0)

    def drive_forward(self):
        print("Forward")
        timer_end = rospy.Time.now().to_sec() + self.motion_time
        while(rospy.Time.now().to_sec() < timer_end):
            self.drive_controller.manual(self.max_vel,0) #forward, angular

    def drive_backward(self):
        print("Backward")
        timer_end = rospy.Time.now().to_sec() + self.motion_time
        while(rospy.Time.now().to_sec() < timer_end):
            self.drive_controller.manual(-self.max_vel,0)

    def turn_right(self):
        print("Right")
        timer_end = rospy.Time.now().to_sec() + self.motion_time
        while(rospy.Time.now().to_sec() < timer_end):
            self.drive_controller.manual(0,self.max_ang_vel)

    def turn_left(self):
        print("Left")
        timer_end = rospy.Time.now().to_sec() + self.motion_time
        while(rospy.Time.now().to_sec() < timer_end):
            self.drive_controller.manual(0,-self.max_ang_vel)

    def open_gripper(self):
        print("Opening Gripper")
        self.move_locobot_arm.open_gripper()
    
    def close_gripper(self):
        print("Closing Gripper")
        self.move_locobot_arm.close_gripper()
    
    def waiting_arm_state(self):
        print("Moving arm to wait under camera")
        self.move_locobot_arm.move_arm_down_for_camera()

    def grasping_arm_state(self):
        print("Moving arm to grasp")
        self.move_locobot_arm.move_gripper_down_to_grasp()

    def tilt_camera(self):
        print("Tilting camera")
        self.orient_camera.tilt_camera()
    

if __name__ == "__main__":
    arg = False
    if (sys.argv[1] == "true"):
        arg = True
    motion = Motion(run_on_robot=arg)

    print("Begin Test sequence: ")
    motion.drive_forward()
    motion.drive_backward()
    motion.stop()
    motion.turn_left()
    motion.turn_right()
    motion.grasping_arm_state()
    motion.open_gripper()
    motion.close_gripper()
    motion.waiting_arm_state()
    motion.tilt_camera()
