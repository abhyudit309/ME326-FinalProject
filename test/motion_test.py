#!/usr/bin/env python3

# Make sure a locobot works by testing every essential maneuver
# Set run_on_robot to true if running on real locobot
# Set use_interbotix to true if using interbotix object instead of moveit object

import time
import rospy
import sys
import traceback
from drive_controller import DriveController
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from locobot_motion_example import OrientCamera, MoveLocobotArm
import moveit_commander
from interbotix_xs_modules.locobot import InterbotixLocobotXS

class Motion(object):
    def __init__(self, run_on_robot = False, use_interbotix = False):
        self.run_on_robot = run_on_robot
        self.use_interbotix = use_interbotix
        self.x = 0
        self.y = 0
        self.max_ang_vel = 0.3
        self.max_vel = 0.5
        self.motion_time = 2 #seconds

        self.drive_controller = DriveController(run_on_robot = run_on_robot)
        
        if use_interbotix == True:
            self.locobot = InterbotixLocobotXS(robot_model="locobot_wx250s", arm_model="mobile_wx250s", use_move_base_action=True)

        self.orient_camera = OrientCamera()
        moveit_commander.roscpp_initialize(sys.argv)
        self.moveit = MoveLocobotArm(moveit_commander)
        
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
        if(self.use_interbotix == True):
            self.locobot.gripper.open()
        else: 
            self.moveit.open_gripper()
    
    def close_gripper(self):
        print("Closing Gripper")
        if(self.use_interbotix == True):
            self.locobot.gripper.close()
        else: 
            self.moveit.close_gripper()
    
    def waiting_arm_state(self):
        print("Moving arm to wait under camera")
        if(self.use_interbotix == True):
            self.locobot.arm.go_to_home_pose()
        else: 
            self.moveit.move_arm_down_for_camera()
            
    def grasping_arm_state(self):
        print("Moving arm to grasp")
        self.moveit.move_gripper_down_to_grasp()

    def set_arm_to(self, x, z):
        if(self.use_interbotix == True):
            print("Moving arm to exact position (x=", x, "z=", z, ")")
            self.locobot.arm.set_ee_cartesian_trajectory(x, z)
    
    def sleeping_arm_state(self, x, z):
        if(self.use_interbotix == True):
            print("Moving arm to sleep")
            self.locobot.arm.go_to_sleep_pose()

    def tilt_camera(self):
        print("Tilting camera")
        self.orient_camera.tilt_camera()
    

if __name__ == "__main__":
    arg_1 = False
    arg_2 = False
    if (sys.argv[1] == "true"):
        arg_1 = True
    if (sys.argv[2] == "true"):
        arg_2 = True
    motion = Motion(run_on_robot=arg_1, use_interbotix=arg_2)

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
    motion.set_arm_to(x=0.1, z=0.25)
    motion.sleeping_arm_state()

