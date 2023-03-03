#!/usr/bin/env python3

import time
import rospy
import tf
import numpy as np
import cv2
import sys

from occupancy_grid import OccupancyGrid
from path_planner import PathPlanner
from station_tracker import StationTracker
from drive_controller import DriveController
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from locobot_motion_example import OrientCamera, MoveLocobotArm
import moveit_commander

class Conductor:
    def __init__(self, run_on_robot = False):
        print("Conductor Starting")
        self.run_on_robot = run_on_robot
        rospy.init_node("conductor")
        self.state = 0
        # State 0: Thinking
        # State 1: Moving to next block
        # State 2: Grabbing block
        # State 3: Moving to block location
        # State 4: Placing block

        self.x = 0
        self.y = 0
        
        self.listener = tf.TransformListener()
        self.tf_time = 0
        
        self.occupancy_grid = OccupancyGrid(run_on_robot = run_on_robot)
        self.path_planner = PathPlanner(self.occupancy_grid)
        self.drive_controller = DriveController(run_on_robot = run_on_robot)
        self.station_tracker = StationTracker(self.occupancy_grid, self.drive_controller)
        
        moveit_commander.roscpp_initialize(sys.argv)
        self.orient_camera = OrientCamera()
        print("argv:", sys.argv)
        #self.move_locobot_arm = MoveLocobotArm(moveit_commander) # Issue here

        self.replan_every = 0.5 #s
        self.replan_time = -99999999

        self.close_enough = 0.5 #m

        self.display_every = 0.5 #s
        self.display_time = -99999999
        
        self.spin_speed = 0.5 # rad/s
        self.spin_time = 2*np.pi / self.spin_speed #s
        self.stop_time = 0.5 #s

        self.get_block_from = np.zeros(2)
        self.bring_block_to = np.zeros(2)
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
        self.tf_time = data.header.stamp
        self.x_init = (self.x, self.y)
    
    def state_machine(self):
        starting_state = self.state
        
        if(self.state == 0):
            self.spin_scan()
            self.get_block_from, self.bring_block_to = self.station_tracker.get_next_move()
            if not(self.get_block_from is None):
                self.state += 1
        elif(self.state == 1):
            self.driving_state(self.bring_block_to, self.get_block_from)
        elif(self.state == 2):
            self.grasping(self.get_block_from)
            print("closing gripper!!")
            #self.move_locobot_arm.close_gripper() # ISSUES with gripper!
            self.state += 1
        elif(self.state == 3):
            self.driving_state(self.get_block_from, self.bring_block_to)
        elif(self.state == 4):
            self.grasping(self.bring_block_to)
            print("opening gripper!!")
            #self.move_locobot_arm.open_gripper() # ISSUES with gripper!
            self.state = 0
        else:
            print("State number not in range:", self.state)

        if (self.state != starting_state):
            print("Swapping to state:", self.state)
        self.display_map()

    def driving_state(self, start, target):
        pos = self.drive_controller.get_P_pos()
        if (np.linalg.norm(pos-target) < self.close_enough):
            self.drive_controller.manual(0,0)
            print("stopping")
            self.state += 1
        else:
            time = rospy.Time.now().to_sec()
            if (time - self.replan_time > self.replan_every):
                self.path_planner.generate_obs_grid()       
                self.path_planner.plan(self.x_init, target)
                self.path_planner.path_publisher()
                self.replan_time = time

    def grasping(self, target_pose):
        try:
            (trans, rot) = self.listener.lookupTransform('locobot/base_footprint', 'locobot/odom', self.tf_time)
            tf_matrix = tf.transformations.compose_matrix(translate=trans, angles=tf.transformations.euler_from_quaternion(rot))
        except (tf.LookupException, tf.ConnectivityException):
            pass
        pose = np.matmul(tf_matrix, np.array([target_pose[0], target_pose[1], 0, 1]))
        self.orient_camera.tilt_camera(angle=-0.5)
        print("going down!!", pose[0], pose[1])
        #self.move_locobot_arm.move_gripper_down_to_grasp(pose[0], pose[1])
        #self.move_locobot_arm.move_arm_down_for_camera()
        self.orient_camera.tilt_camera()

    def spin_scan(self):
        print("Spinning")
        spin_timer_end = rospy.Time.now().to_sec() + self.spin_time
        
        while(rospy.Time.now().to_sec() < spin_timer_end):
            self.drive_controller.manual(0,self.spin_speed)
            self.display_map()
        
    def stop(self):
        stop_timer_end = rospy.Time.now().to_sec() + self.stop_time
        while(rospy.Time.now().to_sec() < stop_timer_end):
            self.drive_controller.manual(0,0)
            
    def display_map(self):
        time = rospy.Time.now().to_sec()
        if (time - self.display_time > self.display_every):
            self.occupancy_grid.display_occupancy()
            self.display_time = time

if __name__ == "__main__":
    np.set_printoptions(precision=5, edgeitems=5, linewidth=250)
    arg = False
    if (sys.argv[1] == "true"):
        arg = True
    conductor = Conductor(run_on_robot=arg)
    conductor.stop()
    while True:
        conductor.state_machine()
