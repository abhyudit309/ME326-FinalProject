#!/usr/bin/env python3

#roslaunch interbotix_xslocobot_control xslocobot_control.launch robot_model:=locobot_wx250s show_lidar:=true use_camera:=true use_rviz:=false align_depth:=true use_base:=true use_static_transform_pub:=true
#roslaunch interbotix_xslocobot_moveit xslocobot_moveit.launch robot_model:=locobot_wx250s show_lidar:=true use_actual:=false use_camera:=true use_gazebo:=false use_moveit_rviz:=false dof:=6

#Don't use the commands below
#roslaunch interbotix_xslocobot_moveit xslocobot_moveit.launch robot_model:=locobot_wx250s show_lidar:=true use_actual:=true use_camera:=true use_mobile_base:=true use_gazebo:=false use_moveit_rviz:=false align_depth:=true use_nav:=true dof:=6
#roslaunch interbotix_xslocobot_control xslocobot_control.launch robot_model:=locobot_wx250s show_lidar:=true use_actual:=true use_camera:=true use_mobile_base:=true use_gazebo:=false use_moveit_rviz:=false align_depth:=true use_nav:=true dof:=6

import time
import rospy
import tf
import numpy as np
import cv2
import sys
import traceback

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
        self.orient_speed = 0.3 # rad/s
        
        self.listener = tf.TransformListener()
        self.tf_time = 0
        
        self.occupancy_grid = OccupancyGrid(run_on_robot = run_on_robot)
        self.drive_controller = DriveController(run_on_robot = run_on_robot)
        self.station_tracker = StationTracker(self.occupancy_grid, self.drive_controller)
        self.path_planner = PathPlanner(self.occupancy_grid, self.station_tracker)
        
        moveit_commander.roscpp_initialize(sys.argv)
        self.orient_camera = OrientCamera()
        print("argv:", sys.argv)
        self.move_locobot_arm = MoveLocobotArm(moveit_commander)

        self.replan_every = 0.5 #s
        self.replan_time = -99999999

        self.close_enough = 0.35 #m

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
            self.driving_state(self.get_block_from)
        elif(self.state == 2):
            self.grasping(self.get_block_from)
            self.state += 1
        elif(self.state == 3):
            self.driving_state(self.bring_block_to)
        elif(self.state == 4):
            self.grasping(self.bring_block_to)
            self.state = 0
        else:
            print("State number not in range:", self.state)

        if (self.state != starting_state):
            print("Swapping to state:", self.state)
        self.display_map()

    def driving_state(self, target):
        pos = self.drive_controller.get_P_pos()
        if (np.linalg.norm(pos-target) < self.close_enough):
            self.drive_controller.manual(0, 0)
            print("stopping")
            self.state += 1
        else:
            time = rospy.Time.now().to_sec()
            if (time - self.replan_time > self.replan_every):
                start = pos
                ### Do we need this ? ###
                if self.state == 1:
                    temp_block_from, temp_block_to = self.station_tracker.get_next_move()
                    if not(temp_block_from is None):
                        self.get_block_from = temp_block_from
                        self.bring_block_to = temp_block_to
                        target = temp_block_from
                ### ---------------- ###
                self.path_planner.generate_obs_grid()       
                self.path_planner.plan(self.x_init, target)
                self.path_planner.path_publisher()
                self.replan_time = time

    def grasping(self, target_pose):
        target_pose = np.round(target_pose, 1)
        print("rounded pose:", target_pose)
        ### orients locobot with block to be grasped ###
        try:
            if self.run_on_robot:
                tf_matrix = np.linalg.inv(self.occupancy_grid.real_world_matrix)
            else:
                (trans, rot) = self.listener.lookupTransform('locobot/base_footprint', 'locobot/odom', self.tf_time)
                tf_matrix = tf.transformations.compose_matrix(translate=trans, angles=tf.transformations.euler_from_quaternion(rot))
        except (tf.LookupException, tf.ConnectivityException):
            pass
        if self.state == 2:
            pose = np.matmul(tf_matrix, np.array([target_pose[0], target_pose[1], 0, 1]))
            angle = np.arctan2(pose[1], pose[0])
            orient_time = rospy.Time.now().to_sec() + np.abs(angle) / self.orient_speed
            while rospy.Time.now().to_sec() < orient_time:
                self.drive_controller.manual(0, np.sign(angle) * self.orient_speed)
            self.drive_controller.manual(0, 0)
            print("oriented with block!!")
        ### ----------- ###
        
        pose = np.matmul(tf_matrix, np.array([target_pose[0], target_pose[1], 0, 1]))
        self.occupancy_grid.do_scan = False
        # self.orient_camera.tilt_camera(angle=-0.5)
        print("going down!!", pose[0], pose[1])
        self.move_locobot_arm.move_gripper_down_to_grasp(pose[0], pose[1])
        if self.state == 2:
            print("closing gripper!!")
            self.move_locobot_arm.close_gripper()
        elif self.state == 4:
            print("opening gripper!!")
            self.move_locobot_arm.open_gripper()
        else:
            pass          
        self.move_locobot_arm.move_arm_down_for_camera()
        self.occupancy_grid.do_scan = True
        # self.orient_camera.tilt_camera()

    def spin_scan(self):
        print("Spinning")
        spin_timer_end = rospy.Time.now().to_sec() + self.spin_time
        
        while(rospy.Time.now().to_sec() < spin_timer_end):
            self.drive_controller.manual(0, self.spin_speed)
            self.display_map()
        
    def stop(self):
        stop_timer_end = rospy.Time.now().to_sec() + self.stop_time
        while(rospy.Time.now().to_sec() < stop_timer_end):
            self.drive_controller.manual(0, 0)
            
    def display_map(self):
        time = rospy.Time.now().to_sec()
        if (time - self.display_time > self.display_every):
            self.occupancy_grid.display_occupancy()
            self.display_time = time

if __name__ == "__main__":
    np.set_printoptions(precision=5, edgeitems=30, linewidth=250)
    arg = False
    if (sys.argv[1] == "true"):
        arg = True
    conductor = Conductor(run_on_robot=arg)
    conductor.stop()
    if conductor.run_on_robot:
        conductor.move_locobot_arm.move_arm_down_for_camera()
        conductor.orient_camera.tilt_camera()
    while True:
        try:
            conductor.state_machine()
        except:
            print(traceback.format_exc())
            print("Shutting down")
            break
