#!/usr/bin/env python3

import time
import rospy
import tf
import numpy as np
import cv2

from occupancy_grid import OccupancyGrid
from path_planner import PathPlanner
from station_tracker import StationTracker
from drive_controller import DriveController

class Conductor:
    def __init__(self):
        rospy.init_node("conductor")
        self.state = 0
        # State 0: Thinking
        # State 1: Moving to next block
        # State 2: Grabbing block
        # State 3: Moving to block location
        # State 4: Placing block

        self.occupancy_grid = OccupancyGrid()
        self.path_planner = PathPlanner(self.occupancy_grid)
        self.drive_controller = DriveController()
        self.station_tracker = StationTracker(self.occupancy_grid)

        self.replan_every = 0.5 #s
        self.replan_time = -99999999
        self.close_enough = 0.005 #m

        self.get_block_from = np.zeros(2)
        self.bring_block_to = np.zeros(2)

    def state_machine(self):
        starting_state = self.state
        self.get_block_from, self.bring_block_to = self.station_tracker.get_next_move()
        if(self.state == 0):
            self.get_block_from, self.bring_block_to = self.station_tracker.get_next_move()
            self.state += 1
        elif(self.state == 1):
            self.driving_state(self.get_block_from)
        elif(self.state == 2):
            self.state += 1 # not implemented
            pass
        elif(self.state == 3):
            self.driving_state(self.bring_block_to)
        elif(self.state == 4):
            self.state += 1 # not implemented
            pass
        else:
            print("State number not in range:", self.state)
        if (self.state != starting_state):
            print("Swapping to state:", self.state)

    def driving_state(self,target):
        time = rospy.Time.now().to_sec()
        if (time - self.replan_time > self.replan_every):
            self.path_planner.plan(target)
            self.replan_time = time

        self.drive_controller.set_target(self.path_planner.current_target())
        pos = self.drive_controller.get_P_pos()

        if (np.linalg.norm(pos-target) < self.close_enough):
            self.state += 1

if __name__ == "__main__":
    np.set_printoptions(precision=5, edgeitems=30, linewidth=250)
    conductor = Conductor()
    rospy.sleep(2.) #wait for vision
    while True:
        conductor.state_machine()