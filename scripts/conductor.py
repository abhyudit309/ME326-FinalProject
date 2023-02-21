#!/usr/bin/env python3

import time
import rospy
import tf
import numpy as np
import cv2

from occupancy_grid import OccupancyGrid
from path_planner import PathPlanner

class Conductor:
    def __init__(self):
        self.state = 0
        # State 0: Thinking
        # State 1: Moving to next block
        # State 2: Grabbing block
        # State 3: Moving to block location
        # State 4: Placing block

        self.occupancy_grid = OccupancyGrid()
        self.path_planner = PathPlanner(self.occupancy_grid)
        self.drive_controller = DriveController()
        self.station_tracker = StationTracker()

        self.replan_every = 0.5 #s
        self.replan_time = -99999999
        self.close_enough = 0.005 #m

        self.get_block_from = np.zero(2)
        self.bring_block_to = np.zero(2)

    def state_machine(self):
        if(self.state = 0):
            self.get_block_from, self.bring_block_to = station_tracker.get_next_move()
            self.state += 1
        elif(self.state = 1):
            self.driving_state(self.get_block_from)
        elif(self.state = 2):
            self.state += 1 # not implemented
            pass
        elif(self.state = 3):
            self.driving_state(self.bring_block_to)
        elif(self.state = 4):
            self.state += 1 # not implemented
            pass
        else:
            print("State number not in range:", self.state)

    def driving_state(self,target):
        time = rospy.Time.now().to_sec()
        if (time - self.replan_time > self.replan_every):
            self.path_planner.plan(target)
            self.replan_time = time

        pos = drive_controller.set_target(path_planner.current_target())

        if (np.linalg.norm(pos-target) < self.close_enough):
            self.state += 1
