#!/usr/bin/env python3

import time
import rospy
import tf
import numpy as np
import cv2

from occupancy_grid import OccupancyGrid


class StationTracker:
    def __init__(self, occupancy_grid):
    	self.occupancy_grid = occupancy_grid