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

    def get_next_move(self):
        self.occupancy_grid.thread_lock.acquire()
        occ_grid = self.occupancy_grid.grid
        self.occupancy_grid.thread_lock.release()

        #fix: right now just getting the most red pixel
        
        red = occ_grid[:,:,1]
        max_red = np.array(np.unravel_index(np.argmax(red), red.shape))
        #print("Argmax:", max_red)
        get_block_from = self.occupancy_grid.to_world(max_red)
        bring_block_to = np.zeros(2)
        return get_block_from, bring_block_to
