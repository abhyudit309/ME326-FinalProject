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
        self.station_locations = np.array([[0,0],[1,0],[0,1]])
        self.radius = 0.25

        self.team = 0 #  Red+Green:0  Yellow+Blue:1

        self.oracle = np.array([
        [2,1,0],  #Red
        [0,1,1],  #Yellow
        [0,1,0],  #Green
        [1,0,2]]) #Blue
        cube_side =(self.occupancy_grid.cube_size / self.occupancy_grid.grid_size)

        #Blob dector
        params = cv2.SimpleBlobDetector_Params()
        params.thresholdStep = 10
        params.minThreshold = 120
        params.maxThreshold = 255
        params.minDistBetweenBlobs = cube_side * np.sqrt(2) + 2
        params.filterByColor = False
        params.filterByArea = False
        params.minArea = cube_side**2
        params.maxArea = cube_side**2 + 4 * cube_side
        params.filterByCircularity = False
        params.minCircularity = 0.4
        params.filterByConvexity = False
        #params.minConvexity = 0.8
        params.filterByInertia = False
        #params.minInertiaRatio = 0.01
        '''print("ThresholdStep:", params.thresholdStep)
        print("MinThreshold:", params.minThreshold)
        print("MaxThreshold:", params.maxThreshold)
        print("MinRepeatability:", params.minRepeatability)
        print("MinDistBetweenBlobs:", params.minDistBetweenBlobs)
        print("FilterByColor:", params.filterByColor)
        print("BlobColor:", params.blobColor)
        print("FilterByArea:", params.filterByArea)
        print("MinArea:", params.minArea)
        print("MaxArea:", params.maxArea)
        print("FilterByCircularity:", params.filterByCircularity)
        print("MinCircularity:", params.minCircularity)
        print("MaxCircularity:", params.maxCircularity)
        print("FilterByInertia:", params.filterByInertia)
        print("MinInertiaRatio:", params.minInertiaRatio)
        print("MaxInertiaRatio:", params.maxInertiaRatio)
        print("FilterByConvexity:", params.filterByConvexity)
        print("MinConvexity:", params.minConvexity)
        print("MaxConvexity:", params.maxConvexity)'''
        self.detector = cv2.SimpleBlobDetector_create(params)

    def count(self):
        self.occupancy_grid.thread_lock.acquire()
        occ_grid = self.occupancy_grid.grid
        normalized_grid = occ_grid[:,:,1:5]
        normalized_grid = normalized_grid / (1e-9+normalized_grid + occ_grid[:,:,0,np.newaxis])
        self.occupancy_grid.thread_lock.release()

        normalized_grid = (normalized_grid.clip(0, 1) * 255).astype(np.uint8)
        station_counts = np.zeros((4,3))
        blocks = [] # [x,y,Color]  | red:0 | yellow:1 | green:2 | blue:3 | 
        
        print(normalized_grid[:,:,0].max())
        print(normalized_grid[:,:,1].max())
        print(normalized_grid[:,:,2].max())
        print(normalized_grid[:,:,3].max())
        for i in range(4):
            keypoints = self.detector.detect(normalized_grid[:,:,i])
            for kp in keypoints:
                world_pt = self.occupancy_grid.to_world(kp.pt)
                blocks.append([world_pt[0],world_pt[1],i])
        if len(blocks) > 0:
            blocks = np.array(blocks)
            sqr_dist = np.zeros((blocks.shape[0], 4))
            sqr_dist[:,3] = self.radius**2 #Set the dist to no station

            print(blocks)
            sqr_dist[:,0:3] = np.sum((blocks[:,np.newaxis,0:2] - self.station_locations[np.newaxis,...])**2, axis = 2)
            print(sqr_dist)
        else:
            print("No blocks found")

        

    def get_next_move(self):
        self.occupancy_grid.thread_lock.acquire()
        occ_grid = self.occupancy_grid.grid.copy()
        self.occupancy_grid.thread_lock.release()

        #fix: right now just getting the most red pixel
        
        red = occ_grid[:,:,1]
        max_red = np.array(np.unravel_index(np.argmax(red), red.shape))
        #print("Argmax:", max_red)
        get_block_from = self.occupancy_grid.to_world(max_red)
        bring_block_to = np.zeros(2)
        return get_block_from, bring_block_to
