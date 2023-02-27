#!/usr/bin/env python3

import time
import rospy
import tf
import numpy as np
import cv2

from occupancy_grid import OccupancyGrid
from drive_controller import DriveController


class StationTracker:
    def __init__(self, occupancy_grid, drive_controller):
        self.occupancy_grid = occupancy_grid
        self.drive_controller = drive_controller

        self.station_locations = np.array([[0,0],[1,0],[0,1]])
        self.radius = 0.25

        self.team_colors = [0,1,2,3] #  red:0 | yellow:1 | green:2 | blue:3 
        self.color_names = ["red","yellow","green","blue"]
        self.pick_up_eq_dist = 1 #m represents pickup time / speed

        self.oracle = np.array([
        [2,1,0],  #Red
        [0,1,1],  #Yellow
        [0,1,0],  #Green
        [1,0,2]]) #Blue

        choice_permutations = np.array([
        [0,1,2],
        [2,0,1],
        [1,2,0],
        [0,2,1],
        [1,0,2],
        [2,1,0]])

        self.oracle_choice = np.zeros((6,self.oracle.shape[0],self.oracle.shape[1]))
        for i in range(self.oracle_choice.shape[0]):
            self.oracle_choice[i,:,[0,1,2]] = self.oracle[:,choice_permutations[i]].T

        #print(self.oracle_choice)

        self.scanned_blocks = []
        self.moveable_blocks = []

        self.moveable_to_station_dist = None

        self.current_station_counts = np.zeros((4, 4))

        cube_side =(self.occupancy_grid.cube_size / self.occupancy_grid.grid_size)

        #Blob dector
        params = cv2.SimpleBlobDetector_Params()
        params.thresholdStep = 10
        params.minThreshold = 10
        params.maxThreshold = 255
        params.minDistBetweenBlobs = cube_side * np.sqrt(2) + 2
        params.filterByColor = False
        params.filterByArea = True
        params.minArea = 0
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
        
        '''print(normalized_grid[:,:,0].max())
        print(normalized_grid[:,:,1].max())
        print(normalized_grid[:,:,2].max())
        print(normalized_grid[:,:,3].max())'''
        for i in range(4):
            keypoints = self.detector.detect(normalized_grid[:,:,i])
            for kp in keypoints:
                world_pt = self.occupancy_grid.to_world(kp.pt)
                blocks.append([world_pt[0],world_pt[1],i])
        if len(blocks) > 0:
            self.scanned_blocks = np.array(blocks)
            sqr_dist = np.zeros((self.scanned_blocks.shape[0], 4))
            sqr_dist[:,3] = self.radius**2 #Set the dist to no station
            sqr_dist[:,0:3] = np.sum((self.scanned_blocks[:,np.newaxis,0:2] - self.station_locations[np.newaxis,...])**2, axis = 2)
            
            closest = np.argmin(sqr_dist, axis = 1)

            self.scanned_blocks = np.hstack((self.scanned_blocks,closest[...,np.newaxis])) # [block num, (pos.x, pos.y, color, current_station)] 

            moveable_mask = np.isin(self.scanned_blocks[:, 2], self.team_colors)
            self.moveable_blocks = self.scanned_blocks[moveable_mask]
            self.moveable_to_station_dist = np.sqrt(sqr_dist)[moveable_mask]
            #print("Station Dist Movable \n",self.moveable_to_station_dist)

            color_station_pairs = (self.scanned_blocks[:,2] * 4 + closest).astype(np.int32)

            #print("csp \n", color_station_pairs)
            self.current_station_counts = np.bincount(color_station_pairs, minlength = 16).reshape(4,4)
        else:
            print("No blocks found")
            self.scanned_blocks = []
            self.moveable_blocks = []

        print("current_blocks \n",self.current_station_counts)
        print("moveable_blocks \n",self.moveable_blocks)

        
        

    def get_next_move(self):
        self.count()
        if (len(self.moveable_blocks) == 0):
            return None,None

        robot_pos = self.drive_controller.get_P_pos()
        to_block_dist = np.linalg.norm(self.moveable_blocks[:,0:2] - robot_pos[np.newaxis,...], axis = 1)
        total_dist = self.moveable_to_station_dist + to_block_dist[...,np.newaxis]
        #print("Total Dist \n",total_dist)

        p = self.station_probabilities()

        color_from = self.moveable_blocks[:,2:4].astype(np.int32)
        #print(color_from)
        new_blocks = np.zeros((self.moveable_blocks.shape[0],4,self.oracle.shape[0],self.oracle.shape[1]))
        for n in range(new_blocks.shape[0]):
            for s in range(new_blocks.shape[1]):
                move_matrix = np.zeros_like(self.current_station_counts)
                move_matrix[color_from[n,0], color_from[n,1]] = -1
                move_matrix[color_from[n,0], s] = 1
                new_blocks[n,s,:,:] = (move_matrix + self.current_station_counts)[:,0:3]

        #print("new_blocks \n", new_blocks)
        d_score = self.score(new_blocks) - self.score(self.current_station_counts[:,0:3])
        #print("d_score \n", d_score)

        expected_score = np.sum(d_score * p[np.newaxis,np.newaxis,:], axis = 2)
        print("expected_score \n",expected_score)
        expected_score_dt = expected_score / (total_dist + self.pick_up_eq_dist)
        print("expected_score_dt \n",expected_score_dt)

        # Choose best move
        best_cube_num, best_destination_station = np.unravel_index(expected_score_dt.argmax(), expected_score_dt.shape)
        best_cube_num = int(best_cube_num)
        best_destination_station = int(best_destination_station)

        if (expected_score_dt[best_cube_num, best_destination_station] <= 0): #if all moves decrease score
            return None,None # Do nothing
        
        best_cube_pos = self.moveable_blocks[best_cube_num, 0:2]
        best_cube_color = self.color_names[int(self.moveable_blocks[best_cube_num, 2])]
        dest_pos = self.station_locations[best_destination_station]

        print("Bringing",best_cube_color, "cube",best_cube_pos.ravel(),"to station @",dest_pos.ravel())
        
        return best_cube_pos, dest_pos

    def score(self, station_counts):
        k = len(station_counts.shape)-2
        dif = station_counts[..., np.newaxis, :, :] - np.broadcast_to(self.oracle_choice, (1,) * k + self.oracle_choice.shape)
        score = np.where(dif > 0, -2 * dif, 0) + np.where(dif < 0, -25 * -dif, 0)
        total_score = np.sum((score),axis = (-1,-2)) + 25 * np.sum(self.oracle)
        return(total_score)

    def station_probabilities(self):
        errors = np.sum(np.abs(self.current_station_counts[np.newaxis,:,0:3] - self.oracle_choice),axis=(1,2))
        total_blocks = np.sum(self.oracle)

        norm_const = 1/600

        P = np.power(total_blocks / (errors + norm_const), 3)
        p = P/np.sum(P)
        
        return p

