#!/usr/bin/env python3
import numpy as np
import cv2
from occupancy_grid import OccupancyGrid
import threading


class PathPlanner:
    def __init__(self, occupancy_grid):

        self.occupancy_grid = occupancy_grid

        self.obs_grid_size = 0.1 #m

        self.scale_ratio = self.occupancy_grid.grid_size / self.obs_grid_size

        self.obs_grid_dim = np.rint(np.array(self.occupancy_grid.grid.shape[0:2]) * self.scale_ratio).astype(int)
        self.obs_grid = np.zeros(self.obs_grid_dim)

        self.spacing = 0.3 #m

        r_grid = np.ceil(self.spacing/self.obs_grid_size)
        v,u = np.meshgrid(np.arange(-r_grid,r_grid+1), np.arange(-r_grid,r_grid+1), indexing='ij')

        self.filter = np.array(np.less_equal(v**2 + u**2, r_grid**2),np.float32)

    def generate_obs_grid(self):
        self.occupancy_grid.thread_lock.acquire()
        occ_grid = self.occupancy_grid.grid
        self.occupancy_grid.thread_lock.release()


        obs_num = 32767
        large_obs = np.array(np.where(occ_grid[:,:,0] < np.sum(occ_grid[:,:,1:6], axis = 2),obs_num,0), dtype='uint16')
        unspaced_obs = cv2.resize(large_obs,(self.obs_grid_dim[0],self.obs_grid_dim[1]))
        #print(large_obs[550-3:550+3,400-3:400+3])
        #print(unspaced_obs)
        obs_grid = cv2.filter2D(unspaced_obs, ddepth=-1, kernel=self.filter)
        self.obs_grid = np.where(obs_grid > 0, 1, 0)
        print(self.obs_grid)


if __name__ == "__main__":
    np.set_printoptions(precision=5, edgeitems=30, linewidth=250)
    occ_grid = OccupancyGrid()
    path_planner = PathPlanner(occ_grid)
    while True:
        path_planner.generate_obs_grid()