#!/usr/bin/env python3
import numpy as np
import cv2
import rospy
from occupancy_grid import OccupancyGrid
from astar import AStar
from std_msgs.msg import Float32MultiArray
import threading
from matplotlib import pyplot as plt


class PathPlanner:

    def __init__(self, occupancy):
        self.occupancy = occupancy
        occupancy_grid = self.occupancy.grid
        dim = occupancy_grid.shape[0]      
        self.statespace_hi = self.occupancy.to_world(np.array([dim, dim]))
        self.statespace_lo = self.occupancy.to_world(np.array([0, 0]))

        self.path = None #np.zeros((1,2))
        self.path_time = 0
        self.target = np.zeros(2)

        #self.time_between_path_points = 0.1 #s        

        self.obs_grid_size = 0.05 #m
        self.scale_ratio = self.occupancy.grid_size / self.obs_grid_size

        self.obs_grid_dim = np.rint(np.array(self.occupancy.grid.shape[0:2]) * self.scale_ratio).astype(int)
        self.obs_grid = np.zeros(self.obs_grid_dim)

        self.spacing = 0.35 #m

        r_grid = np.ceil(self.spacing/self.obs_grid_size)
        v,u = np.meshgrid(np.arange(-r_grid,r_grid+1), np.arange(-r_grid,r_grid+1), indexing='ij')

        self.filter = np.array(np.less_equal(v**2 + u**2, r_grid**2),np.float32)

        self.pub = rospy.Publisher('path_publisher', Float32MultiArray, queue_size=10)
     
    def generate_obs_grid(self):    
        self.occupancy.thread_lock.acquire()
        occ_grid = self.occupancy.grid.copy()
        self.occupancy.thread_lock.release()

        obs_num = 32767
        large_obs = np.array(np.where(occ_grid[:,:,0] < np.sum(occ_grid[:,:,5:6], axis = 2),obs_num,0), dtype='uint16')
        unspaced_obs = cv2.resize(large_obs,(self.obs_grid_dim[0],self.obs_grid_dim[1]))
        #print(large_obs[550-3:550+3,400-3:400+3])
        #print(unspaced_obs)
        obs_grid = cv2.filter2D(unspaced_obs, ddepth=-1, kernel=self.filter)
        self.obs_grid = np.where(obs_grid > 0, 1, 0) 
    
    def plan(self, x_init, x_goal):
        #self.x_goal = x_goal
        self.astar = AStar(self.statespace_lo, self.statespace_hi, x_init, x_goal, self.occupancy, self.obs_grid, self.obs_grid_size)
        if not self.astar.solve():
            print("No path found !!")
            return []
        else:
            print("Path found !!")
            self.path = np.array(self.astar.path  + [x_goal])
            #print("Path:", self.path)
            self.x_goal = x_goal
            self.path_time = rospy.Time.now().to_sec()
            return self.path

    def path_publisher(self):
        rate = rospy.Rate(100) # 100hz
        published_path = Float32MultiArray()
        published_path.data = self.path.ravel().tolist()
        #print("Pup Path:", published_path.data)
        self.pub.publish(published_path)
            
    
    def current_target(self): # NOT USED
        pass

if __name__ == "__main__":
    np.set_printoptions(precision=5, edgeitems=30, linewidth=250)
    occupancy = OccupancyGrid()
    path_planner = PathPlanner(occupancy)
    while True:
        pass
        path_planner.generate_obs_grid()
