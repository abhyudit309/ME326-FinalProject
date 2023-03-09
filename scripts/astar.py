import numpy as np
from occupancy_grid import OccupancyGrid

class AStar(object):

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, obs_grid, resolution, obs_spacing)->None:
        self.statespace_lo = statespace_lo                       # state space lower bound (e.g., [-5, -5])
        self.statespace_hi = statespace_hi                       # state space upper bound (e.g., [5, 5])
        self.occupancy = occupancy                               # occupancy grid (a DetOccupancyGrid2D object)
        self.obs_grid = obs_grid.T                                 # obstacle grid
        self.resolution = resolution                             # resolution of the discretization of state space (cell/m)
        self.scale = self.occupancy.grid_size / self.resolution  # scaling factor between occupancy grid and obstacle grid
        
        self.obs_spacing = obs_spacing


        self.arm_reach = 0.001

        self.x_init = self.snap_to_grid(x_init) # initial state
        self.x_goal = self.snap_to_grid(x_goal) # goal state

        self.closed_set = set()    # the set containing the states that have been visited
        self.open_set = set()      # the set containing the states that are condidate for future expension
        self.est_cost_through = {} # 2d map of the estimated cost from start to goal passing through state
        self.cost_to_arrive = {}   # 2d map of the cost-to-arrive at state from start
        self.came_from = {}        # dictionary keeping track of each state's parent to reconstruct the path

        self.open_set.add(self.x_init)
        self.cost_to_arrive[self.x_init] = 0
        self.est_cost_through[self.x_init] = self.distance(self.x_init,self.x_goal)
        
        self.path = None        # the final path as a list of states

    def is_free(self, x):
        # Return true (i) if the state is inside the bounds of the map and not inside any obstacle
        # OR (ii) if it is the initial state OR (iii) if it is the goal state
        if x == self.x_init or x == self.x_goal:
            return True
        x_array = np.array(x)  # converts the state tuple x to an array

        if not((x_array >= self.statespace_lo).all() and (x_array <= self.statespace_hi).all()):
            return False
        
        x_grid_pt = (self.occupancy.to_grid(x_array) * self.scale).astype(int)
        idx = self.obs_grid[x_grid_pt[0], x_grid_pt[1]]
        
        return True
        '''
        if idx < 5:
            return True
        else:
            return False
            '''

    def obs_cost(self, x):
        x_array = np.array(x)  # converts the state tuple x to an array
        if np.linalg.norm(x_array - self.x_goal) <= self.obs_spacing:
            return 0
        x_grid_pt = (self.occupancy.to_grid(x_array) * self.scale).astype(int)
        idx = self.obs_grid[x_grid_pt[0], x_grid_pt[1]]
        return idx

    def distance(self, x1, x2):  
        return np.linalg.norm(np.array(x1) - np.array(x2))
        
    def snap_to_grid(self, x):
        return (self.resolution*round(x[0]/self.resolution), self.resolution*round(x[1]/self.resolution))

    def get_neighbors(self, x):
        neighbors = []
        # Compute 8 initial neighbors while snapping them to the grid
        neighbor1 = self.snap_to_grid((x[0] + self.resolution, x[1]))
        neighbor2 = self.snap_to_grid((x[0] + self.resolution, x[1] + self.resolution))
        neighbor3 = self.snap_to_grid((x[0], x[1] + self.resolution))
        neighbor4 = self.snap_to_grid((x[0] - self.resolution, x[1] + self.resolution))
        neighbor5 = self.snap_to_grid((x[0] - self.resolution, x[1]))
        neighbor6 = self.snap_to_grid((x[0] - self.resolution, x[1] - self.resolution))
        neighbor7 = self.snap_to_grid((x[0], x[1] - self.resolution))
        neighbor8 = self.snap_to_grid((x[0] + self.resolution, x[1] - self.resolution))
        initial_neighbors = [neighbor1, neighbor2,
                             neighbor3, neighbor4,
                             neighbor5, neighbor6,
                             neighbor7, neighbor8]

        # Check whether each neighbor is free, and if so, add them to 'neighbors'
        for initial_neighbor in initial_neighbors:
            if self.is_free(initial_neighbor):
                neighbors.append(initial_neighbor)
            else:
                pass

        return neighbors

    def find_best_est_cost_through(self):
        return min(self.open_set, key=lambda x: self.est_cost_through[x])

    def reconstruct_path(self):    
        path = [self.x_goal]
        current = path[-1]
        while current != self.x_init:
            path.append(self.came_from[current])
            current = path[-1]
        return list(reversed(path))

    def solve(self):
        while len(self.open_set) > 0:
            x_current = self.find_best_est_cost_through()
            if np.linalg.norm(np.array(x_current) - np.array(self.x_goal)) <= self.arm_reach:
                self.x_goal = x_current
                self.path = self.reconstruct_path()
                return True
            self.open_set.remove(x_current)
            self.closed_set.add(x_current)
            for x_neigh in self.get_neighbors(x_current):
                if x_neigh in self.closed_set:
                    continue
                tentative_cost_to_arrive = self.cost_to_arrive[x_current] + self.distance(x_current, x_neigh)
                tentative_cost_to_arrive += self.obs_cost(x_neigh)
                if x_neigh not in self.open_set:
                    self.open_set.add(x_neigh)
                elif tentative_cost_to_arrive > self.cost_to_arrive[x_neigh]:
                    continue
                self.came_from[x_neigh] = x_current
                self.cost_to_arrive[x_neigh] = tentative_cost_to_arrive
                self.est_cost_through[x_neigh] = tentative_cost_to_arrive + self.distance(x_neigh, self.x_goal)

        return False
