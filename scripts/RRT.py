import numpy as np
from random import random
from occupancy_grid import OccupancyGrid

# Rapidly-exploring random tree (RRT) path planning algorithm 

# Algorithm BuildRRT Psuedocode from lecture
#    Input: Initial configuration qinit, number of vertices in RRT K, incremental distance Δq
#    Output: RRT graph G

#    G.init(qinit)
#    for k = 1 to K do
#        qrand ← RAND_CONF()
#        qnear ← NEAREST_VERTEX(qrand, G)
#        qnew ← NEW_CONF(qnear, qrand, Δq)
#        G.add_vertex(qnew)
#        G.add_edge(qnear, qnew)
#    return G

###
class Line():
    def __init__(self, pos1, pos2):
        self.pos1 = np.array(pos1)
        self.pos2 = np.array(pos2)
        self.dist = pos2 - pos1 #line vector
        self.dir = self.dist/np.linalg.norm(pos2 - pos1) #unit vector along direction

    def step(self, step_size):
        return self.pos1 + step_size * self.dir


class RRTGraph:
    def __init__(self, x_init, x_goal, resolution, upper, lower, occupancy, obs_grid, arm_reach, step_size):
        self.x_init = self.snap_to_grid(x_init)
        self.x_goal = self.snap_to_grid(x_goal)
        self.resolution = resolution
        self.upper = upper
        self.lower = lower
        self.arm_reach = arm_reach
        self.step_size = step_size

        self.occupancy = occupancy  # occupancy grid (a DetOccupancyGrid2D object)
        self.obs_grid = obs_grid.T  # obstacle grid

        self.closed_set = [self.x_init]
        self.open_set = []

        self.success = False

    def snap_to_grid(self, x):
        return (self.resolution * round(x[0]/self.resolution), self.resolution * round(x[1]/self.resolution))

    def randomPosition(self):
        randx = random.uniform(self.lower[0], self.upper[0])
        randy = random.uniform(self.lower[1], self.upper[1])

        return randx, randy

    def isObstructed(self, x):
        x_grid_pt = (self.occupancy.to_grid(np.array(x)) * self.scale).astype(int)
        obs_dist = self.obs_grid[x_grid_pt[0], x_grid_pt[1]]

        if obs_dist < self.arm_reach:
            return True
        else:
            return False

    def nearestPosition(self, randpos):
        nearpos = None
        minDist = 99999

        for pos in self.closed_set:
            line = Line(pos, randpos)
            if self.isPathObstructed(line):
                continue

            if line.dist < minDist:
                minDist = line.dist
                nearpos = pos

        return nearpos

    def isPathObstructed(self, line):
        for obstacle in self.obs_grid:
            if self.intersects(line, obstacle):
                return True
        return False

    
    def intersects(self, line, obstacle):
        a = line.dir.dot(line.dir)
        b = 2 * np.dot(line.dir, line.pos1 - obstacle)
        c = np.dot(line.pos1 - obstacle, line.pos1 - obstacle) - self.arm_reach * self.arm_reach

        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            return False

        t1 = (-b + np.sqrt(discriminant)) / (2 * a)
        t2 = (-b - np.sqrt(discriminant)) / (2 * a)

        if (t1 < 0 and t2 < 0) or (t1 > line.dist and t2 > line.dist):
            return False

        return True

    def newPos(self, nearpos, randpos):
        line = Line(nearpos,randpos)
        newpos = line.step(self.step_size)
        self.closed_set.append(newpos)#Add this new vertex to our graph
        #Find the distance from the nearest vertex to the goal vertex
        goal_dist = np.linalg.norm(np.array(newpos) - np.array(self.x_goal)) 
        if goal_dist < self.arm_reach: #If the new vertex is within a circle surrounding the goal state
            self.closed_set.append(self.x_goal) #Add the goal state to the graph
            self.success = True #Finish
        return newpos



def RRT(x_init, x_goal, resolution, upper, lower, occupancy, obs_grid, num_vertices, arm_reach, stepSize):

    G = RRTGraph(x_init, x_goal, resolution, upper, lower, occupancy, obs_grid, arm_reach, stepSize) #Initialize RRT graph with initial state and goal state?

    for i in range(num_vertices): #num_vertices should be the number of vertices in the graph
        randpos = G.randomPosition() #Pick a random vertex. I should pick from open states
        
        if G.isObstructed(randpos): #Check if the random vertex is in an obstacle
            continue #If it is, then continue

        nearpos = G.nearestPosition(randpos) #Find the nearest vertex in our graph to the random vertex
        
        if nearpos is None: #If there is no nearest vertex, then continue
            continue
        
        newpos = G.newPos(nearpos, randpos) #Move towards the random vertex from the nearest vertex by an incremental step

        #Add the distance from the new vertex to the goal state to the graph
        if(G.success == True):
            break

    return G
