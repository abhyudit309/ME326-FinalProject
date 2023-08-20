# ME326: FinalProject

https://github.com/abhyudit309/ME326-FinalProject/assets/71165429/856a7d0d-83db-4cc6-8dfd-dd9d724524a8

[![Thumbnail](https://img.youtube.com/vi/F3Mx3NWDFKo/sddefault.jpg)](https://www.youtube.com/watch?v=F3Mx3NWDFKo)

## Overview

Our project revolves around the collaboration of two robots to gather resources without digital communication. The robots are given station locations and resource requests but are not told which request should go to each station. Each robot can only manipulate some of the block colors. The robots must collaborate to determine when and where to gather each resource and decide on a shared configuration despite the absence of digital communication.

## Problem Statement

This project involves the collaboration of two robots to gather resources without digital communication. During runtime, a configuration file will describe the number and configuration of the blocks, including the designated block colors for each team, and the number of blocks, by color, per station, but without specifying their location. The arena bounds will be described by a tag located at the arena center. The robots must collaborate to determine when and where to gather each resource, despite the absence of digital communication.

The rules for the task are as follows: Each block present at a station will earn 25 points, but every additional block at a station will deduct 2 points. Only one block color is allowed per station, and teams have the freedom to choose which resource type to use. A block can only be counted at one station. During gametime, only the designated team can add or remove blocks from a station. Each team has their own blocks, and moving the other team's blocks into a station will deduct 5 points, as well as additional points for any extra blocks. The game time is limited to 10 minutes, and both teams work simultaneously. Finally, teams cannot preemptively agree on strategy, such as going clockwise, to ensure fair play.

## System Architecture

An overview of the system architecture is shown below:

![image](https://i.imgur.com/nKLskFs.png)

### Conductor Module

The conductor module is a state machine that orchestrates the other systems and decides what the next task is. The first input is the occupancy grid, which represents the environment and identifies the locations of the blocks and other obstacles. The second input is the highest priority block move. The module then passes the relevant information to each subsystem. The conductor also orchestrates the state of the robot, telling the drive controller when and where to drive and the arm controller when and where to pick and place.

### Occupancy Grid Module

The occupancy grid module allows the Locobot to track observed objects in a world space grid. The main advantage of using an occupancy grid is that it enables negative sensing, which means that measurements, where a block is not seen in a location,n can lower the belief that a block is there. This would be much harder to implement if the detection happened in camera space. Additionally, the module can remember the state of blocks near the station, allowing computation of what blocks need to go where even if the station is not in line of sight. By taking the position-weighted average of neighboring cells, the module can get a good idea of where the center of a block is within a cell by fitting Gaussian distribution with OpenCV blob detection. This allows us to reduce the cell size for faster computation without losing accuracy when we go to pick up the block.

The module takes RGB, depth camera data, and Locobot pose as inputs and outputs the occupancy grid. The transformation between camera UV space and world space is done using vectorized numpy equations (no python for-loops) for speed. The occupancy grid classifies objects by color and by height. If an object falls into the red, yellow, green, or blue HSV ranges and is cube height (1-2 cm tall) it is classified as a cube. The height requirement was important for differentiating between the cubes and the colored tape on the floor of the lab. Objects taller than 3cm are classified as obstacles to be avoided.  The grid encompasses the playable area, with a grid resolution of approximately 1 cm. This resolution can be decreased for smaller blocks, but this will incur more computation time. 

### Path Planner Module

The Path Planner module utilizes the A-Star algorithm or the Rapidly-exploring Random Tree (RRT) algorithm in combination with the Occupancy Grid to plan the optimal path to the target location. The main goal of this module is to avoid any obstacles that may be present. 

The module takes in the target location, robot pose, and occupancy grid as inputs and outputs the desired path. Both A-Star and RRT use the occupancy grid to compute the shortest path to the target location while planning around potential obstacles. It defines safety zones to ensure that the bot does not collide with blocks or the environment during its travel. It then ensures that the planned trajectory does not intersect any of these zones.  The planner also takes into account the robot's current position and heading, and will continuously replan the path every few seconds to account for any moving obstacles (such as other robots) or errors in path following. 

### Station Tracker Module

The Station Tracker module is responsible for keeping track of the resources in each station, estimating the other robot’s belief, and determining the next resource that needs to be moved. By analyzing the occupancy grid, we can determine the count of each resource in every station. From these counts, our belief of their belief (a meta-belief if you will) can be estimated. The closer a station is to being in a particular state, the higher the likelihood that the other team has been placing blocks there, so our system will assign it a higher probability. The system will compute the expected score gain of each move with these probabilities and choose the move with the largest expected value per second. In other words, the move with the largest expected score gain (conditioned on the meta-belief) divided by the amount of time that it would take to complete the move. This way moves that require less effort but yield the same amount of points are prioritized. By approximating the other team’s belief using the board state, this method implicitly takes the initial state of the board into account. The initial state acts like a prior to the meta-belief; if we assume that the other robot is also a competent agent, we can guess that they are more likely to choose a configuration that is closer to being done. If the other robot decides to follow our lead, this prior also helps our robot carry out a solution with fewer moves, as configurations with closer station completions are assigned higher probabilities and thus higher expected scores.

The station tracker module takes in a configuration file, occupancy grid, and station identification (which may be provided by a collaborator) as inputs, and outputs the most important block move - i.e., which block needs to be moved next and where it needs to be moved from and to (outside the field, Station 1, Station 2, etc.). Once the module has determined the highest priority block move, it outputs it to the Conductor module, which then distributes this information to the other subsystems (such as the Path Planner, Drive Controller, and Arm Controller) to carry out the task.

### Arm Controller Module

The Arm Controller module receives arm commands from the conductor module and generates rostopic arm commands to be executed. It takes into consideration the kinematics of the robot arm and generates joint angles to achieve the desired end-effector pose.

### Drive Controller Module

The Drive Controller module takes in the desired path from the Path Planner and outputs the linear and angular velocities needed to follow that path. It uses the method from Homework One to control the Locobot’s movement and maintain this desired trajectory. The control point P is set 10cm in front of the center of the Locobot, such that the arm and gripper can easily place or pick up a block.
