import rospy
import numpy as np
import scipy as sp
from scipy import linalg
import geometry_msgs

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

from nav_msgs.msg import Odometry

from visualization_msgs.msg import Marker

class BlockTracker(object):
    def __init__(self):
        self.blocks = []
        self.K = np.identity(3)



    def pub_markers(self):

        for i in range(len(blocks)):
            block = blocks[i]
            marker = Marker()
            marker.header.frame_id = "Block #" + i
            marker.header.stamp = rospy.Time.now()
            marker.id = i
            marker.type = Marker.SPHERE 
            # Set the marker scale
            marker.scale.x = 0.1  # radius of the sphere
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            # Set the marker pose
            marker.pose.position.x = block.lastPos
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0

            # Set the marker color
            marker.color.a = 1.0 #transparency

            if block.color == 'r'
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif block.color == 'y'
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif block.color == 'g'
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif block.color == 'b'
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            else:
                print("Color " + block.color + " not implemented")

            # Publish the marker
            self.point_P_control_point_visual.publish(marker)

class Block():
    self.errorDistance = 0.1 #m
    self.gamma = 0.9 #Time discounting (1/s)
    self.maxSamples = 100 #number of positions held

    def __init__(self, pos, orientation, color):
        time = #rospy time
        self.positions = np.array([pos])
        self.orientations = np.array([orientation])
        self.times = np.array([time])
        self.color = color

    def GetEstPose(self):
        time =  #rospy time
        timeDiscounted = np.power(self.gamma, time - self.times)
        estPos = numpy.average(self.positions, weights=timeDiscounted)
        estOrientation = numpy.average(self.orientations, weights=timeDiscounted) #may not work bc of wrapping
        return estPos, estOrientation


    def PossibleObservation(self, pos, orientation, color):
        time = #rospy time
        if (color != self.color):
            return false
        
        estPos, estOrientation = self.GetEstPose()

        if (np.linalg.norm(estPos - pos) > self.errorDistance):
            return false
        
        #Object found

        self.positions = numpy.concatenate([pos], self.positions, axis = 0)
        self.orientations = numpy.concatenate([pos], self.orientations, axis = 0)
        self.times = numpy.concatenate([time], self.times, axis = 0)

        if (self.positions.shape[0] > self.maxSamples):
            self.positions = self.positions[0:self.maxSamples,:]
            self.orientations = self.orientations[0:self.maxSamples,:]
            self.times = self.times[0:self.maxSamples,:]

        return true
