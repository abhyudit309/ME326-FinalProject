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

    def camera_info_callback(self, data):
        self.K = data.K

    def camera_callback(self, data):
        
        img = data

    def mask_img(color_img, color_mask='r'):

        hsv_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2HSV)

        #Step 2: prep the mask
        if (color_mask == 'r'):
          lower_bound = np.array([0,128,10])
          upper_bound = np.array([8,256,256])
        elif (color_mask == 'y'):
          lower_bound = np.array([23,128,10])
          upper_bound = np.array([33,256,256])
        elif (color_mask == 'g'):
          lower_bound = np.array([33,60,10])
          upper_bound = np.array([90,256,256])
        elif (color_mask == 'b'):
          lower_bound = np.array([100,200,10])
          upper_bound = np.array([130,256,256])
        else:
          print("Color '" + color_mask + "' not implemented")
          lower_bound = np.array([0,0,0])
          upper_bound = np.array([0,0,0])

        mask = cv2.inRange(hsv_img, lower_bound, upper_bound)
        
        if (color_mask == 'r'):
          lower_bound = np.array([167,128,128])
          upper_bound = np.array([180,255,255])
          other_side_red_mask = cv2.inRange(hsv_img, lower_bound, upper_bound)
          mask = mask + other_side_red_mask
        #Step 3: Apply the mask; black region in the mask is 0, so when multiplied with original image removes all non-selected color 
        mask_img = cv2.bitwise_and(color_img, color_img, mask = mask)
        return mask_img


class Block():
    def __init__(self, pos, orientation, color):
        self.lastPos = pos
        self.lastOrientation = orientation
        self.color = color