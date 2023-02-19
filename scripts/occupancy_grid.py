#!/usr/bin/env python3

import time
import rospy
import tf
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import threading

from me326_locobot_example.srv import PixtoPoint, PixtoPointResponse


class OccupancyGrid:
    def __init__(self):
        self.bridge = CvBridge()
        self.thread_lock = threading.Lock() #threading # self.thread_lock.acquire() # self.thread_lock.release()

        #Set the image topics from param server: http://wiki.ros.org/rospy/Overview/Parameter%20Server 
        self.color_image_topic  = rospy.get_param('pt_srv_color_img_topic', '/locobot/camera/color/image_raw')
        self.depth_image_topic = rospy.get_param('pt_srv_depth_img_topic', '/locobot/camera/aligned_depth_to_color/image_raw')
        self.depth_img_camera_info = rospy.get_param('pt_srv_depth_img_cam_info_topic', '/locobot/camera/aligned_depth_to_color/camera_info')
    
        self.image_color_filt_pub = rospy.Publisher("/locobot/camera/block_color_filt_img",Image,queue_size=1,latch=True)

        # create a tf listener
        self.listener = tf.TransformListener()


        self.cube_size = 0.02 #m
        self.grid_size = 0.01 #m
        self.field_size = 8 #m
        self.obs_height = 0.04 #m

        grid_dim = int(self.field_size / self.grid_size)
        self.grid_center = np.array([int(self.grid_dim / 2),int(self.grid_dim / 2)])
        self.grid = np.zeros((grid_dim, grid_dim, 6)) #[blank, red, yellow, green, blue, obs]


        self recency_bias = 0.8

        '''
        self.camera_cube_locator_marker = rospy.Publisher("/locobot/camera_cube_locator",Marker, queue_size=1)
        self.camera_cube_locator_marker1 = rospy.Publisher("/locobot/camera_cube_locator1",Marker, queue_size=1)
        self.camera_cube_locator_marker2 = rospy.Publisher("/locobot/camera_cube_locator2",Marker, queue_size=1)

        
        self.point = np.zeros(3)
        self.point1 = np.zeros(3)
        self.point2 = np.zeros(3)
        '''
       
        self.info_sub = rospy.Subscriber(self.depth_img_camera_info, CameraInfo, self.info_callback)
        
        self.depth_sub = rospy.Subscriber(self.depth_image_topic, Image, self.depth_callback)
        self.image_sub = rospy.Subscriber(self.color_image_topic, Image, self.color_image_callback)

        self.colors = None # r = 1, y = 2, g = 3, b = 4
        self.depth_image = None
     

    def camera_cube_locator_marker_gen(self):
        marker = Marker()
        self.thread_lock.acquire()
        marker.header.frame_id = 'locobot/odom' #/locobot/camera_cube_locator
        self.thread_lock.release()
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.type = Marker.SPHERE
        # Set the marker scale
        marker.scale.x = 0.05  # radius of the sphere
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        # Set the marker pose
        self.thread_lock.acquire()
        marker.pose.position.x = self.point[0]
        marker.pose.position.y = self.point[1]
        marker.pose.position.z = self.point[2]
        self.thread_lock.release()
        # Set the marker color
        marker.color.a = 1.0 #transparency
        marker.color.r = 1.0 #red
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Publish the marker
        self.camera_cube_locator_marker.publish(marker)

        # Set the marker pose
        self.thread_lock.acquire()
        marker.pose.position.x = self.point1[0]
        marker.pose.position.y = self.point1[1]
        marker.pose.position.z = self.point1[2]
        self.thread_lock.release()
        # Set the marker color
        marker.color.a = 1.0 #transparency
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.camera_cube_locator_marker1.publish(marker)

        # Set the marker pose
        self.thread_lock.acquire()
        marker.pose.position.x = self.point2[0]
        marker.pose.position.y = self.point2[1]
        marker.pose.position.z = self.point2[2]
        self.thread_lock.release()
        # Set the marker color
        marker.color.a = 1.0 #transparency
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        self.camera_cube_locator_marker2.publish(marker)

    def color_image_callback(self,color_msg):

        color_img = self.bridge.imgmsg_to_cv2(color_msg, "rgb8")
        
        
        hsv = cv2.cvtColor(color_img, cv2.COLOR_RGB2HSV)

        # Red
        lower_bound = np.array([0, 100, 20])
        upper_bound = np.array([8, 255, 255])

        red_mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Yellow
        lower_bound = np.array([23,100,10])
        upper_bound = np.array([33,256,256])

        yellow_mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Green
        lower_bound = np.array([33,100,10])
        upper_bound = np.array([90,256,256])

        green_mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Blue
        lower_bound = np.array([100,100,10])
        upper_bound = np.array([130,256,256])

        blue_mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Upper Red
        lower_bound = np.array([167,100,128])
        upper_bound = np.array([180,255,255])

        red_mask += cv2.inRange(hsv, lower_bound, upper_bound)

        white_mask = np.where(red_mask + yellow_mask + green_mask + blue_mask < 0.5, 1, 0)

        colors = np.dstack((white_mask, red_mask, yellow_mask, green_mask, blue_mask, white_mask))

        self.thread_lock.acquire()
        self.colors = colors
        self.thread_lock.release()
            

        self.scan()

    def depth_callback(self, depth_msg):
        # convert depth image message to a numpy array

        depth_image = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(depth_msg.height, depth_msg.width)

        self.thread_lock.acquire()
        self.depth_image = depth_image
        self.thread_lock.release()


    def scan(self):
        if (self.colors is None or self.depth_image is None):
            return; #both cameras have not beet read yet

        v,u = np.meshgrid(np.arange(self.colors.shape[0]), np.arange(self.colors.shape[1]), indexing='ij')
        K = self.camera_model.intrinsicMatrix()
        cx = K[0,2]
        cy = K[1,2]
        fov_x = K[0,0]
        fov_y = K[1,1]
        rays = np.dstack((np.ones_like(u), (cx - u)/fov_x, (cy - v)/fov_y))
        #rays = rays / np.linalg.norm(rays, axis = 2)[..., np.newaxis] # Why does it not want to be normalized?
        '''print("center ray: ",rays[int(rays.shape[0]/2),int(rays.shape[1]/2),:])
        print("first ray: ",rays[0,0,:])
        print("last ray: ",rays[-1,-1,:])
        print("center depth: ",self.depth_image[int(self.depth_image.shape[0]/2),int(self.depth_image.shape[1]/2)])
        print("first depth: ",self.depth_image[0,0])
        print("last depth: ",self.depth_image[-1,-1])'''
        
        points = rays * self.depth_image[..., np.newaxis]
        point4s = np.dstack((points,np.ones_like(u)))[..., np.newaxis]
        matrix4x4 = np.zeros((4,4))
        try:
            (trans, rot) = self.listener.lookupTransform('locobot/odom','locobot/camera_link', rospy.Time())
            print("Translation: ", trans)
            matrix4x4 = tf.transformations.compose_matrix(translate=trans, angles=tf.transformations.euler_from_quaternion(rot))

        except (tf.LookupException, tf.ConnectivityException):
            pass
        print(matrix4x4)
        worldPoints = np.matmul(matrix4x4, point4s)[:, :, :3]

        gridPoints = np.round(worldPoints[:, :, :2] / self.grid_size) + self.grid_center #ignore height
        obj_type = colors * np.where(worldPoints[:,:,2] < self.obs_height, np.array([1,1,1,1,1,0]),np.array([0,1,1,1,1,1])) # obs or blank depending on height
        self.grid[gridPoints[:,:,0],gridPoints[:,:,1]] = obj_type + self.recency_bias * self.grid[gridPoints[:,:,0],gridPoints[:,:,1]]

        '''print("center WP:\n",worldPoints[int(rays.shape[0]/2),int(rays.shape[1]/2),:])
        print("first WP:\n",worldPoints[0,0,:])
        print("last WP:\n",worldPoints[-1,-1,:])
        #print("WP:\n",worldPoints)
        #color_tagged_points = np.concatenate((points, self.colors), axis=2)
        #time.sleep(1)
        centerWP = worldPoints[int(rays.shape[0]/2),int(rays.shape[1]/2),:]
        self.point[0] = centerWP[0,0]
        self.point[1] = centerWP[1,0]
        self.point[2] = centerWP[2,0]

        firstWP = worldPoints[0,0,:]
        self.point1[0] = firstWP[0,0]
        self.point1[1] = firstWP[1,0]
        self.point1[2] = firstWP[2,0]

        lastWP = worldPoints[-1,-1,:]
        self.point2[0] = lastWP[0,0]
        self.point2[1] = lastWP[1,0]
        self.point2[2] = lastWP[2,0]

        self.camera_cube_locator_marker_gen()'''


    def info_callback(self, info_msg):
        # create a camera model from the camera info
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(info_msg)

    def service_callback(self,req):
        #this function takes in a requested topic for the image, and returns pixel point
        #now call the other subscribers
        resp = PixtoPointResponse()
        resp.ptCld_point = self.point_3d_cloud
        return resp

if __name__ == "__main__":
    
    rospy.init_node("block_finder")
    occupancy_grid = OccupancyGrid()

    '''rospy.init_node("pixel_cloud_matcher_service")
    matcher = PixelCloudMatcher()
    s = rospy.Service('pix_to_point',PixtoPoint,matcher.service_callback)'''
    rospy.spin()
