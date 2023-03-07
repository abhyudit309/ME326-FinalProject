#!/usr/bin/env python3


import time
import rospy
import tf
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import traceback
import sys

from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import threading

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from matplotlib import pyplot as plt

from me326_locobot_example.srv import PixtoPoint, PixtoPointResponse


class OccupancyGrid:
    def __init__(self, run_on_robot):
        print("Occupancy Grid Starting")
        self.run_on_robot = run_on_robot

        if self.run_on_robot:
            rospy.Subscriber("/camera_frame/mavros/vision_pose/pose", PoseStamped, self.OdometryCallback)
            self.base_frame = 'locobot/base_footprint'
        else:
            self.base_frame = 'locobot/odom'

        self.real_world_matrix = np.identity(4)

        self.bridge = CvBridge()
        self.thread_lock = threading.Lock() #threading # self.thread_lock.acquire() # self.thread_lock.release()

        #Set the image topics from param server: http://wiki.ros.org/rospy/Overview/Parameter%20Server 
        self.color_image_topic  = rospy.get_param('pt_srv_color_img_topic', '/locobot/camera/color/image_raw')
        '''if self.run_on_robot:
            self.depth_d_size = np.uint16
        else:
            self.depth_d_size = np.float32'''
        self.depth_image_topic = rospy.get_param('pt_srv_depth_img_topic', '/locobot/camera/aligned_depth_to_color/image_raw')

        self.depth_img_camera_info = rospy.get_param('pt_srv_depth_img_cam_info_topic', '/locobot/camera/aligned_depth_to_color/camera_info')
    
        self.image_color_filt_pub = rospy.Publisher("/locobot/camera/block_color_filt_img",Image,queue_size=1,latch=True)

        # create a tf listener
        self.listener = tf.TransformListener()


        self.cube_size = 0.02 #m
        self.grid_size = 0.005 #m
        
        self.field_size = 4.5 #m
        self.obs_height = 0.03 #m

        grid_dim = int(self.field_size / self.grid_size)
        self.grid_center = np.rint([grid_dim / 2,grid_dim / 2])
        self.grid = np.zeros((grid_dim, grid_dim, 6)) #[blank, red, yellow, green, blue, obs]


        self.recency_bias = 0.3
        self.i = 0

        self.do_scan = True
       
        self.info_sub = rospy.Subscriber(self.depth_img_camera_info, CameraInfo, self.info_callback, queue_size=1)
        
        self.depth_sub = rospy.Subscriber(self.depth_image_topic, Image, self.depth_callback, queue_size=1, buff_size=2**24)
        self.image_sub = rospy.Subscriber(self.color_image_topic, Image, self.color_image_callback, queue_size=1, buff_size=2**24)

        self.color_time = 0
        self.colors = None 
        self.depth_image = None

        self.last_update_time = 0
     

    def display_occupancy(self):
        red = np.array([1,0,0])
        yellow = np.array([0.878, 0.722, 0])
        green = np.array([0,1,0])
        blue = np.array([0,0,1])
        white = np.array([1,1,1])
        obs = np.array([0.463,0,0.769])
        self.thread_lock.acquire()
        gridEA = self.grid[..., np.newaxis]
        self.thread_lock.release()

        count = np.sum(self.grid, axis = 2)[..., np.newaxis]

        imgRGB = (gridEA[:,:,0] * white 
            + gridEA[:,:,1] * red
            + gridEA[:,:,2] * yellow
            + gridEA[:,:,3] * green
            + gridEA[:,:,4] * blue
            + gridEA[:,:,5] * obs) / np.where(count > 0, count, 1)
        
        '''red_ind = np.unravel_index(np.argmax(imgRGB[:,:,1]),imgRGB.shape[0:2])
        print("max red index:", red_ind)
        print("max red:", imgRGB[red_ind[0],red_ind[1],:])'''

        plt.imshow(imgRGB, origin='lower')
        plt.show(block=False)
        plt.pause(0.001)

    def color_image_callback(self,color_msg):
        if not self.do_scan:
            return
        color_img = self.bridge.imgmsg_to_cv2(color_msg, "rgb8")
        #print("Occupancy Grid Recieved Color image:", color_img.shape), 
            #"\n Red \n", color_img[:,:,0],
            #"\n Green \n", color_img[:,:,1],
            #"\n Blue \n", color_img[:,:,2])
        
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
        self.color_time = color_msg.header.stamp
        self.thread_lock.release()
        '''
        print(" ")
        print("Now:", rospy.Time())
        print("Image time:", color_msg.header.stamp)'''

        self.scan()

    def depth_callback(self, depth_msg):
        # convert depth image message to a numpy array
        try:
            depth_image_cv2 = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)
        depth_image = np.array(depth_image_cv2, dtype=np.float32)

        if self.run_on_robot:
            depth_image = depth_image / 1000
            
        self.thread_lock.acquire()
        self.depth_image = depth_image
        self.thread_lock.release()


    def scan(self):
        #print("scanning", self.colors, self.depth_image)
        if (self.colors is None or self.depth_image is None):
            return; #both cameras have not beet read yet
        #print("pass if")

        v,u = np.meshgrid(np.arange(self.colors.shape[0]), np.arange(self.colors.shape[1]), indexing='ij')
        K = self.camera_model.intrinsicMatrix()
        cx = K[0,2]
        cy = K[1,2]
        fov_x = K[0,0]
        fov_y = K[1,1]
        rays = np.dstack((np.ones_like(u), (cx - u)/fov_x, (cy - v)/fov_y))
        #rays = rays / np.linalg.norm(rays, axis = 2)[..., np.newaxis] # Why does it not want to be normalized?
        
        points = rays * self.depth_image[..., np.newaxis]
        point4s = np.dstack((points,np.ones_like(u)))[..., np.newaxis]
        matrix4x4 = np.zeros((4,4))
        
        try:    
            if self.run_on_robot:
                last_tf_time = self.listener.getLatestCommonTime(self.base_frame,'locobot/camera_link')
                (translation, rot) = self.listener.lookupTransform(self.base_frame,'locobot/camera_link', last_tf_time)
                matrix4x4 = tf.transformations.compose_matrix(translate=translation, angles=tf.transformations.euler_from_quaternion(rot))
                matrix4x4 = self.real_world_matrix @ matrix4x4 #Apply real world pose
            else:
                (translation, rot) = self.listener.lookupTransform(self.base_frame,'locobot/camera_link', self.color_time)
                matrix4x4 = tf.transformations.compose_matrix(translate=translation, angles=tf.transformations.euler_from_quaternion(rot))


        except:
            print(traceback.format_exc())
            print("ERROR IN OG TRY")
            pass
        
        #print("Camera Pose Matrix 4x4: \n", np.around(matrix4x4, 2))
        worldPoints = np.matmul(matrix4x4, point4s)[:, :, :3, 0]
        worldPoints = worldPoints[..., [1,0,2]] #swap x,y

        gridPoints = self.to_grid(worldPoints)
        out_of_bounds = (np.greater_equal(gridPoints[:,:,0], self.grid.shape[0])
        + np.greater_equal(gridPoints[:,:,1], self.grid.shape[1])
        + np.less(gridPoints[:,:,0], 0)
        + np.less(gridPoints[:,:,1], 0))[..., np.newaxis]

        gridPoints = np.where(out_of_bounds > 0, np.array([0,0]), gridPoints)
        tall = np.where(worldPoints[:,:,2] < self.obs_height, 0,1) # obs or blank depending on height
        
        self.thread_lock.acquire()

        # self.colors[:,:,0:5] *= (1-tall)
        self.colors[:,:,0:5] *= (1-tall[:, :, np.newaxis])
        self.colors[:,:,5] *= tall
        
        self.grid[gridPoints[:,:,0],gridPoints[:,:,1],:] = self.colors + self.recency_bias * self.grid[gridPoints[:,:,0],gridPoints[:,:,1],:]
        self.grid[0,0] = np.array([1,0,0,0,0,0])
        self.i += 1
        self.thread_lock.release()

        now = rospy.Time.now().to_sec()
        #print("dt:", now - self.last_update_time)
        self.thread_lock.acquire()
        self.last_update_time = now
        self.thread_lock.release()

        #print(np.round(self.grid[550-3:550+3,400-3:400+3,0] * 100) / 100)
        #print(np.round(self.grid[550-3:550+3,400-3:400+3,4] * 100) / 100)

        '''red_ind = np.unravel_index(np.argmax(self.grid[:,:,1]),self.grid.shape[0:2])
        print("max red index:", red_ind)
        print("max red:", self.grid[red_ind[0],red_ind[1],:])'''

        '''if (self.i % 10 == 0):
            self.display_occupancy()'''


    def to_grid(self,points):
        return np.rint(points[..., :2] / self.grid_size + self.grid_center).astype(int) #ignore height

    def to_world(self,grid_points):
        return ((grid_points - self.grid_center) * self.grid_size)

    def info_callback(self, info_msg):
        # create a camera model from the camera info
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(info_msg)
        #print("Occupancy Grid Recieved Camera Info:\n", self.camera_model)

    def service_callback(self,req):
        #this function takes in a requested topic for the image, and returns pixel point
        #now call the other subscribers
        resp = PixtoPointResponse()
        resp.ptCld_point = self.point_3d_cloud
        return resp

    def OdometryCallback(self, data):
        if self.run_on_robot:
            translation = data.pose.position
            translation = [translation.x, translation.y, translation.z]
            rot = data.pose.orientation
            rot = [rot.x, rot.y, rot.z, rot.w]
        else: 
            translation = data.pose.pose.position
            rot = data.pose.pose.orientation
        self.thread_lock.acquire()
        self.real_world_matrix = tf.transformations.compose_matrix(translate=translation, angles=tf.transformations.euler_from_quaternion(rot))
        self.thread_lock.release()
        #print("Occupancy Grid Recieved Robot Position:\n", translation)
        #print("Occupancy Grid Recieved Robot Orientation:\n", rot)
