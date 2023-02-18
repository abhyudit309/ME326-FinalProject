#!/usr/bin/env python3

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


class PixelCloudMatcher:
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

        self.uv_pix = [0,0] #find the pixel index

        self.camera_cube_locator_marker = rospy.Publisher("/locobot/camera_cube_locator",Marker, queue_size=1)

        self.point_3d_cloud = PointStamped()
       
        self.info_sub = rospy.Subscriber(self.depth_img_camera_info, CameraInfo, self.info_callback)
        
        self.depth_sub = rospy.Subscriber(self.depth_image_topic, Image, self.depth_callback)
        self.image_sub = rospy.Subscriber(self.color_image_topic, Image, self.color_image_callback)

        self.colors = None # r = 1, y = 2, g = 3, b = 4
        self.depth_image = None

        self.cube_size = 0.08
     

    def camera_cube_locator_marker_gen(self):
        #this is very simple because we are just putting the point P in the base_link frame (it is static in this frame)
        marker = Marker()
        self.thread_lock.acquire()
        marker.header.frame_id = self.point_3d_cloud.header.frame_id #"locobot/camera_depth_link"
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
        marker.pose.position.x = self.point_3d_cloud.point.x
        marker.pose.position.y = self.point_3d_cloud.point.y
        marker.pose.position.z = self.point_3d_cloud.point.z
        self.thread_lock.release()
        # Set the marker color
        marker.color.a = 1.0 #transparency
        marker.color.r = 1.0 #red
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Publish the marker
        self.camera_cube_locator_marker.publish(marker)


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

        colors = red_mask + 2 * yellow_mask + 3 * green_mask + 4 * blue_mask

        self.thread_lock.acquire()
        self.colors = colors
        self.thread_lock.release()

        self.get_cubes()

    def depth_callback(self, depth_msg):
        # convert depth image message to a numpy array

        depth_image = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(depth_msg.height, depth_msg.width)

        self.thread_lock.acquire()
        self.depth_image = depth_image
        self.thread_lock.release()


    def get_cubes(self):
        if (self.colors is None or self.depth_image is None):
            return; #both cameras have not beet read yet

        v,u = np.meshgrid(np.arange(self.colors.shape[0]), np.arange(self.colors.shape[1]), indexing='ij')
        print("u: ", u)
        print("v: ", v)
        K = self.camera_model.intrinsicMatrix()
        cx = K[0,2]
        cy = K[1,2]
        fov_x = K[0,0]
        fov_y = K[1,1]
        rays = np.dstack(((u - cx)/fov_x, (v - cy)/fov_y, np.ones_like(u)))
        rays = rays / np.linalg.norm(rays, axis = 2)[..., np.newaxis]
        print(rays)
        
        points = rays * self.depth_image

        color_tagged_points = np.concatenate((points, self.colors), axis=2)


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
    matcher = PixelCloudMatcher()

    '''rospy.init_node("pixel_cloud_matcher_service")
    matcher = PixelCloudMatcher()
    s = rospy.Service('pix_to_point',PixtoPoint,matcher.service_callback)'''
    rospy.spin()
