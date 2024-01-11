"""
Detect the patholes using haar cascade
Calculate distance and coordinates relative to camera
Get the robot camera coordination from the TF
Calculate the x and y coordinates of each potholes relative to odom frame
Use marker to publish detected coordinates
"""
# Python libs
import rclpy
from rclpy.node import Node
from rclpy import qos

# Numpy
import numpy as np

# OpenCV
import cv2
from cv2 import namedWindow, waitKey
from tf2_ros import Buffer, TransformListener, TransformException

import math

# ROS libraries
import image_geometry
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from tf2_geometry_msgs import do_transform_pose
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion

# Import trained haar cascade model
cascade = cv2.CascadeClassifier('cascade.xml')

font = cv2.FONT_HERSHEY_SIMPLEX

class PotholeDetector(Node):

    object_id_counter = 0  # Class variable for persistent IDs
    camera_model = None
    image_depth_ros = None
    visualisation = True
    color2depth_aspect = 1.0  

    def __init__(self):    
        super().__init__('pothole_detector_real')
        self.bridge = CvBridge()

        self.camera_info_sub = self.create_subscription(CameraInfo, '/limo/depth_camera_link/camera_info', self.camera_info_callback, qos_profile=qos.qos_profile_sensor_data)
        self.image_sub = self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.image_color_callback, qos_profile=qos.qos_profile_sensor_data)
        self.image_sub = self.create_subscription(Image, '/limo/depth_camera_link/depth/image_raw', self.image_depth_callback, qos_profile=qos.qos_profile_sensor_data)
        self.image_pub = self.create_publisher(Image, '/limo/depth_camera_link/image_detect', 10)   # Create a publisher to publish pothole marked image
        self.marker_pub = self.create_publisher(Marker, '/marker', 1)                               # Create a Marker publisher to publish detected pothole coordinates

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.point_id = 0

    def get_tf_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return None

    def camera_info_callback(self, data):
        if not self.camera_model:
            self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)

    def image_depth_callback(self, data):
        self.image_depth_ros = data

    def image_color_callback(self, data):
       
        # wait for camera_model and depth image to arrive
        if self.camera_model is None:
            return

        if self.image_depth_ros is None:
            return

        # covert images to open_cv
        try:
            image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            print(e)

        # Convert BGR color to HSV
        hsv = cv2.cvtColor(image_color, cv2.COLOR_BGR2HSV)

        def search_contours(hsv):
            # Detect the objects using the model
            detect = cascade.detectMultiScale(hsv, 1.2, 4, maxSize=(60, 60))

            for (x, y, w, h) in detect:
                # Draw rectangle around the detected objects
                cv2.rectangle(image_color, (x, y), (x+w,y+h), (0, 0, 255), 1)
                # Get the centroid
                cx = int((x + x + w)/2)
                cy = int((y + y + h)/2)
                image_coords = (cx, cy)

                # Map from color image to depth image
                depth_coords = (image_depth.shape[0]/2 + (image_coords[0] - image_color.shape[0]/2)*self.color2depth_aspect, image_depth.shape[1]/2 + (image_coords[1] - image_color.shape[1]/2)*self.color2depth_aspect)
                
                # Get the depth reading at the pathole's centroid locations
                depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])] 

                # Calculate pothole's's 3d location in camera coords
                camera_coords = self.camera_model.projectPixelTo3dRay((image_coords[1], image_coords[0]))   # Project the image coords (x,y) into 3D ray in camera coords
                camera_coords = [x/camera_coords[2] for x in camera_coords]                                 # Adjust the resulting vector so that z = 1
                camera_coords = [x*depth_value for x in camera_coords]                                      # Multiply the vector by depth

                # Get depth_links coordinates relative to odom frame from TF
                trans = None
                try:
                    now = rclpy.time.Time()
                    trans = self.tf_buffer.lookup_transform('odom', 'depth_link', now)
                except TransformException as ex:
                    self.get_logger().info(f'Could not transform odom to depth_link: {ex}')
                    return
                pos_x = trans.transform.translation.x
                pos_y = trans.transform.translation.y
                pos_rx = trans.transform.rotation.x
                pos_ry = trans.transform.rotation.y
                pos_rz = trans.transform.rotation.z
                pos_rw = trans.transform.rotation.w

                # Convert quaternion values to roll, pitch and yaw
                (roll, pitch, yaw) = euler_from_quaternion([pos_rx, pos_ry, pos_rz, pos_rw])
        
                # Compute transformation matrix elements
                cos_theta = math.cos(yaw)
                sin_theta = math.sin(yaw)
                
                # Calculate the pothole's coordinates relative to the odom position
                x_coords = pos_x + camera_coords[0] * cos_theta - camera_coords[2] * sin_theta
                y_coords = pos_y + camera_coords[0] * sin_theta + camera_coords[2] * cos_theta

                # print('Coords : ', x_coords, y_coords)

                # Publish detected points of patholes using Marker
                marker = Marker()
                marker.header.frame_id = 'odom'  
                marker.id = self.point_id
                marker.type = 2
                marker.action = Marker.ADD
                marker.lifetime.sec = 0
                marker.pose.position.x = x_coords
                marker.pose.position.y = y_coords
                marker.pose.position.z = 0.0
                marker.pose.orientation.w = 0.0
                marker.scale.x = 0.03
                marker.scale.y = 0.03
                marker.scale.z = 0.03
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0

                self.marker_pub.publish(marker)
                self.point_id += 1

        search_contours(hsv)

        #cv2.imshow("image color", image_color)

        # Pulish image with contors
        image_detect = self.bridge.cv2_to_imgmsg(image_color, "bgr8")
        self.image_pub.publish(image_detect)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    pothole_detector = PotholeDetector()
    rclpy.spin(pothole_detector)
    pothole_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()