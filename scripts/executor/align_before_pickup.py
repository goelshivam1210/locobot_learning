#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from tf.transformations import euler_from_quaternion
from locobot_learning.srv import AlignRobot, AlignRobotResponse

class AlignBeforePickup(object):
    def __init__(self):
        # Initialization and other methods remain the same
        self.service = rospy.Service('align_before_pickup', AlignRobot, self.align_service_callback)
        
        # Initialize the CV Bridge
        self.bridge = CvBridge()
        
        # Initialize the color codes
        self.color_codes = {
            "red": ((12, 33, 208), "red_ball"),
            "green": ((0, 255, 220), "green_ball"),
            "orange":((0, 150, 255), "orange_marker"),
            "yellow":((0, 209, 243), "yellow_marker")
        }
        
        # Initialize object_positions dictionary
        self.object_positions = {v[1]: None for _, v in self.color_codes.items()}
        
        # Initialize the tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Initialize the RGB image subscriber
        self.rgb_image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_image_callback)
        
        # Initialize the Depth image subscriber
        self.depth_image_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_image_callback)
        
        # Initialize the publisher for sending movement commands to the robot base
        self.cmd_vel_publisher = rospy.Publisher('/mobile_base/cmd_vel', Twist, queue_size=10)
        
        # # Initialize the service for aligning and picking up objects
        # self.service = rospy.Service('align_and_pickup', Trigger, self.service_callback)  # Replace with your custom service type
        
        # rospy.spin()

    def align_service_callback(self, req):
        object_name = req.object_name
        success = self.align_to_object(object_name)
        if success:
            return AlignRobotResponse(success=True, message='Alignment successful')
        else:
            return AlignRobotResponse(success=False, message='Alignment failed, object not found')

    def rgb_image_callback(self, msg):
        """
        Callback function for the RGB image subscriber. 
        It converts the ROS image message to an OpenCV image and detects colored objects.
        """

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.detect_color_objects(cv_image)
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")

    def depth_image_callback(self, msg):
        """
        Callback function for the depth image subscriber.
        It converts the ROS image message to an OpenCV image and stores it for later use.
        """

        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")
    
    def detect_color_objects(self, cv_image):
        """
        Detects colored objects in the RGB image, computes their centroids, and transforms their positions 
        to the map frame using the corresponding depth information.
        """

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        fx, fy = 384.845458984375, 384.845458984375  # Extracted focal lengths in x and y
        cx, cy = 321.1991882324219, 238.43983459472656  # Extracted principal points in x and y
        
        for color, (bgr, object_name) in self.color_codes.items():
            lower_bound = np.array(bgr) - 40
            upper_bound = np.array(bgr) + 40
            mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                cxi, cyi = np.mean(contour, axis=0)[0]  # pixel coordinates
                depth = self.depth_image[int(cyi), int(cxi)]/1000. # convert mm into m

                # rospy.loginfo(f"Depth at ({cxi}, {cyi}): {depth}")
                
                if not np.isfinite(depth) or depth <= 0:
                    continue  # Skip invalid depth values
                
                point_camera_frame = PointStamped()
                point_camera_frame.header.frame_id = 'locobot/camera_tower_link'
                
                point_camera_frame.point.x = (cxi - cx) * depth / fx
                point_camera_frame.point.y = (cyi - cy) * depth / fy
                point_camera_frame.point.z = depth
                # rospy.loginfo(f"3D Position in Camera Frame: X: {point_camera_frame.point.x}, Y: {point_camera_frame.point.y}, Z: {point_camera_frame.point.z}")
                
                try:
                    point_map_frame = self.tf_buffer.transform(point_camera_frame, 'map', timeout=rospy.Duration(1.0))
                    self.object_positions[object_name] = point_map_frame.point
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logerr(f"Error transforming point: {e}")
                
    def align_to_object(self, object_name):
        tolerance = 0.05  # Define the alignment tolerance in radians
        gain = 0.5  # Define the proportional gain for the rotation command
        
        robot_position, robot_orientation = self.get_robot_pose_orientation()
        _, _, theta_r = euler_from_quaternion(robot_orientation)
        
        object_position = self.object_positions.get(object_name)
        if object_position is None:
            rospy.logerr(f"Object {object_name} not found.")
            return False
        
        x_o, y_o = object_position.x, object_position.y
        theta_d = np.arctan2(y_o - robot_position[1], x_o - robot_position[0])
        
        delta_theta = theta_d - theta_r
        delta_theta = (delta_theta + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]
        
        twist_cmd = Twist()
        while np.abs(delta_theta) > tolerance:
            twist_cmd.angular.z = delta_theta * gain
            self.cmd_vel_publisher.publish(twist_cmd)
            
            _, _, theta_r = euler_from_quaternion(self.get_robot_pose_orientation()[1])
            delta_theta = theta_d - theta_r
            delta_theta = (delta_theta + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]
            
            rospy.sleep(0.1)
        
        twist_cmd.angular.z = 0
        self.cmd_vel_publisher.publish(twist_cmd)
        return True
    
    def get_robot_pose_orientation(self):
        """
        Retrieves the current pose and orientation of the robot in the map frame.
        """

        try:
            transform = self.tf_buffer.lookup_transform('map', 'locobot/base_link', rospy.Time())
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            return np.array([translation.x, translation.y, translation.z]), [rotation.x, rotation.y, rotation.z, rotation.w]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Error getting transform: {e}")
            return None, None
        
if __name__ == '__main__':
    align_before_pickup = AlignBeforePickup()
    rospy.spin()

