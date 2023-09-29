#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs # this helps in the tf2 transform error and exception
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from tf.transformations import euler_from_quaternion
from locobot_learning.srv import AlignRobot, AlignRobotResponse

class AlignBeforePickup(object):

    def __init__(self):
        # # Initialization and other methods remain the same
        rospy.init_node('AlignRobot', anonymous=True)
        
        # Initialize the CV Bridge
        self.bridge = CvBridge()
        # camera topics
        self.rgb_image_sub = rospy.Subscriber('/locobot/camera/color/image_raw', Image, self.rgb_image_callback)
        self.depth_image_sub = rospy.Subscriber('/locobot/camera/depth/image_rect_raw', Image, self.depth_image_callback)
        # Initialize the publisher for sending movement commands to the robot base
        self.cmd_vel_publisher = rospy.Publisher('/mobile_base/cmd_vel', Twist, queue_size=10)
        
        self.service = rospy.Service('align_before_pickup', AlignRobot, self.align_service_callback)

        # Initialize the color codes
        self.color_codes = {
            "red": ((12, 33, 208), "red_ball"),
            "green": ((0, 255, 220), "green_ball"),
            "orange":((0, 150, 255), "orange_marker"),
            "yellow":((0, 209, 243), "yellow_marker"),
            "blue":((112,100,0), "blue_ball")
        }
        
        # Initialize object_positions dictionary
        self.object_positions = {v[1]: None for _, v in self.color_codes.items()}
        
        # Initialize the tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.loginfo("Waiting to align the required object")
        rospy.spin()

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
            lower_bound = np.array(bgr) - 50
            upper_bound = np.array(bgr) + 50
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
                # print ("point camera frame = {}".format(point_camera_frame))
                try:
                    point_map_frame = self.tf_buffer.transform(point_camera_frame, 'map', timeout=rospy.Duration(1.0))
                    self.object_positions[object_name] = point_map_frame.point
                    # print ("self.object_positions = {}".format(self.object_positions))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logerr(f"Error transforming point: {e}")
                
    def align_to_object(self, object_name):
        print ("object_name = {}".format(object_name))
        print ("object position = ".format(self.object_positions.get(object_name)))
        tolerance = 0.05  # Define the alignment tolerance in radians
        gain = 0.01  # Define the proportional gain for the rotation command
        max_iterations = 15 # Maximum number of iterations to prevent infinite loop in case of non-convergence

        
        robot_position, robot_orientation = self.get_robot_pose_orientation()
        _, _, theta_r = euler_from_quaternion(robot_orientation)
        print ("object_positions = {}".format(self.object_positions))
        object_position = self.object_positions.get(object_name)
        if object_position is None:
            rospy.logerr(f"Object {object_name} not found.")
            return False
        
        x_o, y_o = object_position.x, object_position.y
        theta_d = np.arctan2(y_o - robot_position[1], x_o - robot_position[0])
        
        delta_theta = theta_d - theta_r
        print("delta_theta = {}".format(delta_theta))
        delta_theta = (delta_theta + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]
        iteration_count = 0
        twist_cmd = Twist()
        while np.abs(delta_theta) > tolerance:
            if iteration_count > max_iterations:
                rospy.logerr("Alignment failed to converge after %d iterations", max_iterations)
                twist_cmd.angular.z = 0  # Stop turning
                self.cmd_vel_publisher.publish(twist_cmd)
                return False
            
            twist_cmd.angular.z = delta_theta * gain
            self.cmd_vel_publisher.publish(twist_cmd)
            
            _, _, theta_r = euler_from_quaternion(self.get_robot_pose_orientation()[1])
            delta_theta = theta_d - theta_r
            delta_theta = (delta_theta + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]
            
            rospy.loginfo("Iteration: %d, delta_theta: %f, theta_d: %f, theta_r: %f, gain: %f, tolerance: %f",
                        iteration_count, delta_theta, theta_d, theta_r, gain, tolerance)
            
            iteration_count += 1
            rospy.sleep(0.1)  # Adjust the sleep duration as needed
        
        # Stop the robot after alignment
        twist_cmd.angular.z = 0
        self.cmd_vel_publisher.publish(twist_cmd)
        rospy.loginfo("Alignment successful")
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
    AlignBeforePickup()

