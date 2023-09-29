#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from locobot_learning.srv import Facing, FacingResponse
import tf2_ros
import tf2_geometry_msgs #pleasthis helps in the tf2 transform error and exception
from geometry_msgs.msg import PointStamped
from tf.transformations import euler_from_quaternion


class RealRobotFacing(object):
    """
    The RealRobotFacing class initializes the ROS node and provides methods to detect colored objects 
    and estimate whether the robot is facing them, based on the camera and depth images.
    """

    def __init__(self):
        """
        Initializes the RealRobotFacing object, sets up subscribers, service, and transformation listener.
        """

        rospy.init_node('RealRobotFacing', anonymous=True)
        
        self.bridge = CvBridge()
        self.rgb_image_sub = rospy.Subscriber('/locobot/camera/color/image_raw', Image, self.rgb_image_callback)
        self.depth_image_sub = rospy.Subscriber('/locobot/camera/depth/image_rect_raw', Image, self.depth_image_callback)
        self.at_srv = rospy.Service('facing', Facing, self.facing_callback)
        
        self.color_codes = {
            "red": ((12, 33, 208), "red_ball"),
            "green": ((0, 255, 220), "green_ball"),
            "orange":((0, 150, 255), "orange_marker"),
            "yellow":((0, 209, 243), "yellow_marker"),
            "blue":((112,100,0), "blue_ball")
        }
        
        self.object_positions = {v[1]: {} for k, v in self.color_codes.items()}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.loginfo("Waiting for request for facing")
        rospy.spin()

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
                try:
                    point_map_frame = self.tf_buffer.transform(point_camera_frame, 'map', timeout=rospy.Duration(1.0))
                    self.object_positions[object_name] = point_map_frame.point
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logerr(f"Error transforming point: {e}")

    def facing_callback(self, req):
        """
        Service callback function to check whether the robot is facing a specified object.
        It calculates the angle and distance between the robot and the object and responds whether
        the robot is facing the object based on certain thresholds.
        """

        obj = req.obj
        rospy.loginfo(f"Received request to check if robot is facing {obj}.")
        
        if obj not in self.object_positions:
            rospy.logerr(f"Object {obj} not found.")
            return FacingResponse(False)
        
        robot_position, robot_orientation = self.get_robot_pose_orientation()
        print ("robot posisiton = {}".format(robot_position))
        
        if robot_position is None or robot_orientation is None:
            rospy.logerr("Error getting robot pose and orientation.")
            return FacingResponse(False)
        print ("all object positions = {}".format(self.object_positions))
        object_position_msg = self.object_positions[obj]
        object_position = np.array([object_position_msg.x, object_position_msg.y, object_position_msg.z])
        direction_to_object = object_position - robot_position
        direction_to_object_xy = direction_to_object[:2]  # Projecting onto XY plane
        direction_to_object_xy /= np.linalg.norm(direction_to_object_xy)  # Normalizing
        
        _, _, yaw = euler_from_quaternion(robot_orientation)
        robot_facing_dir = np.array([np.cos(yaw), np.sin(yaw)])
        angle = np.arccos(np.clip(np.dot(robot_facing_dir, direction_to_object_xy), -1, 1))
        distance = np.linalg.norm(direction_to_object)
        print ("distance of robot from {} = {}".format(req.obj, distance))
        print ("angle difference between {} and robot {}".format(req.obj,angle))
        if angle < 1.5:  # This value may need refining.
            rospy.loginfo(f"Robot is facing {obj}.")
            return FacingResponse(True)
        else:
            rospy.loginfo(f"Robot is not facing {obj}.")
            return FacingResponse(False)

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


if __name__ == "__main__":
    RealRobotFacing()
