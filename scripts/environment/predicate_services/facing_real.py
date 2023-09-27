#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from locobot_learning.srv import Facing, FacingResponse
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from tf.transformations import euler_from_quaternion


class RealRobotFacing(object):
    def __init__(self):
        rospy.init_node('RealRobotFacing', anonymous=True)
        
        self.bridge = CvBridge()
        self.rgb_image_sub = rospy.Subscriber('/locobot/camera/color/image_raw', Image, self.rgb_image_callback)
        self.depth_image_sub = rospy.Subscriber('/locobot/camera/depth/image_rect_raw', Image, self.depth_image_callback)
        self.at_srv = rospy.Service('facing', Facing, self.facing_callback)
        
        self.color_codes = {
            "red": ((31, 37, 253), "object_1"),
            "green": ((0, 255, 0), "object_2"),
            "orange":((0, 165, 255), "object_3")
        }
        rospy.loginfo("self.color_codes")
        self.object_positions = {v[1]: {} for k, v in self.color_codes.items()}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.spin()

    def rgb_image_callback(self, msg):
        # rospy.loginfo("I am in rgb callback")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # print("{}".format(cv_image))
            self.detect_color_objects(cv_image)
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")
            
    def depth_image_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_image = depth_image
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")
        
    def detect_color_objects(self, cv_image):
        # print ("I am here {}".format(cv_image))
        for color, (bgr, object_name) in self.color_codes.items():
            mask = cv2.inRange(cv_image, np.array(bgr) - 20, np.array(bgr) + 20)
            # print ("mask {}".format(mask))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # print ("contours {}".format(contours))
            for contour in contours:
                cx, cy = np.mean(contour, axis=0)[0]
                depth = self.depth_image[int(cy), int(cx)]
                point_camera_frame = PointStamped()
                point_camera_frame.header.frame_id = 'locobot/camera_tower_link'
                point_camera_frame.point.x = cx * depth
                point_camera_frame.point.y = cy * depth
                point_camera_frame.point.z = depth
                
                try:
                    point_map_frame = self.tf_buffer.transform(point_camera_frame, 'map', timeout=rospy.Duration(1.0))
                    self.object_positions[object_name] = point_map_frame.point
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logerr(f"Error transforming point: {e}")

    def facing_callback(self, req):
        obj = req.obj
        rospy.loginfo(f"Received request to check if robot is facing {obj}.")
        
        robot_position, robot_orientation = self.get_robot_pose_orientation()
        print("object positions is")
        print("robot pose {} robot orientation {}".format(robot_position, robot_orientation))
        print(self.object_positions)

        if obj in self.object_positions:
            object_position_msg = self.object_positions[obj]
            object_position = np.array([object_position_msg.x, object_position_msg.y, object_position_msg.z])
            print(object_position)
            
            direction_to_object = object_position - robot_position
            direction_to_object_xy = np.array([direction_to_object[0], direction_to_object[1]])  # Projecting onto XY plane
            direction_to_object_xy /= np.linalg.norm(direction_to_object_xy)  # Normalizing
            
            _, _, yaw = euler_from_quaternion([robot_orientation[0], robot_orientation[1], robot_orientation[2], robot_orientation[3]])
            robot_facing_dir = np.array([np.cos(yaw), np.sin(yaw)])
            
            angle = np.arccos(np.dot(robot_facing_dir, direction_to_object_xy))
            distance = np.linalg.norm(object_position - robot_position)
            rospy.loginfo(f"Angle: {angle}, Distance: {distance}")

            # rospy.loginfo(f"Threshold: {self.param_facing_thresh.get(obj, 0)}")
            
            # if angle < 0.25 and distance < self.param_facing_thresh.get(obj, 0):
            if angle < 5:
                rospy.loginfo(f"Robot is facing {obj}.")
                return FacingResponse(True)
            else:
                rospy.loginfo(f"Robot is not facing {obj}.")
                return FacingResponse(False)
                
        else:
            rospy.logerr(f"Object {obj} not found.")
            return FacingResponse(False)
        
    def get_robot_pose_orientation(self):
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
