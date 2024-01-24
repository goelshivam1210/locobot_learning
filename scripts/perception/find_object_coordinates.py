#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import time
import tf2_geometry_msgs #pleasthis helps in the tf2 transform error and exception


class TransformAndPublish:
    def __init__(self):
        rospy.init_node("transform_and_publish_node")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.transformed_coordinates_publisher = rospy.Publisher("transformed_coordinates", PoseStamped, queue_size=10)
        rospy.Subscriber("/locobot/pc_filter/markers/objects", Marker, self.marker_callback)
    
    def marker_callback(self, marker_msg):
        rospy.logdebug("HERE")
        try:
            # Use the latest transform available at the time of receiving the marker message
            # transform = self.tf_buffer.lookup_transform("locobot/arm_base_link", "locobot/camera_tower_link", marker_msg.header.stamp, rospy.Duration(1.0))

            # Extract x, y, and z coordinates from marker_msg.pose.position
            x = marker_msg.pose.position.x
            y = marker_msg.pose.position.y
            z = marker_msg.pose.position.z

            # Create a PointStamped message to perform the transformation
            point_in_camera = PoseStamped()
            point_in_camera.header.stamp = rospy.Time.now()
            point_in_camera.header.frame_id = marker_msg.header.frame_id
            point_in_camera.pose.position.x = x
            point_in_camera.pose.position.y = y
            point_in_camera.pose.position.z = z

            transform = self.tf_buffer.lookup_transform("locobot/arm_base_link", "locobot/camera_color_optical_frame", rospy.Time())
            rospy.logerr(transform)

            # Create a PoseStamped message for the transformed coordinates
            transformed_pose = PoseStamped()
            transformed_pose.header.stamp = rospy.Time.now()
            transformed_pose.header.frame_id = "locobot/arm_base_link"
            transformed_pose.pose.position = transform.translation
            transformed_pose.pose.orientation = transform.rotation


            # Publish the transformed coordinates
            self.transformed_coordinates_publisher.publish(transformed_pose)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to transform coordinates from camera frame to arm_base_link frame")

if __name__ == "__main__":
    TransformAndPublish()
    rospy.spin()
