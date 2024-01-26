#!/usr/bin/env python3

import rospy
import tf2_ros
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PointStamped
import time
import tf2_geometry_msgs #his helps in the tf2 transform error and exception


class TransformAndPublish:
    def __init__(self):
        rospy.init_node("transform_and_publish_node")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.transformed_coordinates_publisher = rospy.Publisher("transformed_coordinates", PoseStamped, queue_size=10)
        rospy.Subscriber("/locobot/pc_filter/markers/objects", Marker, self.marker_callback)
    
    def marker_callback(self, marker_msg):
        # rospy.logdebug("HERE")
        try:
            # rospy.loginfo(marker_msg)
            transform = self.tf_buffer.lookup_transform("locobot/arm_base_link", "locobot/camera_color_optical_frame", rospy.Time())

            # Transform the marker coordinates to arm_base_link frame
            point_in_camera = PointStamped()
            point_in_camera.header = marker_msg.header
            point_in_camera.point = Point(x=marker_msg.pose.position.x, y=marker_msg.pose.position.y, z=marker_msg.pose.position.z)
            point_in_camera.header.frame_id = marker_msg.header.frame_id
            # rospy.loginfo(point_in_camera)
            point_in_base = tf2_geometry_msgs.do_transform_point(point_in_camera, transform)

            # Create a PoseStamped message with the transformed coordinates
            pose_stamped = PoseStamped()
            pose_stamped.header = marker_msg.header
            pose_stamped.header.frame_id = "locobot/arm_base_link"
            pose_stamped.pose.position = point_in_base.point
            pose_stamped.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # Set orientation as needed

            # Publish the transformed coordinates
            self.transformed_coordinates_publisher.publish(pose_stamped)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to transform coordinates from camera frame to arm_base_link frame")

if __name__ == "__main__":
    TransformAndPublish()
    rospy.spin()
