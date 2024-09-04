#!/usr/bin/env python3

import rospy
import tf2_ros
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Quaternion
import tf2_geometry_msgs


class TransformAndPublish:
    def __init__(self):
        rospy.init_node("transform_and_publish_node")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publisher for PoseStamped (single point)
        self.transformed_coordinates_publisher = rospy.Publisher("transformed_coordinates", PoseStamped, queue_size=10)
        
        # Publisher for Marker array (multiple points)
        self.transformed_marker_publisher = rospy.Publisher("transformed_marker_array", Marker, queue_size=10)
        
        rospy.Subscriber("/locobot/pc_filter/markers/objects", Marker, self.marker_callback)
    
    def marker_callback(self, marker_msg):
        try:
            # Look up the transform between the camera frame and the arm base link frame
            transform = self.tf_buffer.lookup_transform("locobot/arm_base_link", "locobot/camera_color_optical_frame", rospy.Time())
            
            # Create a new Marker message to store the transformed points
            transformed_marker = Marker()
            transformed_marker.header.frame_id = "locobot/arm_base_link"
            transformed_marker.header.stamp = rospy.Time.now()
            transformed_marker.ns = marker_msg.ns
            transformed_marker.id = marker_msg.id
            transformed_marker.type = Marker.POINTS
            transformed_marker.action = Marker.ADD
            transformed_marker.scale.x = 0.1  # Adjust scale for visualization
            transformed_marker.scale.y = 0.1
            transformed_marker.color = marker_msg.color

            # Transform each point in the marker's points array
            for point in marker_msg.points:
                point_in_camera = PointStamped()
                point_in_camera.header = marker_msg.header
                point_in_camera.point = point

                # Transform the point to the arm_base_link frame
                point_in_base = tf2_geometry_msgs.do_transform_point(point_in_camera, transform)

                # Add the transformed point to the new Marker
                transformed_marker.points.append(point_in_base.point)

                # Publish the transformed single point as PoseStamped
                pose_stamped = PoseStamped()
                pose_stamped.header = marker_msg.header
                pose_stamped.header.frame_id = "locobot/arm_base_link"
                pose_stamped.pose.position = point_in_base.point
                pose_stamped.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
                self.transformed_coordinates_publisher.publish(pose_stamped)

            # Publish the transformed marker array with the array of points
            self.transformed_marker_publisher.publish(transformed_marker)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to transform coordinates from camera frame to arm_base_link frame")


if __name__ == "__main__":
    TransformAndPublish()
    rospy.spin()
