#!/usr/bin/env python3

import rospy
import tf2_ros
from locobot_learning.srv import Contain, ContainResponse
from shapely.geometry import Point, Polygon
from visualization_msgs.msg import Marker
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped


class RealRobotContainService(object):

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('RealRobotContainService', anonymous=True)

        # Fetch the boundaries from the parameter server
        try:
            self.param_boundaries = rospy.get_param("at_boundaries")
        except (KeyError, rospy.ROSException):
            rospy.logerr("Error getting boundaries parameters.")
            raise ValueError

        # Define the service for checking the containment
        self.contain_srv = rospy.Service('contain', Contain, self.contain_callback)

        # TF buffer and listener for transforming coordinates
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.spin()

    def transform_marker_to_map_frame(self, marker_position):
        """
        Transforms the marker position to the map frame.
        """
        try:
            transform = self.tf_buffer.lookup_transform('map', 'locobot/camera_color_optical_frame', rospy.Time())
            
            # Transform the marker coordinates to the map frame
            point_in_camera = PointStamped()
            point_in_camera.header.frame_id = 'locobot/camera_color_optical_frame'
            point_in_camera.point = marker_position

            point_in_map = tf2_geometry_msgs.do_transform_point(point_in_camera, transform)
            return [point_in_map.point.x, point_in_map.point.y]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Error transforming marker: {e}")
            return None

    def contain_callback(self, req):
        """
        Service callback function to check if the given object is contained in the bin.
        """
        obj = req.obj
        container = req.container

        # Check if the container is a bin
        if container != "bin_1":
            return ContainResponse(container_contains_obj=False)

        # Get the bin boundary from the parameter server
        bin_boundary = self.param_boundaries.get("bin_1")
        if bin_boundary is None:
            rospy.logerr("No bin boundary found in the parameters.")
            return ContainResponse(container_contains_obj=False)

        # Dynamically query the marker data to check if the object is detected
        try:
            marker = rospy.wait_for_message("/locobot/pc_filter/markers/objects", Marker, timeout=1.0)
            if not marker.pose.position:
                rospy.loginfo(f"Object {obj} is not detected.")
                return ContainResponse(container_contains_obj=False)
        except rospy.ROSException:
            rospy.loginfo(f"Object {obj} is not detected.")
            return ContainResponse(container_contains_obj=False)

        # Transform the marker position to the map frame
        transformed_marker = self.transform_marker_to_map_frame(marker.pose.position)
        if transformed_marker is None:
            rospy.logerr(f"Failed to transform marker position to map frame.")
            return ContainResponse(container_contains_obj=False)

        # Log transformed marker and bin boundary for debugging
        rospy.loginfo(f"Transformed marker position: {transformed_marker}")
        rospy.loginfo(f"Bin boundary: {bin_boundary}")

        # Check if the marker's position is inside the bin boundary
        if self.is_point_inside_polygon(transformed_marker, bin_boundary):
            rospy.loginfo(f"Object {obj} is inside the bin.")
            return ContainResponse(container_contains_obj=True)

        rospy.loginfo(f"Object {obj} is not inside the bin.")
        return ContainResponse(container_contains_obj = False)

    def is_point_inside_polygon(self, point_coords, boundary_coords):
        """
        Check if a given point is inside a polygon defined by boundary coordinates.
        """
        point = Point(point_coords)
        polygon = Polygon(boundary_coords)
        return point.within(polygon)


if __name__ == "__main__":
    RealRobotContainService()
