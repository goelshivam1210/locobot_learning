#!/usr/bin/env python3

import rospy
import tf2_ros
from locobot_learning.srv import Contain, ContainResponse, Hold, HoldRequest, Facing, FacingRequest, At, AtRequest
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

        # Define the service for checking containment
        self.contain_srv = rospy.Service('contain', Contain, self.contain_callback)

        # TF buffer and listener for transforming coordinates
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("Contain service is ready.")
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

    def check_robot_holding(self):
        """
        Check if the robot is holding any object.
        """
        try:
            hold_service = rospy.ServiceProxy('/hold', Hold)
            response = hold_service(HoldRequest(obj=""))
            rospy.loginfo(f"Robot holding object: {response.robot_holding_obj}")
            return not response.robot_holding_obj  # True if robot is holding nothing
        except rospy.ServiceException as e:
            rospy.logerr(f"Error checking hold status: {e}")
            return False

    def check_robot_facing_bin(self):
        """
        Check if the robot is facing the bin.
        """
        try:
            facing_service = rospy.ServiceProxy('/facing', Facing)
            response = facing_service(FacingRequest(obj="bin_1"))
            rospy.loginfo(f"Robot facing bin: {response.robot_facing_obj}")
            return response.robot_facing_obj
        except rospy.ServiceException as e:
            rospy.logerr(f"Error checking facing status: {e}")
            return False

    def check_object_in_room(self, obj, room):
        """
        Check if the object is in the specified room.
        """
        try:
            at_service = rospy.ServiceProxy('/at', At)
            response = at_service(AtRequest(room=room, obj=obj))
            rospy.loginfo(f"Object {obj} in {room}: {response.obj_at_room}")
            return response.obj_at_room
        except rospy.ServiceException as e:
            rospy.logerr(f"Error checking object's room: {e}")
            return False

    def contain_callback(self, req):
        """
        Service callback function to check if the given object is contained in the bin.
        """
        obj = req.obj
        container = req.container

        # Check if the container is a bin
        if container != "bin_1":
            return ContainResponse(container_contains_obj=False)

        # Check preconditions
        rospy.loginfo(f"Checking containment for {obj} in {container}.")

        # 1. Check if robot is holding nothing
        if not self.check_robot_holding():
            rospy.loginfo(f"Robot is holding something. Precondition failed.")
            return ContainResponse(container_contains_obj=False)

        # 2. Check if robot is facing the bin
        if not self.check_robot_facing_bin():
            rospy.loginfo(f"Robot is not facing the bin. Precondition failed.")
            return ContainResponse(container_contains_obj=False)

        # 3. Check if the object is in room 2
        if not self.check_object_in_room(obj, "room_2"):
            rospy.loginfo(f"Object {obj} is not in room_2. Precondition failed.")
            return ContainResponse(container_contains_obj=False)

        # 4. Check transformed coordinates (optional)
        try:
            marker = rospy.wait_for_message("/locobot/pc_filter/markers/objects", Marker, timeout=1.0)
            transformed_marker = self.transform_marker_to_map_frame(marker.pose.position)
            if transformed_marker is None:
                rospy.logwarn("No transformed coordinates available.")
            else:
                rospy.loginfo(f"Transformed marker position: {transformed_marker}")
                # Check if the marker's position is inside the bin boundary
                bin_boundary = self.param_boundaries.get("bin_1")
                if bin_boundary and self.is_point_inside_polygon(transformed_marker, bin_boundary):
                    rospy.loginfo(f"Object {obj} is inside the bin.")
                    return ContainResponse(container_contains_obj=True)
        except rospy.ROSException:
            rospy.logwarn("Could not retrieve transformed coordinates.")

        # If all conditions fail, return false
        rospy.loginfo(f"Object {obj} is NOT contained in {container}.")
        return ContainResponse(container_contains_obj=False)

    def is_point_inside_polygon(self, point_coords, boundary_coords):
        """
        Check if a given point is inside a polygon defined by boundary coordinates.
        """
        point = Point(point_coords)
        polygon = Polygon(boundary_coords)
        return point.within(polygon)


if __name__ == "__main__":
    RealRobotContainService()