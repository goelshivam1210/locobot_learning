#!/usr/bin/env python3
import rospy
from shapely.geometry import Point, Polygon
import tf2_ros
import numpy as np

from locobot_learning.srv import At, AtResponse

class AtService(object):

    def __init__(self):
        rospy.init_node('AtService', anonymous=True)
        self.at_srv = rospy.Service('at', At, self.at_callback)

        try:
            self.param_at_boundaries = rospy.get_param("at_boundaries")
        except (KeyError, rospy.ROSException):
            rospy.logerr("Error getting 'at_boundaries' parameter. Shutting down node.")
            rospy.signal_shutdown("Parameter error")
            return

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("At service is ready.")
        rospy.spin()

    def at_callback(self, req):
        room = req.room
        obj = req.obj
        rospy.loginfo(f"Received request to check if {obj} is in {room}.")

        # Map objects like ball_1 or can_1 to 'generic_object'
        obj = self.map_to_generic_object(obj)

        if obj == "robot_1":
            return self.is_robot_in_room(room)
        elif obj == "marker_1" or obj == "generic_object":
            return self.is_marker_in_room(room)  # Use marker logic for generic objects
        elif obj == "bin_1":
            return self.is_bin_in_room(room)
        else:
            rospy.logwarn(f"Object {obj} is not recognized.")
            return AtResponse(False)

    def map_to_generic_object(self, obj: str) -> str:
        """
        Map specific objects like ball_1 or can_1 to 'generic_object'.
        """
        if obj in ["ball_1", "can_1"]:
            return "generic_object"
        return obj

    def is_robot_in_room(self, room):
        robot_position, _ = self.get_robot_pose_orientation()
        if robot_position is None:
            return AtResponse(False)

        point = (robot_position[0], robot_position[1])
        boundary = self.param_at_boundaries.get(room)

        if boundary is None:
            rospy.logwarn(f"Room {room} boundary is not defined.")
            return AtResponse(False)

        if self.is_point_inside_polygon(point, boundary):
            rospy.loginfo(f"Robot is in {room}.")
            return AtResponse(True)
        else:
            rospy.loginfo(f"Robot is NOT in {room}.")
            return AtResponse(False)

    def is_marker_in_room(self, room):
        # Simplified logic: check if the marker is in a specific room (can be expanded)
        if room == "room_1":  # Assume the generic object is in room_1 for now
            return AtResponse(True)
        else:
            return AtResponse(False)

    def is_bin_in_room(self, room):
        if room == "room_2":  # Simplified logic, assumes bin is always in room_2
            return AtResponse(True)
        else:
            return AtResponse(False)

    def is_point_inside_polygon(self, point_coords, boundary_coords):
        point = Point(point_coords)
        poly = Polygon(boundary_coords)
        return point.within(poly)

    def get_robot_pose_orientation(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'locobot/base_link', rospy.Time())
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            return np.array([translation.x, translation.y, translation.z]), [rotation.x, rotation.y, rotation.z, rotation.w]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Error getting robot pose: {e}")
            return None, None


if __name__ == "__main__":
    AtService()