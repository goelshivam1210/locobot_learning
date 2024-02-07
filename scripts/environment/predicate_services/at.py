#!/usr/bin/env python3
import rospy
from shapely.geometry import Point
from shapely.geometry import Polygon
import tf2_ros
import numpy as np

from locobot_learning.srv import At, AtResponse

class RecycleBotAt(object):

    def __init__(self):

        #TO DO - bin always in room 2, won't change
        #If gripper has picked up marker, then marker at loc of robot
        #If gripper not picked up marker, in room1

        rospy.init_node('RecycleBotAt', anonymous=True)

        self.at_srv = rospy.Service('at', At, self.at_callback)

        try:
            self.param_at_boundaries = rospy.get_param("at_boundaries")
        except (KeyError, rospy.ROSException):
            
            rospy.logerr("Error getting parameters.")
            raise ValueError

        #Deleted other mappings not relevant (coke can, etc)
        self.model_to_pddl_mapping = {
            "robot_1": "locobot",
            "marker_1": "marker",
            "bin_1": "bin"
        }

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        while not rospy.is_shutdown():
            rospy.spin()
    

    def at_callback(self, req):
        #Determine if request for marker, bin or robot
        room = req.room
        obj = req.obj
        rospy.loginfo(f"Received request to check if {obj} is in {room}.")

        if obj not in self.model_to_pddl_mapping:
            return AtResponse(False)
        
        if obj == "robot_1":
            return self.is_robot_in_room(room)
        elif obj == "marker_1":
            return self.is_marker_in_room(room)
        else:
            return self.is_bin_in_room(room)


    def is_robot_in_room(self, room):
        # model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates)
        robot_position, _ = self.get_robot_pose_orientation()
        x_pos, y_pos = robot_position[0], robot_position[1]
        rospy.loginfo(f"Robot X position: {x_pos} \n Y Pos: {y_pos}")

        point = (x_pos, y_pos)

        # Check if point is in requested room
        boundary = self.param_at_boundaries[room]

        if self.is_point_inside_polygon(point, boundary):
            rospy.loginfo(f"Object robot_1 is in {room}.")
            return AtResponse(True)
        else:
            rospy.loginfo(f"Object robot_1 is NOT in {room}.")
            return AtResponse(False)
    
    def is_marker_in_room(self, room):
        #To Do - Get whether marker picked up, if so get location of robot
        if room == "room_1":
            return AtResponse(True)
        else:
            return AtResponse(False)
    
    def is_bin_in_room(self, room):
        if room == "room_2":
            return AtResponse(True)
        else:
            return AtResponse(False)

        
    def is_point_inside_polygon(self, point_coords, boundary_coords):
        """
        Check if a given point is inside a polygon defined by boundary coordinates.
        :param point_coords: Tuple of (x, y) coordinates for the point.
        :param boundary_coords: List of tuples, each tuple being (x, y) coordinates of a vertex of the polygon.
        :return: True if the point is inside the polygon, False otherwise.        if self.is_point_inside_polygon(point, self.param_at_boundaries['room_1']):
            rospy.loginfo(f"Point {point} is inside room_1 boundaries.")
        else:
            rospy.loginfo(f"Point {point} is NOT inside room_1 boundaries.")
        """
        point = Point(point_coords)
        poly = Polygon(boundary_coords)
        return point.within(poly)
    

    
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
    RecycleBotAt()