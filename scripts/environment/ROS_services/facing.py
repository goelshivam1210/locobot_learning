#!/usr/bin/env python3

import rospy
from locobot_learning.srv import Facing, FacingResponse
from visualization_msgs.msg import Marker
from shapely.geometry import Point, Polygon
import tf2_ros
import numpy as np
from tf.transformations import euler_from_quaternion
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

class RealRobotFacing(object):
    """
    The RealRobotFacing class initializes the ROS node and provides methods to detect objects 
    and estimate whether the robot is facing them, based on camera detection (non-stationary) or
    its position and orientation (stationary).
    """

    def __init__(self):
        """
        Initializes the RealRobotFacing object, sets up subscribers, service, and transformation listener.
        """
        self.tolerance = 0.15
        rospy.init_node('RealRobotFacing', anonymous=True)

        # Fetch parameters from ROS param server (facing boundaries and navigation goals)
        try:
            self.param_facing_boundaries = rospy.get_param("facing_boundaries")
            self.param_nav_goals = rospy.get_param("real_nav_goals")  # Correctly fetching the real_nav_goals
        except (KeyError, rospy.ROSException):
            rospy.logerr("Error getting parameters.")
            raise ValueError

        # Service
        self.facing_srv = rospy.Service('facing', Facing, self.facing_callback)
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Map models to objects in the domain
        self.model_to_pddl_mapping = {
            "robot_1": "locobot",
            "generic_object": "generic_object",
            "door_1": "door",
            "bin_1": "bin",
            "table": "table",
            "nothing": "nothing",
            "atdoor": "atdoor",
            "postdoor": "postdoor",
            "doorway_1": "doorway_1"
        }

    def facing_callback(self, req):
        """
        Service callback function to check whether the robot is facing a specified object.
        """
        if req.obj not in self.model_to_pddl_mapping:
            rospy.loginfo("Object not in mapped models")
            return FacingResponse(False)

        model = self.model_to_pddl_mapping[req.obj]

        # Special handling for postdoor
        if model == "postdoor":
            return self.facing_nothing()

        if model in ["door", "bin", "table", "atdoor", "doorway_1"]:
            return self.facing_zone(model, req.obj)

        elif model == "generic_object":
            if not self.is_facing_any_stationary_object():
                return self.facing_generic_object()

        return self.facing_nothing()



    def facing_doorway(self, model):
        """
        Logic for checking if the robot is facing the doorway.
        """
        robot_position, robot_pose = self.get_robot_pose_orientation()
        if robot_position is None or robot_pose is None:
            return FacingResponse(False)

        x_pos, y_pos = robot_position[0], robot_position[1]

        # Convert orientation to yaw
        _, _, yaw = euler_from_quaternion(robot_pose)

        # Get doorway boundary and orientation
        boundary = self.param_facing_boundaries.get(model, {}).get('boundary', None)
        if not boundary:
            rospy.logerr(f"Boundary not found for {model}.")
            return FacingResponse(False)

        yaw_threshold = self.param_facing_boundaries.get(model, {}).get('threshold', 0.5)
        goal_yaw = self.get_goal_yaw_from_nav_goals(model)

        if self.is_point_inside_polygon((x_pos, y_pos), boundary) and (goal_yaw - yaw_threshold <= yaw <= goal_yaw + yaw_threshold):
            rospy.loginfo(f"Robot is facing the doorway: {model}.")
            return FacingResponse(True)

        rospy.loginfo(f"Robot is NOT facing the doorway: {model}.")
        return FacingResponse(False)

    def facing_zone(self, model, obj):
        """
        Logic to check if the robot is facing a stationary zone, such as a doorway, bin, table, or doorways (atdoor/postdoor).
        """
        if model not in self.param_facing_boundaries:
            rospy.logerr(f"No boundary information found for {model}.")
            return FacingResponse(False)

        robot_position, robot_pose = self.get_robot_pose_orientation()
        if robot_position is None or robot_pose is None:
            return FacingResponse(False)

        x_pos, y_pos = robot_position[0], robot_position[1]
        _, _, yaw = euler_from_quaternion(robot_pose)

        boundary = self.param_facing_boundaries.get(model, {}).get('boundary', None)
        if not boundary:
            rospy.logerr(f"Boundary not found for {model}")
            return FacingResponse(False)

        yaw_threshold = self.param_facing_boundaries.get(model, {}).get('threshold', 0.5)
        goal_yaw = self.get_goal_yaw_from_nav_goals(model)

        if self.is_point_inside_polygon((x_pos, y_pos), boundary) and (goal_yaw - yaw_threshold <= yaw <= goal_yaw + yaw_threshold):
            rospy.loginfo(f"Robot is facing {obj}.")
            return FacingResponse(True)

        rospy.loginfo(f"Robot is NOT facing {obj}.")
        return FacingResponse(False)

    def facing_nothing(self):
        """
        Determine if the robot is facing 'nothing' based on its position and orientation.
        """
        robot_position, robot_pose = self.get_robot_pose_orientation()
        if robot_position is None or robot_pose is None:
            return FacingResponse(False)

        x_pos, y_pos = robot_position[0], robot_position[1]
        _, _, yaw = euler_from_quaternion(robot_pose)

        # Define thresholds for being "postdoor" and "nothing"
        postdoor_boundary = self.param_facing_boundaries.get("postdoor", {}).get("boundary", [])
        yaw_threshold = self.param_facing_boundaries.get("postdoor", {}).get("threshold", 0.5)

        if not postdoor_boundary:
            rospy.logerr("Boundary for postdoor not found.")
            return FacingResponse(False)

        # Extract goal yaw for postdoor
        postdoor_goal_yaw = self.get_goal_yaw_from_nav_goals("postdoor")

        # Check if the robot is in the postdoor region
        if self.is_point_inside_polygon((x_pos, y_pos), postdoor_boundary):
            lower_orientation = postdoor_goal_yaw - yaw_threshold
            upper_orientation = postdoor_goal_yaw + yaw_threshold

            rospy.loginfo(f"Yaw: {yaw}, Postdoor range: {lower_orientation} to {upper_orientation}")

            # If the robot is in postdoor region and not oriented towards the doorway, it is facing nothing
            if not (lower_orientation <= yaw <= upper_orientation):
                rospy.loginfo("Robot is in postdoor region and oriented away. Facing nothing.")
                return FacingResponse(True)

        # If no other objects are detected, fallback to facing nothing
        if not self.is_facing_any_stationary_object():
            rospy.loginfo("No other objects detected. Robot is facing nothing.")
            return FacingResponse(True)

        rospy.loginfo("Robot is NOT facing nothing.")
        return FacingResponse(False)



    def get_goal_yaw_from_nav_goals(self, model):
        try:
            quaternion = self.param_nav_goals[model]['orientation']
            _, _, goal_yaw = euler_from_quaternion([quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']])
            return goal_yaw
        except KeyError:
            rospy.logerr(f"No yaw found for {model} in real_nav_goals.yaml.")
            return 0

    def is_facing_any_stationary_object(self):
        for obj in ["door", "bin", "table", "atdoor", "postdoor"]:
            if self.facing_zone(obj, obj).robot_facing_obj:
                return True
        return False

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
            rospy.logerr(f"Error getting transform: {e}")
            return None, None

if __name__ == "__main__":
    RealRobotFacing()
    rospy.spin()
