#!/usr/bin/env python3

import rospy
from locobot_learning.srv import Facing, FacingRequest, FacingResponse
from shapely.geometry import Point, Polygon
import tf2_ros
import numpy as np
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped


class RealRobotFacing:
    def __init__(self):
        """
        Initializes the RealRobotFacing service and loads parameters.
        """
        rospy.init_node('real_robot_facing', anonymous=True)

        # Load parameters from the parameter server
        try:
            self.param_facing_boundaries = rospy.get_param("facing_boundaries")
            self.param_nav_goals = rospy.get_param("real_nav_goals")  # For generic object boundaries
            rospy.loginfo("Parameters successfully loaded from the parameter server.")
        except KeyError:
            rospy.logerr("Required parameters not found.")
            raise

        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Define service
        self.facing_srv = rospy.Service('/facing', Facing, self.facing_callback)
        rospy.loginfo("Facing service initialized.")
        rospy.loginfo("Facing service is ready.")


    def facing_callback(self, req):
        """
        Callback to check if the robot is facing a specific object.
        """
        obj = req.obj
        rospy.loginfo(f"Checking if robot is facing: {obj}")

        # Default to True for 'nothing'
        if obj == "nothing":
            if not self.is_facing_any_specific_object():
                rospy.loginfo("Robot is not facing any specific object. Defaulting to facing nothing.")
                return FacingResponse(True)
            rospy.loginfo("Robot is facing another object. Not facing nothing.")
            return FacingResponse(False)

        # Handle specific objects
        # 2025-05-05: Added "generic_object" to the list of objects because SymbolicState.py
        # is checking for "generic_object" in the facing service.
        if obj in ["ball_1", "can_1", "generic_object"]:  # Generic objects
            return self.check_facing_generic_object()

        if obj == "bin_1":  # Bin
            return self.check_facing_boundary("bin")

        if obj == "table":  # Table
            return self.check_facing_boundary("table")

        if obj == "doorway_1":  # Doorway
            return self.check_facing_boundary("atdoor")

        rospy.logwarn(f"Unknown object: {obj}. Defaulting to not facing.")
        return FacingResponse(False)

    def is_facing_any_specific_object(self):
        """
        Check if the robot is facing any specific object.
        """
        rospy.loginfo("Checking if the robot is facing any specific object.")
        for obj in ["ball_1", "can_1", "bin_1", "table", "doorway_1"]:
            response = self.facing_callback(FacingRequest(obj=obj))
            if response.robot_facing_obj:
                rospy.loginfo(f"Robot is facing: {obj}")
                return True
        return False

    def check_facing_boundary(self, boundary_name):
        """
        Check if the robot is within the boundary and orientation thresholds of a specified object.
        """
        robot_position, robot_orientation = self.get_robot_pose_orientation()
        if robot_position is None or robot_orientation is None:
            rospy.logerr("Failed to retrieve robot position or orientation.")
            return FacingResponse(False)

        rospy.loginfo(f"Robot position: {robot_position}, Robot yaw: {robot_orientation[2]}")

        boundary = self.param_facing_boundaries.get(boundary_name, {}).get("boundary")
        yaw_threshold = self.param_facing_boundaries.get(boundary_name, {}).get("threshold", 0.5)

        if not boundary:
            rospy.logerr(f"Boundary not defined for {boundary_name}.")
            return FacingResponse(False)

        yaw = robot_orientation[2]
        goal_yaw = self.get_goal_yaw(boundary_name)

        def get_angle_difference(yaw1, yaw2):
            """ Compute the shortest difference between two angles, handling wrap-around at ±π. """
            return abs((yaw1 - yaw2 + np.pi) % (2 * np.pi) - np.pi)

        point_inside = self.is_point_inside_polygon(robot_position, boundary)
        # yaw_diff = abs(yaw - goal_yaw)
        yaw_diff = get_angle_difference(yaw, goal_yaw)

        rospy.loginfo(f"Boundary: {boundary}, Goal yaw: {goal_yaw}, Yaw threshold: {yaw_threshold}")
        rospy.loginfo(f"Point inside boundary: {point_inside}, Yaw difference: {yaw_diff}")

        if point_inside and yaw_diff <= yaw_threshold:
            rospy.loginfo(f"Robot is facing {boundary_name}.")
            return FacingResponse(True)

        rospy.loginfo(f"Robot is NOT facing {boundary_name}.")
        return FacingResponse(False)

    def check_facing_generic_object(self):
        """
        Check if the robot is facing a generic object using transformed coordinates and boundaries.
        """
        transformed_coords = self.get_transformed_coordinates()
        robot_position, _ = self.get_robot_pose_orientation()

        rospy.loginfo(f"Robot position: {robot_position}, Transformed coordinates: {transformed_coords}")

        # Retrieve boundary for generic objects from `real_nav_goals`
        generic_object_boundary = self.get_generic_object_boundary()

        rospy.loginfo(f"Generic object boundary: {generic_object_boundary}")

        if generic_object_boundary and self.is_point_inside_polygon(robot_position, generic_object_boundary):
            if transformed_coords is not None:
                rospy.loginfo("Robot is facing a generic object.")
                return FacingResponse(True)

        rospy.loginfo("Robot is NOT facing a generic object.")
        return FacingResponse(False)

    def get_generic_object_boundary(self):
        """
        Retrieve the boundary for generic objects from `real_nav_goals`.
        """
        try:
            boundary = self.param_facing_boundaries["generic_object"]["boundary"]
            return boundary
        except KeyError:
            rospy.logwarn("Boundary for generic objects not defined in real_nav_goals.")
            return None

    def get_transformed_coordinates(self):
        """
        Retrieve transformed coordinates from the relevant ROS topic.
        """
        try:
            pose = rospy.wait_for_message("/transformed_coordinates", PoseStamped, timeout=1.0)
            rospy.loginfo(f"Transformed coordinates: {pose.pose.position}")
            return pose.pose.position
        except rospy.ROSException:
            rospy.logwarn("No transformed coordinates available.")
            return None

    def get_robot_pose_orientation(self):
        """
        Retrieve the robot's position and orientation in the map frame.
        """
        try:
            transform = self.tf_buffer.lookup_transform('map', 'locobot/base_link', rospy.Time())
            position = [transform.transform.translation.x, transform.transform.translation.y]
            orientation = euler_from_quaternion([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ])
            return position, orientation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to get robot transform.")
            return None, None

    def is_point_inside_polygon(self, point_coords, boundary_coords):
        """
        Check if a point is within a given polygon.
        """
        point = Point(point_coords)
        polygon = Polygon(boundary_coords)
        result = point.within(polygon)
        rospy.loginfo(f"Point {point_coords} within polygon {boundary_coords}: {result}")
        return result

    def get_goal_yaw(self, boundary_name):
        """
        Retrieve the goal yaw for a specific boundary from the parameters.
        """
        try:
            return self.param_facing_boundaries[boundary_name].get('goal_yaw', 0.0)
        except KeyError:
            rospy.logerr(f"Goal yaw not defined for {boundary_name}.")
            return 0.0


if __name__ == "__main__":
    RealRobotFacing()
    rospy.spin()
