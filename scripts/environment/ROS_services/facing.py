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

        # Service and subscriber
        self.facing_srv = rospy.Service('facing', Facing, self.facing_callback)
        
        # Create a buffer to store recent marker messages from the topic
        self.marker_sub = rospy.Subscriber("/locobot/pc_filter/markers/objects", Marker, self.marker_callback)
        self.recent_marker = None  # Store the recent marker message

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
            "atdoor": "atdoor",       # Add atdoor
            "postdoor": "postdoor"    # Add postdoor
        }

    def facing_callback(self, req):
        """
        Service callback function to check whether the robot is facing a specified object.
        """
        if req.obj not in self.model_to_pddl_mapping:
            rospy.loginfo("Object not in mapped models")
            return FacingResponse(False)

        model = self.model_to_pddl_mapping[req.obj]

        if model in ["door", "bin", "table", "atdoor", "postdoor"]:
            return self.facing_zone(model, req.obj)
        elif model == "generic_object":
            if not self.is_facing_any_stationary_object():
                return self.facing_generic_object()

        return self.facing_nothing()

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

        rospy.loginfo(f"Robot X position: {x_pos}, Y position: {y_pos}")

        point = (x_pos, y_pos)

        # Convert the robot's quaternion orientation to Euler angles (yaw)
        _, _, yaw = euler_from_quaternion(robot_pose)

        # Extract the boundary and threshold for stationary objects
        boundary = self.param_facing_boundaries.get(model, {}).get('boundary', None)
        if not boundary:
            rospy.logerr(f"Boundary not found for {model}")
            return FacingResponse(False)

        yaw_threshold = self.param_facing_boundaries.get(model, {}).get('threshold', 0.5)  # Default yaw threshold

        # Extract target yaw from real_nav_goals.yaml
        goal_yaw = self.get_goal_yaw_from_nav_goals(model)

        # Yaw thresholds for model (e.g., table, bin, atdoor, or postdoor)
        lower_orientation = goal_yaw - yaw_threshold
        upper_orientation = goal_yaw + yaw_threshold
        
        rospy.loginfo(f"Yaw: {yaw}")
        rospy.loginfo(f"Yaw lower limit: {lower_orientation}, Yaw upper limit: {upper_orientation}")

        if self.is_point_inside_polygon(point, boundary):
            # Check if the robot is facing the stationary object (table/bin/door/atdoor/postdoor)
            if lower_orientation <= yaw <= upper_orientation:
                rospy.loginfo(f"Robot is facing {obj}.")
                return FacingResponse(True)

        rospy.loginfo(f"Robot is NOT facing {obj}.")
        return FacingResponse(False)

    def get_goal_yaw_from_nav_goals(self, model):
        """
        Extract yaw (orientation) from real_nav_goals.yaml for the given model (table, bin, atdoor, postdoor, etc.).
        """
        try:
            quaternion = self.param_nav_goals[model]['orientation']
            _, _, goal_yaw = euler_from_quaternion([quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']])
            return goal_yaw
        except KeyError:
            rospy.logerr(f"No yaw found for {model} in real_nav_goals.yaml.")
            return 0  # Default value

    def facing_generic_object(self):
        """
        Logic for facing a generic object:
        - Check if a marker has been detected dynamically.
        - Transform marker coordinates and use robot's orientation to determine facing.
        """
        if self.recent_marker is None:
            rospy.loginfo("No recent marker detected, not facing the generic object.")
            return FacingResponse(False)

        # Transform the marker position to the 'map' frame
        transformed_marker_position = self.transform_marker_to_map_frame(self.recent_marker.pose.position)
        if transformed_marker_position is None:
            return FacingResponse(False)

        robot_position, robot_pose = self.get_robot_pose_orientation()
        if robot_position is None or robot_pose is None:
            return FacingResponse(False)

        rospy.loginfo("Marker detected, robot is facing the generic object.")
        return FacingResponse(True)

    def facing_nothing(self):
        """
        Check if the robot is facing 'nothing', meaning no object or marker is detected.
        """
        if self.recent_marker is None:
            rospy.loginfo("Robot is facing nothing.")
            return FacingResponse(True)
        else:
            rospy.loginfo("Robot is NOT facing nothing, an object is detected.")
            return FacingResponse(False)

    def is_facing_any_stationary_object(self):
        """
        Check if the robot is currently facing any stationary object (door, bin, table, atdoor, postdoor).
        """
        for obj in ["door", "bin", "table", "atdoor", "postdoor"]:
            if self.facing_zone(obj, obj).robot_facing_obj:
                return True
        return False

    def marker_callback(self, marker_msg):
        """
        Callback to update the recent marker information from the topic.
        """
        if marker_msg.pose.position:  # Check if there is any position data in the message
            rospy.loginfo(f"Detected object position: ({marker_msg.pose.position.x}, {marker_msg.pose.position.y})")
            self.recent_marker = marker_msg  # Store the recent marker message
        else:
            rospy.loginfo("No object detected.")
            self.recent_marker = None  # No object detected

    def transform_marker_to_map_frame(self, marker_position):
        """
        Transforms the marker position to the 'map' frame.
        """
        try:
            transform = self.tf_buffer.lookup_transform('map', 'locobot/camera_color_optical_frame', rospy.Time())

            point_in_camera = PointStamped()
            point_in_camera.header.frame_id = 'locobot/camera_color_optical_frame'
            point_in_camera.point = marker_position

            point_in_map = tf2_geometry_msgs.do_transform_point(point_in_camera, transform)
            return [point_in_map.point.x, point_in_map.point.y]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Error transforming marker: {e}")
            return None

    def is_point_inside_polygon(self, point_coords, boundary_coords):
        """
        Check if a given point is inside a polygon defined by boundary coordinates.
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
    RealRobotFacing()
    rospy.spin()
