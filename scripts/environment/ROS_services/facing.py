#!/usr/bin/env python3

import rospy
from locobot_learning.srv import Facing, FacingResponse
from visualization_msgs.msg import Marker
from shapely.geometry import Point
from shapely.geometry import Polygon
import tf2_ros
import numpy as np
from tf.transformations import euler_from_quaternion


class RealRobotFacing(object):
    """
    The RealRobotFacing class initializes the ROS node and provides methods to detect colored objects 
    and estimate whether the robot is facing them, based on the camera and depth images.
    """

    def __init__(self):
        """
        Initializes the RealRobotFacing object, sets up subscribers, service, and transformation listener.
        """

        self.measurements = [] * 10
        self.valCounter = 0
        self.tolerance = 0.15


        rospy.init_node('RealRobotFacing', anonymous=True)

        try:
            self.param_facing_boundaries = rospy.get_param("facing_boundaries")
        except (KeyError, rospy.ROSException):
            rospy.logerr("Error getting parameters.")
            raise ValueError


        self.at_srv = rospy.Service('facing', Facing, self.facing_callback)
        rospy.Subscriber("/locobot/pc_filter/markers/objects", Marker, self.marker_callback)

        #Need to see how PDDL objects defined
        self.model_to_pddl_mapping = {
            "robot_1": "locobot",
            "marker_y": "yellow_marker",
            "marker_g": "green_marker",
            "o_ball": "orange_ball",
            "r_ball": "red_ball",
            "door_1": "door",
            "bin_1": "bin"
        }

        self.obj_color_mapping = {
            "yellow_marker": {"r": 0.594,
                            "g": 0.495,
                            "b": 0.287},
            "green_marker": {"r": 0.328,
                            "g": 0.499,
                            "b": 0.447},
            "orange_ball": {"r": 0.72,
                            "g": 0.6,
                            "b": 0.2},
            "red_ball": {"r": 0.72,
                            "g": 0.6,
                            "b": 0.2},
            "bin": {"r": 0.72,
                            "g": 0.6,
                            "b": 0.2},
            "door": {}
        }

        self.val_orientation_mapping = {
            #Need to move this out to config
            #Yaw lower and upper bound parameters
            "door": {
                "yl": -2.0,
                "yu": -1.0
            },
            "bin": {
                "yl": -0.45,
                "yu": 1.45
            }
        }

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        

    
    def facing_callback(self, req):
        """
        Service callback function to check whether the robot is facing a specified object.
        It calculates the angle and distance between the robot and the object and responds whether
        the robot is facing the object based on certain thresholds.
        """

        if req.obj not in self.model_to_pddl_mapping:
            rospy.loginfo("Object not in mapped models")
            return FacingResponse(False)

        model = self.model_to_pddl_mapping[req.obj]

        if model == "door" or model == "bin":
            return self.facing_zone(model, req.obj)
        else:
            return self.facing_color_obj(model, req.obj)

        


    def facing_zone(self, model, obj):
        #Point in polygon
        #Within upper/lower bounds
        robot_position, robot_pose = self.get_robot_pose_orientation()
        x_pos, y_pos = robot_position[0], robot_position[1]
        
        rospy.loginfo(f"Robot X position: {x_pos} \n Y Pos: {y_pos}")

        point = (x_pos, y_pos)

        _, _, yaw = euler_from_quaternion(robot_pose)

        lowerOrientation = self.val_orientation_mapping[model]["yl"]
        upperOrientation = self.val_orientation_mapping[model]["yu"]
        rospy.loginfo(f"Yaw: {yaw}")
        rospy.loginfo(f"YL: {lowerOrientation}")
        rospy.loginfo(f"YU: {upperOrientation}")

        # Check if point is in requested room
        boundary = self.param_facing_boundaries[model]
        rospy.loginfo(self.is_point_inside_polygon(point, boundary))

        if (self.is_point_inside_polygon(point, boundary) 
            and lowerOrientation <= yaw <= upperOrientation):
            rospy.loginfo(f"Robot is facing {obj}.")
            return FacingResponse(True)
        else:
            rospy.loginfo(f"Robot is NOT facing {obj}.")
            return FacingResponse(False)
        


    def facing_color_obj(self, model, obj):
        obj_colors = self.obj_color_mapping[model]


        red = 0
        green = 0
        blue = 0
        for m in self.measurements:
            red += m.r
            green += m.g
            blue += m.b
        
        if len(self.measurements) > 0:
            red = red/len(self.measurements)
            green = green/len(self.measurements)
            blue = blue/len(self.measurements)
        else:
            rospy.loginfo("No measurement data recorded")
            return FacingResponse(False)
        

        rospy.loginfo("Measurements: ")
        rospy.loginfo(self.measurements)
        rospy.loginfo(f"Averages --- Red: {red} Green: {green} Blue: {blue}")
        rospy.loginfo(obj_colors)

        if ((obj_colors["r"] - self.tolerance  <= red <= obj_colors["r"] + self.tolerance) 
            and (obj_colors["g"] - self.tolerance  <= green <= obj_colors["g"] + self.tolerance)
            and (obj_colors["b"] - self.tolerance  <= blue <= obj_colors["b"] + self.tolerance)):
                rospy.loginfo(f"Object {req.obj} in tolerance range and detected!")
                return FacingResponse(True)
        else:
            rospy.loginfo(f"Object {obj} NOT detected")
            rospy.loginfo(f"Object {obj} NOT detected")
            return FacingResponse(False)


    def marker_callback(self, marker_msg):
        # rospy.loginfo("At marker callback")
        # rospy.loginfo(self.measurements)
        if len(self.measurements) < 10:
            self.measurements.append(marker_msg.color)
        else:
            self.measurements[self.valCounter] = marker_msg.color
            self.valCounter += 1
        
        if self.valCounter > 9:
            self.valCounter = 0

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
    RealRobotFacing()
    rospy.spin()