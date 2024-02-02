#!/usr/bin/env python3

import rospy
from locobot_learning.srv import Facing, FacingResponse
from visualization_msgs.msg import Marker


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
        self.tolerance = 0.1


        rospy.init_node('RealRobotFacing', anonymous=True)

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
            "yellow_marker": {"r": 0.72,
                            "g": 0.6,
                            "b": 0.2},
            "green_marker": {"r": 0.72,
                            "g": 0.6,
                            "b": 0.2},
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
        

    
    def facing_callback(self, req):
        """
        Service callback function to check whether the robot is facing a specified object.
        It calculates the angle and distance between the robot and the object and responds whether
        the robot is facing the object based on certain thresholds.
        """
        #To Do: Incorporate facing the doorway

        if req.obj not in self.model_to_pddl_mapping:
            return FacingResponse(False)

        model = self.model_to_pddl_mapping[req.obj]
        obj_colors = self.obj_color_mapping[model]


        red = 0
        green = 0
        blue = 0
        for m in self.measurements:
            red, green, blue += m.r, m.g, m.b
        
        red = red/len(self.measurements)
        green = green/len(self.measurements)
        blue = blue/len(self.measurements)

        rospy.loginfo("Measurements: " + self.measurements)
        rospy.loginfo("Averages --- Red: " + red + " Green: " + green + " Blue: " + blue)

        if ((obj_colors["red"] - self.tolerance  <= red <= obj_colors["red"] + self.tolerance) 
            and (obj_colors["green"] - self.tolerance  <= green <= obj_colors["green"] + self.tolerance)
            and (obj_colors["blue"] - self.tolerance  <= blue <= obj_colors["blue"] + self.tolerance)):
                rospy.loginfo(f"Object {req.obj} in tolerance range and detected!")
                return FacingResponse(True)
        else:
            rospy.loginfo(f"Object {req.obj} NOT detected")
            return FacingResponse(False)



    def marker_callback(self, marker_msg):
        if len(self.measurements) < 10:
            self.measurements.append(marker_msg.color)
        else:
            self.measurements[self.valCounter] = marker_msg.color
            self.valCounter += 1
        
        if self.valCounter > 9:
            self.valCounter = 0


if __name__ == "__main__":
    RealRobotFacing()
    rospy.spin()
