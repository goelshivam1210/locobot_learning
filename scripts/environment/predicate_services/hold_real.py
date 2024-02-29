#!/usr/bin/env python3
import rospy
import time
from locobot_learning.srv import Hold, HoldResponse
from sensor_msgs.msg import JointState
from interbotix_xs_modules.locobot import InterbotixLocobotXS


class RealRobotHold(object):
    """
    The RealRobotFacing class initializes the ROS node and provides methods to detect colored objects 
    and estimate whether the robot is facing them, based on the camera and depth images.
    """

    def __init__(self):
        """
        Initializes the RealRobotFacing object, sets up subscribers, service, and transformation listener.
        """


        # rospy.init_node('RealRobotHold', anonymous=True)
        # rospy.loginfo("Initialized here")

        self.bot = InterbotixLocobotXS("locobot_wx200", arm_model="mobile_wx200")
        self.left_finger = 0
        self.right_finger = 0
        self.tolerance = 0.015 

        self.at_srv = rospy.Service('holding_obj', Hold, self.hold_callback)
        rospy.Subscriber("/locobot/joint_states", JointState, self.gripper_callback)

    def hold_callback(self, req):
        # self.bot.gripper.open()
        # Close gripper
        self.bot.gripper.close()
        time.sleep(1)
        rospy.loginfo("Close called")
        rospy.loginfo(f"Left: {self.left_finger}")
        rospy.loginfo(f"Right: {self.right_finger}")

        if (0 <= self.left_finger <= self.tolerance
            and -self.tolerance <= self.right_finger <= 0):
            return HoldResponse(False)
        else:
            return HoldResponse(True)
        

    def gripper_callback(self, gripper_msg):
        self.left_finger, self.right_finger = gripper_msg.position[-4], gripper_msg.position[-3]
        # rospy.loginfo(f"Left Pos: {self.left_finger}\nRight Pos: {self.right_finger}")

    
if __name__ == "__main__":
    RealRobotHold()
    rospy.spin()
