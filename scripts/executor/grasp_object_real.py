#!/usr/bin/env python3

import rospy
import math
import time
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, Quaternion, Point
from moveit_msgs.msg import PositionIKRequest
import moveit_commander
from moveit_msgs.srv import GetPositionIK
from interbotix_xs_modules.locobot import InterbotixLocobotXS
from sensor_msgs.msg import JointState
# from visualization_msgs.msg import Marker
import tf2_geometry_msgs #pleasthis helps in the tf2 transform error and exception


class Grasp:
    def __init__(self) -> None:
        self.arm_tf = "locobot/arm_base_link"

        # This initializes the robots's arm
        self.bot = InterbotixLocobotXS("locobot_wx200", arm_model="mobile_wx200")
        self.bot.camera.pan_tilt_move(0, 0.75)  # Set camera tilt angle

        # Resets arm position to ensure arm is ready to move to any position
        self.bot.arm.set_ee_pose_components(x=0.3, z=0.2)  # Y defaults to 0
        self.bot.arm.go_to_sleep_pose()

        self.pub_coordinates = rospy.Publisher("grasp_pose", PoseStamped, queue_size=1000)

        self.arm_group = moveit_commander.MoveGroupCommander(
            robot_description="/locobot/robot_description",
            name="interbotix_arm",
            ns="locobot",
        )
        
        self.arm_moving = False  # Flag to track if the arm is already moving

        # Subscribe to the transformed_markers topic
        # rospy.Subscriber("/locobot/transformed_markers", Marker, self.marker_callback)
        # rospy.Subscriber("/locobot/pc_filter/markers/objects", Marker, self.marker_callback)
        rospy.Subscriber("/transformed_coordinates", PoseStamped, self.marker_callback)
    

    def marker_callback(self, marker_msg):
        rospy.loginfo("callback is called")
        # Extract the position from the Marker message
        x = marker_msg.pose.position.x
        y = marker_msg.pose.position.y
        z = marker_msg.pose.position.z

        # Create a PoseStamped message for arm movement
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.arm_tf
        pose.pose.position = Point(x, y, z)
        pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # Set orientation as needed
        
        # Log the pose information
        rospy.loginfo(f"Received marker pose: x={x}, y={y}, z={z}")

        # Move the arm to the specified pose
        self.move_arm(pose)

    def move_arm(self, pose):
        if not self.arm_moving:
            self.pub_coordinates.publish(pose)

            # Extract position and orientation from the pose
            x, y, z = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
            rospy.loginfo(f"Moving arm to pose: x={x}, y={y}, z={z}")
            roll, pitch, yaw = 0.0, 0.0, 0.0  # Set the desired orientation as needed
            yaw = math.atan2(pose.pose.position.y, pose.pose.position.x)


            # Open gripper
            rospy.loginfo("Opening gripper")
            self.bot.gripper.open()
            time.sleep(0.2)

            # Move the arm
            rospy.loginfo("Moving arm")
            self.bot.arm.set_ee_pose_components(x, y, z, roll, pitch, yaw)
            time.sleep(0.2)

            # Close gripper
            self.bot.gripper.close()
            time.sleep(0.2)

            # Go to sleep pose
            self.bot.arm.go_to_sleep_pose()
            time.sleep(0.1)
            self.arm_moving = True  # Reset the flag once the arm movement is complete
            rospy.loginfo("Done")
            time.sleep(5)

            # Unsubscribe from the /transformed_coordinates topic to prevent further arm movements
            rospy.loginfo("Unsubscribing from /transformed_coordinates topic")
            self.subscriber.unregister()
            rospy.Subscriber("/transformed_coordinates", PoseStamped, self.marker_callback, callback_args=None, queue_size=1)
if __name__ == "__main__":
    # rospy.init_node("grasp_node")  # Initialize the ROS node here
    Grasp()
    rospy.spin()
