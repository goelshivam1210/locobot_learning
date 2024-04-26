#!/usr/bin/env python3

import rospy
import math
import time
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
import tf2_geometry_msgs  # this helps in the tf2 transform error and exception
from sensor_msgs.msg import JointState
from locobot_learning.srv import GraspObject, GraspObjectResponse
from locobot_learning.srv import DropObject, DropObjectResponse
from locobot_learning.srv import DropObjectAside, DropObjectAsideResponse
from locobot_learning.srv import Hold, HoldResponse
from moveit_msgs.msg import PositionIKRequest
import moveit_commander
from moveit_msgs.srv import GetPositionIK
from interbotix_xs_modules.locobot import InterbotixLocobotXS

class ArmServer:
    def __init__(self):
        # Initialize the ROS node
        # rospy.init_node("arm_server")

        # Initialize the robot arm
        self.bot = InterbotixLocobotXS("locobot_wx200", arm_model="mobile_wx200")
        self.bot.camera.pan_tilt_move(0, 0.75)  # Set camera tilt angle
        self.arm_group = moveit_commander.MoveGroupCommander(
            robot_description="/locobot/robot_description",
            name="interbotix_arm",
            ns="locobot",
        )

        self.intermediate_sleep_pose = [-0.00920388475060463,
                       -1.3054176568984985, 1.5646604299545288,0.2,
                         0.0]
        self.place_pose = [-0.07363107800483704, 0.22549518942832947, 0.06289321184158325,
                           0.22396120429039001, -0.004601942375302315]

        self.pub_coordinates = rospy.Publisher("grasp_pose", PoseStamped, queue_size=1000)
        self.arm_tf = "locobot/arm_base_link"


        # Define the arm pick and drop services
        self.grasp_service = rospy.Service('grasp_object', GraspObject, self.handle_grasp_object)
        self.drop_service = rospy.Service('drop_object', DropObject, self.handle_drop_object)
        self.drop_aside_service = rospy.Service('drop_aside', DropObjectAside, self.handle_drop_aside)

        # Define the predicate services
        self.holding_service = rospy.Service('hold', Hold, self.handle_hold)
        # holding service global variables
        self.left_finger = 0
        self.right_finger = 0
        self.tolerance = 0.015 

        # Subscribe to the transformed coordinates topic
        rospy.Subscriber("/transformed_coordinates", PoseStamped, self.marker_callback)
        rospy.Subscriber("/locobot/joint_states", JointState, self.gripper_callback)


        self.pose = None

    def marker_callback(self, marker_msg):
        # Extract the position from the Marker message
        x = marker_msg.pose.position.x
        y = marker_msg.pose.position.y
        z = marker_msg.pose.position.z

        # Create a PoseStamped message for arm movement
        self.pose = PoseStamped()
        self.pose.header.stamp = rospy.Time.now()
        self.pose.header.frame_id = self.arm_tf
        self.pose.pose.position = Point(x, y, z)
        self.pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # Set orientation as needed
        
        # Log the pose information
        rospy.loginfo_once(f"Received marker pose: x={x}, y={y}, z={z}")


    def handle_grasp_object(self, request):
        # Ensure that cartesian coordinates are available
        if self.pose is None:
            rospy.logwarn("Cartesian coordinates not available. Cannot perform grasp.")
            return GraspObjectResponse(success=False)

        # Extract cartesian coordinates
        x, y, z = self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z

        # Move the arm to the specified pose
        flag = False
        if request.target_object == 'novel':
            flag = True 
            z += 0.07  # Adjust the z-coordinate if necessary
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "locobot/arm_base_link"
        pose.pose.position = Point(x, y, z)
        pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # Set orientation as needed
        
        # Open gripper
        rospy.loginfo("Opening gripper")
        self.bot.gripper.open()
        time.sleep(0.2)

        # move arm
        self.move_arm(pose, flag)

        # Close gripper
        rospy.loginfo("Closing gripper")
        self.bot.gripper.close()
        time.sleep(0.2)

        rospy.loginfo("Object grasped.")

        rospy.loginfo("Return to intermediate sleep pose")
        self.bot.arm.go_to_home_pose()
        self.bot.arm.set_joint_positions(joint_positions = self.intermediate_sleep_pose)
        return GraspObjectResponse(success=True)

    def move_arm(self, pose, flag):
        # Publish the pose for visualization
        self.pub_coordinates.publish(pose)
        # Extract position and orientation from the pose
        x, y, z = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z 
        roll, pitch, yaw = 0.0, 0.0, 0.0  # Set the desired orientation as needed
        yaw = math.atan2(pose.pose.position.y, pose.pose.position.x)  
        pitch = 0.0   
        if flag:
            z +=0.02
        # Move the arm
        rospy.loginfo(f"Moving arm to pose: x={x}, y={y}, z={z}")
        self.bot.arm.set_ee_pose_components(x, y, z, roll, pitch, yaw)
        time.sleep(0.2)

    def handle_drop_object(self, request):
        
        self.bot.arm.go_to_home_pose()
        self.bot.gripper.open()
        time.sleep(0.2)
        self.bot.gripper.close()
        time.sleep(0.2)
        self.bot.arm.go_to_sleep_pose()

        return DropObjectResponse(success=True)

    def handle_drop_aside(self, request):
        # this will be later called placce object
        
        drop_aside_pose = [-1.1, 0, 0, 0, 0]
        drop_aside_pose = [0, 0, 0, 0.2, 0]
        self.bot.arm.set_joint_positions(drop_aside_pose)
        # self.bot.arm.set_joint_positions(self.place_pose)
        # self.bot.arm.go_to_home_pose()
        self.bot.gripper.open()
        time.sleep(0.2)
        # self.bot.arm.go_to_home_pose()
        self.bot.arm.go_to_sleep_pose()
        self.bot.gripper.close()
        # time.sleep(0.2)
        
        return DropObjectAsideResponse(success = True)
    

    def handle_hold(self, request):

        self.bot.gripper.close()
        time.sleep(1)
        rospy.loginfo("Close called")
        rospy.loginfo(f"Left: {self.left_finger}")
        rospy.loginfo(f"Right: {self.right_finger}")

        if (0 <= self.left_finger <= self.tolerance
            and -self.tolerance <= self.right_finger <= 0):
            return HoldResponse(robot_holding_obj = False)
        else:
            return HoldResponse(robot_holding_obj = True)
        
    def gripper_callback(self, gripper_msg):
        self.left_finger, self.right_finger = gripper_msg.position[-4], gripper_msg.position[-3]
        rospy.loginfo_once(f"Left Pos: {self.left_finger}\nRight Pos: {self.right_finger}")

if __name__ == "__main__":
    ArmServer()
    rospy.spin()
