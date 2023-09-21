#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPoseSetter:
    def __init__(self):
        rospy.init_node('initial_pose_setter', anonymous=True)
        
        print("[INFO] Initializing initial_pose_setter node...")

        # Publisher for the initial pose
        self.initial_pose_pub = rospy.Publisher('/locobot/rtabmap/localization_pose', PoseWithCovarianceStamped, queue_size=10)
        
        # Flag to ensure we only set the initial pose once
        self.initial_pose_set = False
        rospy.sleep(0.1)
        # Subscribe to the gazebo model states topic
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)
        
        print("[INFO] Waiting for messages on /gazebo/model_states...")
        
        # Wait for messages
        rospy.spin()
        

    def model_states_callback(self, data):
        # If we've already set the initial pose, return early
        if self.initial_pose_set:
            print("[INFO] Initial pose already set. Ignoring subsequent messages.")
            return

        # The robot's name in Gazebo is 'locobot', find its index
        try:
            locobot_index = data.name.index('locobot')
            print("[INFO] Found locobot in model states!")
        except ValueError:
            rospy.logerr("Locobot model not found in Gazebo model states!")
            return
        
        # Fetch the pose of the locobot
        locobot_pose = data.pose[locobot_index]
        
        print(f"[INFO] Locobot Pose - Position: {locobot_pose.position}, Orientation: {locobot_pose.orientation}")
        
        # Create the initial pose message
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.stamp = rospy.Time.now()
        initial_pose_msg.header.frame_id = "map"
        initial_pose_msg.pose.pose = locobot_pose
        
        # Set a small covariance value to indicate high confidence in the initial pose
        covariance_value = 0.1
        initial_pose_msg.pose.covariance = [covariance_value] * 36  # 6x6 covariance matrix
        
        # Publish the initial pose
        self.initial_pose_pub.publish(initial_pose_msg)
        
        print("[INFO] Published initial pose to /locobot/initialpose!")
        
        # Set the flag to indicate we've set the initial pose
        self.initial_pose_set = True

if __name__ == '__main__':
    InitialPoseSetter()