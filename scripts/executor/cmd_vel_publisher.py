#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys

def publish_velocity(linear_vel, angular_vel, duration=10):
    # Initialize the ROS node
    rospy.init_node('cmd_vel_publisher_node')
    
    # Create a publisher for the /locobot/cmd_vel topic
    cmd_vel_pub = rospy.Publisher('/locobot/cmd_vel', Twist, queue_size=10)
    
    # Construct the Twist message
    twist_msg = Twist()
    twist_msg.linear.x = linear_vel
    twist_msg.angular.z = angular_vel
    
    # Publish the message for the specified duration
    rate = rospy.Rate(10)  # 10Hz
    end_time = rospy.Time.now() + rospy.Duration(duration)
    while rospy.Time.now() < end_time:
        cmd_vel_pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: cmd_vel_publisher.py <linear_velocity> <angular_velocity>")
        sys.exit(1)

    linear_velocity = float(sys.argv[1])
    angular_velocity = float(sys.argv[2])
    
    publish_velocity(linear_velocity, angular_velocity)