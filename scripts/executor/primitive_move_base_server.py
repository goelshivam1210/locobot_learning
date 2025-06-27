#!/usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import Twist
from locobot_learning.srv import PrimitiveBase, PrimitiveBaseResponse

class PrimitiveBaseServer:
    def __init__(self):
        rospy.init_node('primitive_base_server')

        # Determine the mode and choose cmd_vel topic
        mode = rospy.get_param("~mode", "real_robot")
        if mode == "simulation":
            cmd_vel_topic = "/locobot/cmd_vel"
        elif mode == "real_robot":
            cmd_vel_topic = "mobile_base/cmd_vel"
        elif mode == "kobuki_base":
            cmd_vel_topic = "/mobile_base/commands/velocity"
        else:
            rospy.logerr(f"[PrimitiveBaseServer] Invalid mode '{mode}'. Exiting.")
            sys.exit(1)

        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self.service = rospy.Service('primitive_base_service', PrimitiveBase, self.handle_primitive_action)

        rospy.loginfo(f"[PrimitiveBaseServer] Initialized on {cmd_vel_topic}")

    def handle_primitive_action(self, req):
        rospy.loginfo(f"[PrimitiveBaseServer] Received action: {req.action_type}, value: {req.value}")

        cmd = Twist()
        if req.action_type == "move_forward":
            cmd.linear.x = req.value
        elif req.action_type == "turn_left":
            cmd.angular.z = req.value
        elif req.action_type == "turn_right":
            cmd.angular.z = -req.value
        else:
            rospy.logerr(f"[PrimitiveBaseServer] Invalid action type: {req.action_type}")
            return PrimitiveBaseResponse(success=False)

        # Publish command for short duration
        rate = rospy.Rate(10)  # 10 Hz
        duration = rospy.Duration(1.0)  # 1 second
        end_time = rospy.Time.now() + duration

        while rospy.Time.now() < end_time:
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        # Stop the robot after action
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        rospy.loginfo(f"[PrimitiveBaseServer] Action {req.action_type} completed.")

        return PrimitiveBaseResponse(success=True)

if __name__ == "__main__":
    server = PrimitiveBaseServer()
    rospy.spin()
