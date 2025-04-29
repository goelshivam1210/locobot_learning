#!/usr/bin/env python3

import rospy
import numpy as np
from locobot_learning.srv import At, AtRequest, Hold, HoldRequest, Facing, FacingRequest

class SymbolicState:
    def __init__(self):
        """
        Initializes SymbolicState by setting up ROS service proxies for AT, HOLD, and FACING.
        """
        rospy.wait_for_service('/at')
        rospy.wait_for_service('/hold')
        rospy.wait_for_service('/facing')

        self.at_client = rospy.ServiceProxy('/at', At)
        self.hold_client = rospy.ServiceProxy('/hold', Hold)
        self.facing_client = rospy.ServiceProxy('/facing', Facing)

        self.room_options = ["room_1", "room_2"]
        self.facing_options = ["generic_object", "table", "bin_1", "doorway_1", "nothing"]
        # Load the nav_goals from the parameter server
        try:
            self.nav_goals = rospy.get_param("real_nav_goals")
        except rospy.ROSException as e:
            rospy.logwarn("real_nav_goals not found on param server.")
            self.nav_goals = {}

    def get_symbolic_state(self):
        """
        Queries symbolic services and returns a single concatenated observation vector.
        """

        room_encoding = self._query_robot_room()
        holding_encoding = self._query_robot_holding()
        facing_encoding = self._query_robot_facing()

        obs_vector = np.array(room_encoding + holding_encoding + facing_encoding, dtype=np.float32)
        return obs_vector

    def _query_robot_room(self):
        room_encoding = [0.0] * len(self.room_options)
        for idx, room in enumerate(self.room_options):
            try:
                req = AtRequest(obj="robot_1", room=room)
                resp = self.at_client(req)
                if resp.obj_at_room:
                    room_encoding[idx] = 1.0
                    break
            except rospy.ServiceException as e:
                rospy.logerr(f"Error calling /at service for robot location: {e}")
        return room_encoding

    def _query_robot_holding(self):
        try:
            req = HoldRequest(obj="ball_1")  # Assuming ball_1 is the object to check
            resp = self.hold_client(req)
            return [1.0] if resp.robot_holding_obj else [0.0]
        except rospy.ServiceException as e:
            rospy.logerr(f"Error calling /hold service: {e}")
            return [0.0]

    def _query_robot_facing(self):
        facing_encoding = [0.0] * len(self.facing_options)
        for idx, obj in enumerate(self.facing_options):
            try:
                req = FacingRequest(obj=obj)
                resp = self.facing_client(req)
                if resp.robot_facing_obj:
                    facing_encoding[idx] = 1.0
                    break
            except rospy.ServiceException as e:
                rospy.logerr(f"Error calling /facing service: {e}")
        return facing_encoding

if __name__ == "__main__":
    rospy.init_node('symbolic_state_tester')
    state_module = SymbolicState()
    rospy.sleep(1.0)  # Give time for services to stabilize

    try:
        obs_vector = state_module.get_symbolic_state()
        print("Symbolic Observation Vector:", obs_vector)
        print("Size of observation:", obs_vector.shape)
    except rospy.ROSInterruptException:
        pass
