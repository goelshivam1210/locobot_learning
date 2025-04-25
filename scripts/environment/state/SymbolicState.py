#!/usr/bin/env python3

import rospy
from locobot_learning.srv import At, Hold, Facing
import numpy as np

class SymbolicState:
    def __init__(self):
        """
        Initializes the SymbolicState module by connecting to the necessary ROS services.
        """
        rospy.wait_for_service('at')
        rospy.wait_for_service('hold')
        rospy.wait_for_service('facing')

        self.at_srv = rospy.ServiceProxy('at', At)
        self.hold_srv = rospy.ServiceProxy('hold', Hold)
        self.facing_srv = rospy.ServiceProxy('facing', Facing)

        self.facing_options = ["generic_object", "table", "bin_1", "doorway_1", "nothing"]
        self.room_options = ["room_1", "room_2"]

    def get_symbolic_state(self):
        """
        Queries the ROS services and returns a one-hot encoded symbolic state.
        """
        # Determine current room
        room_encoding = [0.0] * len(self.room_options)
        for i, room in enumerate(self.room_options):
            try:
                res = self.at_srv("robot_1", room)
                if res.obj_at_location:
                    room_encoding[i] = 1.0
                    break
            except rospy.ServiceException as e:
                rospy.logerr(f"AT service failed: {e}")

        # Holding status (True/False)
        try:
            hold_res = self.hold_srv("ball_1")
            holding_encoding = [1.0] if hold_res.holding else [0.0]
        except rospy.ServiceException as e:
            rospy.logerr(f"HOLD service failed: {e}")
            holding_encoding = [0.0]

        # Facing one-hot
        facing_encoding = [0.0] * len(self.facing_options)
        for i, obj in enumerate(self.facing_options):
            try:
                face_res = self.facing_srv(obj)
                if face_res.robot_facing_obj:
                    facing_encoding[i] = 1.0
                    break
            except rospy.ServiceException as e:
                rospy.logerr(f"FACING service failed: {e}")

        return np.array(room_encoding + holding_encoding + facing_encoding, dtype=np.float32)

if __name__ == "__main__":
    rospy.init_node("symbolic_state_tester")
    state = SymbolicState()
    rospy.sleep(1.0)  # allow time for services to be ready
    symbolic_vec = state.get_symbolic_state()
    print("Symbolic state vector:", symbolic_vec)
