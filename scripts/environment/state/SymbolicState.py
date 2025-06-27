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

    def predicates_from_vector(self, vector: np.ndarray):
        """
        Converts the observation vector into a set of predicates.

        Args:
            vector (np.ndarray): The observation vector, in the format returned by get_symboilic_state().
        
        Returns:
            set: A set of predicates representing the symbolic state. Each predicate is a tuple of strings of the form (predicate_name, arg1, ...).
        
        """
        
        start_index = 0
        # The first len(self.room_options) elements are room encodings
        room_dict = {room: vector[start_index + idx] == 1.0 for idx, room in enumerate(self.room_options)}
        start_index += len(self.room_options)
        # The next element is the holding encoding
        holding = vector[start_index] == 1.0
        start_index += 1
        # The remaining elements are facing encodings
        facing_dict = {obj: vector[start_index + idx] == 1.0 for idx, obj in enumerate(self.facing_options)}

        predicates = set()

        for room_id, present in room_dict.items():
            if present:
                # Add an `at` predicate for the robot's location
                predicates.add(('at', room_id, 'robot_1'))
                # There can be only one room where the robot is located
                # so we can break after finding the first one
                break
        
        if holding:
            # If the robot is holding an object, add a `hold` predicate (assume the object is 'ball_1')
            predicates.add(('hold', 'ball_1'))

        for obj_id, present in facing_dict.items():
            if present:
                # Add a `facing` predicate for the object the robot is facing
                predicates.add(('facing', obj_id))
                # There can be only one object the robot is facing
                # so we can break after finding the first one
                break
        
        return predicates

    def get_current_predicates(self):
        """
        Returns the current predicates based on the symbolic state.

        This is a convenience wrapper around get_symbolic_state() and predicates_from_vector().

        Returns:
            set: A set of predicates representing the current symbolic state.
        """
        obs_vector = self.get_symbolic_state()
        return self.predicates_from_vector(obs_vector)

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
