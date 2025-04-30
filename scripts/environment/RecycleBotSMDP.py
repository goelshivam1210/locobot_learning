#!/usr/bin/env python3

import rospy
import random
import sys
import os

# sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'core')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'action')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'state')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'agent', 'core')))


from action_space import ActionSpace  # Custom action space class
from locobot_learning.srv import PrimitiveBase  # ROS service definition
from PDDLActions import PDDLActions  # Custom PDDL actions class
from PDDLPredicates import PDDLPredicates  # Custom PDDL predicates class
from observation_space import ObservationSpace  # Custom observation space class

class RecycleBotSMDP:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("recyclebot_action_test", anonymous=True)

        # Generate grounded symbolic actions
        all_actions = self.generate_grounded_symbolic_actions()
        self.grounded_actions = self.filter_valid_actions(all_actions)

        # Initialize ActionSpace
        self.action_space = ActionSpace(self.grounded_actions)

        # Initialize PDDL actions client
        self.pddl_actions_client = PDDLActions()

        # Initialize PDDL predicates client
        self.pddl_predicates_client = PDDLPredicates()

        self.observation_space = ObservationSpace()

    def primitive_client(self, action_name, value=0.2):
        """
        Sends primitive move commands via ROS service.
        """
        rospy.wait_for_service('/primitive_base_service')
        try:
            primitive_service = rospy.ServiceProxy('/primitive_base_service', PrimitiveBase)
            primitive_service(action_type=action_name, value=value)
            rospy.loginfo(f"[PrimitiveClient] Executed primitive action: {action_name}")
        except rospy.ServiceException as e:
            rospy.logerr(f"[PrimitiveClient] Service call failed: {e}")

    def step(self, action_id):
        """
        Executes the action and returns (obs, reward, done, info)
        """
        executed = self.action_space.execute_action(
            action_id,
            primitive_client=self.primitive_client,
            pddl_actions_client=self.pddl_actions_client,
            predicate_checker=self.pddl_predicates_client
        )

        if not executed:
            # Penalize invalid symbolic actions
            return self.observation_space.get_observation(), -1.0, False, {"failure": "precondition_failed"}

        obs = self.observation_space.get_observation()
        # reward = self.compute_reward(obs)
        reward = 0.0
        # done = self.check_done(obs)
        done = False
        return obs, reward, done, {}

    def generate_grounded_symbolic_actions(self):
        """
        Returns a static list of grounded symbolic actions (name, parameters).
        """
        objects = {
            "object": ["ball_1", "can_1"],
            "room": ["room_1", "room_2"],
            "container": ["bin_1"],
            "doorway": ["doorway_1"]
        }

        actions = []

        for obj in objects["object"] + objects["container"] + objects["doorway"]:
            for room in objects["room"]:
                actions.append(("approach", [obj, room, "nothing"]))

        for obj in objects["object"]:
            for room in objects["room"]:
                actions.append(("pick", [obj, room]))

        for obj in objects["object"]:
            for room in objects["room"]:
                for container in objects["container"]:
                    actions.append(("place", [obj, room, container]))

        for r1 in objects["room"]:
            for r2 in objects["room"]:
                if r1 != r2:
                    for doorway in objects["doorway"]:
                        actions.append(("pass_through_door", [r1, r2, doorway]))

        return actions
    
    def filter_valid_actions(self, actions):
        """
        Filters grounded symbolic actions based on known truths about the world.
        """
        valid = []
        for name, params in actions:
            # Filter out any actions involving can_1
            if "can_1" in params:
                continue

            # Filter: ball_1 is not in room_2
            if name in ["approach", "pick"] and params[0] == "ball_1" and "room_2" in params:
                continue

            # Filter: bin_1 is not in room_1
            if name == "approach" and params[0] == "bin_1" and params[1] == "room_1":
                continue
            if name == "place" and params[2] == "bin_1" and params[1] == "room_1":
                continue

            valid.append((name, params))
        return valid

def main():
    env = RecycleBotSMDP()

    rospy.loginfo(f"Action space size: {env.action_space.size}")
    rospy.loginfo(f"Grounded symbolic actions: {env.grounded_actions}")
    rospy.loginfo(f"Action space: {env.action_space.action_list}")
    rate = rospy.Rate(0.5)  # One action every 2 seconds

    for i in range(10):
        if rospy.is_shutdown():
            break

        action_id = random.randint(0, env.action_space.size - 1)
        action = env.action_space.get_action(action_id)
        rospy.loginfo(f"[MAIN] Executing action {action_id}: {action}")
        env.step(action_id)
        rate.sleep()

if __name__ == "__main__":
    main()
