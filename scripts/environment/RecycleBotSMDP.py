#!/usr/bin/env python3

import rospy
import random
import sys
import os
import math

# sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'core')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'action')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'state')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'reward')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'agent', 'core')))

from action_space import ActionSpace  # Custom action space class
from locobot_learning.srv import PrimitiveBase, Approach # ROS service definition
from PDDLActions import PDDLActions  # Custom PDDL actions class
from PDDLPredicates import PDDLPredicates  # Custom PDDL predicates class
from observation_space import ObservationSpace  # Custom observation space class
from reward_function import RewardFunction  # Custom reward function class
from planner.planner import Action

class RecycleBotSMDP:
    def __init__(
        self,
        failed_operator: Action,
        reward_function=None,
        include_local_view=True,
        include_symbolic_actions=False
    ):
        """
        Initializes the RecycleBotSMDP environment.
        Args:
            reward_function: Optional custom reward function. If None, uses default.
            include_local_view: Whether to include local view in the subsymbolic observation space, or only the is_obstructed bit.
        """

        # Initialize ROS node
        # rospy.init_node("recyclebot_action_test", anonymous=True)

        # Generate grounded symbolic actions
        all_actions = self.generate_grounded_symbolic_actions()
        self.grounded_actions = self.filter_valid_actions(all_actions) if include_symbolic_actions else []
        self.failed_operator = failed_operator

        # Initialize ActionSpace
        self.action_space = ActionSpace(
            self.grounded_actions,
            skip_action=failed_operator,
        )

        # Initialize PDDL actions client
        self.pddl_actions_client = PDDLActions()

        # Initialize PDDL predicates client
        self.pddl_predicates_client = PDDLPredicates()

        self.observation_space = ObservationSpace(include_local_view=include_local_view)
        rospy.wait_for_service('approach')

        self.approach_service = rospy.ServiceProxy('approach', Approach)

        self.reward_function = reward_function if reward_function else RewardFunction(set())

    def primitive_client(self, action_name, params):
        """
        Sends primitive move commands via ROS service.
        """
        rospy.wait_for_service('/primitive_base_service')
        try:
            primitive_service = rospy.ServiceProxy('/primitive_base_service', PrimitiveBase)
            velocity = params["velocity"]
            primitive_service(action_type=action_name, value=velocity)
            rospy.loginfo(f"[PrimitiveClient] Executed primitive action: {action_name}")
        except rospy.ServiceException as e:
            rospy.logerr(f"[PrimitiveClient] Service call failed: {e}")

    def prepare_for_reset(self):
        input("Resetting robot for new episode. Press Enter key when environment is set for reset.")
        approach_point = None
        # Currently hard-coded for curtain novelty
        if self.failed_operator.name == "pass_through_door":
            approach_point = "atdoor" 
        # ...or for bin obstruction novelty
        elif self.failed_operator.name == "approach" and self.failed_operator.parameters[0] == "bin_1":
            approach_point = "postdoor"
        else:
            rospy.logwarn(f"[RecycleBotSMDP] reset(): Don't know how to automatically reset for operator {self.failed_operator}. Please manually reset the environment.")
        if approach_point is not None:
            try:
                response = self.approach_service(approach_point)
                if not response.success:
                    rospy.logerr(f"[RecycleBotSMDP] reset(): Approaching '{approach_point}' was unsuccessful")

            except rospy.ServiceException as e:
                rospy.logerr(f"[RecycleBotSMDP] reset(): Service call failed: {e}")


    def prompt_for_learning(self):
      input("Resetting complete. Press Enter when robot should start learning again.")

    def reset(self):
        """
        Resets the state of the robot in order to prepare for starting a new episode.

        Currently relies on human intervention to do anything necessary to allow it to
        complete its reset (e.g. moving curtain away).
        """

        self.prepare_for_reset()
        self.prompt_for_learning()

  
    def step(self, action_id):
        """
        Executes the action and returns (obs, reward, done, info)
        """

        action = self.action_space.get_action(action_id)
        if (self.is_noop_action(action)):
            return self.observation_space.get_observation(), 0.0, False, {}

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

        reward, done = self.reward_function.compute_reward(self.observation_space)
        rospy.loginfo(f"[RecycleBotSMDP] Action executed: {action['name']}, params: {action['params']}, reward: {reward}, done: {done}")

        return obs, reward, done, {}
    
    def check_forward_collision(self):
        obs = self.observation_space.subsymbolic_state.get_subsymbolic_observation()
        if obs is None:
            rospy.logwarn("[RecycleBotSMDP] Subsymbolic observation is None; cannot check for collision.")
            return False
        parsed_state = self.observation_space.subsymbolic_state.parse_observation(obs)
        
        return parsed_state.is_obstructed

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

        #DEBUG: Right now, for the Curtain Novelty, we are hard coding these values
        #TODO: Determine these values based on the failed operator
        include_arm_actions = False
        include_move_actions = True
        #END DEBUG
        valid = []
        for name, params in actions:
            if name in ["pick", "place"] and not include_arm_actions:
                continue
            if name in ["approach", "pass_through_door"] and not include_move_actions:
                continue
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
    
    def is_noop_action(self, action):
        """
        Checks if the action is a no-op (does nothing).
        """
        
        if action["type"] == "primitive" and action["name"] == "move_forward":
            if self.check_forward_collision():
                rospy.loginfo("[RecycleBotSMDP] move_forward action would result in collision; not executing action")
                return True
        
        # Prevent any action that would take the robot into room 1
        # TODO: Eliminate this code
        if action["type"] == "symbolic":
            if  action["name"] == "approach":
                room = action["params"][1]  # The room to approach
                if room == "room_1":
                    rospy.loginfo("[RecycleBotSMDP] approach action would take robot into room 1; not executing action")
                    return True

        return False

def main():
    rospy.init_node("recyclebot_node", anonymous=True)
    rospy.loginfo("Starting RecycleBotSMDP node...")
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
