# core/HybridAgent.py

import sys
import os
import rospy

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'environment')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'planner')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'learner')))

from planner.planner import Planner
from PDDLActions import PDDLActions
from PDDLPredicates import PDDLPredicates
from learner.LearningAgent import LearningAgent
from Agent import Agent
from exceptions import ActionExecutionError

from learner.LearningAgent import LearningAgent
from environment.reward.reward_function import RewardFunction
from environment.RecycleBotSMDP import RecycleBotSMDP
from learner.PPO import PPO

class HybridAgent:
    def __init__(self, domain_file, predicate_funcs, objects, max_retries=3):
        """
        Initializes the hybrid agent with planning and learning capabilities.
        """
        self.planner = Planner(domain_file, predicate_funcs)
        self.actions = PDDLActions()
        self.predicates = PDDLPredicates()
        self.agent = Agent(self.planner, self.actions, self.predicates)
        self.objects = objects
        self.max_retries = max_retries

    def run(self):
        """
        Execute the plan and handle failures dynamically.
        """

        try:
            self.agent.run(self.objects)
        except ActionExecutionError as e:
            rospy.logwarn(f"[HybridAgent] Action failed: {e} | Action: {e.action_name}, Params: {e.params}")
            self.invoke_learning(e.action_name, e.params)

        rospy.loginfo("Plan executed successfully.")


    def invoke_learning(self, action_name, params):
        """
        Handles transition to learning mode on execution failure.
        Computes plannable states for recovery.
        """
        rospy.logwarn(f"[HybridAgent] Switching to learning mode for action: {action_name} {params}")

        failed_op = self.find_failed_operator(action_name, params)
        if failed_op is None:
            rospy.logerr("[HybridAgent] Cannot compute plannable states without failed operator.")
            return

        plan = self.planner.plan
        if plan is None:
            rospy.logerr("[HybridAgent] Planner has no saved plan.")
            return

        plannable_state = self.planner.compute_plannable_states(plan, failed_op)
        rospy.loginfo(f"[HybridAgent] Computed plannable state: {plannable_state}")

        # Create RewardFunction with computed plannable state
        reward_function = RewardFunction(plannable_state)

        # Create a new environment instance with reward function
        env = RecycleBotSMDP(reward_function=reward_function)
        
        # Get observation + action space sizes
        state_dim = env.observation_space.size
        action_dim = env.action_space.size

        # Create PPO learner
        ppo_model = PPO(
            state_dim=state_dim,
            action_dim=action_dim,
            lr_actor=0.0003,
            lr_critic=0.001,
            gamma=0.99,
            K_epochs=4,
            eps_clip=0.2
        )

        # Create LearningAgent
        learner = LearningAgent(env=env, learner_model=ppo_model)

        success = learner.learn()

        if success:
            rospy.loginfo("[HybridAgent] Learning successful. Retrying plan.")
            self.run()
        else:
            rospy.logerr("[HybridAgent] Learning failed. Aborting.")

    def find_failed_operator(self, action_name, params):
        """
        Finds the failed operator in the current plan based on the action name and parameters.
        """
        for op in self.planner.plan:
            if op.name == action_name and list(op.params) == list(params):
                return op
        return None


if __name__ == "__main__":
    domain_file = "path_to_your_domain_file"
    objects = {
        'doorway': ['doorway_1'],
        'room': ['room_1', 'room_2'],
        'ball': ['ball_1'],
        'can': ['can_1'],
        'bin': ['bin_1'],
        'nothing': ['nothing'],
        'robot': ['robot_1']
    }

    agent = HybridAgent(domain_file, None, objects)
    agent.run()
