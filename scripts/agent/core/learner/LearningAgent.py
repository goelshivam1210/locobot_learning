# core/learner/LearningAgent.py

import sys
import os
import rospy
import torch
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'learner')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'planner')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'environment')))

from PPOLearner import PPOLearner
from environment.RecycleBotSMDP import RecycleBotSMDP  # Our RL environment
from environment.state.SymbolicState import SymbolicState  # Tracks world state
from environment.reward.reward_function import RewardFunction  # Defines rewards

class LearningAgent:
    def __init__(self, domain_file, problem_file):
        """
        Initialize the LearningAgent, setting up the environment and learning model.
        """
        self.domain_file = domain_file
        self.problem_file = problem_file

        # Initialize the environment with the PDDL domain and problem files
        self.env = RecycleBotSMDP(domain_file, problem_file)

        # Define symbolic state tracking
        self.symbolic_state = SymbolicState()

        # Define a reward function
        self.reward_function = RewardFunction()

        # Initialize PPO-based reinforcement learning model
        self.learner = PPOLearner(self.env)

    def execute_executor(self, action_name, params):
        """
        Attempt to execute a learned policy for the given action.
        Returns True if the action succeeds, otherwise False.
        """
        rospy.loginfo(f"Executing learned policy for action: {action_name} with params: {params}")

        try:
            # Convert action and params into the correct observation format
            observation = self.symbolic_state.get_state_representation()

            # Select action using learned policy
            action = self.learner.select_action(observation)

            # Execute the action in the environment
            next_obs, reward, done, info = self.env.step(action)

            # If done and reward is positive, assume success
            if done and reward > 0:
                rospy.loginfo(f"Successfully executed {action_name} via learned policy.")
                return True

            rospy.logwarn(f"Execution of {action_name} failed in learned mode.")
            return False

        except Exception as e:
            rospy.logerr(f"Error executing policy for {action_name}: {e}")
            return False

    def learn(self):
        """
        Trigger reinforcement learning to learn a policy for a failed action.
        Returns True if learning is successful, otherwise False.
        """
        rospy.loginfo("Starting RL training...")

        # Reset the environment
        observation = self.env.reset()
        done = False

        while not done:
            # Select an action based on the policy
            action = self.learner.select_action(observation)

            # Execute the action in the environment
            next_obs, reward, done, _ = self.env.step(action)

            # Update the policy
            self.learner.update(observation, action, reward, done)

            # Move to the next observation
            observation = next_obs

        rospy.loginfo("Learning process completed.")
        return True  # Indicate success

if __name__ == "__main__":
    domain_file = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../knowledge/PDDL/recycle_bot/domain.pddl'))
    problem_file = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../knowledge/PDDL/recycle_bot/problem.pddl'))

    agent = LearningAgent(domain_file, problem_file)
    agent.learn()
