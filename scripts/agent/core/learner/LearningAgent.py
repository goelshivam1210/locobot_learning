# core/learner/LearningAgent.py

import sys
import os

# Add the core and environment directories to the Python path
# sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'environment')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'learner')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'planner')))

from PPOLearner import PPOLearner
# from environment.RecycleBot import RecycleBot
# from environment.ObservationGenerator import ObservationGenerator
# from environment.RewardFunction import RewardFunction

class LearningAgent:
    def __init__(self, domain_file, problem_file):
        self.domain_file = domain_file
        self.problem_file = problem_file
        
        # Initialize environment components
        # self.env = RecycleBot(domain_file, problem_file, failed_action=None, planner=None, sym_actions_dict=None, executor_dir=None)
        # self.observation_generator = ObservationGenerator(domain_file, problem_file)
        # self.reward_function = RewardFunction(domain_file, problem_file, failed_action=None)
        
        # Initialize the learner
        # self.learner = PPOLearner(self.env)

    def execute_executor(self, action_name, params):
        """
        Attempt to execute a learned policy for the given action.
        Returns True if the action succeeds, otherwise False.
        """
        print(f"Attempting to execute executor for action: {action_name} with params: {params}")
        try:
            # Assuming execute_policy returns a boolean indicating success/failure
            success = self.learner.execute_policy(action_name, params)
            return success
        except Exception as e:
            print(f"Error during executor execution: {e}")
            return False

    def learn(self):
        """
        Trigger the learning process to learn or improve the policy for the given action.
        Returns True if learning is successful, otherwise False.
        """
        print("Starting learning process...")
        observation = self.env.reset()
        done = False
        while not done:
            action = self.learner.select_action(observation)
            observation, reward, done, _ = self.env.step(action)
            self.learner.update(observation, action, reward, done)
        
        return True  # Indicate learning completed successfully

if __name__ == "__main__":
    domain_file = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../knowledge/PDDL/recycle_bot/domain.pddl'))
    problem_file = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../knowledge/PDDL/recycle_bot/problem.pddl'))

    agent = LearningAgent(domain_file, problem_file)
    agent.learn()