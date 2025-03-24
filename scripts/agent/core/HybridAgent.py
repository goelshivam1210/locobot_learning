# core/HybridAgent.py

import sys
import os
import rospy

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from planner.planner import Planner
from PDDLActions import PDDLActions
from PDDLPredicates import PDDLPredicates
from learner.LearningAgent import LearningAgent

class HybridAgent:
    def __init__(self, domain_file, predicate_funcs, objects, max_retries=3):
        """
        Initializes the hybrid agent with planning and learning capabilities.
        """
        self.planner = Planner(domain_file, predicate_funcs)
        self.actions = PDDLActions()
        self.learner = LearningAgent(domain_file, None)  # Initialize learning module
        self.objects = objects
        self.max_retries = max_retries

    def run(self):
        """
        Execute the plan and handle failures dynamically.
        """
        self.planner.new_problem(self.objects)
        action = self.planner.next_action()

        while action is not None:
            action_name, *params = action

            if not self.planner.verify_preconditions(action_name, *params):
                rospy.logwarn(f"Preconditions failed for {action_name}, invoking learning.")
                self.invoke_learning(action_name, params)
                return

            try:
                self.actions.execute(action_name, params)
            except Exception as e:
                rospy.logwarn(f"Execution failed for {action_name}: {e}, invoking learning.")
                self.invoke_learning(action_name, params)
                return

            if not self.planner.verify_effects(action_name, *params):
                rospy.logwarn(f"Effects not met for {action_name}, invoking learning.")
                self.invoke_learning(action_name, params)
                return

            action = self.planner.next_action()

        rospy.loginfo("Plan executed successfully.")

    def invoke_learning(self, action_name, params):
        """
        Switch to learning mode if execution fails.
        """
        rospy.logwarn(f"Switching to learning mode for action: {action_name} with params: {params}")
        success = self.learner.learn()

        if success:
            rospy.loginfo(f"Successfully learned {action_name}, retrying execution.")
            self.run()  # Retry the entire plan
        else:
            rospy.logerr(f"Learning failed for {action_name}, terminating execution.")

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
