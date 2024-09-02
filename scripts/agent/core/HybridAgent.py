import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


from planner.planner import Planner
from PDDLActions import PDDLActions
from PDDLPredicates import PDDLPredicates
from learner.LearningAgent import LearningAgent  # Adjusted import


class HybridAgent:
    def __init__(self, domain_file, predicate_funcs, objects, max_retries=3):
        # Initialize the planner
        self.planner = Planner(domain_file, predicate_funcs)
        self.actions = PDDLActions()
        self.learner = LearningAgent(domain_file, None)  # Initialize the learner here
        self.objects = objects
        self.max_retries = max_retries

    def run(self):
        """
        Main loop to execute the plan and handle failures.
        """
        self.planner.new_problem(self.objects)
        action = self.planner.next_action()

        while action is not None:
            action_name, *params = action

            if not self.planner.verify_preconditions(action_name, *params):
                print(f"Preconditions not met for {action_name}, entering recovery mode.")
                self.recovery_mode(action_name, params)
                break

            try:
                self.actions.execute(action_name, params)
            except Exception as e:
                print(f"Failed to execute action {action_name}: {e}, entering recovery mode.")
                self.recovery_mode(action_name, params)
                break

            if not self.planner.verify_effects(action_name, *params):
                print(f"Effects not met after {action_name}, entering recovery mode.")
                self.recovery_mode(action_name, params)
                break

            action = self.planner.next_action()

        if action is None:
            print("Plan completed successfully.")
    def recovery_mode(self, action_name, params):
        for attempt in range(self.max_retries):
            print(f"Retrying action {action_name} ({attempt + 1}/{self.max_retries})...")
            try:
                if self.planner.verify_preconditions(action_name, *params):
                    self.actions.execute(action_name, params)
                    if self.planner.verify_effects(action_name, *params):
                        print(f"Action {action_name} succeeded on retry {attempt + 1}.")
                        return
                else:
                    print(f"Preconditions still not met for {action_name} on retry {attempt + 1}.")
            except Exception as e:
                print(f"Retry {attempt + 1} failed for action {action_name}: {e}")
        
        print(f"Retries exhausted for action {action_name}, attempting to execute learned executor.")
        if self.learner:
            success = self.learner.execute_executor(action_name, params)
            if success:
                print(f"Successfully executed learned policy for action {action_name}.")
                return
            else:
                print(f"Learned executor failed for action {action_name}, entering learning mode.")
                self.invoke_learning(action_name, params)
        else:
            print("LearningAgent is not initialized.")

    def invoke_learning(self, action_name, params):
        print(f"Starting learning for action: {action_name} with params: {params}")
        if self.learner:
            success = self.learner.learn()
            if success:
                print(f"Learning successful for action: {action_name}.")
                # self.run()  # Retry the entire plan
            else:
                print(f"Learning failed for action: {action_name}. Stopping execution.")
        else:
            print("LearningAgent is not initialized.")

    def execute_executor(self, action_name, params):
        """
        Attempt to execute the learned policy (if available) for the given action.
        """
        try:
            success = self.learner.execute_executor(action_name, params)
            if success:
                print(f"Executor successfully executed action {action_name}.")
                return True
            else:
                print(f"Executor failed to execute action {action_name}.")
                return False
        except Exception as e:
            print(f"Error executing executor for action {action_name}: {e}")
            return False

if __name__ == "__main__":
    domain_file = "path_to_your_domain_file"
    predicate_funcs = {
        "at": lambda *args: True,
        "connect": lambda *args: True,
        "facing": lambda *args: True,
        "hold": lambda *args: True,
        "contain": lambda *args: True,
    }
    objects = {
        'doorway': ['doorway_1'],
        'room': ['room_1', 'room_2'],
        'ball': ['ball_1'],
        'can': ['can_1'],
        'bin': ['bin_1'],
        'nothing': ['nothing'],
        'robot': ['robot_1']
    }

    agent = HybridAgent(domain_file, predicate_funcs, objects)
    agent.run()