import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from planner.planner import Planner
from PDDLActions import PDDLActions
from PDDLPredicates import PDDLPredicates

class HybridAgent:
    def __init__(self, domain_file, predicate_funcs, objects):
        # Initialize the planner
        self.planner = Planner(domain_file, predicate_funcs)
        self.actions = PDDLActions()
        self.predicates = PDDLPredicates(predicate_funcs)
        self.objects = objects

    def run(self):
        """
        Main loop to execute the plan and handle failures.
        """
        self.planner.new_problem(self.objects)
        action = self.planner.next_action()

        while action is not None:
            action_name, *params = action

            if not self.predicates.check_preconditions(action_name, params):
                print(f"Preconditions not met for {action_name}, entering recovery mode.")
                self.recovery_mode(action_name, params)
                break

            try:
                self.actions.execute(action_name, params)
            except Exception as e:
                print(f"Failed to execute action {action_name}: {e}, entering recovery mode.")
                self.recovery_mode(action_name, params)
                break

            if not self.predicates.check_effects(action_name, params):
                print(f"Effects not met after {action_name}, entering recovery mode.")
                self.recovery_mode(action_name, params)
                break

            action = self.planner.next_action()

        if action is None:
            print("Plan completed successfully.")

    def recovery_mode(self, action_name, params):
        """
        Handle failures by invoking a fallback mechanism or retrying.
        """
        print(f"Starting recovery for action: {action_name} with params: {params}")
        # Add logic for recovery here, for example, retrying the action or modifying the plan.
        # As we are not using the learner, this function can be implemented as needed.
        print("Recovery mode not yet implemented. Stopping execution.")
        return False  # Placeholder

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