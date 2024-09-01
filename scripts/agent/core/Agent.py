# core/Agent.py

from planner.planner import Planner
from PDDLActions import PDDLActions
from PDDLPredicates import PDDLPredicates

class Agent:
    def __init__(self, planner: Planner, actions: PDDLActions, predicates: PDDLPredicates):
        self.planner = planner
        self.actions = actions
        self.predicates = predicates

    def run(self, objects):
        """
        Main loop to execute the plan.
        """
        self.planner.new_problem(objects)
        action = self.planner.next_action()

        while action is not None:
            action_name, *params = action

            if not self.check_preconditions(action_name, params):
                raise RuntimeError(f"Preconditions not met for {action_name} with params {params}")

            try:
                self.execute_action(action_name, params)
            except Exception as e:
                raise RuntimeError(f"Failed to execute action {action_name} with params {params}: {e}")

            if not self.check_effects(action_name, params):
                raise RuntimeError(f"Effects not met after {action_name} with params {params}")

            action = self.planner.next_action()

        print("Plan completed successfully.")

    def check_preconditions(self, action_name: str, params: list) -> bool:
        return self.predicates.check_preconditions(action_name, params)

    def execute_action(self, action_name: str, params: list):
        self.actions.execute(action_name, params)

    def check_effects(self, action_name: str, params: list) -> bool:
        return self.predicates.check_effects(action_name, params)