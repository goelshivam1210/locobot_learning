# core/Agent.py

from planner.planner import Planner
from PDDLActions import PDDLActions
from PDDLPredicates import PDDLPredicates
from exceptions import ActionExecutionError

class Agent:
    def __init__(self, planner: Planner, actions: PDDLActions, predicates: PDDLPredicates):
        self.planner = planner
        self.actions = actions
        self.predicates = predicates

    def run(self, objects):
        """
        Main loop to execute the plan.
        """
        # Dynamically fetch the current state of the world using PDDLPredicates
        current_state = self.fetch_current_state(objects)

        # Pass the current state along with objects to the planner
        self.planner.new_problem(objects, current_state)
        action = self.planner.next_action()

        while action is not None:
            action_name, *params = action

            self.run_action(action_name, list(params))

            action = self.planner.next_action()

        print("Plan completed successfully.")

    def run_action(self, action_name: str, params: list):
        import rospy
        rospy.loginfo(f"[Agent] Executing action: {action_name} with params: {params}")
        if not self.check_preconditions(action_name, params):
            raise ActionExecutionError("Preconditions failed", action_name, params)

        try:
            self.execute_action(action_name, params)
        except Exception as e:
            raise ActionExecutionError(f"Execution failed: {e}", action_name, params)

        if not self.check_effects(action_name, params):
            raise ActionExecutionError("Effects failed", action_name, params)

    def fetch_current_state(self, objects: dict) -> dict:
        """
        Fetch the current state of the world dynamically using PDDLPredicates.
        """
        # Retrieve dynamic state information from predicates

        #DEBUG
        #TODO: Remove debug block
        #We are pretending that the robot is holding "ball_1", so ball_1 needs to be in whatever
        #room the robot is in. This is a temporary hack to get the robot to skip picking up the
        #object and skip straight to passing through the doorway.
        robot_location = self.predicates.get_robot_location()
        current_state = {
            'robot_location': robot_location,
            'robot_facing': self.predicates.get_robot_facing(),
            'robot_holding': self.predicates.get_robot_holding(),
            'object_locations': {
                obj: robot_location if obj == 'ball_1' else self.predicates.get_object_location(obj) 
                    for obj in objects.get('ball', []) + objects.get('bin', [])
            }
        }
        return current_state
        #END DEBUG
        current_state = {
            'robot_location': self.predicates.get_robot_location(),
            'robot_facing': self.predicates.get_robot_facing(),
            'robot_holding': self.predicates.get_robot_holding(),
            'object_locations': {obj: self.predicates.get_object_location(obj) for obj in objects.get('ball', []) + objects.get('bin', [])}
        }
        return current_state

    def check_preconditions(self, action_name: str, params: list) -> bool:
        return self.predicates.check_preconditions(action_name, params)

    def execute_action(self, action_name: str, params: list):
        self.actions.execute(action_name, params)

    def check_effects(self, action_name: str, params: list) -> bool:
        return self.predicates.check_effects(action_name, params)
