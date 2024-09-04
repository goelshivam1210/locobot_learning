# core/PDDLPredicates.py
import rospy
from locobot_learning.srv import At, AtResponse
from locobot_learning.srv import Hold, HoldResponse
from locobot_learning.srv import Facing, FacingResponse
from locobot_learning.srv import Contain, ContainResponse



class PDDLPredicates:
    def __init__(self, predicate_funcs: dict):

                # Initialize ROS service clients
        self.at_service = rospy.ServiceProxy('/at_service', At)
        self.contain_service = rospy.ServiceProxy('/contain_service', Contain)
        self.facing_service = rospy.ServiceProxy('/facing_service', Facing)
        self.hold_service = rospy.ServiceProxy('/hold_service', Hold)

        
        self.predicate_funcs = predicate_funcs

    def check_preconditions(self, action_name: str, params: list) -> bool:
        # Evaluate preconditions using predicate functions
        # In this simple case, we assume action names map directly to predicates, which may need refinement.
        relevant_predicates = {
            "approach": ["at", "facing"],
            "pick": ["at", "facing", "hold"],
            "pass_through_door": ["at", "connect", "facing"],
            "place": ["at", "facing", "hold"],
        }
        
        for pred_name in relevant_predicates.get(action_name, []):
            if not self.predicate_funcs[pred_name](*params):
                return False
        return True

    def check_effects(self, action_name: str, params: list) -> bool:
        # Evaluate effects using predicate functions
        return self._evaluate_predicates(action_name, params)

    def _evaluate_predicates(self, action_name: str, params: list) -> bool:
        try:
            # We now evaluate based on specific predicates related to the action
            relevant_predicates = {
                "approach": ["at", "facing"],
                "pick": ["hold"],
                "pass_through_door": ["at"],
                "place": ["contain"],
            }

            for pred_name in relevant_predicates.get(action_name, []):
                if not self.predicate_funcs[pred_name](*params):
                    return False
            return True
        except Exception as e:
            print(f"Predicate evaluation failed for action {action_name} with params {params}: {e}")
            return False