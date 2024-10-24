import rospy
from locobot_learning.srv import At, AtRequest, Hold, HoldRequest, Facing, FacingRequest, Contain, ContainRequest

class PDDLPredicates:
    def __init__(self):
        # Initialize ROS service clients
        self.at_service = rospy.ServiceProxy('/at', At)
        self.contain_service = rospy.ServiceProxy('/contain', Contain)
        self.facing_service = rospy.ServiceProxy('/facing', Facing)
        self.hold_service = rospy.ServiceProxy('/hold', Hold)

    def map_to_generic_object(self, obj: str) -> str:
        """
        Map specific objects like ball_1 or can_1 to 'generic_object' for ROS service calls.
        """
        if obj in ["ball_1", "can_1"]:
            return "generic_object"
        return obj

    def check_preconditions(self, action_name: str, params: list) -> bool:
        """
        Evaluate preconditions for the given action.
        """
        relevant_predicates = {
            "approach": [("check_at", [0, 1]), ("check_facing", [2])],
            "pick": [("check_at", [0, 1]), ("check_facing", [0]), ("check_hold", [0])],
            "pass_through_door": [("check_at", [0, 1]), ("check_facing", [2])],
            "place": [("check_at", [0, 1]), ("check_facing", [2]), ("check_hold", [0])],
        }

        # Check relevant predicates based on the action
        for pred_name, arg_indices in relevant_predicates.get(action_name, []):
            # Skip 'facing' check if it's against "nothing"
            if pred_name == "check_facing" and params[arg_indices[0]] == "nothing":
                continue

            # Call the predicate function with the correct args
            args = [params[i] for i in arg_indices]
            if not getattr(self, pred_name)(*args):
                return False
        return True

    # ROS service calls for predicates
    def check_at(self, obj: str, room: str) -> bool:
        """
        Check if the object is at the specified room.
        """
        obj = self.map_to_generic_object(obj)  # Map specific object to generic object
        try:
            response = self.at_service(AtRequest(room=room, obj=obj))
            return response.obj_at_room
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def check_facing(self, obj: str) -> bool:
        """
        Check if the robot is facing the specified object.
        """
        obj = self.map_to_generic_object(obj)  # Map specific object to generic object
        try:
            response = self.facing_service(FacingRequest(obj=obj))
            return response.robot_facing_obj
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def check_hold(self, obj: str) -> bool:
        """
        Check if the robot is holding the specified object.
        """
        obj = self.map_to_generic_object(obj)  # Map specific object to generic object
        try:
            response = self.hold_service(HoldRequest(obj=obj))
            return response.robot_holding_obj
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

