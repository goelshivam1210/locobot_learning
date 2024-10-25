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
        Evaluate preconditions for the given action with logging for failures.
        """
        relevant_predicates = {
            "approach": [("check_at", [0, 1]), ("check_facing", [2])],
            "pick": [("check_at", [0, 1]), ("check_facing", [0]), ("check_hold", ["nothing"])],  # Check holding nothing
            "pass_through_door": [("check_at", [0, 1]), ("check_facing", [2])],
            "place": [("check_at", [0, 1]), ("check_facing", [2]), ("check_hold", [0])],
        }

        rospy.loginfo(f"Checking preconditions for action: {action_name}")

        for pred_name, arg_indices in relevant_predicates.get(action_name, []):
            # Skip 'facing' check if it's against "nothing"
            if pred_name == "check_facing" and params[arg_indices[0]] == "nothing":
                continue

            # Handle "hold nothing" check for pick
            if pred_name == "check_hold" and "nothing" in arg_indices:
                args = ["nothing"]  # Correctly pass "nothing" to the hold check
            else:
                args = [self.map_to_generic_object(params[i]) for i in arg_indices]

            rospy.loginfo(f"Checking {pred_name} with args: {args}")
            if not getattr(self, pred_name)(*args):
                rospy.logwarn(f"Precondition failed: {pred_name} with args: {args}")
                return False
        return True



    def check_effects(self, action_name: str, params: list) -> bool:
        """
        Evaluate effects for the given action with logging for failures.
        """
        relevant_predicates = {
            "approach": [("check_facing", [0])],
            "pick": [("check_hold", [0])],
            "pass_through_door": [("check_at", [1, 2])],
            "place": [("check_contain", [0, 1])],
        }

        rospy.loginfo(f"Checking effects for action: {action_name}")

        for pred_name, arg_indices in relevant_predicates.get(action_name, []):
            args = [self.map_to_generic_object(params[i]) for i in arg_indices]
            rospy.loginfo(f"Checking {pred_name} with args: {args}")
            if not getattr(self, pred_name)(*args):
                rospy.logwarn(f"Effect failed: {pred_name} with args: {args}")
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
        Check if the robot is holding an object or holding nothing.
        """
        rospy.loginfo(f"Checking hold status for the robot (expected hold status: {obj})")
        try:
            response = self.hold_service(HoldRequest(obj=""))  # Call without any specific object
            rospy.loginfo(f"robot_holding_obj: {response.robot_holding_obj}")
            
            # If we're checking for "nothing", return True if robot_holding_obj is False (i.e., holding nothing)
            if obj == "nothing":
                return not response.robot_holding_obj  # Return True if the robot is not holding anything
            
            # If checking for a specific object, return True if the robot is holding something (robot_holding_obj is True)
            return response.robot_holding_obj
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False



    def check_contain(self, obj: str, container: str) -> bool:
        """
        Check if the object is contained in the specified container (e.g., bin).
        """
        obj = self.map_to_generic_object(obj)  # Map specific object to generic object
        try:
            response = self.contain_service(ContainRequest(obj=obj, container=container))
            return response.container_contains_obj
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
