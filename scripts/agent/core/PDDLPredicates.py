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
            "pick": [("check_at", [0, 1]), ("check_facing", [0]), ("check_hold", ["nothing"])],
            "pass_through_door": [("check_at", ["robot_1", 0]), ("check_facing", [2])],
            "place": [("check_at", [0, 1]), ("check_facing", [2]), ("check_hold", [0])],
        }


        for pred_name, arg_indices in relevant_predicates.get(action_name, []):
            # Handle constants and dynamic indices
            args = [
                params[i] if isinstance(i, int) else i  # Use index for integers, use the constant for strings
                for i in arg_indices
            ]

            if not getattr(self, pred_name)(*args):
                return False
        return True



    def check_effects(self, action_name: str, params: list) -> bool:
        """
        Evaluate effects for the given action with logging for failures.
        """
        relevant_predicates = {
            "approach": [("check_facing", [0])],
            "pick": [("check_hold", [0])],
            "pass_through_door": [("check_at", ["robot_1", 1]), ("check_facing", ["nothing"])],
            "place": [("check_contain", [0, 1])],
        }

        for pred_name, arg_indices in relevant_predicates.get(action_name, []):
            # Handle constants and dynamic indices
            args = [
                params[i] if isinstance(i, int) else i  # Use index for integers, use the constant for strings
                for i in arg_indices
            ]

            if not getattr(self, pred_name)(*args):
                return False
        return True



    # ROS service calls for predicates
    def check_at(self, obj: str, room: str) -> bool:
        rospy.loginfo(f"Checking if {obj} is in {room}.")
        obj = self.map_to_generic_object(obj)
        try:
            response = self.at_service(AtRequest(room=room, obj=obj))
            rospy.loginfo(f"check_at result: {response.obj_at_room} for {obj} in {room}")
            return response.obj_at_room
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False



    def check_facing(self, obj: str) -> bool:
        """
        Check if the robot is facing the specified object.
        """
        try:
            response = self.facing_service(FacingRequest(obj=obj))
            rospy.loginfo(f"Robot is facing {obj}: {response.robot_facing_obj}")
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


    def get_robot_location(self) -> str:
        """
        Get the current location of the robot.
        """
        try:
            # Assume the robot's location can be determined via the 'at' service
            for room in ["room_1", "room_2"]:  # Iterate over possible rooms
                response = self.check_at("robot_1", room)
                if response:
                    return room
        except Exception as e:
            rospy.logerr(f"Error getting robot location: {e}")
        return "unknown"

    def get_robot_facing(self) -> str:
        """
        Get the object or direction the robot is currently facing.
        """
        try:
            for obj in ["nothing", "ball_1", "can_1", "doorway_1", "bin_1"]:  # Iterate over facable objects
                response = self.check_facing(obj)
                if response:
                    return obj
        except Exception as e:
            rospy.logerr(f"Error getting robot facing direction: {e}")
        return "unknown"

    def get_robot_holding(self) -> str:
        """
        Get the object the robot is currently holding, or "nothing".
        """

        #DEBUG
        #TODO: Remove this debug code
        #Temporarily trick robot into thinking it's holding "ball_1" so that it
        #continues to pass through the doorway
        rospy.logdebug("Forcing robot to think it's holding 'ball_1'")
        return "ball_1"
        #END DEBUG

        try:
            for obj in ["nothing", "ball_1", "can_1"]:  # Iterate over holdable objects
                if self.check_hold(obj):
                    return obj
        except Exception as e:
            rospy.logerr(f"Error getting robot holding state: {e}")
        return "unknown"

    def get_object_location(self, obj: str) -> str:
        """
        Get the current location of the specified object.
        """
        try:
            for room in ["room_1", "room_2"]:  # Iterate over possible rooms
                if self.check_at(obj, room):
                    return room
        except Exception as e:
            rospy.logerr(f"Error getting location for object {obj}: {e}")
        return "unknown"
