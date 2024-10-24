import rospy
from locobot_learning.srv import Approach, GraspObject, DropObject


class PDDLActions:
    def __init__(self):
        # Initialize ROS service proxies
        self.approach_service = rospy.ServiceProxy('/approach', Approach)
        self.grasp_service = rospy.ServiceProxy('/grasp_object', GraspObject)
        self.drop_service = rospy.ServiceProxy('/drop_object', DropObject)

    def execute(self, action_name: str, params: list):
        """
        Executes the given action by calling the respective ROS service.
        """
        rospy.loginfo(f"Executing action: {action_name} with parameters: {params}")
        if action_name == "approach":
            self.approach(*params)
        elif action_name == "pick":
            self.pick(*params)
        elif action_name == "place":
            self.place(*params)
        elif action_name == "pass_through_door":
            self.pass_through_door(*params)
        else:
            rospy.logwarn(f"Unknown action: {action_name}")

    def approach(self, obj, room, facing):
        """
        Calls the approach service.
        """
        rospy.loginfo(f"Approaching {obj} in {room} while facing {facing}.")
        try:
            response = self.approach_service(facing)
            if response.success:
                rospy.loginfo(f"Successfully approached {facing}.")
            else:
                rospy.logerr(f"Failed to approach {facing}.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def pick(self, obj, room):
        """
        Calls the grasp_object service with the 'generic object' argument.
        """
        rospy.loginfo(f"Picking up {obj} in {room}.")
        try:
            response = self.grasp_service("generic_object")
            if response.success:
                rospy.loginfo(f"Successfully picked up {obj}.")
            else:
                rospy.logerr(f"Failed to pick up {obj}.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def place(self, obj, room, container):
        """
        Calls the drop_object service to place an object.
        """
        rospy.loginfo(f"Placing {obj} in {container} in {room}.")
        try:
            response = self.drop_service()
            if response.success:
                rospy.loginfo(f"Successfully placed {obj} in {container}.")
            else:
                rospy.logerr(f"Failed to place {obj} in {container}.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def pass_through_door(self, room1, room2, doorway):
        """
        Calls the approach service with the argument 'postdoor'.
        """
        rospy.loginfo(f"Passing through {doorway} from {room1} to {room2}.")
        try:
            response = self.approach_service("postdoor")
            if response.success:
                rospy.loginfo(f"Successfully passed through the door.")
            else:
                rospy.logerr(f"Failed to pass through the door.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
