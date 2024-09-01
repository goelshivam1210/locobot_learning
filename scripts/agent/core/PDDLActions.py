# this will later be populated with the ROS service calls
class PDDLActions:
    def execute(self, action_name: str, params: list):
        """
        Executes the given action by simulating the operation.
        """
        print(f"Executing action: {action_name} with parameters: {params}")
        if action_name == "approach":
            self.approach(*params)
        elif action_name == "pick":
            self.pick(*params)
        elif action_name == "place":
            self.place(*params)
        elif action_name == "pass_through_door":
            self.pass_through_door(*params)
        else:
            print(f"Unknown action: {action_name}")

    def approach(self, obj, room, facing):
        print(f"Approaching {obj} in {room} while facing {facing}.")

    def pick(self, obj, room):
        print(f"Picking up {obj} in {room}.")

    def place(self, obj, room, container):
        print(f"Placing {obj} in {container} in {room}.")

    def pass_through_door(self, room1, room2, doorway):
        print(f"Passing through {doorway} from {room1} to {room2}.")