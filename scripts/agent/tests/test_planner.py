import sys
import os
import rospy

# Add the core directory to the Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'core')))

script_dir = os.path.dirname(__file__)  # Get the directory of the test_planner.py script

domain_file = os.path.abspath(os.path.join(script_dir, '../../knowledge/PDDL/recycle_bot/domain.pddl'))

from planner.planner import Planner
from PDDLActions import PDDLActions
from Agent import Agent
from PDDLPredicates import PDDLPredicates

def test_planner():
    # Initialize ROS node (if not already initialized)
    if not rospy.get_node_uri():
        rospy.init_node('test_planner_node', anonymous=True)

    # Initialize the PDDLPredicates to query the current state dynamically
    predicates = PDDLPredicates()

    # Dynamically fetch the current state of the world
    current_state = {
        'robot_location': predicates.get_robot_location(),
        'robot_facing': predicates.get_robot_facing(),
        'robot_holding': predicates.get_robot_holding(),
        'object_locations': {
            'ball_1': predicates.get_object_location('ball_1'),
            'bin_1': predicates.get_object_location('bin_1'),
            'can_1': predicates.get_object_location('can_1'),
            # Add other objects dynamically as needed
        }
    }

    # Object initialization (to match the domain/problem PDDL)
    objects = {
        'doorway': ['doorway_1'],
        'room': ['room_1', 'room_2'],
        'ball': ['ball_1'],
        'can': ['can_1'],
        'bin': ['bin_1'],
        'nothing': ['nothing'],
        'robot': ['robot_1']
    }

    # Initialize the planner with the domain
    planner = Planner(domain_file)

    # Generate a new problem dynamically using the current state
    planner.new_problem(objects, current_state)

    # Initialize actions and agent
    actions = PDDLActions()
    agent = Agent(planner, actions, predicates)

    # Run the agent to execute the plan
    agent.run(objects)

    # Check next action in the plan
    action = planner.next_action()
    if action:
        print(f"Next action: {action}")
    else:
        print("No actions in the plan or plan generation failed.")


if __name__ == "__main__":
    test_planner()
