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

    # Initialize the PDDLPredicates with its internal ROS service calls
    predicates = PDDLPredicates()  # No argument required

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

    # Initialize planner without needing predicate_funcs
    planner = Planner(domain_file)  # Remove predicate_funcs here
    planner.new_problem(objects)

    # Initialize actions and agent
    actions = PDDLActions()  # Modify this if action execution needs ROS service calls
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
