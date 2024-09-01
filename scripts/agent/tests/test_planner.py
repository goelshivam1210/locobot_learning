import sys
import os

# Add the core directory to the Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'core')))

script_dir = os.path.dirname(__file__)  # Get the directory of the test_planner.py script

domain_file = os.path.abspath(os.path.join(script_dir, '../../knowledge/PDDL/recycle_bot/domain.pddl'))
problem_file = os.path.abspath(os.path.join(script_dir, '../../knowledge/PDDL/recycle_bot/problem.pddl'))

from planner.planner import Planner
from PDDLActions import PDDLActions
from Agent import Agent
from PDDLPredicates import PDDLPredicates
def test_planner():
    # domain_file = "../../../knowledge/PDDL/recycle_bot/domain.pddl"
    
    def mock_predicate(*args):
        return True

    predicate_funcs = {
        "at": mock_predicate,
        "connect": mock_predicate,
        "facing": mock_predicate,
        "hold": mock_predicate,
        "contain": mock_predicate,
    }

    objects = {
        'doorway': ['doorway_1'],
        'room': ['room_1', 'room_2'],
        'ball': ['ball_1'],
        'can': ['can_1'],
        'bin': ['bin_1'],
        'nothing': ['nothing'],
        'robot': ['robot_1']
    }



    planner = Planner(domain_file, predicate_funcs)
    planner.new_problem(objects)

    actions = PDDLActions()
    predicates = PDDLPredicates(predicate_funcs)
    agent = Agent(planner, actions, predicates)
    agent.run(objects)

    # planner.new_problem(objects)

    action = planner.next_action()
    if action:
        print(f"Next action: {action}")
    else:
        print("No actions in the plan or plan generation failed.")

if __name__ == "__main__":
    test_planner()