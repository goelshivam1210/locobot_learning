import sys
import os

# Add the core directory to the Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'core')))

from HybridAgent import HybridAgent

def test_hybrid_agent():
    domain_file = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../knowledge/PDDL/recycle_bot/domain.pddl'))
    
    # Mock predicates for testing - introducing failure cases
    def mock_predicate(*args):
        # Simulate a failure in the preconditions or effects
        if args[0] == 'ball_1':
            return False  # This will simulate a failure when dealing with ball_1
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

    agent = HybridAgent(domain_file, predicate_funcs, objects)
    agent.run()

if __name__ == "__main__":
    test_hybrid_agent()