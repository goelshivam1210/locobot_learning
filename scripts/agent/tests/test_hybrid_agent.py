import sys
import os

# Add core directory to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'core')))

from HybridAgent import HybridAgent
import rospy

def test_hybrid_agent():
    rospy.init_node("test_hybrid_agent", anonymous=True)

    # Path to PDDL domain file
    domain_file = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../knowledge/PDDL/recycle_bot/domain.pddl'))

    # Define objects for the problem
    objects = {
        'doorway': ['doorway_1'],
        'room': ['room_1', 'room_2'],
        'ball': ['ball_1'],
        'can': ['can_1'],
        'bin': ['bin_1'],
        'nothing': ['nothing'],
        'robot': ['robot_1']
    }

    # Create HybridAgent instance
    agent = HybridAgent(domain_file, objects)

    rospy.loginfo("[test_hybrid_agent] Starting agent run.")
    agent.run()
    rospy.loginfo("[test_hybrid_agent] Agent run completed.")

if __name__ == "__main__":
    test_hybrid_agent()
