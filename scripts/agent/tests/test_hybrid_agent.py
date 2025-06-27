#!/usr/bin/env python3 -u

#NOTE: the -u in the shebang above makes the script output unbuffered when not writing to 
#a TTY (e.g. when using tee or > redirect). If that's no longer needed, it can be removed.

import sys
import os

# Add core directory to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'core')))

from HybridAgent import HybridAgent
import rospy


def test_hybrid_agent(include_local_view=True, num_demonstrations: int = 0, test_only: bool = False):
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
    agent = HybridAgent(
        domain_file,
        objects,
        num_demonstrations=num_demonstrations,
        include_local_view=include_local_view,
        max_steps=25 if test_only else 50,
    )

    if test_only:
        saved_policy_path = agent.get_saved_policy_path()

        if saved_policy_path is not None:
            rospy.loginfo("[test_hybrid_agent] Running in test mode with existing policy.")
            # As a safety measure, prevent the saved policy from being overwritten due to logic errors
            import stat
            import atexit
            # Store the original permissions of the saved policy file
            original_permissions = os.stat(saved_policy_path).st_mode
            # Make the saved policy file read-only
            os.chmod(saved_policy_path, stat.S_IREAD)
            # Ensure the file is restored to its original permissions on exit
            atexit.register(lambda: os.chmod(saved_policy_path, original_permissions))  # Restore original permissions

    rospy.loginfo("[test_hybrid_agent] Starting agent run.")
    agent.run()
    rospy.loginfo("[test_hybrid_agent] Agent run completed.")

if __name__ == "__main__":
    from argparse import ArgumentParser
    parser = ArgumentParser()

    parser.add_argument("-d", "--demonstrations", type=int, nargs="?", help="Number of human demonstrations to request before the agent starts learning", default=0)
    parser.add_argument("--no-local-view", action="store_false", help="Omit local view grid in the agent's observations; include only is_obstructedd bit for subsymbolic state.")
    parser.add_argument("--test_only", action="store_true", help="Run the agent in test mode, without updating weights.")
    args = parser.parse_args()

    test_hybrid_agent(
        include_local_view=args.no_local_view,
        num_demonstrations=args.demonstrations,
        test_only=args.test_only
    )
