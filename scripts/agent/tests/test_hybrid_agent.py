#!/usr/bin/env python3 -u

#NOTE: the -u in the shebang above makes the script output unbuffered when not writing to 
#a TTY (e.g. when using tee or > redirect). If that's no longer needed, it can be removed.

from typing import Union, Literal
from io import TextIOBase
import sys
import os

# Add core directory to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'core')))

from HybridAgent import HybridAgent
import rospy


class Tee:
    def __init__(self, *streams):
        self.streams = streams

    def write(self, data):
        for s in self.streams:
            s.write(data)
        for s in self.streams:
            s.flush()

    def flush(self):
        for s in self.streams:
            s.flush()


def setup_tee(run_dir: str):
    log_file_path = os.path.join(run_dir, "log.txt")
    # line-buffered text file is nice for logs
    log_file = open(log_file_path, "a", buffering=1)

    sys.stdout = Tee(sys.stdout, log_file)
    sys.stderr = Tee(sys.stderr, log_file)

    return log_file  # keep a reference so it doesn't get GC'ed

def test_hybrid_agent(run_dir: str, include_local_view=True, num_demonstrations: int = 0, test_only: bool = False):
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


    print(f"""
===============================================
Starting HybridAgent Test {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
===============================================
    """)

    # Create HybridAgent instance
    agent = HybridAgent(
        domain_file,
        objects,
        num_demonstrations=num_demonstrations,
        include_local_view=include_local_view,
        # DEBUG: Reduce episodes and steps for quicker testing
        # num_episodes=50,
        max_steps=50,
        num_episodes=25,
        # max_steps=10,
        include_symbolic_actions=False,
        run_dir=run_dir,
        policy_checkpoint_frequency=5,
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
    from datetime import datetime
    from argparse import ArgumentParser
    parser = ArgumentParser()

    current_date = datetime.now().strftime("%Y-%m-%d")

    default_stats_file = os.path.abspath(os.path.join(os.path.dirname(__file__), f"./logs/hybrid_agent_stats_{current_date}.csv"))

    parser.add_argument("-d", "--demonstrations", type=int, nargs="?", help="Number of human demonstrations to request before the agent starts learning", default=0)
    parser.add_argument("--no-local-view", action="store_false", help="Omit local view grid in the agent's observations; include only is_obstructedd bit for subsymbolic state.")
    parser.add_argument("--test_only", action="store_true", help="Run the agent in test mode, without updating weights.")
    parser.add_argument("--run-id", type=str, default=f"{current_date}", help="Identifier for this run, used in logging.")
    args = parser.parse_args()

    run_id = args.run_id

    run_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), f"./runs/{run_id}"))

    if not os.path.exists(run_dir):
        os.makedirs(run_dir)
        rospy.loginfo(f"[test_hybrid_agent] Created run directory at {run_dir}")

    log_file = setup_tee(run_dir)

    try:
        test_hybrid_agent(
            include_local_view=args.no_local_view,
            num_demonstrations=args.demonstrations,
            run_dir=run_dir,
            test_only=args.test_only
        )
    finally:
        log_file.close()