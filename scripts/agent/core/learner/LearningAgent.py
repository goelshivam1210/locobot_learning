# learner/LearningAgent.py

from typing import Union, Tuple
import rospy
import sys
import os
import json
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'environment')))
from RecycleBotSMDP import RecycleBotSMDP  # Import the RecycleBotSMDP environment
from .PPO import PPO  # Import the PPO learner model
from .learning_stats import LearningStats  # Import LearningStats for logging


class LearningAgent:
    def __init__(
        self,
        env: RecycleBotSMDP,
        learner_model: PPO,
        max_steps: int,
        run_dir: str,
    ):
        """
        env: instance of RecycleBotSMDP
        learner_model: instance of PPO
        max_steps: maximum steps per learning episode
        run_dir: directory path to save stats and policies
        """
        self.env = env
        self.learner: PPO = learner_model
        self.max_steps = max_steps
        self.run_dir = run_dir
        self.stats_file_path = os.path.join(self.run_dir, "stats.csv")

    def learn(self, stats: LearningStats, episode: int, demonstration=False) -> bool:
        """
        Run PPO learning loop until recovery is achieved (done=True) or max_steps reached.
        """
        rospy.loginfo("[LearningAgent] Starting learning process...")

        obs = self.env.observation_space.get_observation()  # or env.reset() if implemented
        done = False
        step_count = 0

        action_dict = {action_id: f"{action['name']}({action['params'] if action['params'] else ''})" for action_id, action in enumerate(self.env.action_space.action_list)}

        if not os.path.exists(self.stats_file_path):
            os.makedirs(os.path.dirname(self.stats_file_path), exist_ok=True)
                
        with open(self.stats_file_path, 'a+') as stats_file:
            while not done and step_count < self.max_steps and not rospy.is_shutdown():
                rospy.loginfo(f"[LearningAgent] Actions: {action_dict}")
                step_start = rospy.get_time()
                if demonstration:
                    action = self.get_demonstration_action()
                    self.learner.update_buffer(obs, action)
                else:
                    action, action_logprob = self.learner.select_action(obs)
                    self.learner.update_buffer(obs, action, action_logprob)
                next_obs, reward, done, info = self.env.step(action)
                step_end = rospy.get_time()
                stats.write_step_to_file(
                    file=stats_file,
                    episode=episode,
                    step=step_count,
                    reward=reward,
                    duration=step_end - step_start,
                    action=action_dict[action]
                )
                rospy.loginfo(f"[LearningAgent] Step {step_count} written to stats file.")
                self.learner.buffer.rewards.append(reward)
                self.learner.buffer.is_terminals.append(done)

                obs = next_obs
                step_count += 1

                rospy.loginfo(f"[LearningAgent] Step {step_count}: reward={reward}, done={done}")

                # Optionally log info
                if info.get("failure"):
                    rospy.logwarn(f"[LearningAgent] Info: {info}")

        # Trigger PPO update after trajectory collected
        avg_loss, avg_advantage = self.learner.update()

        rospy.loginfo(f"[LearningAgent] PPO update complete: avg_loss={avg_loss:.4f}, avg_adv={avg_advantage:.4f}")

        if done:
            rospy.loginfo(f"[LearningAgent] Plannable state achieved in {step_count} steps → recovery complete.")
            return True
        else:
            rospy.logwarn("[LearningAgent] Max steps reached or aborted → recovery failed.")
            return False
    
    # Prompts the user for a primitive action in a human demonstration..
    def get_demonstration_action(self) -> Union[int, None]:
        valid_actions = {"move_forward", "turn_left", "turn_right", "l", "r", "f"}
        action_map = {
            "f": "move_forward",
            "l": "turn_left",
            "r": "turn_right",
        }
    
        while True:
            action_name = input("Enter an action (move_forward, turn_left, turn_right): ").strip().lower()
            if action_name in action_map:
                action_name = action_map[action_name]
            if action_name in valid_actions:
                action_id = self.env.action_space.get_action_id(action_name)
                if action_id is None:
                    print(f"Action '{action_name}' not found.")
                    continue
                print(f"Chose action: {action_name}")
                return action_id
            else:
                print("Invalid input. Please enter one of: move_forward, turn_left, turn_right.")

    def save_policy(self, filename):
        """
        Save the current policy to a file.
        """
        self.learner.save(filename)
        rospy.loginfo(f"[LearningAgent] Policy saved to {filename}")
        
    def load_policy(self, filename):
        """
        Load a policy from a file.
        """
        self.learner.load(filename)
        rospy.loginfo(f"[LearningAgent] Policy loaded from {filename}")
