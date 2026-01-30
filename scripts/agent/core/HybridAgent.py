# core/HybridAgent.py

from typing import Union
import sys
import os
import rospy
import json

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'environment')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'environment', 'reward')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'planner')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'learner')))

from planner.planner import Planner
from PDDLActions import PDDLActions
from PDDLPredicates import PDDLPredicates
from learner.LearningAgent import LearningAgent
from learner.learning_stats import LearningStats
from Agent import Agent
from exceptions import ActionExecutionError

from learner.LearningAgent import LearningAgent
from reward_function import RewardFunction
from RecycleBotSMDP import RecycleBotSMDP
from learner.PPO import PPO


DEFAULT_MAX_STEPS = 25
DEFAULT_NUM_EPISODES = 30

COMPLETED_POLICY_DIRECTORY = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "policies"))


class HybridAgent:
    def __init__(
        self,
        domain_file,
        objects,
        run_dir: str,
        max_retries=3,
        num_demonstrations=0,
        include_local_view=True,
        max_steps: int = DEFAULT_MAX_STEPS,
        num_episodes: int = DEFAULT_NUM_EPISODES,
        include_symbolic_actions=False,
        policy_checkpoint_frequency: int = 5,
    ):
        """
        Initializes the hybrid agent with planning and learning capabilities.
        domain_file: Path to the PDDL domain file.
        objects: Dictionary of objects in the environment.
        run_dir: Directory to save learning data.
        max_retries: Maximum retries for failed actions.
        num_demonstrations: Number of human demonstrations before learning.
        include_local_view: Whether to include local view in the environment.
        max_steps: Maximum steps per learning episode.
        num_episodes: Number of learning episodes to run.
        include_symbolic_actions: Whether to include symbolic actions in the environment.
        policy_checkpoint_frequency: Frequency (in episodes) to save policy checkpoints.
        """
        self.planner = Planner(domain_file)
        self.actions = PDDLActions()
        self.predicates = PDDLPredicates()
        self.agent = Agent(self.planner, self.actions, self.predicates)
        self.objects = objects
        self.run_dir = run_dir
        self.max_retries = max_retries
        self.num_demonstrations = num_demonstrations
        self.include_local_view = include_local_view
        self.max_steps = max_steps
        self.num_episodes = num_episodes
        self.include_symbolic_actions = include_symbolic_actions
        self.stats_file_path = os.path.join(self.run_dir, "stats.csv")
        self.policy_checkpoint_frequency = policy_checkpoint_frequency
        self.base_policy_directory = os.path.join(self.run_dir, "policies")

    def run(self):
        """
        Execute the plan and handle failures dynamically.
        """

        success = None
        try:
            self.agent.run(self.objects)
        except ActionExecutionError as e:
            rospy.logwarn(f"[HybridAgent] Action failed: {e} | Action: {e.action_name}, Params: {e.params}")
            success = self.invoke_learning(e.action_name, e.params)

        if success == True:
            rospy.loginfo("[HybridAgent] Plan executed successfully.")

    def get_filename_for_operator_episode(self, operator, episode: Union[int, None] = None) -> str:
        """
        Returns the filename for the policy file for a given operator and episode.
        """
        param_string = "_".join(operator.parameters) if operator.parameters else "_"
        episode_string = f"episode_{episode}" if episode is not None else ""
        filename = f"{param_string}_{episode_string}.pth"
        return filename

    
    def get_policy_directory(self, operator) -> str:
        """
        Returns the directory path for the given operator's policies.
        """
        return os.path.join(self.base_policy_directory, operator.name)

    def get_completed_policy_file_path(self, failed_op) -> str:
        """
        Returns the path to the completed policy file, if it exists.
        """
        
        # File path is of the form "approach/bin_1_room_2_doorway_1.pth"
        params_str = "_".join(failed_op.parameters) if failed_op.parameters else ""
        return os.path.join(
            COMPLETED_POLICY_DIRECTORY,
            failed_op.name + params_str + ".pth",
        )


    def get_incomplete_policy_file(self, failed_op):
        """
        Returns the path to the incomplete policy file, if it exists.
        """
        op_directory = self.get_policy_directory(failed_op)
        if not os.path.exists(op_directory):
            return None
        files = os.listdir(op_directory)
        policy_files = [file for file in files if file.endswith('.pth')]
        if len(policy_files) == 0:
            return None
        policy_files.sort(key=lambda x: os.path.getmtime(os.path.join(op_directory, x)), reverse=True)
        incomplete_policy_file = policy_files[0]
        return os.path.join(op_directory, incomplete_policy_file)


    def resume_incomplete_policy(self, model: PPO, failed_op) -> int:
        """
        Resumes the learning process from a saved policy.
        """
        incomplete_policy_file = self.get_incomplete_policy_file(failed_op)
        if incomplete_policy_file is None:
            return 0
        
        continue_policy = input("Incomplete policy found. Do you want to continue learning from it? ([Y]/n): ")
        if continue_policy.strip().lower() == 'n':
            return 0
            
        import re
        episode_number = re.search(r'episode_(\d+)\.pth$', incomplete_policy_file)
        if not episode_number:
            rospy.logerr(f"[HybridAgent] {incomplete_policy_file} is an invalid incomplete policy file name format. Starting learning from beginning.")
            return 0
        episode_number = int(episode_number.group(1))
        model.load(incomplete_policy_file)
        rospy.loginfo(f"[HybridAgent] Resuming learning from episode {episode_number + 1} with policy {incomplete_policy_file}")
        return episode_number + 1
        
    def save_incomplete_policy(self, learner: LearningAgent, episode: int, failed_op):
        """
        Saves the current policy as an incomplete policy.
        """

        filename = self.get_filename_for_operator_episode(failed_op, episode=episode)
        directory = self.get_policy_directory(failed_op)

        if not os.path.exists(directory):
            os.makedirs(directory)
        policy_path = os.path.join(directory, filename)
        rospy.loginfo(f"[HybridAgent] Saving incomplete policy for episode {episode} at {policy_path}")

        learner.save_policy(policy_path)
        rospy.loginfo(f"[HybridAgent] Saved incomplete policy for episode {episode} at {policy_path}")


    def remove_incomplete_policy(self, incomplete_policy_file: Union[str, None]):
        """
        Removes the incomplete policy file if it exists.
        """

        full_path = os.path.join(self.base_policy_directory, incomplete_policy_file) if incomplete_policy_file else None
        if full_path is not None:
            if not os.path.exists(full_path):
                rospy.logwarn(f"[HybridAgent] Incomplete policy file {full_path} does not exist.")
                return
            try:
                os.remove(full_path)
                rospy.loginfo(f"[HybridAgent] Removed incomplete policy file: {full_path}")
                files = os.listdir(os.path.dirname(full_path))
                rospy.loginfo(f"[HybridAgent] Remaining files in incomplete policy directory: {files}")
                if files and len(files) > 0:
                    return
                    # if the directory is empty, coninue below to remove it
            except OSError as e:
                rospy.logerr(f"[HybridAgent] Failed to remove incomplete policy file {full_path}: {e}")
    
    def run_saved_policy(self, model: PPO, env: RecycleBotSMDP, policy_path: str) -> Union[bool, None]:
        """
        Runs the saved policy for the failed operator.
        """
        
        # Load the saved policy
        rospy.loginfo(f"[HybridAgent] Running saved policy at {policy_path}")
        model.load(policy_path)
        done = False
        step = 0
        while step < self.max_steps and not done:
            step += 1
            obs = env.observation_space.get_observation()

            # action = model.select_optimal_action(obs)
            action, action_logprob = model.select_action(obs)
            rospy.loginfo(f"[HybridAgent] Selected action ID: {action}")
            rospy.loginfo(f"[HybridAgent] Executing action: {env.action_space.get_action(action)}")
            _, _, done, _ = env.step(action)
            rospy.loginfo(f"[HybridAgent] Action log probability: {action_logprob}")
        if done:
            rospy.loginfo(f"[HybridAgent] Saved policy execution completed in {step} steps.")
            return True
        else:
            rospy.loginfo(f"[HybridAgent] Saved policy execution failed after {step} steps")
            return False
    

    def write_parameters_json(self, params: dict):
        """
        Write the parameters of the learning agent to a JSON file in the run directory.
        """
        params_file_path = os.path.join(self.run_dir, "learning_agent_params.json")

        if os.path.exists(params_file_path):
            rospy.loginfo(f"[HybridAgent] Parameters file already exists at {params_file_path}, skipping write.")
            return
        with open(params_file_path, 'w') as f:
            json.dump(params, f, indent=4)
        rospy.loginfo(f"[HybridAgent] Parameters written to {params_file_path}")


    def invoke_learning(self, action_name, params):
        """
        Handles transition to learning mode on execution failure.
        Computes plannable states for recovery.
        """
        rospy.logwarn(f"[HybridAgent] Switching to learning mode for action: {action_name} {params}")

        failed_op = self.find_failed_operator(action_name, params)
        if failed_op is None:
            rospy.logerr("[HybridAgent] Cannot compute plannable states without failed operator.")
            return

        plan = self.planner.plan
        if plan is None:
            rospy.logerr("[HybridAgent] Planner has no saved plan.")
            return

        plannable_state = self.planner.compute_plannable_states(plan, failed_op)
        rospy.loginfo(f"[HybridAgent] Computed plannable state: {plannable_state}")

        # Create RewardFunction with computed plannable state
        reward_function = RewardFunction(plannable_state, failed_operator=failed_op)

        # Create a new environment instance with reward function
        env = RecycleBotSMDP(
            reward_function=reward_function,
            include_local_view=self.include_local_view,
            failed_operator=failed_op,
            include_symbolic_actions=self.include_symbolic_actions
        )
        
        # Get observation + action space sizes
        state_dim = env.observation_space.get_observation_size()
        action_dim = env.action_space.size

        rospy.loginfo(f"Action space size: {action_dim}")

        ppo_args = {
            "state_dim": state_dim,
            "action_dim": action_dim,
            "lr_actor": 0.0003,
            "lr_critic": 0.001,
            "gamma": 0.99,
            "K_epochs": 2,
            "eps_clip": 0.2
        }

        # Create PPO learner
        ppo_model = PPO(**ppo_args)

        self.write_parameters_json(ppo_args)

        policy_path = self.get_completed_policy_file_path(failed_op)

        policy_exists = policy_path is not None and os.path.exists(policy_path)

        if policy_exists:
            success = self.run_saved_policy(ppo_model, env, policy_path)
            return success

        # Create LearningAgent
        learner = LearningAgent(
            env=env,
            learner_model=ppo_model,
            max_steps=self.max_steps,
            run_dir=self.run_dir,
        )

        episode = self.resume_incomplete_policy(ppo_model, failed_op)

        if episode == 0:
            demo_count = 0
            # If configured, request human demonstrations before attempting learning by itself
            rospy.loginfo(f"[HybridAgent] Number of human demonstrations: {self.num_demonstrations}")
            while demo_count < self.num_demonstrations:
                rospy.loginfo(f"[HybridAgent] Human demonstration {demo_count + 1}:")
                learner.learn(demonstration=True)
                # Reset the robot in preparation for starting a new episode
                env.reset()
                demo_count += 1

        # Now let it learn by itself
        rospy.loginfo(f"[HybridAgent] Self learning episodes:")

        learning_stats = LearningStats()

        while episode < self.num_episodes:
            episode_start = rospy.get_time()
            rospy.loginfo(f"[HybridAgent] Episode {episode}:")
            success = learner.learn(
                stats=learning_stats,
                episode=episode,
            )
            if success:
                learning_stats.successes += 1
            rospy.loginfo(f"[HybridAgent] Episode {episode} completed. Success: {success}")
            try:
                incomplete_policy_file = self.get_incomplete_policy_file(failed_op)
                self.save_incomplete_policy(learner, episode, failed_op)
                # Remove incomplete policy, if this episode is not one of the checkpoint episodes
                if incomplete_policy_file is not None and episode % self.policy_checkpoint_frequency != 0:
                    self.remove_incomplete_policy(incomplete_policy_file)
            except Exception as e:
                rospy.logerr(f"[HybridAgent] Failed to save incomplete policy: {e}")
            episode_end = rospy.get_time()
            rospy.loginfo(f"[HybridAgent] Episode {episode} duration: {episode_end - episode_start} seconds")
            
            if not success:
                # It's possible that the episode exhausted its step quota but got the environment
                # into a state thate if can execute the action in; e.g. moved an obstacle away but
                # didn't have enough steps left to face the object. In that case, we retry the action.
                # If it can execute it, consider the episode a success.
                success = self.retry_failed_action(action_name, params)
                if success:
                    learning_stats.successes += 1

            episode += 1
            env.prepare_for_reset()
            env.prompt_for_learning()

        if learning_stats.successes > 0:
            # Save the learned policy
            rospy.loginfo(f"[HybridAgent] {learning_stats.successes}/{episode} episodes succeeded ({(learning_stats.successes/episode) * 100}%). Retrying plan.")
            learner.save_policy(policy_path)
            self.remove_incomplete_policy(None)
            rospy.loginfo(f"[HybridAgent] Policy saved to path: {policy_path}")
            self.run()
        else:
            rospy.logerr("[HybridAgent] Learning failed. Aborting.")

        # if self.stats_file_path is not None:
        #     try:
        #         with open(self.stats_file_path, 'w') as stats_file:
        #             learning_stats.write_to_file(stats_file)
        #     except Exception as e:
        #         rospy.logerr(f"[HybridAgent] Failed to write learning stats to file: {e}")

    def retry_failed_action(self, failed_op_name: str, params: list):
        """
        Retries the failed action.

        Arguments:
        failed_op_name -- The failed operator name.
        params -- The parameters for the failed operator.

        Returns True if the action succeeds on retry, False otherwise.
        """
        rospy.loginfo(f"[HybridAgent] Retrying action {failed_op_name} with params {params}.")
        try:
            self.agent.run_action(failed_op_name, list(params))
            rospy.loginfo(f"[HybridAgent] Action {failed_op_name} succeeded on retry.")
            return True
        except ActionExecutionError as e:
            rospy.logwarn(f"[HybridAgent] Retry for action {failed_op_name} failed: {e}")
            return False

    def find_failed_operator(self, action_name, params):
        """
        Finds the failed operator in the current plan based on the action name and parameters.
        """
        for op in self.planner.plan:
            if op.name == action_name and list(op.parameters) == list(params):
                return op
        return None


if __name__ == "__main__":
    domain_file = "path_to_your_domain_file"
    objects = {
        'doorway': ['doorway_1'],
        'room': ['room_1', 'room_2'],
        'ball': ['ball_1'],
        'can': ['can_1'],
        'bin': ['bin_1'],
        'nothing': ['nothing'],
        'robot': ['robot_1']
    }

    agent = HybridAgent(domain_file, None, objects)
    agent.run()
