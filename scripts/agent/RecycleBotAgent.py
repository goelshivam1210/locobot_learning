from Agent import Agent
from planner.Planner import Planner
from ppo_learner import PPOLearner
import sys
sys.path.append("/home/bharatkesari/interbotix_ws/src/locobot_custom/scripts/environment")
from RecycleBot import RecycleBot
from os.path import join

class RecycleBotAgent(Agent):

    def __init__(self, planner: Planner, sym_actions: dict, executor_path: str, num_eps: int) -> None:
        super().__init__(planner, sym_actions)
        self._executor_path = executor_path
        self._num_eps = num_eps

    def learn_executor(self,
                       action: str):
        problem_file = join(self.planner.problem_dir, f"{self.planner.problem_prefix}.pddl")
        self.new_problem() # generate a new problem file
        # env = RecycleBot(self.planner.domain_path, self.planner.problem_path, action)
        env = RecycleBot(self.planner.domain_path, problem_file, action, self.planner, self._sym_actions, self._executor_path)
        learner = PPOLearner(
            obs_space=env.observation_space,
            act_space=env.action_space,
            hidden_sizes=[256]
        )

        env.reset()

        for epi in range(self._num_eps):
            observation, info = env.reset()
            terminated = False
            truncated = False
            epi_len = 0
            total_return = 0

            while not terminated and not truncated:
                # print ("observation: ", observation)
                # print ("observation.shape while doing the forward pass: {} ".format(observation.shape))
                act = learner.get_action(observation)
                prev_obs = observation
                observation, reward, terminated, truncated, info = env.step(act)

                trans_dict = {
                    "obs": prev_obs,
                    "obs_next": observation,
                    "rew": reward,
                    "act": act,
                    "done": terminated or truncated,
                    "truncated": truncated,
                    "terminated": terminated,
                    "info": info
                }
                
                epi_len += 1
                total_return += reward

                learner.update(trans_dict)
            print(f"epi: {epi}, return: {total_return}")

    
    def load_and_run_executor(self, action: str):
        raise NotImplementedError