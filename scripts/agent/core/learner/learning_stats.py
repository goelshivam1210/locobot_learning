import os
from io import TextIOBase

class LearningStats:
    def __init__(self):
        self.step_rewards_by_episode = {}
        self.step_durations_by_episode = {}
        self.successes = 0

    def log_step(self, episode: int, step: int, reward: float, duration: float):
        if episode not in self.step_rewards_by_episode:
            self.step_rewards_by_episode[episode] = {}
        self.step_rewards_by_episode[episode][step] = reward
        if episode not in self.step_durations_by_episode:
            self.step_durations_by_episode[episode] = {}
        self.step_durations_by_episode[episode][step] = duration

    def write_to_file(self, file: TextIOBase):
        # if file is empty, write header
        file.seek(0, os.SEEK_END)
        if file.tell() == 0:
            file.write("episode,step,reward,duration\n")
        for episode in sorted(self.step_rewards_by_episode.keys()):
            for step in sorted(self.step_rewards_by_episode[episode].keys()):
                reward = self.step_rewards_by_episode[episode][step]
                duration = self.step_durations_by_episode[episode].get(step, 0.0)
                file.write(f"{episode},{step},{reward},{duration}\n")