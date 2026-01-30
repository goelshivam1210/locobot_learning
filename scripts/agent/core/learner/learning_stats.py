import os
from io import TextIOBase

class LearningStats:
    def __init__(self):
        self.step_rewards_by_episode = {}
        self.step_durations_by_episode = {}
        self.successes = 0
        self.actions_by_step = {}

    def load_successes_from_file(self, file: TextIOBase):
        file.seek(0)
        lines = file.readlines()
        for line in lines[1:]:  # Skip header
            parts = line.rstrip("\n").split(',')
            if len(parts) == 5:
                try:
                    self.successes = int(parts[4])
                except ValueError:
                    continue

    def write_header_to_file(self, file: TextIOBase):
        file.write("episode,step,reward,duration,total_success,action\n")

    def log_step(self, episode: int, step: int, reward: float, duration: float, action: str):
        if episode not in self.step_rewards_by_episode:
            self.step_rewards_by_episode[episode] = {}
        self.step_rewards_by_episode[episode][step] = reward
        if episode not in self.step_durations_by_episode:
            self.step_durations_by_episode[episode] = {}
        self.step_durations_by_episode[episode][step] = duration
        if episode not in self.actions_by_step:
            self.actions_by_step[episode] = {}
        self.actions_by_step[episode][step] = action

    def write_step_to_file(self, file: TextIOBase, episode: int, step: int, reward: float, duration: float, action: str):
        if file.tell() == 0:
            self.write_header_to_file(file)
        file.write(f"{episode},{step},{reward},{duration},{self.successes},{action}\n")