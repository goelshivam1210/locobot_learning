# environment/state/observation_space.py

import rospy
import numpy as np
from .SubSymbolicState import SubSymbolicState
from .SymbolicState import SymbolicState


class ObservationSpace:
    def __init__(self, use_symbolic=True, use_subsymbolic=True):
        """
        Combines symbolic and subsymbolic states into a unified observation space.
        """
        self.use_symbolic = use_symbolic
        self.use_subsymbolic = use_subsymbolic

        self.subsymbolic_state = SubSymbolicState() if use_subsymbolic else None
        self.symbolic_state = SymbolicState() if use_symbolic else None

        self.current_obs = None

    def get_observation(self):
        """
        Get the current observation vector (concatenated).
        """
        parts = []

        if self.use_subsymbolic:
            parts.append(self.subsymbolic_state.get_flattened_subsymbolic_obs())

        if self.use_symbolic:
            parts.append(self.symbolic_state.get_symbolic_encoding())

        self.current_obs = np.concatenate(parts, axis=0).astype(np.float32)
        return self.current_obs

    def get_observation_size(self):
        """
        Return the dimension of the full observation vector.
        """
        obs = self.get_observation()
        return obs.shape[0]

    def reset(self):
        """
        Reset if needed. Placeholder for future extensions.
        """
        rospy.loginfo("ObservationSpace reset.")
        self.current_obs = None


if __name__ == "__main__":
    rospy.init_node("observation_space_debug", anonymous=True)
    obs_space = ObservationSpace()
    obs = obs_space.get_observation()
    print("Observation shape:", obs.shape)
    print("Observation:", obs)
