class RewardFunction:
    def __init__(self, plannable_state):
        """
        plannable_state: set of predicates that defines recovery state
        """
        self.plannable_state = plannable_state

    def compute_reward(self, current_symbolic_state):
        """
        current_symbolic_state: set of predicates (from SymbolicState)
        Returns: reward (float), done (bool)
        """
        if self.plannable_state.issubset(current_symbolic_state):
            return 1.0, True  # success achieved
        else:
            return 0.0, False
