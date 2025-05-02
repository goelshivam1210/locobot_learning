# learner/LearningAgent.py

import rospy

class LearningAgent:
    def __init__(self, env, learner_model):
        """
        env: instance of RecycleBotSMDP
        learner_model: instance of PPO
        """
        self.env = env
        self.learner = learner_model

    def learn(self, max_steps=5000):
        """
        Run PPO learning loop until recovery is achieved (done=True) or max_steps reached.
        """
        rospy.loginfo("[LearningAgent] Starting learning process...")

        obs = self.env.observation_space.get_observation()  # or env.reset() if implemented
        done = False
        step_count = 0

        while not done and step_count < max_steps and not rospy.is_shutdown():
            action = self.learner.select_action(obs)
            next_obs, reward, done, info = self.env.step(action)

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
            rospy.loginfo("[LearningAgent] Plannable state achieved → recovery complete.")
            return True
        else:
            rospy.logwarn("[LearningAgent] Max steps reached or aborted → recovery failed.")
            return False
