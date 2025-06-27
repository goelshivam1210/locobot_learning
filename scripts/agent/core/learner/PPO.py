import torch
import torch.nn as nn
from torch.distributions import Categorical


device = torch.device("mps") if torch.backends.mps.is_available() else torch.device("cuda" if torch.cuda.is_available() else "cpu")

class RolloutBuffer:
    def __init__(self):
        self.actions = []
        self.states = []
        self.logprobs = []
        self.rewards = []
        self.is_terminals = []
    
    def clear(self):
        del self.actions[:]
        del self.states[:]
        del self.logprobs[:]
        del self.rewards[:]
        del self.is_terminals[:]

class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, attention_net=None):
        super(ActorCritic, self).__init__()
        
        # Attention network (optional)
        self.attention_net = attention_net
    
        # Actor network
        self.actor = nn.Sequential(
            nn.Linear(state_dim, 256),  # Input layer -> Hidden layer: 128
            nn.ReLU(),
            # nn.LayerNorm(256),         # Normalize activations
            # nn.Dropout(0.1),           # Dropout for regularization
            nn.Linear(256, 128),        # Hidden layer: 128 -> Hidden layer: 64
            nn.ReLU(),
            nn.Linear(128, 64),  # Input layer -> Hidden layer: 128
            nn.ReLU(),
            # nn.LayerNorm(64),
            # nn.Dropout(0.1),
            nn.Linear(64, action_dim),  # Output layer: 64 -> Action space
            # Removed softmax at ChatGPT's suggestion to avoid NaNs
            # nn.Softmax(dim=-1)         # Output probabilities
        )

        # Critic network
        self.critic = nn.Sequential(
            nn.Linear(state_dim, 256),  # Input layer: 40 -> Hidden layer: 128
            nn.ReLU(),
            # nn.LayerNorm(128),
            # nn.Dropout(0.1),
            nn.Linear(256, 128),        # Hidden layer: 128 -> Hidden layer: 64
            nn.ReLU(),
            nn.Linear(128, 64),  # Input layer -> Hidden layer: 128
            nn.ReLU(),
            # nn.LayerNorm(64),
            # nn.Dropout(0.1),
            nn.Linear(64, 1)           # Output layer: 64 -> Single value (state value)
        )
    
    def forward(self):
        raise NotImplementedError
    
    def act(self, state, constraints=None):
        # Apply attention before actor if attention is being used
        if self.attention_net and constraints is not None:
            state = self.attention_net(state, constraints)
        
        # Ensure state has at least 2 dimensions: [1, input_features]
        if state.dim() == 1:
            state = state.unsqueeze(0)  # Add batch dimension

        # action_probs = self.actor(state)
        logits = self.actor(state)
        action_probs = torch.softmax(logits, dim=-1)  # still for printing only
        prob_list = [(id, prob) for id, prob in enumerate(action_probs.tolist()[0])]
        prob_list = sorted(prob_list, key=lambda pair: pair[1], reverse=True)
        print(f"[PPO -> ActorCritic] Action probabilities:")
        from pprint import pprint
        pprint(prob_list)
        # dist = Categorical(action_probs)
        dist = Categorical(logits=logits)
        action = dist.sample()
        action_logprob = dist.log_prob(action)
        return action.detach(), action_logprob.detach()
    
    def evaluate(self, state, action, constraints=None):
        # Apply attention before critic and actor if attention is being used
        if self.attention_net and constraints is not None:
            state = self.attention_net(state, constraints)
        
        # action_probs = self.actor(state)
        logits = self.actor(state)
        action_probs = torch.softmax(logits, dim=-1)  # for debug only
        for name, param in self.actor.named_parameters():
            if torch.any(torch.isnan(param)):
                print(f"NaNs in actor param: {name}")
            if torch.any(torch.isinf(param)):
                print(f"Infs in actor param: {name}")
        if torch.any(torch.isnan(action_probs)):
            print("NaNs detected in action_probs")
            print("Input state:", state)
            print("Actor output:", action_probs)
            raise ValueError("Actor output contains NaNs")
        # dist = Categorical(action_probs)
        # Suggested by ChatGPT: let Categorical handle softmax internally to avoid NaNs due to overflow/underflow
        dist = Categorical(logits=logits)

        action_logprobs = dist.log_prob(action)
        dist_entropy = dist.entropy()
        state_value = self.critic(state)
        
        return action_logprobs, state_value, dist_entropy

class PPO:
    def __init__(self, state_dim, action_dim, lr_actor, lr_critic, gamma, K_epochs, eps_clip, use_attention=False, attention_net=None):
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.K_epochs = K_epochs
        self.use_attention = use_attention
        
        self.buffer = RolloutBuffer()

        # Use attention if specified
        self.policy = ActorCritic(state_dim, action_dim, attention_net=attention_net if use_attention else None).to(device)
        
        self.optimizer = torch.optim.Adam([
            {'params': self.policy.actor.parameters(), 'lr': lr_actor},
            {'params': self.policy.critic.parameters(), 'lr': lr_critic}
        ])

        self.policy_old = ActorCritic(state_dim, action_dim, attention_net=attention_net if use_attention else None).to(device)
        self.policy_old.load_state_dict(self.policy.state_dict())
        
        self.MseLoss = nn.MSELoss()

    def select_optimal_action(self, state) -> int:
        with torch.no_grad():
            state = torch.FloatTensor(state).to(device)
        action_probs = self.policy_old.actor(state)
        print(f"[PPO] Selecting optimal action among probabilities: {action_probs}")
        action_id = torch.argmax(action_probs)

        print(f"[PPO] Selected action ID: {action_id}")

        return action_id

    def select_action(self, state, constraints=None):
        with torch.no_grad():
            state = torch.FloatTensor(state).to(device)
            # print(f"[PPO] Selecting random action among probabilities: {action_probs}")
            action, action_logprob = self.policy_old.act(state, constraints)  # Pass constraints
        
        # print(f"State shape before actor: {state.shape}")
        
        return action.item(), action_logprob.item()
    
    def update_buffer(self, state, action, action_logprob=None, testing=False):
        action_id = None
        if not testing:
            with torch.no_grad():
                state = torch.FloatTensor(state).to(device)
                if not isinstance(action, torch.Tensor):
                    action_id = action
                    action = torch.tensor(action).to(device)
            self.buffer.states.append(state)
            self.buffer.actions.append(action)
            if action_logprob is None:
                # Get logprob for action
                action_probs = self.policy_old.actor(state.unsqueeze(0))
                dist = Categorical(action_probs)
                action_logprob = dist.log_prob(action)
                print(f"[PPO] Derived log probability {action_logprob} for action ID {action_id}")
            self.buffer.logprobs.append(action_logprob)

    def update(self):
        # Monte Carlo estimate of returns
        rewards = []
        discounted_reward = 0

        for reward, is_terminal in zip(reversed(self.buffer.rewards), reversed(self.buffer.is_terminals)):
            if is_terminal:
                discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            rewards.insert(0, discounted_reward)
        
        # Convert list to tensor and normalize rewards
        rewards = torch.tensor(rewards, dtype=torch.float32).to(device)
        
        std = rewards.std()
        # The conditional check below was suggested by ChatGPT; it claims that it's more
        # robust in cases where std() is exactly 0 than adding 1e-7 to std.
        if torch.isnan(std) or std.item() < 1e-6:
            normalized_rewards = rewards - rewards.mean()
        else:
            normalized_rewards = (rewards - rewards.mean()) / std
        rewards = normalized_rewards
        # rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-7)
        
        # Convert lists to tensors
        old_states = torch.squeeze(torch.stack(self.buffer.states, dim=0)).detach().to(device)
        old_actions = torch.squeeze(torch.stack(self.buffer.actions, dim=0)).detach().to(device)
        old_logprobs = torch.squeeze(torch.tensor(self.buffer.logprobs, dtype=torch.float32)).detach().to(device)

        total_loss = 0
        all_advantages = []

        # Optimize policy for K epochs
        for _ in range(self.K_epochs):
            # In case you want to use constraints for the update, pass them here
            logprobs, state_values, dist_entropy = self.policy.evaluate(old_states, old_actions)

            state_values = torch.squeeze(state_values)

            # Find the ratio (pi_theta / pi_theta__old)
            ratios = torch.exp(logprobs - old_logprobs.detach())
            
            # Calculate advantages
            advantages = rewards - state_values.detach()
            all_advantages.append(advantages)
            
            # Surrogate loss
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1 - self.eps_clip, 1 + self.eps_clip) * advantages

            # Final loss of clipped objective PPO
            loss = -torch.min(surr1, surr2) + 0.5 * self.MseLoss(state_values, rewards) - 0.01 * dist_entropy
            total_loss += loss.mean().item()
            
            # Take gradient step
            self.optimizer.zero_grad()
            loss.mean().backward()
            # Added the line below at the suggestion of ChatGPT to avoid NaNs in gradients
            torch.nn.utils.clip_grad_norm_(self.policy.parameters(), max_norm=0.5)
            self.optimizer.step()



        # Copy new weights into old policy
        self.policy_old.load_state_dict(self.policy.state_dict())

        # Clear buffer
        self.buffer.clear()
        torch.cuda.empty_cache()

        for param in self.policy.parameters():
            if param.grad is not None:
                # print(f"[DEBUG] Gradient Norm: {torch.norm(param.grad).item()}")
                continue

        return total_loss / self.K_epochs, torch.stack(all_advantages).mean()

    def save(self, checkpoint_path):
        torch.save(self.policy_old.state_dict(), checkpoint_path)

    def load(self, checkpoint_path):
        self.policy_old.load_state_dict(torch.load(checkpoint_path))
        self.policy.load_state_dict(torch.load(checkpoint_path))