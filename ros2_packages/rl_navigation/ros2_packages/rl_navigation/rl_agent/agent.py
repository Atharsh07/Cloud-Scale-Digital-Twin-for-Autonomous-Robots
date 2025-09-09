import numpy as np

class RLAgent:
    """
    Simple Q-learning agent for demonstration.
    Can be extended with DQN or PPO for complex environments.
    """
    def __init__(self, actions):
        self.actions = actions
        self.q_table = {}  # state: action_values
        self.alpha = 0.5
        self.gamma = 0.9
        self.epsilon = 0.1

    def get_state_key(self, state):
        return tuple(np.round(state, 2))  # discretize

    def choose_action(self, state):
        key = self.get_state_key(state)
        if key not in self.q_table:
            self.q_table[key] = np.zeros(len(self.actions))
        if np.random.rand() < self.epsilon:
            return np.random.choice(self.actions)
        return self.actions[np.argmax(self.q_table[key])]

    def update_q(self, state, action, reward, next_state):
        key = self.get_state_key(state)
        next_key = self.get_state_key(next_state)
        if key not in self.q_table:
            self.q_table[key] = np.zeros(len(self.actions))
        if next_key not in self.q_table:
            self.q_table[next_key] = np.zeros(len(self.actions))
        action_idx = self.actions.index(action)
        self.q_table[key][action_idx] += self.alpha * (reward + self.gamma * np.max(self.q_table[next_key]) - self.q_table[key][action_idx])
