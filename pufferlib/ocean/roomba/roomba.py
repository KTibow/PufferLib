'''Roomba navigation environment with continuous physics and basic sensors.'''

import gymnasium
import numpy as np

import pufferlib
from pufferlib.ocean.roomba import binding

dt = 0.05
class Roomba(pufferlib.PufferEnv):
    def __init__(self, num_envs=1, render_mode=None,
                 max_steps=1000, buf=None, seed=0):
        # Observation space: [left_bumper, right_bumper]
        # Binary sensors: 0 = not pressed, 1 = pressed
        self.single_observation_space = gymnasium.spaces.Box(
            low=np.array([0.0, 0.0]),
            high=np.array([1.0, 1.0]),
            shape=(2,), dtype=np.float32)

        # Action space: [left_wheel_speed, right_wheel_speed] normalized to [-1, 1]
        # Will be scaled in the C environment
        self.single_action_space = gymnasium.spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        self.render_mode = render_mode
        self.num_agents = num_envs
        self.max_steps = max_steps

        super().__init__(buf)
        self.c_envs = binding.vec_init(
            self.observations, self.actions, self.rewards,
            self.terminals, self.truncations, num_envs, seed,
            max_steps=max_steps)

    def reset(self, seed=0):
        binding.vec_reset(self.c_envs, seed)
        self.tick = 0
        return self.observations, []

    def step(self, actions):
        self.actions[:] = actions

        self.tick += 1
        binding.vec_step(self.c_envs)

        info = []
        if self.terminals.any():
            info.append(binding.vec_log(self.c_envs))

        return (self.observations, self.rewards,
            self.terminals, self.truncations, info)

    def render(self):
        binding.vec_render(self.c_envs, 0)

    def close(self):
        binding.vec_close(self.c_envs)
