'''Roomba navigation environment with continuous physics and basic sensors.'''

import gymnasium
import numpy as np

import pufferlib
from pufferlib.ocean.roomba import binding

dt = 0.05
class Roomba(pufferlib.PufferEnv):
    def __init__(self, num_envs=1, render_mode=None, log_interval=128,
                 room_width=800.0, room_height=600.0, max_steps=1000, buf=None, seed=0):
        # Observation space: [left_bumper_importance, right_bumper_importance, left_light_bumper_strength, right_light_bumper_strength]
        # Bumper importance: 2.0=just hit, 0.0=not hit recently (decays over 2 seconds)
        # Light bumper strength: continuous from 0.0-1.0 based on distance (closer than 10cm but further than 2cm rises from 0 to 1, closer than 2cm falls to 0)
        self.single_observation_space = gymnasium.spaces.Box(
            low=np.array([0.0, 0.0, 0.0, 0.0]),
            high=np.array([2.0, 2.0, 1.0, 1.0]),
            shape=(4,), dtype=np.float32)

        # Action space: [left_wheel_speed, right_wheel_speed] normalized to [-1, 1]
        # Will be scaled in the C environment
        self.single_action_space = gymnasium.spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        self.render_mode = render_mode
        self.num_agents = num_envs
        self.log_interval = log_interval
        self.room_width = room_width
        self.room_height = room_height
        self.max_steps = max_steps

        super().__init__(buf)
        self.c_envs = binding.vec_init(
            self.observations, self.actions, self.rewards,
            self.terminals, self.truncations, num_envs, seed,
            room_width=room_width, room_height=room_height, max_steps=max_steps)

    def reset(self, seed=0):
        binding.vec_reset(self.c_envs, seed)
        self.tick = 0
        return self.observations, []

    def step(self, actions):
        self.tick += 1

        self.actions[:] = actions
        binding.vec_step(self.c_envs)

        info = []
        if self.tick % self.log_interval == 0:
            info.append(binding.vec_log(self.c_envs))

        return (self.observations, self.rewards,
            self.terminals, self.truncations, info)

    def render(self):
        binding.vec_render(self.c_envs, 0)

    def close(self):
        binding.vec_close(self.c_envs)

if __name__ == '__main__':
    # Simple test of the environment
    N = 1
    env = Roomba(num_envs=N, room_width=600.0, room_height=400.0, max_steps=500)
    env.reset()
    steps = 0

    # Cache some random actions for testing (normalized [-1, 1])
    CACHE = 1024
    actions = np.random.uniform(-1, 1, (CACHE, N, 2))

    import time
    start = time.time()
    while time.time() - start < 10:
        env.step(actions[steps % CACHE])
        steps += 1

    print('Roomba SPS:', int(env.num_agents * steps / (time.time() - start)))
