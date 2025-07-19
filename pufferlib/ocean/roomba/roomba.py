'''Roomba navigation environment with continuous physics and basic sensors.'''

import gymnasium
import numpy as np

import pufferlib
from pufferlib.ocean.roomba import binding

class Roomba(pufferlib.PufferEnv):
    def __init__(self, num_envs=1, render_mode=None, log_interval=128, 
                 room_width=8.0, room_height=6.0, max_steps=1000, buf=None, seed=0):
        # Observation space: [bump_sensor, front_dist, right_dist, back_dist, left_dist]
        self.single_observation_space = gymnasium.spaces.Box(
            low=0.0, high=1.0, shape=(5,), dtype=np.float32)
        
        # Action space: STOP, FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT
        self.single_action_space = gymnasium.spaces.Discrete(5)
        
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
    env = Roomba(num_envs=N, room_width=6.0, room_height=4.0, max_steps=500)
    env.reset()
    steps = 0

    # Cache some random actions for testing
    CACHE = 1024
    actions = np.random.randint(0, 5, (CACHE, N))

    import time
    start = time.time()
    while time.time() - start < 10:
        env.step(actions[steps % CACHE])
        steps += 1

    print('Roomba SPS:', int(env.num_agents * steps / (time.time() - start)))