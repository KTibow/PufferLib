'''Simple grid coverage environment for debugging roomba learning issues.

Agent is constrained to a discrete grid with simple observations and actions.
Designed to help identify where the learning problem actually is.
'''

import gymnasium
import numpy as np

import pufferlib
from pufferlib.ocean.coverage_grid import binding

class CoverageGrid(pufferlib.PufferEnv):
    def __init__(self, num_envs=1, render_mode=None, log_interval=128, 
                 grid_width=8, grid_height=6, max_steps=100, 
                 obs_type='position', buf=None, seed=0):
        
        # Observation space depends on obs_type
        if obs_type == 'position':
            # [x, y] normalized to [0, 1]
            self.single_observation_space = gymnasium.spaces.Box(
                low=0.0, high=1.0, shape=(2,), dtype=np.float32)
        elif obs_type == 'distances':
            # [dist_left, dist_right, dist_up, dist_down] normalized to [0, 1]
            self.single_observation_space = gymnasium.spaces.Box(
                low=0.0, high=1.0, shape=(4,), dtype=np.float32)
        else:
            raise ValueError(f"obs_type must be 'position' or 'distances', got {obs_type}")
        
        # Action space: UP, DOWN, LEFT, RIGHT
        self.single_action_space = gymnasium.spaces.Discrete(4)
        
        self.render_mode = render_mode
        self.num_agents = num_envs
        self.log_interval = log_interval
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.max_steps = max_steps
        self.obs_type = obs_type

        super().__init__(buf)
        self.c_envs = binding.vec_init(
            self.observations, self.actions, self.rewards,
            self.terminals, self.truncations, num_envs, seed, 
            grid_width=grid_width, grid_height=grid_height, 
            max_steps=max_steps, obs_type=obs_type)
 
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
    env = CoverageGrid(num_envs=N, grid_width=4, grid_height=3, max_steps=20, obs_type='position')
    env.reset()
    steps = 0

    # Cache some random actions for testing
    CACHE = 64
    actions = np.random.randint(0, 4, (CACHE, N))

    import time
    start = time.time()
    while time.time() - start < 5:
        env.step(actions[steps % CACHE])
        steps += 1

    print('Coverage Grid SPS:', int(env.num_agents * steps / (time.time() - start)))