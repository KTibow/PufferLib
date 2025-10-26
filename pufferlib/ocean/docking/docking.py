from typing import Any, override, final

import gymnasium
import numpy as np

import pufferlib
from pufferlib.ocean.docking import binding


@final
class Docking(pufferlib.PufferEnv):
    def __init__(
        self,
        num_envs: int = 1,
        render_mode: str | None = None,
        log_interval: int = 128,
        buf: dict[str, np.ndarray[Any, Any]] | None = None,
        seed: int = 0,
    ):
        # Observation: [force_field, green_buoy, red_buoy, angled_active, wall_nearby] (all binary 0/1)
        self.single_observation_space = gymnasium.spaces.Box(
            low=0.0, high=1.0, shape=(5,), dtype=np.float32
        )
        # Action: [left_wheel, right_wheel] each in [-1, 1]
        self.single_action_space = gymnasium.spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32
        )
        self.render_mode = render_mode
        self.num_agents = num_envs

        super().__init__(buf)
        self.c_envs = binding.vec_init(
            self.observations,
            self.actions,
            self.rewards,
            self.terminals,
            self.truncations,
            num_envs,
            seed,
        )

    @override
    def reset(self, seed: int | None = None):
        binding.vec_reset(self.c_envs, seed)
        return self.observations, []

    @override
    def step(self, actions: np.ndarray[Any, Any]):
        self.actions[:] = actions
        binding.vec_step(self.c_envs)
        info = [binding.vec_log(self.c_envs)]
        return (self.observations, self.rewards, self.terminals, self.truncations, info)

    def render(self):
        binding.vec_render(self.c_envs, 0)

    @override
    def close(self):
        binding.vec_close(self.c_envs)


if __name__ == "__main__":
    N = 4096
    env = Docking(num_envs=N)
    _ = env.reset()
    steps = 0

    CACHE = 1024
    # Random continuous actions for left and right wheels
    actions = np.random.uniform(-1.0, 1.0, (CACHE, N, 2)).astype(np.float32)

    import time

    start = time.time()
    while time.time() - start < 10:
        _ = env.step(actions[steps % CACHE])
        steps += 1

    print("SPS:", int(env.num_agents * steps / (time.time() - start)))
