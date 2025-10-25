"""Type stub for the docking binding module (C extension)."""

from typing import Any, NewType
import numpy as np

# Opaque type representing a C environment handle
CEnvHandle = NewType('CEnvHandle', int)

def vec_init(
    observations: np.ndarray[Any, Any],
    actions: np.ndarray[Any, Any],
    rewards: np.ndarray[Any, Any],
    terminals: np.ndarray[Any, Any],
    truncations: np.ndarray[Any, Any],
    num_envs: int,
    seed: int,
    *,
    size: int,
) -> CEnvHandle: ...

def vec_reset(c_envs: CEnvHandle, seed: int | None) -> None: ...

def vec_step(c_envs: CEnvHandle) -> None: ...

def vec_log(c_envs: CEnvHandle) -> dict[str, Any]: ...

def vec_render(c_envs: CEnvHandle, index: int) -> None: ...

def vec_close(c_envs: CEnvHandle) -> None: ...
