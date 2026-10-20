"""Configuration objects shared by the numerical MPC simulation."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PlantConfig:
    """Exact sampled first-order velocity plant parameters."""

    sample_time: float
    tau: np.ndarray
    delay_steps: int

    def __post_init__(self) -> None:
        tau = np.asarray(self.tau, dtype=float)
        if self.sample_time <= 0.0:
            raise ValueError("sample_time must be positive")
        if tau.shape != (3,) or np.any(tau <= 0.0):
            raise ValueError("tau must contain three positive time constants")
        if self.delay_steps < 0:
            raise ValueError("delay_steps must be non-negative")
        object.__setattr__(self, "tau", tau)
