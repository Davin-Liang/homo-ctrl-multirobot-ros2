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


@dataclass(frozen=True)
class MpcConfig:
    """Weights and hard bounds for the convex finite-horizon controller."""

    horizon: int
    q: np.ndarray
    q_terminal: np.ndarray
    r_relative: np.ndarray
    r_slew: np.ndarray
    input_min: np.ndarray
    input_max: np.ndarray
    slew_min: np.ndarray
    slew_max: np.ndarray
    model_delay_steps: int

    def __post_init__(self) -> None:
        arrays = {
            "q": (self.q, 6),
            "q_terminal": (self.q_terminal, 6),
            "r_relative": (self.r_relative, 3),
            "r_slew": (self.r_slew, 3),
            "input_min": (self.input_min, 3),
            "input_max": (self.input_max, 3),
            "slew_min": (self.slew_min, 3),
            "slew_max": (self.slew_max, 3),
        }
        if self.horizon <= 0 or self.model_delay_steps < 0:
            raise ValueError("horizon must be positive and model_delay_steps non-negative")
        normalized = {}
        for name, (value, length) in arrays.items():
            value = np.asarray(value, dtype=float)
            if value.shape != (length,):
                raise ValueError(f"{name} must have {length} entries")
            normalized[name] = value
        if np.any(normalized["q"] < 0.0) or np.any(normalized["q_terminal"] < 0.0):
            raise ValueError("state weights must be non-negative")
        if np.any(normalized["r_relative"] <= 0.0) or np.any(normalized["r_slew"] <= 0.0):
            raise ValueError("input weights must be positive")
        if np.any(normalized["input_min"] > normalized["input_max"]):
            raise ValueError("input lower bounds exceed upper bounds")
        if np.any(normalized["slew_min"] > normalized["slew_max"]):
            raise ValueError("slew lower bounds exceed upper bounds")
        for name, value in normalized.items():
            object.__setattr__(self, name, value)
