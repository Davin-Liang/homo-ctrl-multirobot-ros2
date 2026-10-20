"""Exact ZOH error dynamics and fixed-length input-delay plant."""

from collections import deque

import numpy as np

from .config import PlantConfig


def exact_zoh(config: PlantConfig) -> tuple[np.ndarray, np.ndarray]:
    """Return exact sampled matrices for q_dot=v, v_dot=(u-v)/tau."""
    h = config.sample_time
    decay = np.exp(-h / config.tau)
    H = config.tau * (1.0 - decay)
    ad = np.block(
        [[np.eye(3), np.diag(H)], [np.zeros((3, 3)), np.diag(decay)]]
    )
    bd = np.vstack((np.diag(h - H), np.diag(1.0 - decay)))
    return ad, bd


class DelayedPlant:
    """Discrete plant whose queue head is applied over the next sample."""

    def __init__(
        self, config: PlantConfig, initial_error: np.ndarray, initial_input: np.ndarray
    ) -> None:
        self.ad, self.bd = exact_zoh(config)
        self.error = np.asarray(initial_error, dtype=float).copy()
        initial_input = np.asarray(initial_input, dtype=float)
        if self.error.shape != (6,) or initial_input.shape != (3,):
            raise ValueError("initial_error must have six and initial_input three entries")
        self.queue: deque[np.ndarray] = deque(
            [initial_input.copy() for _ in range(config.delay_steps)]
        )

    def step(self, command: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Advance one sample and return the state and applied queued command."""
        command = np.asarray(command, dtype=float)
        if command.shape != (3,):
            raise ValueError("command must have three entries")
        if self.queue:
            applied = self.queue.popleft()
            self.queue.append(command.copy())
        else:
            applied = command.copy()
        self.error = self.ad @ self.error + self.bd @ applied
        return self.error.copy(), applied.copy()
