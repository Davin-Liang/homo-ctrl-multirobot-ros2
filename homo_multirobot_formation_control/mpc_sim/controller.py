"""Convex delay-augmented linear MPC for map-frame formation error."""

from dataclasses import dataclass
from time import perf_counter

import cvxpy as cp
import numpy as np

from .config import MpcConfig, PlantConfig
from .model import exact_zoh


def augment_delay_model(
    ad: np.ndarray, bd: np.ndarray, delay_steps: int
) -> tuple[np.ndarray, np.ndarray]:
    """Augment x with oldest-to-newest delayed input queue entries."""
    if delay_steps < 0:
        raise ValueError("delay_steps must be non-negative")
    dimensions = 6 + 3 * delay_steps
    az = np.zeros((dimensions, dimensions))
    bz = np.zeros((dimensions, 3))
    az[:6, :6] = ad
    if delay_steps == 0:
        bz[:6, :] = bd
        return az, bz
    az[:6, 6:9] = bd
    for index in range(delay_steps - 1):
        start = 6 + 3 * index
        az[start : start + 3, start + 3 : start + 6] = np.eye(3)
    bz[-3:, :] = np.eye(3)
    return az, bz


@dataclass(frozen=True)
class MpcResult:
    relative_command: np.ndarray
    follower_command: np.ndarray
    status: str
    solve_seconds: float
    predicted_errors: np.ndarray


class DelayAwareMpc:
    """Builds a small OSQP problem per call to preserve transparent semantics."""

    def __init__(self, plant: PlantConfig, config: MpcConfig) -> None:
        if plant.delay_steps != config.model_delay_steps:
            raise ValueError("plant and MPC model delay steps must match")
        self.plant = plant
        self.config = config
        self.ad, self.bd = exact_zoh(plant)
        self.az, self.bz = augment_delay_model(
            self.ad, self.bd, config.model_delay_steps
        )

    def solve(
        self,
        error: np.ndarray,
        queued_relative_inputs: np.ndarray,
        leader_commands: np.ndarray,
        previous_follower_command: np.ndarray,
    ) -> MpcResult:
        """Solve the QP for the first relative velocity command."""
        error = np.asarray(error, dtype=float)
        queued_relative_inputs = np.asarray(queued_relative_inputs, dtype=float)
        leader_commands = np.asarray(leader_commands, dtype=float)
        previous_follower_command = np.asarray(previous_follower_command, dtype=float)
        delay = self.config.model_delay_steps
        if error.shape != (6,) or queued_relative_inputs.shape != (delay, 3):
            raise ValueError("error or delayed relative-input queue has an invalid shape")
        if leader_commands.shape != (self.config.horizon, 3):
            raise ValueError("leader_commands must have one row per horizon step")
        if previous_follower_command.shape != (3,):
            raise ValueError("previous_follower_command must have three entries")

        initial = np.r_[error, queued_relative_inputs.reshape(-1)]
        horizon, dimension = self.config.horizon, self.az.shape[0]
        relative = cp.Variable((horizon, 3))
        state = cp.Variable((horizon + 1, dimension))
        follower = leader_commands + relative
        constraints = [state[0, :] == initial]
        cost = 0.0
        for index in range(horizon):
            constraints.append(
                state[index + 1, :] == self.az @ state[index, :] + self.bz @ relative[index, :]
            )
            constraints.extend(
                [
                    follower[index, :] >= self.config.input_min,
                    follower[index, :] <= self.config.input_max,
                ]
            )
            previous = previous_follower_command if index == 0 else follower[index - 1, :]
            change = follower[index, :] - previous
            constraints.extend(
                [change >= self.config.slew_min, change <= self.config.slew_max]
            )
            cost += cp.sum_squares(cp.multiply(np.sqrt(self.config.q), state[index, :6]))
            cost += cp.sum_squares(cp.multiply(np.sqrt(self.config.r_relative), relative[index, :]))
            cost += cp.sum_squares(cp.multiply(np.sqrt(self.config.r_slew), change))
        cost += cp.sum_squares(
            cp.multiply(np.sqrt(self.config.q_terminal), state[horizon, :6])
        )

        problem = cp.Problem(cp.Minimize(cost), constraints)
        started = perf_counter()
        problem.solve(
            solver=cp.OSQP,
            warm_start=True,
            polishing=True,
            eps_abs=1e-8,
            eps_rel=1e-8,
        )
        elapsed = perf_counter() - started
        if relative.value is None or state.value is None:
            fallback = previous_follower_command.copy()
            return MpcResult(
                fallback - leader_commands[0], fallback, str(problem.status), elapsed,
                np.full((horizon + 1, 6), np.nan),
            )
        return MpcResult(
            relative.value[0].copy(), follower.value[0].copy(), str(problem.status), elapsed,
            state.value[:, :6].copy(),
        )
