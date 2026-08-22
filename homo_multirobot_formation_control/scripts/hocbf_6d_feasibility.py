#!/usr/bin/env python3
"""Oracle numerical utilities for predictor-state HOCBF feasibility studies."""

from dataclasses import dataclass

import numpy as np
from scipy.linalg import expm


@dataclass(frozen=True)
class PlantParams:
    """Discrete simulation parameters for the planar delayed actuator model."""

    tau: float
    delay: float
    dt: float

    def __post_init__(self):
        if self.tau <= 0.0 or self.dt <= 0.0 or self.delay < 0.0:
            raise ValueError("tau and dt must be positive; delay must be non-negative")
        delay_steps = self.delay / self.dt
        if not np.isclose(delay_steps, round(delay_steps), atol=1e-12):
            raise ValueError("delay must be an integer multiple of dt")

    @property
    def delay_steps(self) -> int:
        return int(round(self.delay / self.dt))


@dataclass(frozen=True)
class SafetyFilterResult:
    """Result of the hard constrained planar minimum-correction QP."""

    command: np.ndarray
    feasible: bool
    active_constraints: int


def zoh_matrices(params: PlantParams) -> tuple[np.ndarray, np.ndarray]:
    """Return exact ZOH matrices for p_dot=v and v_dot=(-v+u)/tau."""
    a = np.block(
        [
            [np.zeros((2, 2)), np.eye(2)],
            [np.zeros((2, 2)), -np.eye(2) / params.tau],
        ]
    )
    b = np.vstack((np.zeros((2, 2)), np.eye(2) / params.tau))
    augmented = np.zeros((6, 6))
    augmented[:4, :4] = a
    augmented[:4, 4:] = b
    transition = expm(augmented * params.dt)
    return transition[:4, :4], transition[:4, 4:]


def predict_delayed_state(
    x: np.ndarray,
    queued_commands: list[np.ndarray],
    ad: np.ndarray,
    bd: np.ndarray,
) -> np.ndarray:
    """Propagate x through known commands that will act before a new command."""
    predicted = np.asarray(x, dtype=float).copy()
    if predicted.shape != (4,):
        raise ValueError("x must contain [px, py, vx, vy]")
    for command in queued_commands:
        command = np.asarray(command, dtype=float)
        if command.shape != (2,):
            raise ValueError("queued commands must be two-dimensional")
        predicted = ad @ predicted + bd @ command
    return predicted


def hocbf_halfspace(
    x_pred: np.ndarray,
    obstacle: np.ndarray,
    safe_radius: float,
    tau: float,
    c1: float,
    c2: float,
) -> tuple[np.ndarray, float, float, float]:
    """Return a, b, h, psi1 for the hard constraint a @ u >= b."""
    x_pred = np.asarray(x_pred, dtype=float)
    obstacle = np.asarray(obstacle, dtype=float)
    if x_pred.shape != (4,) or obstacle.shape != (2,):
        raise ValueError("x_pred must have four entries and obstacle two")
    if safe_radius <= 0.0 or tau <= 0.0 or c1 <= 0.0 or c2 <= 0.0:
        raise ValueError("safe_radius, tau, c1, and c2 must be positive")

    position = x_pred[:2]
    velocity = x_pred[2:]
    radial = position - obstacle
    h = float(radial @ radial - safe_radius**2)
    psi1 = float(2.0 * radial @ velocity + c1 * h)
    a = 2.0 * radial / tau
    b = float(
        -2.0 * velocity @ velocity
        + 2.0 * radial @ velocity / tau
        - 2.0 * c1 * radial @ velocity
        - c2 * psi1
    )
    return a, b, h, psi1


def solve_hocbf_qp(
    u_nom: np.ndarray,
    u_prev: np.ndarray,
    halfspaces: list[tuple[np.ndarray, float]],
    vmax: float,
    amax: float,
    dt: float,
) -> SafetyFilterResult:
    """Solve the 2D hard HOCBF QP by enumerating the active constraint set."""
    u_nom = np.asarray(u_nom, dtype=float)
    u_prev = np.asarray(u_prev, dtype=float)
    if u_nom.shape != (2,) or u_prev.shape != (2,):
        raise ValueError("u_nom and u_prev must be two-dimensional")
    if vmax <= 0.0 or amax <= 0.0 or dt <= 0.0:
        raise ValueError("vmax, amax, and dt must be positive")

    lower = np.maximum(-vmax, u_prev - amax * dt)
    upper = np.minimum(vmax, u_prev + amax * dt)
    constraints = [
        (np.array([1.0, 0.0]), float(lower[0])),
        (np.array([-1.0, 0.0]), float(-upper[0])),
        (np.array([0.0, 1.0]), float(lower[1])),
        (np.array([0.0, -1.0]), float(-upper[1])),
    ]
    for a, b in halfspaces:
        a = np.asarray(a, dtype=float)
        if a.shape != (2,):
            raise ValueError("halfspace normals must be two-dimensional")
        constraints.append((a, float(b)))

    def is_feasible(candidate: np.ndarray) -> bool:
        return all(a @ candidate >= b - 1e-10 for a, b in constraints)

    candidates = []
    if is_feasible(u_nom):
        candidates.append(u_nom)

    for a, b in constraints:
        norm_squared = float(a @ a)
        if norm_squared <= 1e-15:
            continue
        candidate = u_nom + (b - a @ u_nom) / norm_squared * a
        if is_feasible(candidate):
            candidates.append(candidate)

    for i, (a_i, b_i) in enumerate(constraints):
        for a_j, b_j in constraints[i + 1 :]:
            matrix = np.vstack((a_i, a_j))
            if abs(np.linalg.det(matrix)) <= 1e-12:
                continue
            candidate = np.linalg.solve(matrix, np.array([b_i, b_j]))
            if is_feasible(candidate):
                candidates.append(candidate)

    if not candidates:
        return SafetyFilterResult(np.zeros(2), False, 0)

    command = min(candidates, key=lambda value: float(np.sum((value - u_nom) ** 2)))
    active = sum(abs(a @ command - b) <= 1e-9 for a, b in constraints)
    return SafetyFilterResult(command, True, active)
