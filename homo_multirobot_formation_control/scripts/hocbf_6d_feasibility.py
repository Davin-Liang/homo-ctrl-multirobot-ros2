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


@dataclass(frozen=True)
class ScenarioConfig:
    """One oracle scenario with a static circular obstacle."""

    plant: PlantParams
    obstacle: np.ndarray
    safe_radius: float
    initial_state: np.ndarray
    nominal_command: np.ndarray
    vmax: float
    amax: float
    c1: float
    c2: float
    duration: float
    predictor_delay: float | None = None

    def __post_init__(self):
        if self.safe_radius <= 0.0 or self.vmax <= 0.0 or self.amax <= 0.0:
            raise ValueError("safe_radius, vmax, and amax must be positive")
        if self.c1 <= 0.0 or self.c2 <= 0.0 or self.duration <= 0.0:
            raise ValueError("c1, c2, and duration must be positive")
        if np.asarray(self.obstacle).shape != (2,):
            raise ValueError("obstacle must be two-dimensional")
        if np.asarray(self.initial_state).shape != (4,):
            raise ValueError("initial_state must contain [px, py, vx, vy]")
        if np.asarray(self.nominal_command).shape != (2,):
            raise ValueError("nominal_command must be two-dimensional")
        if self.predictor_delay is not None:
            if self.predictor_delay < 0.0:
                raise ValueError("predictor_delay must be non-negative")
            if self.predictor_delay > self.plant.delay + 1e-12:
                raise ValueError("predictor_delay must not exceed plant delay")


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


def _braking_command(u_prev: np.ndarray, amax: float, dt: float) -> np.ndarray:
    delta = amax * dt
    return np.sign(u_prev) * np.maximum(np.abs(u_prev) - delta, 0.0)


def simulate_scenario(config: ScenarioConfig) -> dict[str, np.ndarray]:
    """Simulate the delayed plant with predictor-state HOCBF filtering."""
    actual_ad, actual_bd = zoh_matrices(config.plant)
    predictor_delay = (
        config.plant.delay
        if config.predictor_delay is None
        else config.predictor_delay
    )
    predictor_params = PlantParams(
        tau=config.plant.tau,
        delay=predictor_delay,
        dt=config.plant.dt,
    )
    predictor_ad, predictor_bd = zoh_matrices(predictor_params)
    steps = int(round(config.duration / config.plant.dt))
    if steps <= 0:
        raise ValueError("duration must include at least one sample")

    state = np.asarray(config.initial_state, dtype=float).copy()
    obstacle = np.asarray(config.obstacle, dtype=float)
    nominal_command = np.asarray(config.nominal_command, dtype=float)
    queue = [np.zeros(2) for _ in range(config.plant.delay_steps)]
    previous_command = np.zeros(2)

    times = []
    states = []
    commands = []
    h_values = []
    psi1_values = []
    psi2_values = []
    feasible_values = []
    braking_values = []

    for index in range(steps):
        prediction_queue = queue[: predictor_params.delay_steps]
        predicted = predict_delayed_state(
            state, prediction_queue, predictor_ad, predictor_bd
        )
        a, b, _, psi1 = hocbf_halfspace(
            predicted,
            obstacle,
            config.safe_radius,
            config.plant.tau,
            config.c1,
            config.c2,
        )
        result = solve_hocbf_qp(
            nominal_command,
            previous_command,
            [(a, b)],
            config.vmax,
            config.amax,
            config.plant.dt,
        )
        braking = not result.feasible
        command = (
            _braking_command(previous_command, config.amax, config.plant.dt)
            if braking
            else result.command
        )

        actual_h = float(
            np.sum((state[:2] - obstacle) ** 2) - config.safe_radius**2
        )
        times.append(index * config.plant.dt)
        states.append(state.copy())
        commands.append(command.copy())
        h_values.append(actual_h)
        psi1_values.append(psi1)
        psi2_values.append(float(a @ command - b))
        feasible_values.append(result.feasible)
        braking_values.append(braking)

        if queue:
            applied = queue.pop(0)
            queue.append(command.copy())
        else:
            applied = command
        state = actual_ad @ state + actual_bd @ applied
        previous_command = command

    return {
        "time": np.asarray(times),
        "state": np.asarray(states),
        "command": np.asarray(commands),
        "h": np.asarray(h_values),
        "psi1": np.asarray(psi1_values),
        "psi2": np.asarray(psi2_values),
        "feasible": np.asarray(feasible_values, dtype=bool),
        "braking": np.asarray(braking_values, dtype=bool),
    }
