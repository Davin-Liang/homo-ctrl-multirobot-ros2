#!/usr/bin/env python3
"""Oracle numerical utilities for predictor-state HOCBF feasibility studies."""

import argparse
import csv
import itertools
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
from scipy.linalg import expm


@dataclass(frozen=True)
class PlantParams:
    """Time scales for the planar delayed actuator model."""

    tau: float
    delay: float
    integration_dt: float
    control_dt: float

    def __post_init__(self):
        if (
            self.tau <= 0.0
            or self.integration_dt <= 0.0
            or self.control_dt <= 0.0
            or self.delay < 0.0
        ):
            raise ValueError(
                "tau and time steps must be positive; delay must be non-negative"
            )
        delay_steps = self.delay / self.integration_dt
        if not np.isclose(delay_steps, round(delay_steps), atol=1e-12):
            raise ValueError("delay must be an integer multiple of integration_dt")
        control_steps = self.control_dt / self.integration_dt
        if not np.isclose(control_steps, round(control_steps), atol=1e-12):
            raise ValueError("control_dt must be an integer multiple of integration_dt")

    @property
    def delay_steps(self) -> int:
        return int(round(self.delay / self.integration_dt))

    @property
    def control_steps(self) -> int:
        return int(round(self.control_dt / self.integration_dt))


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
    transition = expm(augmented * params.integration_dt)
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
        integration_dt=config.plant.integration_dt,
        control_dt=config.plant.control_dt,
    )
    predictor_ad, predictor_bd = zoh_matrices(predictor_params)
    steps = config.duration / config.plant.control_dt
    if not np.isclose(steps, round(steps), atol=1e-12) or steps <= 0:
        raise ValueError("duration must be an integer number of control samples")
    control_samples = int(round(steps))

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
    internal_times = [0.0]
    internal_states = [state.copy()]

    for index in range(control_samples):
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
            config.plant.control_dt,
        )
        braking = not result.feasible
        command = (
            _braking_command(
                previous_command, config.amax, config.plant.control_dt
            )
            if braking
            else result.command
        )

        actual_h = float(
            np.sum((state[:2] - obstacle) ** 2) - config.safe_radius**2
        )
        times.append(index * config.plant.control_dt)
        states.append(state.copy())
        commands.append(command.copy())
        h_values.append(actual_h)
        psi1_values.append(psi1)
        psi2_values.append(float(a @ command - b))
        feasible_values.append(result.feasible)
        braking_values.append(braking)

        for substep in range(config.plant.control_steps):
            if queue:
                applied = queue.pop(0)
                queue.append(command.copy())
            else:
                applied = command
            state = actual_ad @ state + actual_bd @ applied
            internal_times.append(
                index * config.plant.control_dt
                + (substep + 1) * config.plant.integration_dt
            )
            internal_states.append(state.copy())
        previous_command = command

    internal_state_array = np.asarray(internal_states)
    internal_h = (
        np.sum((internal_state_array[:, :2] - obstacle) ** 2, axis=1)
        - config.safe_radius**2
    )
    return {
        "time": np.asarray(times),
        "state": np.asarray(states),
        "command": np.asarray(commands),
        "h": np.asarray(h_values),
        "psi1": np.asarray(psi1_values),
        "psi2": np.asarray(psi2_values),
        "feasible": np.asarray(feasible_values, dtype=bool),
        "braking": np.asarray(braking_values, dtype=bool),
        "time_internal": np.asarray(internal_times),
        "state_internal": internal_state_array,
        "h_internal": internal_h,
    }


METRICS_FIELDNAMES = [
    "tau",
    "delay_model",
    "delay_actual",
    "initial_clearance",
    "min_h",
    "min_distance",
    "min_psi2",
    "max_command_norm",
    "infeasible_steps",
    "braking_steps",
]


def scan_envelope(
    tau_values: list[float],
    delay_values: list[float],
    clearances: list[float],
    delay_mismatches: list[float],
) -> list[dict[str, float | int]]:
    """Run the head-on scenario over model, plant, and initial-condition grids."""
    rows = []
    for tau, delay_model, clearance, delay_mismatch in itertools.product(
        tau_values, delay_values, clearances, delay_mismatches
    ):
        delay_actual = delay_model + delay_mismatch
        config = ScenarioConfig(
            plant=PlantParams(
                tau=tau,
                delay=delay_actual,
                integration_dt=0.01,
                control_dt=0.05,
            ),
            predictor_delay=delay_model,
            obstacle=np.zeros(2),
            safe_radius=0.8,
            initial_state=np.array([0.8 + clearance, 0.0, -0.1, 0.0]),
            nominal_command=np.array([-0.8, 0.0]),
            vmax=1.0,
            amax=20.0,
            c1=2.0,
            c2=2.0,
            duration=4.0,
        )
        result = simulate_scenario(config)
        distances = np.linalg.norm(
            result["state_internal"][:, :2] - config.obstacle, axis=1
        )
        rows.append(
            {
                "tau": tau,
                "delay_model": delay_model,
                "delay_actual": delay_actual,
                "initial_clearance": clearance,
                "min_h": float(result["h_internal"].min()),
                "min_distance": float(distances.min()),
                "min_psi2": float(result["psi2"].min()),
                "max_command_norm": float(
                    np.linalg.norm(result["command"], axis=1).max()
                ),
                "infeasible_steps": int((~result["feasible"]).sum()),
                "braking_steps": int(result["braking"].sum()),
            }
        )
    return rows


def write_metrics_csv(rows: list[dict[str, float | int]], output: Path) -> None:
    """Write scan metrics with a stable column order and LF line endings."""
    if not rows:
        raise ValueError("rows must not be empty")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=METRICS_FIELDNAMES, lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)


def compare_sampling_rates(
    config: ScenarioConfig, reference_dt: float = 0.001
) -> dict[str, float]:
    """Compare the configured controller rate against a high-rate reference."""
    if reference_dt <= 0.0 or reference_dt > config.plant.control_dt:
        raise ValueError("reference_dt must be positive and no larger than control_dt")

    low_rate = simulate_scenario(config)
    reference_plant = replace(
        config.plant,
        integration_dt=reference_dt,
        control_dt=reference_dt,
    )
    reference = simulate_scenario(replace(config, plant=reference_plant))
    low_distance = np.linalg.norm(
        low_rate["state_internal"][:, :2] - config.obstacle, axis=1
    ).min()
    reference_distance = np.linalg.norm(
        reference["state_internal"][:, :2] - config.obstacle, axis=1
    ).min()
    min_h_20hz = float(low_rate["h_internal"].min())
    min_h_1khz = float(reference["h_internal"].min())
    return {
        "control_dt": config.plant.control_dt,
        "reference_dt": reference_dt,
        "min_h_20hz": min_h_20hz,
        "min_h_1khz": min_h_1khz,
        "min_distance_20hz": float(low_distance),
        "min_distance_1khz": float(reference_distance),
        "min_h_difference": min_h_20hz - min_h_1khz,
    }


def write_rate_comparison_csv(row: dict[str, float], output: Path) -> None:
    """Write one sampling-rate comparison row with LF line endings."""
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=list(row), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerow(row)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Scan static-obstacle predictor-state HOCBF feasibility."
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(
            "homo_multirobot_formation_control/analysis/results/"
            "6d_artstein_hocbf_feasibility/scan.csv"
        ),
    )
    args = parser.parse_args()
    rows = scan_envelope(
        tau_values=[0.30, 0.43, 0.55],
        delay_values=[0.0, 0.22],
        clearances=[0.4, 0.8, 1.2],
        delay_mismatches=[0.0, 0.02, 0.05],
    )
    write_metrics_csv(rows, args.output)
    comparison = compare_sampling_rates(
        ScenarioConfig(
            plant=PlantParams(
                tau=0.43,
                delay=0.22,
                integration_dt=0.01,
                control_dt=0.05,
            ),
            obstacle=np.zeros(2),
            safe_radius=0.8,
            initial_state=np.array([1.6, 0.0, -0.1, 0.0]),
            nominal_command=np.array([-0.8, 0.0]),
            vmax=1.0,
            amax=20.0,
            c1=2.0,
            c2=2.0,
            duration=4.0,
        )
    )
    comparison_output = args.output.with_name("sampling_rate_compare.csv")
    write_rate_comparison_csv(comparison, comparison_output)
    print(
        f"wrote {len(rows)} rows to {args.output} and "
        f"sampling comparison to {comparison_output}"
    )


if __name__ == "__main__":
    main()
