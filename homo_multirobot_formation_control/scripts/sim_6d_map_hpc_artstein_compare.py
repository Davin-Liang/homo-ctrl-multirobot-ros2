#!/usr/bin/env python3
"""Map-frame 6D regularized-HPC and Artstein comparison utilities."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import expm, solve_continuous_lyapunov


def wrap_angle(angle: float) -> float:
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


def rot(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def build_nominal_model(
    mass: float, inertia: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if mass <= 0.0 or inertia <= 0.0:
        raise ValueError("mass and inertia must be positive")
    a = np.zeros((6, 6))
    a[0, 3] = a[1, 4] = a[2, 5] = 1.0
    d_inv = np.diag([1.0 / mass, 1.0 / mass, 1.0 / inertia])
    b = np.vstack([np.zeros((3, 3)), d_inv])
    g0 = np.diag([-1.0, -1.0, -1.0, 0.0, 0.0, 0.0])
    return a, b, g0, np.diag([mass, mass, inertia])


def map_error(
    leader: np.ndarray,
    follower: np.ndarray,
    offset_map: np.ndarray,
    dtheta: float = 0.0,
) -> np.ndarray:
    return np.r_[
        follower[:2] - leader[:2] - offset_map,
        wrap_angle(follower[2] - leader[2] - dtheta),
        rot(follower[2]) @ follower[3:5] - rot(leader[2]) @ leader[3:5],
        follower[5] - leader[5],
    ]


def hnorm(error: np.ndarray, gd: np.ndarray, p: np.ndarray, nmax: int = 40) -> float:
    if np.linalg.norm(error) < 1e-14:
        return 0.0
    lower, upper = -1.0, 1.0
    while float((expm(-gd * lower) @ error).T @ p @ (expm(-gd * lower) @ error)) < 1.0:
        lower *= 2.0
    while float((expm(-gd * upper) @ error).T @ p @ (expm(-gd * upper) @ error)) > 1.0:
        upper *= 2.0
    for _ in range(nmax):
        midpoint = 0.5 * (lower + upper)
        scaled = expm(-gd * midpoint) @ error
        if float(scaled.T @ p @ scaled) > 1.0:
            lower = midpoint
        else:
            upper = midpoint
    return float(np.exp(0.5 * (lower + upper)))


class RegularizedMapHpc:
    """Existing project's regularized engineering HPC, in map-frame error coordinates."""

    def __init__(self, mass: float, inertia: float, mu: float, kp: float, kv: float,
                 c_min: float):
        if not 0.0 < c_min <= 1.0:
            raise ValueError("c_min must be in (0, 1]")
        self.a, self.b, g0, scale = build_nominal_model(mass, inertia)
        self.mu = mu
        self.gd = np.eye(6) + mu * g0
        self.k = -scale @ np.hstack([kp * np.eye(3), kv * np.eye(3)])
        self.p = solve_continuous_lyapunov((self.a + self.b @ self.k).T, -2.0 * np.eye(6))
        self.c_min = c_min

    def command(self, error: np.ndarray) -> np.ndarray:
        if np.linalg.norm(error) < 1e-14:
            return np.zeros(3)
        c = np.clip(hnorm(error, self.gd, self.p), self.c_min, 1.0)
        return c ** (1.0 + self.mu) * self.k @ expm(
            self.gd * (1.0 - np.log(c))
        ) @ error


def map_to_body(theta: float, velocity_map: np.ndarray) -> np.ndarray:
    return rot(theta).T @ velocity_map


def actuator_matrices_4d(tau: float) -> tuple[np.ndarray, np.ndarray]:
    return np.array([
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, -1.0 / tau, 0.0],
        [0.0, 0.0, 0.0, -1.0 / tau],
    ]), np.array([
        [0.0, 0.0], [0.0, 0.0], [1.0 / tau, 0.0], [0.0, 1.0 / tau],
    ])


def actuator_matrices_2d(tau: float) -> tuple[np.ndarray, np.ndarray]:
    return np.array([[0.0, 1.0], [0.0, -1.0 / tau]]), np.array([[0.0], [1.0 / tau]])


def artstein_integral(history, a: np.ndarray, b: np.ndarray, td: float,
                      control_dt: float) -> np.ndarray:
    samples = max(1, int(np.ceil(td / control_dt)))
    integral = np.zeros(a.shape[0])
    for index in range(min(samples, len(history))):
        weight = control_dt if index < samples - 1 else td - (samples - 1) * control_dt
        integral += expm(a * (index * control_dt - td)) @ b @ np.atleast_1d(history[index]) * weight
    return integral


def predict_from_artstein(z: np.ndarray, current_command: np.ndarray, a: np.ndarray,
                          tau: float, td: float) -> np.ndarray:
    delay_free = expm(a * td) @ z
    decay = np.exp(-1.0)
    channels = delay_free.shape[0] // 2
    position = delay_free[:channels] + current_command * tau + tau * (1.0 - decay) * (
        delay_free[channels:] - current_command
    )
    velocity = current_command + decay * (delay_free[channels:] - current_command)
    return np.r_[position, velocity]


def predict_map_state(state: np.ndarray, history, td: float, tau: float,
                      control_dt: float) -> np.ndarray:
    """Direction-A Artstein integral plus one-tau forward prediction.

    The discrete integral is intentionally identical to the established
    ``sim_6d_disc_artstein_compare.py`` implementation; history[0] is newest.
    """
    if td == 0.0 and tau == 0.0:
        return state.copy()
    if td < 0.0 or tau < 0.0 or control_dt <= 0.0:
        raise ValueError("td/tau must be non-negative and control_dt positive")
    if tau == 0.0:
        raise ValueError("Artstein predictor requires positive tau when td is non-zero")
    velocity_map = rot(state[2]) @ state[3:5]
    current_command = np.asarray(history[0], dtype=float) if history else np.r_[velocity_map, state[5]]
    a4, b4 = actuator_matrices_4d(tau)
    z4 = np.r_[state[:2], velocity_map] + artstein_integral(
        deque(command[:2] for command in history), a4, b4, td, control_dt
    )
    predicted4 = predict_from_artstein(z4, current_command[:2], a4, tau, td)
    a2, b2 = actuator_matrices_2d(tau)
    z2 = np.array([state[2], state[5]]) + artstein_integral(
        deque(np.array([command[2]]) for command in history), a2, b2, td, control_dt
    )
    predicted2 = predict_from_artstein(z2, np.array([current_command[2]]), a2, tau, td)
    theta = wrap_angle(predicted2[0])
    return np.array([
        predicted4[0], predicted4[1], theta,
        *map_to_body(theta, predicted4[2:4]), predicted2[1],
    ])


def verify_nominal_identities(
    mass: float, inertia: float, mu: float, kp: float, kv: float
) -> dict[str, float]:
    if not -1.0 < mu < 0.0:
        raise ValueError("mu must be in (-1, 0)")
    if kp <= 0.0 or kv <= 0.0:
        raise ValueError("kp and kv must be positive")
    a, b, g0, scale = build_nominal_model(mass, inertia)
    gd = np.eye(6) + mu * g0
    k = -scale @ np.hstack([kp * np.eye(3), kv * np.eye(3)])
    acl = a + b @ k
    if np.max(np.real(np.linalg.eigvals(acl))) >= 0.0:
        raise ValueError("linear feedback is not Hurwitz")
    p = solve_continuous_lyapunov(acl.T, -2.0 * np.eye(6))
    if np.min(np.linalg.eigvalsh(p)) <= 0.0:
        raise ValueError("Lyapunov solution is not positive definite")
    if np.min(np.linalg.eigvalsh(p @ gd + gd.T @ p)) <= 0.0:
        raise ValueError("dilation compatibility is not positive definite")
    controllability = np.hstack([b, a @ b])
    return {
        "controllability_rank": int(np.linalg.matrix_rank(controllability, tol=1e-10)),
        "g0_b": float(np.linalg.norm(g0 @ b)),
        "ag0_commutator": float(np.linalg.norm(a @ g0 - g0 @ a - a)),
        "agd_commutator": float(np.linalg.norm(a @ gd - gd @ a - mu * a)),
        "gd_b": float(np.linalg.norm(gd @ b - b)),
    }


@dataclass(frozen=True)
class SimulationConfig:
    tmax: float = 60.0
    control_dt: float = 0.05
    plant_dt: float = 0.01
    td: float = 0.22
    tau: float = 0.43
    mass: float = 2.0
    inertia: float = 1.0
    mu: float = -0.25
    kp: float = 1.2
    kv: float = 2.0
    c_min: float = 0.5
    offset_map: tuple[float, float] = (-1.0, 0.0)
    leader_radius: float = 2.0
    leader_speed: float = 0.45
    yaw_step_time: float = 30.0
    yaw_step_angle: float = np.pi / 2.0
    output_dir: Path = Path(
        "homo_multirobot_formation_control/analysis/results/6d_map_hpc_artstein"
    )


@dataclass
class SimulationResult:
    name: str
    td: float
    initial_follower: np.ndarray
    time: np.ndarray
    leader: np.ndarray
    follower: np.ndarray
    command_map: np.ndarray
    error: np.ndarray


def circle_leader_state(t: float, radius: float, speed: float) -> np.ndarray:
    path_rate = speed / radius
    phase = path_rate * t
    theta = wrap_angle(phase + np.pi / 2.0)
    return np.array([
        radius * np.cos(phase), radius * np.sin(phase), theta,
        speed, 0.0, path_rate,
    ])


def yaw_step_leader_state(t: float, radius: float, speed: float,
                          step_time: float = 30.0,
                          step_angle: float = np.pi / 2.0) -> np.ndarray:
    """Circle leader with a yaw-reference step and continuous map translation."""
    state = circle_leader_state(t, radius, speed)
    if t >= step_time:
        velocity_map = rot(state[2]) @ state[3:5]
        state[2] = wrap_angle(state[2] + step_angle)
        state[3:5] = map_to_body(state[2], velocity_map)
    return state


def predict_leader_from_observation(leader: np.ndarray, time: float, horizon: float,
                                    radius: float, speed: float) -> np.ndarray:
    """Predict a planned circle without non-causally seeing a future yaw step."""
    nominal_now = circle_leader_state(time, radius, speed)
    future = circle_leader_state(time + horizon, radius, speed)
    yaw_offset = wrap_angle(leader[2] - nominal_now[2])
    velocity_map = rot(future[2]) @ future[3:5]
    future[2] = wrap_angle(future[2] + yaw_offset)
    future[3:5] = map_to_body(future[2], velocity_map)
    return future


def _state_with_map_velocity(state: np.ndarray, velocity_map: np.ndarray,
                             yaw_rate: float) -> np.ndarray:
    result = state.copy()
    result[3:5] = map_to_body(result[2], velocity_map)
    result[5] = yaw_rate
    return result


def step_delayed_plant(state: np.ndarray, command_map: np.ndarray, dt: float,
                       tau: float) -> np.ndarray:
    velocity_map = rot(state[2]) @ state[3:5]
    yaw_rate = state[5]
    next_velocity = velocity_map + dt * (command_map[:2] - velocity_map) / tau
    next_rate = yaw_rate + dt * (command_map[2] - yaw_rate) / tau
    result = state.copy()
    result[:2] += dt * next_velocity
    result[2] = wrap_angle(result[2] + dt * next_rate)
    return _state_with_map_velocity(result, next_velocity, next_rate)


def step_ideal_plant(state: np.ndarray, command_map: np.ndarray, dt: float) -> np.ndarray:
    result = state.copy()
    result[:2] += dt * command_map[:2]
    result[2] = wrap_angle(result[2] + dt * command_map[2])
    return _state_with_map_velocity(result, command_map[:2], command_map[2])


def _delay_line(initial: np.ndarray, delay: float, dt: float) -> deque[np.ndarray]:
    steps = delay / dt
    if not np.isclose(steps, round(steps), atol=1e-12):
        raise ValueError("td must be an integer multiple of plant_dt")
    return deque([initial.copy() for _ in range(int(round(steps)))])


def _advance_delay(line: deque[np.ndarray], command: np.ndarray) -> np.ndarray:
    if not line:
        return command.copy()
    delayed = line.popleft()
    line.append(command.copy())
    return delayed


def _command_from_force(state: np.ndarray, force_moment: np.ndarray,
                        config: SimulationConfig) -> np.ndarray:
    velocity_map = rot(state[2]) @ state[3:5]
    command = np.r_[
        velocity_map + config.control_dt * force_moment[:2] / config.mass,
        state[5] + config.control_dt * force_moment[2] / config.inertia,
    ]
    command[:2] = np.clip(command[:2], -1.0, 1.0)
    command[2] = np.clip(command[2], -0.8, 0.8)
    return command


def simulate_case(kind: str, config: SimulationConfig) -> SimulationResult:
    if kind not in {"ideal", "delayed", "artstein"}:
        raise ValueError(f"unsupported case: {kind}")
    substeps = config.control_dt / config.plant_dt
    if not np.isclose(substeps, round(substeps), atol=1e-12):
        raise ValueError("control_dt must be an integer multiple of plant_dt")
    substeps = int(round(substeps))
    controller = RegularizedMapHpc(
        config.mass, config.inertia, config.mu, config.kp, config.kv, config.c_min
    )
    x2 = np.array([3.8, -0.5, -0.5, 0.0, 0.0, 0.0])
    initial_follower = x2.copy()
    command = np.zeros(3)
    delay = _delay_line(command, 0.0 if kind == "ideal" else config.td, config.plant_dt)
    history: deque[np.ndarray] = deque(maxlen=max(2, int(np.ceil(config.td / config.control_dt)) + 2))
    history.appendleft(command.copy())
    rows = []
    t = 0.0
    while t < config.tmax - 1e-12:
        leader = yaw_step_leader_state(
            t, config.leader_radius, config.leader_speed,
            config.yaw_step_time, config.yaw_step_angle,
        )
        if kind == "artstein":
            horizon = config.td + config.tau
            leader_ctrl = predict_leader_from_observation(
                leader, t, horizon, config.leader_radius, config.leader_speed,
            )
            follower_ctrl = predict_map_state(x2, history, config.td, config.tau, config.control_dt)
        else:
            leader_ctrl, follower_ctrl = leader, x2
        error = map_error(leader_ctrl, follower_ctrl, np.asarray(config.offset_map))
        command = _command_from_force(follower_ctrl, controller.command(error), config)
        for _ in range(substeps):
            if kind == "ideal":
                x2 = step_ideal_plant(x2, command, config.plant_dt)
            else:
                x2 = step_delayed_plant(
                    x2, _advance_delay(delay, command), config.plant_dt, config.tau
                )
        history.appendleft(command.copy())
        sample_time = t + config.control_dt
        leader_at_sample = yaw_step_leader_state(
            sample_time, config.leader_radius, config.leader_speed,
            config.yaw_step_time, config.yaw_step_angle,
        )
        actual_error = map_error(leader_at_sample, x2, np.asarray(config.offset_map))
        rows.append((sample_time, leader_at_sample, x2.copy(), command.copy(), actual_error))
        t += config.control_dt
    return SimulationResult(
        kind, 0.0 if kind == "ideal" else config.td, initial_follower,
        np.array([row[0] for row in rows]), np.array([row[1] for row in rows]),
        np.array([row[2] for row in rows]), np.array([row[3] for row in rows]),
        np.array([row[4] for row in rows]),
    )


def _write_outputs(results: list[SimulationResult], config: SimulationConfig) -> list[Path]:
    config.output_dir.mkdir(parents=True, exist_ok=True)
    plot_path = config.output_dir / "comparison.png"
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for result in results:
        axes[0, 0].plot(result.follower[:, 0], result.follower[:, 1], label=result.name)
        axes[0, 1].plot(result.time, np.linalg.norm(result.error[:, :2], axis=1), label=result.name)
        axes[1, 0].plot(result.time, np.abs(result.error[:, 2]), label=result.name)
        axes[1, 1].plot(result.time, result.command_map[:, 0], label=f"{result.name} vx")
    for axis in (axes[0, 1], axes[1, 0], axes[1, 1]):
        axis.axvline(config.yaw_step_time, color="0.35", linestyle="--", linewidth=1.0)
    axes[0, 0].plot(results[0].leader[:, 0], results[0].leader[:, 1], "k--", label="leader")
    for axis, title, ylabel in zip(
        axes.ravel(), ("trajectory", "position error", "yaw error", "map vx command"),
        ("y (m)", "m", "rad", "m/s"),
    ):
        axis.set(title=title, xlabel="t (s)" if axis is not axes[0, 0] else "x (m)", ylabel=ylabel)
        axis.grid(True); axis.legend(frameon=False)
    axes[0, 0].axis("equal")
    fig.savefig(plot_path, dpi=180); plt.close(fig)

    summary_path = config.output_dir / "summary_metrics.csv"
    lines = ["case,max_position_error,tail_mean_position_error,final_position_error,tail_mean_yaw_error,final_yaw_error,post_step_peak_position_error,post_step_peak_yaw_error"]
    timeseries = ["case,t,leader_x,leader_y,follower_x,follower_y,follower_yaw,cmd_vx_map,cmd_vy_map,cmd_w,ex,ey,etheta,evx,evy,eomega"]
    for result in results:
        position = np.linalg.norm(result.error[:, :2], axis=1); yaw = np.abs(result.error[:, 2])
        tail = slice(int(.7 * len(position)), None)
        post_step = result.time >= config.yaw_step_time
        post_position = position[post_step]
        post_yaw = yaw[post_step]
        post_peak_position = post_position.max() if post_position.size else float("nan")
        post_peak_yaw = post_yaw.max() if post_yaw.size else float("nan")
        lines.append(f"{result.name},{position.max():.6f},{position[tail].mean():.6f},{position[-1]:.6f},{yaw[tail].mean():.6f},{yaw[-1]:.6f},{post_peak_position:.6f},{post_peak_yaw:.6f}")
        for index, time in enumerate(result.time):
            row = np.r_[result.leader[index, :2], result.follower[index, :3], result.command_map[index], result.error[index]]
            timeseries.append(result.name + "," + f"{time:.6f}," + ",".join(f"{value:.6f}" for value in row))
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    timeseries_path = config.output_dir / "timeseries.csv"
    timeseries_path.write_text("\n".join(timeseries) + "\n", encoding="utf-8")
    diagnostics_path = config.output_dir / "diagnostics.txt"
    diagnostics = verify_nominal_identities(config.mass, config.inertia, config.mu, config.kp, config.kv)
    diagnostics_path.write_text("\n".join(f"{key}={value}" for key, value in diagnostics.items()) + "\n", encoding="utf-8")
    return [plot_path, summary_path, timeseries_path, diagnostics_path]


def run_experiment(config: SimulationConfig) -> list[Path]:
    return _write_outputs([simulate_case(name, config) for name in ("ideal", "delayed", "artstein")], config)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, default=SimulationConfig.output_dir)
    parser.add_argument("--tmax", type=float, default=SimulationConfig.tmax)
    args = parser.parse_args()
    for path in run_experiment(SimulationConfig(tmax=args.tmax, output_dir=args.out_dir)):
        print(path)


if __name__ == "__main__":
    main()
