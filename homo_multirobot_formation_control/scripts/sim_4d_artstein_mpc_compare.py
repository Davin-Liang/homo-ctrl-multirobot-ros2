#!/usr/bin/env python3
"""
Numerical comparison for 4D Artstein-HPC and 4D Artstein-MPC.

The MPC controller intentionally replaces only the upper translational law.
Leader prediction, follower Artstein / motor prediction, discrete formation
target switching, and final velocity post-processing follow the existing 4D
Artstein-HPC numerical experiment as closely as possible.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from sim_4d_hpc_artstein_compare import (  # noqa: E402
    Hpc4D,
    add_measurement_noise,
    artstein_integral,
    circle_leader_state,
    predict_follower_state_from_artstein,
    predict_leader_state,
)


def double_integrator_zoh(mass: float, dt: float) -> tuple[np.ndarray, np.ndarray]:
    Ad = np.array(
        [
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    Bd = np.array(
        [
            [0.5 * dt * dt / mass, 0.0],
            [0.0, 0.5 * dt * dt / mass],
            [dt / mass, 0.0],
            [0.0, dt / mass],
        ]
    )
    return Ad, Bd


@dataclass
class MpcSolution:
    x_pred: np.ndarray
    u_pred: np.ndarray
    status: str
    iterations: int
    solve_ms: float


@dataclass
class SimRow:
    t: float
    leader: np.ndarray
    follower: np.ndarray
    leader_ctrl: np.ndarray
    follower_ctrl: np.ndarray
    cmd_raw: np.ndarray
    cmd: np.ndarray
    selected_error: np.ndarray
    distance: float
    target_idx: int
    solve_ms: float
    solve_iters: int
    solver_status: str
    speed_clipped: bool
    accel_clipped: bool


class Mpc4D:
    """Condensed linear MPC for the 4D double integrator.

    The QP is solved with a compact ADMM loop so the simulation can run on a
    plain SciPy-free/OSQP-free ROS workstation. It mirrors the OSQP structure
    planned for C++: quadratic tracking cost, force-like input box constraints,
    and predicted map-frame velocity box constraints.
    """

    def __init__(
        self,
        mass: float = 2.0,
        dt: float = 0.05,
        horizon: int = 40,
        radius: float = 2.0,
        m_p: int = 4,
        tol: float = 0.1,
        q: tuple[float, float, float, float] = (10.0, 10.0, 1.0, 1.0),
        r: tuple[float, float] = (0.05, 0.05),
        terminal_factor: float = 10.0,
        max_speed: float = 0.5,
        max_accel: float = 0.4,
        max_iter: int = 120,
        eps_abs: float = 1e-4,
        eps_rel: float = 1e-3,
        rho: float = 1.0,
    ):
        self.mass = mass
        self.dt = dt
        self.N = horizon
        self.radius = radius
        self.m_p = m_p
        self.tol = tol
        self.max_speed = max_speed
        self.max_force = mass * max_accel
        self.max_iter = max_iter
        self.eps_abs = eps_abs
        self.eps_rel = eps_rel
        self.rho = rho

        self.Ad, self.Bd = double_integrator_zoh(mass, dt)
        self.Sx, self.Su = self._build_prediction_matrices()

        q_stage = np.diag(q)
        q_terminal = terminal_factor * q_stage
        self.Qbar = np.zeros((4 * (self.N + 1), 4 * (self.N + 1)))
        for k in range(self.N):
            self.Qbar[4 * k : 4 * (k + 1), 4 * k : 4 * (k + 1)] = q_stage
        self.Qbar[4 * self.N : 4 * (self.N + 1), 4 * self.N : 4 * (self.N + 1)] = q_terminal
        self.Rbar = np.kron(np.eye(self.N), np.diag(r))

        self.H = 2.0 * (self.Su.T @ self.Qbar @ self.Su + self.Rbar)
        self.H += 1e-9 * np.eye(2 * self.N)

        self.vel_selector = np.zeros((2 * self.N, 4 * (self.N + 1)))
        for k in range(1, self.N + 1):
            self.vel_selector[2 * (k - 1), 4 * k + 2] = 1.0
            self.vel_selector[2 * (k - 1) + 1, 4 * k + 3] = 1.0

        self.A_cons = np.vstack([np.eye(2 * self.N), self.vel_selector @ self.Su])
        self.K_admm = self.H + rho * (self.A_cons.T @ self.A_cons)
        self.K_chol = np.linalg.cholesky(self.K_admm)

        self.dl = np.column_stack(
            [
                np.array(
                    [
                        -radius * np.cos(2.0 * np.pi * i / m_p),
                        -radius * np.sin(2.0 * np.pi * i / m_p),
                        0.0,
                        0.0,
                    ]
                )
                for i in range(m_p)
            ]
        )
        self.target_idx = 0
        self.d = self.dl[:, 0].copy()
        self.last_u_sequence = np.zeros(2 * self.N)
        self.last_solution = MpcSolution(
            x_pred=np.zeros((self.N + 1, 4)),
            u_pred=np.zeros((self.N, 2)),
            status="not_run",
            iterations=0,
            solve_ms=0.0,
        )

    def init(self, x1: np.ndarray, x2: np.ndarray):
        self.target_idx = self._best_target_idx(x1, x2)
        self.d = self.dl[:, self.target_idx].copy()

    def command(self, x1: np.ndarray, x2: np.ndarray) -> np.ndarray:
        self._switch_if_needed(x1, x2)
        refs = self._reference_trajectory(x1)
        lower, upper = self._constraint_bounds(x2)
        f = 2.0 * self.Su.T @ self.Qbar @ (self.Sx @ x2 - refs.reshape(-1))

        started = time.perf_counter()
        u_flat, status, iterations = self._solve_admm(f, lower, upper)
        solve_ms = (time.perf_counter() - started) * 1000.0

        x_pred = (self.Sx @ x2 + self.Su @ u_flat).reshape(self.N + 1, 4)
        u_pred = u_flat.reshape(self.N, 2)
        self.last_solution = MpcSolution(x_pred, u_pred, status, iterations, solve_ms)
        self.last_u_sequence = np.r_[u_flat[2:], u_flat[-2:]]
        return x_pred[1, 2:4].copy()

    def distance(self, x1: np.ndarray, x2: np.ndarray) -> float:
        return float(min(np.linalg.norm(x2 - x1 - self.dl[:, i]) for i in range(self.m_p)))

    def selected_error(self, x1: np.ndarray, x2: np.ndarray) -> np.ndarray:
        return x2 - x1 - self.d

    def _build_prediction_matrices(self) -> tuple[np.ndarray, np.ndarray]:
        Sx = np.zeros((4 * (self.N + 1), 4))
        Su = np.zeros((4 * (self.N + 1), 2 * self.N))
        powers = [np.eye(4)]
        for _ in range(self.N):
            powers.append(powers[-1] @ self.Ad)
        for k in range(self.N + 1):
            Sx[4 * k : 4 * (k + 1), :] = powers[k]
            for j in range(k):
                Su[4 * k : 4 * (k + 1), 2 * j : 2 * (j + 1)] = powers[k - 1 - j] @ self.Bd
        return Sx, Su

    def _reference_trajectory(self, x1: np.ndarray) -> np.ndarray:
        refs = np.zeros((self.N + 1, 4))
        for k in range(self.N + 1):
            ref = x1.copy()
            ref[0:2] += k * self.dt * x1[2:4]
            refs[k, :] = ref + self.d
        return refs

    def _constraint_bounds(self, x0: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        input_lower = -self.max_force * np.ones(2 * self.N)
        input_upper = self.max_force * np.ones(2 * self.N)

        vel_offset = self.vel_selector @ (self.Sx @ x0)
        vel_lower = -self.max_speed * np.ones(2 * self.N) - vel_offset
        vel_upper = self.max_speed * np.ones(2 * self.N) - vel_offset
        return np.r_[input_lower, vel_lower], np.r_[input_upper, vel_upper]

    def _solve_admm(self, f: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> tuple[np.ndarray, str, int]:
        u = self.last_u_sequence.copy()
        z = np.clip(self.A_cons @ u, lower, upper)
        y = np.zeros_like(z)
        status = "max_iter"

        for iteration in range(1, self.max_iter + 1):
            rhs = -f + self.rho * self.A_cons.T @ (z - y)
            u = self._chol_solve(rhs)
            au = self.A_cons @ u
            z_prev = z
            z = np.clip(au + y, lower, upper)
            y = y + au - z

            primal = np.linalg.norm(au - z)
            dual = self.rho * np.linalg.norm(self.A_cons.T @ (z - z_prev))
            eps_primal = self.eps_abs * np.sqrt(z.size) + self.eps_rel * max(np.linalg.norm(au), np.linalg.norm(z))
            eps_dual = self.eps_abs * np.sqrt(u.size) + self.eps_rel * np.linalg.norm(self.A_cons.T @ y)
            if primal <= eps_primal and dual <= eps_dual:
                status = "solved"
                return u, status, iteration
        return u, status, self.max_iter

    def _chol_solve(self, rhs: np.ndarray) -> np.ndarray:
        y = np.linalg.solve(self.K_chol, rhs)
        return np.linalg.solve(self.K_chol.T, y)

    def _best_target_idx(self, x1: np.ndarray, x2: np.ndarray) -> int:
        distances = [np.linalg.norm(x2 - x1 - self.dl[:, i]) for i in range(self.m_p)]
        return int(np.argmin(distances))

    def _switch_if_needed(self, x1: np.ndarray, x2: np.ndarray):
        best_idx = self._best_target_idx(x1, x2)
        current_dist = np.linalg.norm(x2 - x1 - self.d)
        best_dist = np.linalg.norm(x2 - x1 - self.dl[:, best_idx])
        if best_dist + self.tol < current_dist:
            self.target_idx = best_idx
            self.d = self.dl[:, best_idx].copy()


def target_idx_from_hpc(ctrl: Hpc4D) -> int:
    distances = [np.linalg.norm(ctrl.d - ctrl.dl[:, i]) for i in range(ctrl.m_p)]
    return int(np.argmin(distances))


def selected_error(ctrl: Hpc4D | Mpc4D, x1: np.ndarray, x2: np.ndarray) -> np.ndarray:
    if isinstance(ctrl, Mpc4D):
        return ctrl.selected_error(x1, x2)
    return x2 - x1 - ctrl.d


def clip_norm(v: np.ndarray, max_norm: float) -> tuple[np.ndarray, bool]:
    norm = float(np.linalg.norm(v))
    if max_norm > 0.0 and norm > max_norm:
        return v * (max_norm / norm), True
    return v, False


def constrain_velocity_command(
    raw_cmd: np.ndarray,
    prev_cmd: np.ndarray,
    dt: float,
    max_speed: float,
    max_accel: float,
) -> tuple[np.ndarray, bool, bool]:
    cmd, speed_clipped_1 = clip_norm(raw_cmd, max_speed)
    delta, accel_clipped = clip_norm(cmd - prev_cmd, max_accel * dt)
    cmd = prev_cmd + delta
    cmd, speed_clipped_2 = clip_norm(cmd, max_speed)
    return cmd, speed_clipped_1 or speed_clipped_2, accel_clipped


def make_controller(controller_kind: str, dt: float, args) -> Hpc4D | Mpc4D:
    if controller_kind == "hpc":
        return Hpc4D(
            mass=args.mass,
            radius=args.radius,
            m_p=args.m_p,
            tol=args.tol,
            c_min=args.hpc_c_min,
            initial_min_lambda=args.initial_min_lambda,
            switch_min_lambda=args.switch_min_lambda,
        )
    if controller_kind == "mpc":
        return Mpc4D(
            mass=args.mass,
            dt=dt,
            horizon=args.mpc_horizon,
            radius=args.radius,
            m_p=args.m_p,
            tol=args.tol,
            q=(args.q_px, args.q_py, args.q_vx, args.q_vy),
            r=(args.r_ux, args.r_uy),
            terminal_factor=args.terminal_factor,
            max_speed=args.max_linear_vel,
            max_accel=args.max_linear_accel,
            max_iter=args.mpc_max_iter,
            eps_abs=args.mpc_eps_abs,
            eps_rel=args.mpc_eps_rel,
            rho=args.mpc_rho,
        )
    raise ValueError(f"unknown controller kind: {controller_kind}")


def raw_velocity_command(ctrl: Hpc4D | Mpc4D, x1_ctrl: np.ndarray, x2_ctrl: np.ndarray, dt: float, mass: float):
    if isinstance(ctrl, Mpc4D):
        return ctrl.command(x1_ctrl, x2_ctrl)
    accel = ctrl.accel(x1_ctrl, x2_ctrl)
    return x2_ctrl[2:4] + dt * (accel / mass)


def simulate_circle_case(
    controller_kind: str,
    compensation_kind: str,
    tmax: float,
    dt: float,
    tau: float,
    Td: float,
    max_speed: float = 0.5,
    max_accel: float = 0.4,
    pos_noise: float = 0.0,
    vel_noise: float = 0.0,
    seed: int = 7,
    args=None,
) -> list[SimRow]:
    if args is None:
        args = argparse.Namespace(
            mass=2.0,
            radius=2.0,
            m_p=4,
            tol=0.1,
            hpc_c_min=0.1,
            initial_min_lambda=1.5,
            switch_min_lambda=4.0,
            mpc_horizon=30,
            q_px=40.0,
            q_py=40.0,
            q_vx=1.0,
            q_vy=1.0,
            r_ux=0.02,
            r_uy=0.02,
            terminal_factor=10.0,
            max_linear_vel=max_speed,
            max_linear_accel=max_accel,
            mpc_max_iter=320,
            mpc_eps_abs=1e-4,
            mpc_eps_rel=1e-3,
            mpc_rho=1.0,
            leader_radius=2.0,
            leader_omega=0.1,
        )

    rng = np.random.default_rng(seed)
    ctrl = make_controller(controller_kind, dt, args)
    x1 = circle_leader_state(0.0, radius=args.leader_radius, omega=args.leader_omega)
    x2 = np.array([4.5, 0.0, 0.0, 0.0])

    use_compensation = compensation_kind == "artstein"
    use_delay_plant = compensation_kind in ("artstein", "none_with_delay")
    delay_steps = max(1, int(np.ceil(Td / dt)))
    delay_line = deque([x2[2:4].copy() for _ in range(delay_steps + 1)], maxlen=delay_steps + 1)
    hist_len = max(1, int(np.ceil(Td / dt))) + 2
    cmd_history = deque([x2[2:4].copy() for _ in range(hist_len)], maxlen=hist_len)
    prev_cmd = x2[2:4].copy()

    x1_meas = add_measurement_noise(x1, pos_noise, vel_noise, rng)
    x2_meas = add_measurement_noise(x2, pos_noise, vel_noise, rng)
    if use_compensation:
        z2 = x2_meas + artstein_integral(cmd_history, tau, Td, dt)
        x2_ctrl = predict_follower_state_from_artstein(z2, prev_cmd, tau, Td)
        x1_ctrl = predict_leader_state(x1_meas, tau, Td)
    else:
        x1_ctrl = x1_meas
        x2_ctrl = x2_meas
    ctrl.init(x1_ctrl, x2_ctrl)

    rows: list[SimRow] = []
    t = 0.0
    while t < tmax - 1e-12:
        x1 = circle_leader_state(t, radius=args.leader_radius, omega=args.leader_omega)
        x1_meas = add_measurement_noise(x1, pos_noise, vel_noise, rng)
        x2_meas = add_measurement_noise(x2, pos_noise, vel_noise, rng)

        if use_compensation:
            z2 = x2_meas + artstein_integral(cmd_history, tau, Td, dt)
            x2_ctrl = predict_follower_state_from_artstein(z2, prev_cmd, tau, Td)
            x1_ctrl = predict_leader_state(x1_meas, tau, Td)
        else:
            x1_ctrl = x1_meas
            x2_ctrl = x2_meas

        cmd_raw = raw_velocity_command(ctrl, x1_ctrl, x2_ctrl, dt, args.mass)
        cmd, speed_clipped, accel_clipped = constrain_velocity_command(
            cmd_raw,
            prev_cmd,
            dt,
            args.max_linear_vel,
            args.max_linear_accel,
        )

        if use_delay_plant:
            delay_line.appendleft(cmd.copy())
            delayed_cmd = delay_line[-1]
            x2[2:4] += dt * ((delayed_cmd - x2[2:4]) / tau)
        else:
            x2[2:4] = cmd.copy()
        x2[0:2] += dt * x2[2:4]

        prev_cmd = cmd.copy()
        cmd_history.appendleft(cmd.copy())
        t += dt

        target_idx = ctrl.target_idx if isinstance(ctrl, Mpc4D) else target_idx_from_hpc(ctrl)
        solution = ctrl.last_solution if isinstance(ctrl, Mpc4D) else None
        rows.append(
            SimRow(
                t=t,
                leader=x1.copy(),
                follower=x2.copy(),
                leader_ctrl=x1_ctrl.copy(),
                follower_ctrl=x2_ctrl.copy(),
                cmd_raw=cmd_raw.copy(),
                cmd=cmd.copy(),
                selected_error=selected_error(ctrl, x1, x2).copy(),
                distance=ctrl.distance(x1, x2),
                target_idx=target_idx,
                solve_ms=solution.solve_ms if solution else 0.0,
                solve_iters=solution.iterations if solution else 0,
                solver_status=solution.status if solution else "analytic",
                speed_clipped=speed_clipped,
                accel_clipped=accel_clipped,
            )
        )
    return rows


def rows_to_arrays(rows: list[SimRow]) -> dict[str, np.ndarray]:
    return {
        "t": np.array([row.t for row in rows]),
        "leader": np.column_stack([row.leader for row in rows]),
        "follower": np.column_stack([row.follower for row in rows]),
        "cmd_raw": np.column_stack([row.cmd_raw for row in rows]),
        "cmd": np.column_stack([row.cmd for row in rows]),
        "err": np.column_stack([row.selected_error for row in rows]),
        "distance": np.array([row.distance for row in rows]),
        "target": np.array([row.target_idx for row in rows]),
        "solve_ms": np.array([row.solve_ms for row in rows]),
        "solve_iters": np.array([row.solve_iters for row in rows]),
    }


def metric_row(name: str, rows: list[SimRow]) -> dict[str, float | int | str]:
    dist = np.array([row.distance for row in rows])
    tail = dist[int(0.7 * len(dist)) :]
    cmd = np.column_stack([row.cmd for row in rows])
    cmd_diff = np.diff(cmd, axis=1) if cmd.shape[1] > 1 else np.zeros((2, 1))
    solve_ms = np.array([row.solve_ms for row in rows])
    statuses = [row.solver_status for row in rows]
    solver_failures = sum(1 for status in statuses if status not in ("solved", "analytic"))
    return {
        "case": name,
        "initial_distance": float(dist[0]),
        "max_distance": float(np.max(dist)),
        "tail_mean_distance": float(np.mean(tail)),
        "tail_rms_distance": float(np.sqrt(np.mean(tail * tail))),
        "tail_std_distance": float(np.std(tail)),
        "final_distance": float(dist[-1]),
        "max_cmd_speed": float(np.max(np.linalg.norm(cmd, axis=0))),
        "mean_cmd_delta": float(np.mean(np.linalg.norm(cmd_diff, axis=0))),
        "speed_clip_ratio": float(np.mean([row.speed_clipped for row in rows])),
        "accel_clip_ratio": float(np.mean([row.accel_clipped for row in rows])),
        "mean_solve_ms": float(np.mean(solve_ms)),
        "max_solve_ms": float(np.max(solve_ms)),
        "mean_solve_iters": float(np.mean([row.solve_iters for row in rows])),
        "solver_failures": int(solver_failures),
    }


def write_summary(out_path: Path, rows_by_name: dict[str, list[SimRow]]) -> Path:
    fieldnames = list(metric_row(next(iter(rows_by_name)), next(iter(rows_by_name.values()))).keys())
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for name, rows in rows_by_name.items():
            writer.writerow(metric_row(name, rows))
    return out_path


def write_timeseries(out_path: Path, rows_by_name: dict[str, list[SimRow]]) -> Path:
    fieldnames = [
        "case",
        "t",
        "leader_px",
        "leader_py",
        "follower_px",
        "follower_py",
        "cmd_vx",
        "cmd_vy",
        "err_px",
        "err_py",
        "err_vx",
        "err_vy",
        "distance",
        "target_idx",
        "solve_ms",
        "solve_iters",
        "solver_status",
    ]
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for name, rows in rows_by_name.items():
            for row in rows:
                writer.writerow(
                    {
                        "case": name,
                        "t": row.t,
                        "leader_px": row.leader[0],
                        "leader_py": row.leader[1],
                        "follower_px": row.follower[0],
                        "follower_py": row.follower[1],
                        "cmd_vx": row.cmd[0],
                        "cmd_vy": row.cmd[1],
                        "err_px": row.selected_error[0],
                        "err_py": row.selected_error[1],
                        "err_vx": row.selected_error[2],
                        "err_vy": row.selected_error[3],
                        "distance": row.distance,
                        "target_idx": row.target_idx,
                        "solve_ms": row.solve_ms,
                        "solve_iters": row.solve_iters,
                        "solver_status": row.solver_status,
                    }
                )
    return out_path


def plot_group(title: str, rows_by_name: dict[str, list[SimRow]], out_path: Path) -> Path:
    arrays = {name: rows_to_arrays(rows) for name, rows in rows_by_name.items()}
    fig, axs = plt.subplots(2, 3, figsize=(15, 8), constrained_layout=True)

    first = next(iter(arrays.values()))
    axs[0, 0].plot(first["leader"][0], first["leader"][1], "k--", label="leader")
    for name, arr in arrays.items():
        axs[0, 0].plot(arr["follower"][0], arr["follower"][1], label=name)
    axs[0, 0].axis("equal")
    axs[0, 0].set(xlabel="x (m)", ylabel="y (m)", title="trajectory")

    for name, arr in arrays.items():
        axs[0, 1].plot(arr["t"], arr["distance"], label=name)
    axs[0, 1].set(xlabel="t (s)", ylabel="nearest 4D error", title="formation distance")

    for name, arr in arrays.items():
        axs[0, 2].plot(arr["t"], arr["err"][0], label=f"{name} ex")
        axs[0, 2].plot(arr["t"], arr["err"][1], linestyle="--", label=f"{name} ey")
    axs[0, 2].set(xlabel="t (s)", ylabel="position error", title="selected target error")

    for name, arr in arrays.items():
        axs[1, 0].plot(arr["t"], arr["cmd"][0], label=f"{name} vx")
        axs[1, 0].plot(arr["t"], arr["cmd"][1], linestyle="--", label=f"{name} vy")
    axs[1, 0].set(xlabel="t (s)", ylabel="cmd velocity (m/s)", title="map velocity command")

    for name, arr in arrays.items():
        axs[1, 1].plot(arr["t"], np.linalg.norm(arr["cmd"], axis=0), label=name)
    axs[1, 1].set(xlabel="t (s)", ylabel="|cmd| (m/s)", title="command speed")

    for name, arr in arrays.items():
        if np.any(arr["solve_ms"] > 0.0):
            axs[1, 2].plot(arr["t"], arr["solve_ms"], label=name)
    axs[1, 2].set(xlabel="t (s)", ylabel="solve time (ms)", title="MPC solve time")

    for ax in axs.ravel():
        ax.grid(True)
        ax.legend(frameon=False, fontsize=8)
    fig.suptitle(title)
    fig.savefig(out_path, dpi=180)
    plt.close(fig)
    return out_path


def run_experiments(args) -> dict[str, list[SimRow]]:
    return {
        "artstein_hpc_no_delay": simulate_circle_case(
            "hpc", "none", args.circle_tmax, args.dt, args.tau, args.Td, args.max_linear_vel, args.max_linear_accel, args=args
        ),
        "artstein_mpc_no_delay": simulate_circle_case(
            "mpc", "none", args.circle_tmax, args.dt, args.tau, args.Td, args.max_linear_vel, args.max_linear_accel, args=args
        ),
        "original_4d_delay": simulate_circle_case(
            "hpc",
            "none_with_delay",
            args.circle_tmax,
            args.dt,
            args.tau,
            args.Td,
            args.max_linear_vel,
            args.max_linear_accel,
            args=args,
        ),
        "artstein_hpc_delay": simulate_circle_case(
            "hpc",
            "artstein",
            args.circle_tmax,
            args.dt,
            args.tau,
            args.Td,
            args.max_linear_vel,
            args.max_linear_accel,
            args=args,
        ),
        "artstein_mpc_delay": simulate_circle_case(
            "mpc",
            "artstein",
            args.circle_tmax,
            args.dt,
            args.tau,
            args.Td,
            args.max_linear_vel,
            args.max_linear_accel,
            args=args,
        ),
    }


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", default="homo_multirobot_formation_control/analysis/results/4d_artstein_mpc")
    parser.add_argument("--circle-tmax", type=float, default=45.0)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--tau", type=float, default=0.43)
    parser.add_argument("--Td", type=float, default=0.22)
    parser.add_argument("--mass", type=float, default=2.0)
    parser.add_argument("--radius", type=float, default=2.0)
    parser.add_argument("--m-p", dest="m_p", type=int, default=4)
    parser.add_argument("--tol", type=float, default=0.1)
    parser.add_argument("--leader-radius", type=float, default=2.0)
    parser.add_argument("--leader-omega", type=float, default=0.1)
    parser.add_argument("--max-linear-vel", type=float, default=0.5)
    parser.add_argument("--max-linear-accel", type=float, default=0.4)
    parser.add_argument("--hpc-c-min", type=float, default=0.1)
    parser.add_argument("--initial-min-lambda", type=float, default=1.5)
    parser.add_argument("--switch-min-lambda", type=float, default=4.0)
    parser.add_argument("--mpc-horizon", type=int, default=30)
    parser.add_argument("--q-px", type=float, default=40.0)
    parser.add_argument("--q-py", type=float, default=40.0)
    parser.add_argument("--q-vx", type=float, default=1.0)
    parser.add_argument("--q-vy", type=float, default=1.0)
    parser.add_argument("--r-ux", type=float, default=0.02)
    parser.add_argument("--r-uy", type=float, default=0.02)
    parser.add_argument("--terminal-factor", type=float, default=10.0)
    parser.add_argument("--mpc-max-iter", type=int, default=320)
    parser.add_argument("--mpc-eps-abs", type=float, default=1e-4)
    parser.add_argument("--mpc-eps-rel", type=float, default=1e-3)
    parser.add_argument("--mpc-rho", type=float, default=1.0)
    return parser.parse_args()


def main():
    args = parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    rows_by_name = run_experiments(args)
    outputs = [
        plot_group(
            "4D Artstein-HPC vs 4D Artstein-MPC, no delay",
            {
                "Artstein-HPC no delay": rows_by_name["artstein_hpc_no_delay"],
                "Artstein-MPC no delay": rows_by_name["artstein_mpc_no_delay"],
            },
            out_dir / "circle_no_delay_hpc_vs_mpc.png",
        ),
        plot_group(
            "4D delay baselines with shared Artstein prediction layer",
            {
                "original 4D + delay": rows_by_name["original_4d_delay"],
                "Artstein-HPC + delay": rows_by_name["artstein_hpc_delay"],
                "Artstein-MPC + delay": rows_by_name["artstein_mpc_delay"],
            },
            out_dir / "circle_delay_hpc_vs_mpc.png",
        ),
        plot_group("4D Artstein-MPC full comparison", rows_by_name, out_dir / "circle_all_compare.png"),
        write_summary(out_dir / "summary_metrics.csv", rows_by_name),
        write_timeseries(out_dir / "timeseries_circle_compare.csv", rows_by_name),
    ]
    for path in outputs:
        print(path)


if __name__ == "__main__":
    main()
