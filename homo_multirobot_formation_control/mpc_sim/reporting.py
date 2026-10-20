"""Persistent numeric and visual evidence for MPC experiments."""

import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .simulation import SimulationLog


def compute_metrics(log: SimulationLog) -> dict[str, float]:
    position = np.linalg.norm(log.error[:, :2], axis=1)
    tail_start = int(0.8 * len(position))
    sent = log.sent_follower
    amplitude = np.logical_or(sent < log.input_min - 1e-8, sent > log.input_max + 1e-8)
    slew = np.abs(np.diff(sent, axis=0)) > log.slew_max + 1e-8
    return {
        "position_rmse": float(np.sqrt(np.mean(position ** 2))),
        "position_max": float(np.max(position)),
        "position_tail_mean": float(np.mean(position[tail_start:])),
        "yaw_rmse": float(np.sqrt(np.mean(log.error[:, 2] ** 2))),
        "velocity_rmse": float(np.sqrt(np.mean(log.error[:, 3:] ** 2))),
        "constraint_violations": float(np.count_nonzero(amplitude) + np.count_nonzero(slew)),
        "nonoptimal_solves": float(sum(status not in {"optimal", "optimal_inaccurate"} for status in log.statuses)),
        "solve_ms_mean": float(1000.0 * np.mean(log.solve_seconds)),
        "solve_ms_max": float(1000.0 * np.max(log.solve_seconds)),
    }


def write_case_results(path: Path, log: SimulationLog, metrics: dict[str, float]) -> None:
    path.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "time", "leader_x", "leader_y", "leader_theta", "follower_x", "follower_y", "follower_theta",
        "ex", "ey", "e_theta", "evx", "evy", "e_omega", "uf_x", "uf_y", "uf_w",
        "ul_x", "ul_y", "ul_w", "applied_uf_x", "applied_uf_y", "applied_uf_w", "solve_ms", "qp_status",
    ]
    with (path / "timeseries.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for index in range(len(log.sent_follower)):
            writer.writerow({
                "time": log.time[index],
                "leader_x": log.leader_state[index, 0], "leader_y": log.leader_state[index, 1], "leader_theta": log.leader_state[index, 2],
                "follower_x": log.follower_state[index, 0], "follower_y": log.follower_state[index, 1], "follower_theta": log.follower_state[index, 2],
                "ex": log.error[index, 0], "ey": log.error[index, 1], "e_theta": log.error[index, 2],
                "evx": log.error[index, 3], "evy": log.error[index, 4], "e_omega": log.error[index, 5],
                "uf_x": log.sent_follower[index, 0], "uf_y": log.sent_follower[index, 1], "uf_w": log.sent_follower[index, 2],
                "ul_x": log.sent_leader[index, 0], "ul_y": log.sent_leader[index, 1], "ul_w": log.sent_leader[index, 2],
                "applied_uf_x": log.applied_follower[index, 0], "applied_uf_y": log.applied_follower[index, 1], "applied_uf_w": log.applied_follower[index, 2],
                "solve_ms": 1000.0 * log.solve_seconds[index], "qp_status": log.statuses[index],
            })
    with (path / "metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(metrics))
        writer.writeheader(); writer.writerow(metrics)

    fig, axis = plt.subplots(figsize=(6, 5))
    axis.plot(log.leader_state[:, 0], log.leader_state[:, 1], label="leader")
    axis.plot(log.follower_state[:, 0], log.follower_state[:, 1], label="follower")
    axis.set(xlabel="map x (m)", ylabel="map y (m)", title=f"{log.scenario} trajectory")
    axis.axis("equal"); axis.legend(); fig.tight_layout(); fig.savefig(path / "trajectory.png", dpi=150); plt.close(fig)
    fig, axes = plt.subplots(3, 1, figsize=(7, 7), sharex=True)
    axes[0].plot(log.time, np.linalg.norm(log.error[:, :2], axis=1)); axes[0].set_ylabel("position (m)")
    axes[1].plot(log.time, log.error[:, 2]); axes[1].set_ylabel("yaw (rad)")
    axes[2].plot(log.time, log.error[:, 3:]); axes[2].set_ylabel("velocity"); axes[2].set_xlabel("time (s)")
    fig.tight_layout(); fig.savefig(path / "errors.png", dpi=150); plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(7, 6), sharex=True)
    axes[0].plot(log.time[:-1], log.sent_follower); axes[0].set_ylabel("sent follower command")
    axes[1].plot(log.time[:-1], 1000.0 * log.solve_seconds); axes[1].set_ylabel("solve (ms)"); axes[1].set_xlabel("time (s)")
    fig.tight_layout(); fig.savefig(path / "inputs_and_solver.png", dpi=150); plt.close(fig)
