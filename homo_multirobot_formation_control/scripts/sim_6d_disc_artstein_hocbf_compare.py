#!/usr/bin/env python3
"""Ideal HOCBF adapter for the existing 6D Artstein Disc simulator."""

from pathlib import Path
import sys
from collections import deque
import argparse

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))

from sim_6d_disc_artstein_compare import (
    Hpc6DDisc, body_to_map, map_to_body, circle_leader_state, make_delay_line,
    advance_delay_line, predict_follower_direction_a,
    se2_predict_constant_twist, step_plant, simulate_case, rows_to_arrays, rot,
)
from hocbf_6d_feasibility import hocbf_halfspace, solve_hocbf_qp


class LocalReferenceGovernor:
    def __init__(self, activation_radius, release_radius, return_tolerance,
                 return_alpha=0.05):
        self.activation_radius = activation_radius
        self.release_radius = release_radius
        self.return_tolerance = return_tolerance
        self.return_alpha = return_alpha
        self.state = "NORMAL"
        self.side = 1.0
        self.reference = None

    def update(self, follower, obstacle, target):
        delta = follower - obstacle
        distance = np.linalg.norm(delta)
        if self.state == "NORMAL" and distance < self.activation_radius:
            normal = delta / max(distance, 1e-9)
            tangent = np.array([-normal[1], normal[0]])
            candidates = [
                obstacle + self.release_radius * tangent,
                obstacle - self.release_radius * tangent,
            ]
            costs = [
                np.linalg.norm(follower - q) + 1.5 * np.linalg.norm(q - target)
                for q in candidates
            ]
            best = int(np.argmin(costs))
            self.side = 1.0 if best == 0 else -1.0
            self.reference = candidates[best]
            self.state = "BYPASS"
        elif self.state == "BYPASS" and distance > self.release_radius:
            self.state = "RETURN"
        elif self.state == "BYPASS":
            phi = np.arctan2(delta[1], delta[0])
            lookahead = self.side * np.pi / 3.0
            self.reference = obstacle + self.release_radius * np.array([
                np.cos(phi + lookahead), np.sin(phi + lookahead)
            ])
        elif self.state == "RETURN":
            self.reference += self.return_alpha * (target - self.reference)
            if np.linalg.norm(self.reference - target) < self.return_tolerance:
                self.state = "NORMAL"
                self.reference = target.copy()
        if self.reference is None:
            self.reference = target.copy()
        return self


def filter_translation_command(
    x_pred: np.ndarray,
    yaw_meas: float,
    cmd_nom_body: np.ndarray,
    obstacles: list[np.ndarray],
    safe_radius: float,
    tau: float,
    c1: float,
    c2: float,
    previous_map_command: np.ndarray,
    vmax: float,
    amax: float,
    dt: float,
) -> tuple[np.ndarray, np.ndarray, bool, float]:
    """Apply map-frame HOCBF to translation while retaining nominal yaw command."""
    cmd_nom_body = np.asarray(cmd_nom_body, dtype=float)
    cmd_nom_map = body_to_map(yaw_meas, cmd_nom_body[:2])
    v_pred_map = body_to_map(x_pred[2], x_pred[3:5])
    halfspaces = []
    h_values = []
    for obstacle in obstacles:
        a, b, h, _ = hocbf_halfspace(
            np.r_[x_pred[:2], v_pred_map], obstacle, safe_radius, tau, c1, c2
        )
        halfspaces.append((a, b))
        h_values.append(h)
    result = solve_hocbf_qp(
        cmd_nom_map, previous_map_command, halfspaces, vmax, amax, dt
    )
    safe_map = result.command if result.feasible else np.zeros(2)
    safe_body = map_to_body(yaw_meas, safe_map)
    return (
        np.r_[safe_body, cmd_nom_body[2]],
        safe_map,
        result.feasible,
        min(h_values),
    )


def simulate_compensated_hocbf(Tmax, h, tau_v, tau_w, Td, obstacles, safe_radius,
                               leader_heading_fixed=False, leader_speed=0.45):
    """Existing compensated loop with final command replaced by HOCBF output."""
    ctrl = Hpc6DDisc(control_period=h)
    x1 = circle_leader_state(0.0, speed=leader_speed, heading_fixed=leader_heading_fixed)
    x2 = np.array([4.2, -0.4, np.pi / 2, 0.0, 0.0, 0.0])
    hist_len = max(1, int(np.ceil(Td / h))) + 2
    hist = deque([body_to_map(x2[2], x2[3:5]) for _ in range(hist_len)], maxlen=hist_len)
    whist = deque([np.array([x2[5]]) for _ in range(hist_len)], maxlen=hist_len)
    delay = make_delay_line(x2[3:6], Td, 0.01)
    last_map = hist[0].copy(); last_w = 0.0; prev_map = last_map.copy()
    x1c = se2_predict_constant_twist(x1, Td + tau_v)
    x2c = predict_follower_direction_a(x2, hist, whist, last_map, last_w, tau_v, tau_w, Td, h)
    ctrl.init(x1c, x2c); rows=[]; t=0.0
    while t < Tmax - 1e-12:
        x1 = circle_leader_state(t, speed=leader_speed, heading_fixed=leader_heading_fixed)
        yaw_meas = x2[2]
        x1c = se2_predict_constant_twist(x1, Td + tau_v)
        x2c = predict_follower_direction_a(x2, hist, whist, last_map, last_w, tau_v, tau_w, Td, h)
        nominal = np.clip(ctrl.command(x1c, x2c), [-1,-1,-.8], [1,1,.8])
        safe, safe_map, feasible, hval = filter_translation_command(
            x2c, yaw_meas, nominal, obstacles, safe_radius, tau_v, 2.0, 2.0,
            prev_map, 1.0, 20.0, h)
        for _ in range(int(round(h / .01))):
            x2 = step_plant(x2, advance_delay_line(delay, safe), .01, tau_v, tau_w)
        # Match the ROS node: history stores the command published in the
        # current measured body frame, not the predicted control-frame yaw.
        last_map = body_to_map(yaw_meas, safe[:2]); last_w = safe[2]
        hist.appendleft(last_map.copy()); whist.appendleft(np.array([last_w])); prev_map=safe_map
        t += h
        rows.append({"t":t, "x1":x1.copy(), "x2":x2.copy(), "cmd_nom_body":nominal,
                     "cmd_safe_body":safe, "cmd_safe_map":safe_map, "h":hval,
                     "correction_norm":float(np.linalg.norm(safe_map-body_to_map(x2[2], nominal[:2]))),
                     "feasible":feasible})
    return rows


def obstacle_from_path(path_xy, index, offset, side):
    tangent = path_xy[:, index + 1] - path_xy[:, index - 1]
    tangent /= np.linalg.norm(tangent)
    normal = np.array([-tangent[1], tangent[0]])
    return path_xy[:, index] + side * offset * normal


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", default="homo_multirobot_formation_control/analysis/results/6d_artstein_disc_hocbf")
    parser.add_argument("--tmax", type=float, default=30.0)
    parser.add_argument("--obstacle-x", type=float, default=None)
    parser.add_argument("--obstacle-y", type=float, default=None)
    parser.add_argument("--obstacle2-x", type=float, default=None)
    parser.add_argument("--obstacle2-y", type=float, default=None)
    parser.add_argument("--auto-obstacle-offset", type=float, default=None)
    parser.add_argument("--auto-two-cylinders", action="store_true")
    parser.add_argument("--auto-two-offset", type=float, default=0.5)
    parser.add_argument("--safe-radius", type=float, default=0.8)
    parser.add_argument("--physical-radius", type=float, default=0.8)
    parser.add_argument("--leader-heading-fixed", action="store_true")
    parser.add_argument("--leader-speed", type=float, default=0.45)
    args = parser.parse_args()
    out = Path(args.out_dir); out.mkdir(parents=True, exist_ok=True)
    base = simulate_case("compensated", args.tmax, .05, .43, .43, .22,
                         leader_heading_fixed=args.leader_heading_fixed,
                         leader_speed=args.leader_speed)
    _, _, x2, *_ = rows_to_arrays(base)
    if args.auto_two_cylinders:
        i1 = len(base) // 2
        i2 = int(0.7 * (len(base) - 1))
        obstacles = [
            obstacle_from_path(x2[:2], i1, args.auto_two_offset, 1.0),
            obstacle_from_path(x2[:2], i2, args.auto_two_offset, -1.0),
        ]
        obstacle = obstacles[0]
    elif args.auto_obstacle_offset is not None:
        i = len(base) // 2
        point = x2[:2, i]
        tangent = x2[:2, i + 1] - x2[:2, i - 1]
        normal = np.array([-tangent[1], tangent[0]]) / np.linalg.norm(tangent)
        obstacle = point + args.auto_obstacle_offset * normal
    else:
        obstacle = (np.array([args.obstacle_x, args.obstacle_y])
                if args.obstacle_x is not None and args.obstacle_y is not None
                else x2[:2, len(base) // 2].copy())
    radius = args.safe_radius
    if not args.auto_two_cylinders:
        obstacles = [obstacle]
    if args.obstacle2_x is not None and args.obstacle2_y is not None:
        obstacles.append(np.array([args.obstacle2_x, args.obstacle2_y]))
    safe = simulate_compensated_hocbf(args.tmax, .05, .43, .43, .22, obstacles,
                                      radius, args.leader_heading_fixed, args.leader_speed)
    _, x1, x2, *_ = rows_to_arrays(base)
    sx = np.array([r["x2"] for r in safe]); st = np.array([r["t"] for r in safe])
    fig, ax = plt.subplots(1, 2, figsize=(11, 4), constrained_layout=True)
    ax[0].plot(x1[0], x1[1], "k--", label="leader")
    ax[0].plot(x2[0], x2[1], label="6D Artstein")
    ax[0].plot(sx[:,0], sx[:,1], label="6D Artstein + HOCBF")
    for idx, obs in enumerate(obstacles):
        label = "physical safety" if idx == 0 else None
        ax[0].add_patch(plt.Circle(obs, args.physical_radius, fill=False, color="r", linewidth=2, label=label))
        ax[0].add_patch(plt.Circle(obs, radius, fill=False, color="orange", linestyle="--", linewidth=2, label="HOCBF filter" if idx == 0 else None))
    ax[0].axis("equal"); ax[0].legend(); ax[0].set_title("trajectory")
    ax[1].plot(st, np.linalg.norm(sx[:,:2]-obstacle, axis=1), label="HOCBF")
    ax[1].axhline(radius, color="r", linestyle="--"); ax[1].legend(); ax[1].set_title("obstacle distance")
    fig.savefig(out / "coupled_compare.png", dpi=180)
    np.savetxt(out / "coupled_timeseries.csv",
               np.column_stack([st, sx[:,0], sx[:,1],
                                np.array([r["h"] for r in safe]),
                                np.array([r["correction_norm"] for r in safe])]),
               delimiter=",", header="t,x,y,h,correction_norm", comments="")
    print(out / "coupled_compare.png")


if __name__ == "__main__":
    main()
