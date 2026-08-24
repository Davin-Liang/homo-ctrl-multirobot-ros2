#!/usr/bin/env python3
"""Ideal HOCBF adapter for the existing 6D Artstein Disc simulator."""

from pathlib import Path
import sys
from collections import deque
import argparse
from dataclasses import dataclass

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


@dataclass(frozen=True)
class ObstacleSpec:
    """Static cylinder geometry after robot and safety margins are included."""
    center: np.ndarray
    physical_radius: float
    filter_radius: float


def make_obstacle_specs(centers, cylinder_radii, follower_radius, clearance,
                        filter_margin):
    if len(centers) != len(cylinder_radii):
        raise ValueError("centers and cylinder_radii must have equal length")
    if follower_radius < 0.0 or clearance < 0.0 or filter_margin < 0.0:
        raise ValueError("safety radius components must be non-negative")
    specs = []
    for center, cylinder_radius in zip(centers, cylinder_radii):
        if cylinder_radius <= 0.0:
            raise ValueError("all cylinder radii must be positive")
        physical_radius = follower_radius + cylinder_radius + clearance
        specs.append(ObstacleSpec(
            np.asarray(center, dtype=float), physical_radius,
            physical_radius + filter_margin,
        ))
    return specs


def parse_radius_list(value):
    try:
        radii = [float(part) for part in value.split(",")]
    except ValueError as error:
        raise ValueError("cylinder radii must be comma-separated numbers") from error
    if not radii or any(radius <= 0.0 for radius in radii):
        raise ValueError("all cylinder radii must be positive")
    return radii


def coerce_obstacle_specs(obstacles, safe_radius):
    """Keep old point-plus-radius calls usable while the CLI uses ObstacleSpec."""
    if isinstance(obstacles, np.ndarray) and obstacles.shape == (2,):
        obstacles = [obstacles]
    if all(isinstance(item, ObstacleSpec) for item in obstacles):
        return obstacles
    return [ObstacleSpec(np.asarray(item, dtype=float), safe_radius, safe_radius)
            for item in obstacles]


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
            if np.linalg.norm(follower - target) < self.return_tolerance:
                self.state = "NORMAL"
                self.reference = target.copy()
                return self
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
    obstacles: list[ObstacleSpec],
    tau: float,
    c1: float,
    c2: float,
    previous_map_command: np.ndarray,
    vmax: float,
    amax: float,
    dt: float,
) -> tuple[np.ndarray, np.ndarray, bool, float, np.ndarray]:
    """Apply map-frame HOCBF to translation while retaining nominal yaw command."""
    cmd_nom_body = np.asarray(cmd_nom_body, dtype=float)
    cmd_nom_map = body_to_map(yaw_meas, cmd_nom_body[:2])
    v_pred_map = body_to_map(x_pred[2], x_pred[3:5])
    halfspaces = []
    h_values = []
    for obstacle in obstacles:
        a, b, h, _ = hocbf_halfspace(
            np.r_[x_pred[:2], v_pred_map], obstacle.center,
            obstacle.filter_radius, tau, c1, c2
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
        np.asarray(h_values),
    )


def simulate_compensated_hocbf(Tmax, h, tau_v, tau_w, Td, obstacles=None, safe_radius=0.8,
                               leader_heading_fixed=False, leader_speed=0.45,
                               obstacle=None):
    """Existing compensated loop with final command replaced by HOCBF output."""
    if obstacle is not None:
        obstacles = [obstacle]
    if obstacles is None:
        obstacles = [np.array([2.0, 0.0])]
    obstacles = coerce_obstacle_specs(obstacles, safe_radius)
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
        safe, safe_map, feasible, hval, h_values = filter_translation_command(
            x2c, yaw_meas, nominal, obstacles, tau_v, 2.0, 2.0,
            prev_map, 1.0, 20.0, h)
        for _ in range(int(round(h / .01))):
            x2 = step_plant(x2, advance_delay_line(delay, safe), .01, tau_v, tau_w)
        # Match the ROS node: history stores the command published in the
        # current measured body frame, not the predicted control-frame yaw.
        last_map = body_to_map(yaw_meas, safe[:2]); last_w = safe[2]
        hist.appendleft(last_map.copy()); whist.appendleft(np.array([last_w])); prev_map=safe_map
        t += h
        obstacle_distances = np.array([
            np.linalg.norm(x2[:2] - item.center) for item in obstacles
        ])
        rows.append({"t":t, "x1":x1.copy(), "x2":x2.copy(), "cmd_nom_body":nominal,
                     "cmd_safe_body":safe, "cmd_safe_map":safe_map, "h":hval,
                     "h_values":h_values,
                     "obstacle_distances":obstacle_distances,
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
    parser.add_argument("--cylinder-radii", default="0.25",
                        help="comma-separated physical cylinder radii [m]")
    parser.add_argument("--follower-radius", type=float, default=0.15)
    parser.add_argument("--clearance", type=float, default=0.10)
    parser.add_argument("--filter-margin", type=float, default=0.15)
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
        obstacle_centers = [
            obstacle_from_path(x2[:2], i1, args.auto_two_offset, 1.0),
            obstacle_from_path(x2[:2], i2, args.auto_two_offset, -1.0),
        ]
    elif args.auto_obstacle_offset is not None:
        i = len(base) // 2
        point = x2[:2, i]
        tangent = x2[:2, i + 1] - x2[:2, i - 1]
        normal = np.array([-tangent[1], tangent[0]]) / np.linalg.norm(tangent)
        obstacle_centers = [point + args.auto_obstacle_offset * normal]
    else:
        obstacle_centers = [(np.array([args.obstacle_x, args.obstacle_y])
                if args.obstacle_x is not None and args.obstacle_y is not None
                else x2[:2, len(base) // 2].copy())]
    if args.obstacle2_x is not None and args.obstacle2_y is not None:
        obstacle_centers.append(np.array([args.obstacle2_x, args.obstacle2_y]))
    cylinder_radii = parse_radius_list(args.cylinder_radii)
    obstacles = make_obstacle_specs(
        obstacle_centers, cylinder_radii, args.follower_radius, args.clearance,
        args.filter_margin,
    )
    safe = simulate_compensated_hocbf(args.tmax, .05, .43, .43, .22, obstacles,
                                      leader_heading_fixed=args.leader_heading_fixed,
                                      leader_speed=args.leader_speed)
    _, x1, x2, *_ = rows_to_arrays(base)
    sx = np.array([r["x2"] for r in safe]); st = np.array([r["t"] for r in safe])
    fig, ax = plt.subplots(1, 2, figsize=(11, 4), constrained_layout=True)
    ax[0].plot(x1[0], x1[1], "k--", label="leader")
    ax[0].plot(x2[0], x2[1], label="6D Artstein")
    ax[0].plot(sx[:,0], sx[:,1], label="6D Artstein + HOCBF")
    for idx, obstacle in enumerate(obstacles):
        ax[0].add_patch(plt.Circle(
            obstacle.center, obstacle.physical_radius, fill=False, color="r",
            linewidth=2, label="physical safety" if idx == 0 else None,
        ))
        ax[0].add_patch(plt.Circle(
            obstacle.center, obstacle.filter_radius, fill=False, color="orange",
            linestyle="--", linewidth=2, label="HOCBF filter" if idx == 0 else None,
        ))
    ax[0].axis("equal"); ax[0].legend(); ax[0].set_title("trajectory")
    distances = np.array([row["obstacle_distances"] for row in safe])
    for idx, obstacle in enumerate(obstacles):
        ax[1].plot(st, distances[:, idx], label=f"distance obs{idx + 1}")
        ax[1].axhline(obstacle.physical_radius, color="r", linestyle="-", alpha=.6)
        ax[1].axhline(obstacle.filter_radius, color="orange", linestyle="--", alpha=.7)
    ax[1].legend(); ax[1].set_title("obstacle distances")
    fig.savefig(out / "coupled_compare.png", dpi=180)
    columns = [st, sx[:, 0], sx[:, 1], np.array([row["h"] for row in safe]),
               np.array([row["correction_norm"] for row in safe])]
    header = ["t", "x", "y", "h", "correction_norm"]
    for idx, obstacle in enumerate(obstacles):
        distance = distances[:, idx]
        columns.extend([distance, distance - obstacle.physical_radius,
                        distance - obstacle.filter_radius])
        header.extend([f"distance_obs{idx + 1}",
                       f"physical_margin_obs{idx + 1}",
                       f"filter_margin_obs{idx + 1}"])
        print(
            f"obs{idx + 1}: cylinder_radius={cylinder_radii[idx]:.3f} m, "
            f"R_physical={obstacle.physical_radius:.3f} m, "
            f"R_filter={obstacle.filter_radius:.3f} m, "
            f"min_distance={distance.min():.3f} m, "
            f"min_physical_margin={(distance - obstacle.physical_radius).min():.3f} m, "
            f"min_filter_margin={(distance - obstacle.filter_radius).min():.3f} m, "
            f"physical_violation={bool(np.any(distance < obstacle.physical_radius))}"
        )
    csv_path = out / "coupled_timeseries.csv"
    np.savetxt(csv_path, np.column_stack(columns), delimiter=",",
               header=",".join(header), comments="")
    print(out / "coupled_compare.png")
    print(csv_path)


if __name__ == "__main__":
    main()
