#!/usr/bin/env python3
"""Measure motion-capture velocity step responses and compare simple models."""

import argparse
import csv
import json
import math
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np


@dataclass
class FitResult:
    prediction: np.ndarray
    parameters: dict
    sse: float
    rmse: float
    mae: float
    aic: float


def scalar_response(mode, vx_map, vy_map, wz):
    if mode == "move":
        return math.hypot(vx_map, vy_map)
    if mode == "turn":
        return wz
    raise ValueError(f"unsupported mode: {mode}")


def _result(prediction, speed, parameters):
    error = prediction - speed
    sse = float(np.dot(error, error))
    count = len(speed)
    return FitResult(prediction, parameters, sse,
                     math.sqrt(sse / count), float(np.mean(np.abs(error))),
                     count * math.log(max(sse / count, 1e-12)) + 2.0)


def fit_first_order(time_s, speed, target):
    time_s, speed = np.asarray(time_s), np.asarray(speed)
    initial = speed[0]
    best = None
    for tau in np.linspace(0.02, 3.0, 300):
        prediction = target + (initial - target) * np.exp(-time_s / tau)
        result = _result(prediction, speed, {"tau": float(tau)})
        if best is None or result.sse < best.sse:
            best = result
    return best


def fit_constant_accel(time_s, speed, target):
    time_s, speed = np.asarray(time_s), np.asarray(speed)
    initial = speed[0]
    direction = 1.0 if target >= initial else -1.0
    upper = max(abs(target - initial) / max(time_s[-1], 1e-6) * 4.0, 0.01)
    best = None
    for magnitude in np.linspace(0.001, upper, 300):
        acceleration = direction * magnitude
        prediction = np.clip(initial + acceleration * time_s,
                             min(initial, target), max(initial, target))
        result = _result(prediction, speed, {"acceleration": float(acceleration)})
        if best is None or result.sse < best.sse:
            best = result
    return best


def choose_model(first_order, constant_accel):
    return "first_order" if first_order.aic < constant_accel.aic else "constant_accel"


SAMPLE_FIELDS = ("trial", "phase", "elapsed_s", "cmd_vx", "cmd_vy", "cmd_wz",
                 "vx_map", "vy_map", "wz", "response")


def write_outputs(output_dir, samples, summaries, metadata):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "samples.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=SAMPLE_FIELDS)
        writer.writeheader()
        writer.writerows(samples)
    summary_fields = sorted({key for row in summaries for key in row}) or ["status"]
    with (output_dir / "fit_summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=summary_fields)
        writer.writeheader()
        writer.writerows(summaries)
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")


def command_values(mode, target):
    return (target, 0.0, 0.0) if mode == "move" else (0.0, 0.0, target)


def summarize_samples(samples, target):
    rows = []
    for trial in sorted({sample["trial"] for sample in samples}):
        for phase, phase_target in (("hold", target), ("stop", 0.0)):
            part = [sample for sample in samples
                    if sample["trial"] == trial and sample["phase"] == phase]
            if len(part) < 2:
                rows.append({"trial": trial, "phase": phase, "status": "invalid"})
                continue
            t = np.array([sample["elapsed_s"] for sample in part])
            y = np.array([sample["response"] for sample in part])
            first, accel = fit_first_order(t, y, phase_target), fit_constant_accel(t, y, phase_target)
            rows.append({"trial": trial, "phase": phase, "status": "ok",
                         "first_tau": first.parameters["tau"], "first_rmse": first.rmse,
                         "first_aic": first.aic, "accel": accel.parameters["acceleration"],
                         "accel_rmse": accel.rmse, "accel_aic": accel.aic,
                         "preferred": choose_model(first, accel)})
    return rows


def plot_samples(output_dir, samples):
    if not samples:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axis = plt.subplots()
    for trial in sorted({sample["trial"] for sample in samples}):
        part = [sample for sample in samples if sample["trial"] == trial]
        axis.plot([sample["response"] for sample in part], label=f"trial {trial}")
    axis.set_xlabel("sample")
    axis.set_ylabel("measured response")
    axis.grid()
    axis.legend()
    figure.savefig(Path(output_dir) / "comparison.png", dpi=150)
    plt.close(figure)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("move", "turn"), required=True)
    parser.add_argument("--target", type=float, required=True,
                        help="move: m/s; turn: rad/s")
    parser.add_argument("--namespace", default="/robot2")
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--hold-seconds", type=float, default=3.0)
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    parser.add_argument("--mocap-timeout", type=float, default=0.20)
    parser.add_argument("--out-dir", default="")
    return parser.parse_args(argv)


def main():
    args = parse_args()
    import rclpy
    from geometry_msgs.msg import Twist
    from geometry_msgs.msg import TwistStamped
    from rclpy.node import Node

    class Meter(Node):
        def __init__(self):
            super().__init__("mocap_velocity_model_meter")
            ns = args.namespace.rstrip("/")
            self.publisher = self.create_publisher(Twist, f"{ns}/cmd_vel", 10)
            self.latest = None
            self.samples = []
            self.create_subscription(TwistStamped, f"{ns}/mocap/twist", self.callback, 10)

        def callback(self, message):
            self.latest = (time.monotonic(), message)

        def publish_stop(self):
            self.publisher.publish(Twist())

        def command(self, active):
            msg = Twist()
            if active:
                msg.linear.x, msg.linear.y, msg.angular.z = command_values(args.mode, args.target)
            self.publisher.publish(msg)
            return msg

        def collect(self, trial, phase, duration, active):
            started = time.monotonic()
            while time.monotonic() - started < duration:
                rclpy.spin_once(self, timeout_sec=0.02)
                if self.latest is None or time.monotonic() - self.latest[0] > args.mocap_timeout:
                    raise RuntimeError("motion-capture velocity timed out")
                cmd = self.command(active)
                msg = self.latest[1]
                self.samples.append({"trial": trial, "phase": phase,
                    "elapsed_s": time.monotonic() - started,
                    "cmd_vx": cmd.linear.x, "cmd_vy": cmd.linear.y, "cmd_wz": cmd.angular.z,
                    "vx_map": msg.twist.linear.x, "vy_map": msg.twist.linear.y,
                    "wz": msg.twist.angular.z,
                    "response": scalar_response(args.mode, msg.twist.linear.x,
                                                msg.twist.linear.y, msg.twist.angular.z)})

    rclpy.init()
    meter = Meter()
    root = Path(args.out_dir) if args.out_dir else Path(__file__).resolve().parents[1] / "robot_traj/real/model_identification"
    output = root / datetime.now().strftime("%Y%m%d_%H%M%S")
    try:
        for trial in range(1, args.trials + 1):
            meter.collect(trial, "settle", args.settle_seconds, False)
            meter.collect(trial, "hold", args.hold_seconds, True)
            meter.collect(trial, "stop", args.settle_seconds, False)
        write_outputs(output, meter.samples, summarize_samples(meter.samples, args.target),
                      vars(args) | {"completed": True})
        plot_samples(output, meter.samples)
        print(output)
    finally:
        meter.publish_stop()
        meter.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
