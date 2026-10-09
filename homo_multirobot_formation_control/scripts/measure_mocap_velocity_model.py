#!/usr/bin/env python3
"""Measure motion-capture velocity step responses and compare simple models."""

import argparse
import math
from dataclasses import dataclass

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


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("move", "turn"), required=True)
    parser.add_argument("--target", type=float, required=True,
                        help="move: m/s; turn: rad/s")
    return parser.parse_args(argv)


def main():
    parse_args()
    raise RuntimeError("ROS collection is not implemented yet")


if __name__ == "__main__":
    main()
