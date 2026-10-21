"""Deterministic Leader commands for numerical formation experiments."""

from dataclasses import dataclass
from typing import Callable

import numpy as np


@dataclass(frozen=True)
class Scenario:
    name: str
    duration: float
    initial_error: np.ndarray
    offset_map: np.ndarray
    leader_command_at: Callable[[float], np.ndarray]

    def __post_init__(self) -> None:
        if self.duration <= 0.0:
            raise ValueError("duration must be positive")
        if np.asarray(self.initial_error).shape != (6,):
            raise ValueError("initial_error must have six entries")
        if np.asarray(self.offset_map).shape != (3,):
            raise ValueError("offset_map must have three entries")


def make_static_scenario(
    initial_error: np.ndarray | None = None, duration: float = 12.0
) -> Scenario:
    return Scenario(
        "static",
        duration,
        np.array([1.0, -0.7, 0.5, 0.0, 0.0, 0.0])
        if initial_error is None
        else np.asarray(initial_error, dtype=float),
        np.array([-1.0, 0.0, 0.0]),
        lambda _: np.zeros(3),
    )


def make_straight_scenario(duration: float = 16.0) -> Scenario:
    return Scenario(
        "straight",
        duration,
        np.array([1.0, -0.5, 0.35, 0.0, 0.0, 0.0]),
        np.array([-1.0, 0.0, 0.0]),
        lambda _: np.array([0.28, 0.06, 0.0]),
    )


def make_curve_scenario(duration: float = 20.0) -> Scenario:
    def command(time: float) -> np.ndarray:
        return np.array(
            [0.22 + 0.10 * np.cos(0.45 * time), 0.12 * np.sin(0.35 * time), 0.32 * np.cos(0.30 * time)]
        )

    return Scenario(
        "curve",
        duration,
        np.array([1.0, -0.6, 0.45, 0.0, 0.0, 0.0]),
        np.array([-1.0, 0.0, 0.0]),
        command,
    )


def make_circle_scenario(duration: float = 32.0) -> Scenario:
    """Leader command tracing a 2 m map-frame circle after actuator transients."""
    radius, omega = 2.0, 0.25

    def command(time: float) -> np.ndarray:
        phase = omega * time
        return np.array(
            [
                -radius * omega * np.sin(phase),
                radius * omega * np.cos(phase),
                omega,
            ]
        )

    return Scenario(
        "circle",
        duration,
        np.array([1.0, -0.6, 0.45, 0.0, 0.0, 0.0]),
        np.array([-1.0, 0.0, 0.0]),
        command,
    )


def make_circle_far3_scenario(duration: float = 32.0) -> Scenario:
    """The circular Leader case with a fixed 3 m map-frame Follower offset."""
    base = make_circle_scenario(duration)
    return Scenario(
        "circle_far3",
        duration,
        base.initial_error.copy(),
        np.array([-3.0, 0.0, 0.0]),
        base.leader_command_at,
    )


def make_circle_large_initial_separation_scenario(
    duration: float = 32.0, initial_distance: float = 3.0
) -> Scenario:
    """Original 1 m target offset with Follower initially behind Leader."""
    if initial_distance <= 0.0:
        raise ValueError("initial_distance must be positive")
    base = make_circle_scenario(duration)
    initial_error_x = -initial_distance - base.offset_map[0]
    return Scenario(
        "circle_large_init",
        duration,
        np.array([initial_error_x, 0.0, 0.45, 0.0, 0.0, 0.0]),
        base.offset_map.copy(),
        base.leader_command_at,
    )
