import importlib.util
import math
from pathlib import Path
import sys

import numpy as np


SCRIPT_PATH = (
    Path(__file__).parents[1] / "scripts" / "leader_eight_closed_loop_map.py"
)
SPEC = importlib.util.spec_from_file_location(
    "leader_eight_closed_loop_map", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_eight_reference_starts_and_returns_to_origin():
    p0 = np.array([1.5, -0.25])
    omega = MODULE.reference_omega(2.0, 1.0, 0.4)
    period = 2.0 * math.pi / omega

    position, _ = MODULE.eight_reference(p0, 2.0, 1.0, 0.4, 0.0)
    returned, _ = MODULE.eight_reference(p0, 2.0, 1.0, 0.4, period)

    np.testing.assert_allclose(position, p0)
    np.testing.assert_allclose(returned, p0, atol=1e-12)


def test_eight_reference_peak_speed_and_zero_speed():
    p0 = np.zeros(2)
    speed = 0.4
    omega = MODULE.reference_omega(2.0, 1.0, speed)
    period = 2.0 * math.pi / omega
    norms = [
        np.linalg.norm(MODULE.eight_reference(p0, 2.0, 1.0, speed, time)[1])
        for time in np.linspace(0.0, period, 1001)
    ]

    assert max(norms) <= speed + 1e-12

    position, velocity = MODULE.eight_reference(p0, 2.0, 1.0, 0.0, 99.0)
    np.testing.assert_allclose(position, p0)
    np.testing.assert_allclose(velocity, [0.0, 0.0])


def test_launch_forwards_map_and_mocap_parameters():
    launch_path = (
        Path(__file__).parents[1]
        / "launch"
        / "leader_eight_closed_loop_map.launch.py"
    )
    source = launch_path.read_text(encoding="utf-8")

    for value in (
        "amplitude_x",
        "amplitude_y",
        "speed",
        "state_source",
        "mocap_pose_topic",
        "mocap_twist_topic",
        "map_frame",
        "leader_eight_closed_loop_map.py",
    ):
        assert value in source
