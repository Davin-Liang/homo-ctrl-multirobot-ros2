import importlib.util
from pathlib import Path

import pytest


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = PACKAGE_DIR / "scripts" / "analyze_no_feedforward_sensitivity.py"
SPEC = importlib.util.spec_from_file_location(
    "analyze_no_feedforward_sensitivity", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


def test_compute_trial_metrics_uses_absolute_error_and_last_ten_seconds():
    rows = [
        {"time_s": 0.0, "distance_m": 0.8,
         "leader_vx_map_ms": 1.0, "leader_vy_map_ms": 0.0,
         "follower_vx_map_ms": 0.0, "follower_vy_map_ms": 0.0},
        {"time_s": 5.0, "distance_m": 1.0,
         "leader_vx_map_ms": 1.0, "leader_vy_map_ms": 0.0,
         "follower_vx_map_ms": 1.0, "follower_vy_map_ms": 0.0},
        {"time_s": 10.0, "distance_m": 1.2,
         "leader_vx_map_ms": 0.0, "leader_vy_map_ms": 1.0,
         "follower_vx_map_ms": 0.0, "follower_vy_map_ms": 1.0},
        {"time_s": 15.0, "distance_m": 1.1,
         "leader_vx_map_ms": 0.0, "leader_vy_map_ms": 1.0,
         "follower_vx_map_ms": 0.0, "follower_vy_map_ms": 2.0},
    ]

    metrics = module.compute_trial_metrics(rows, radius_m=1.0)

    assert metrics["mean_abs_distance_error_m"] == pytest.approx(0.125)
    assert metrics["rms_distance_error_m"] == pytest.approx(0.15)
    assert metrics["tail_mean_abs_distance_error_m"] == pytest.approx(0.1)
    assert metrics["tail_distance_error_std_m"] == pytest.approx(0.081649658)
    assert metrics["mean_relative_velocity_error_mps"] == pytest.approx(0.5)
    assert metrics["tail_mean_relative_velocity_error_mps"] == pytest.approx(1.0 / 3.0)


def test_aggregate_trials_uses_sample_standard_deviation():
    first = {key: 0.1 for key in module.METRIC_KEYS}
    second = {key: 0.2 for key in module.METRIC_KEYS}
    third = {key: 0.3 for key in module.METRIC_KEYS}
    summary = module.aggregate_trials([
        first,
        second,
        third,
    ])

    assert summary["mean_abs_distance_error_m"] == pytest.approx((0.2, 0.1))
