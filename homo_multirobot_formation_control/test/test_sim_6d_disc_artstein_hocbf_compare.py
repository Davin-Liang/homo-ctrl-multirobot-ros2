import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest


PATH = Path(__file__).resolve().parents[1] / "scripts" / "sim_6d_disc_artstein_hocbf_compare.py"


def load_module():
    spec = importlib.util.spec_from_file_location("sim_6d_disc_artstein_hocbf_compare", PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_filter_preserves_yaw_and_nominal_command_when_obstacle_is_far():
    module = load_module()
    obstacle = module.ObstacleSpec(np.array([10.0, 0.0]), 0.8, 0.8)
    cmd, cmd_map, feasible, h, h_values = module.filter_translation_command(
        np.array([0.0, 0.0, 0.0, 0.1, 0.0, 0.2]),
        0.0, np.array([0.2, 0.0, 0.3]), [obstacle],
        0.43, 2.0, 2.0, np.zeros(2), 1.0, 20.0, 0.05)
    np.testing.assert_allclose(cmd, [0.2, 0.0, 0.3])
    np.testing.assert_allclose(cmd_map, [0.2, 0.0])
    assert feasible and h > 0.0
    assert h_values.shape == (1,)


def test_coupled_simulation_records_safe_command_history_fields():
    module = load_module()
    rows = module.simulate_compensated_hocbf(
        Tmax=0.1, h=0.05, tau_v=0.43, tau_w=0.43, Td=0.22,
        obstacle=np.array([2.0, 0.0]), safe_radius=0.8)
    assert len(rows) == 2
    assert rows[0]["cmd_safe_map"].shape == (2,)
    assert "h" in rows[0] and "correction_norm" in rows[0]
    assert rows[0]["obstacle_distances"].shape == (1,)
    assert rows[0]["h_values"].shape == (1,)


def test_obstacle_specs_derive_distinct_physical_and_filter_radii():
    module = load_module()
    specs = module.make_obstacle_specs(
        [np.array([1.0, 2.0]), np.array([3.0, 4.0])],
        [0.20, 0.35], follower_radius=0.15, clearance=0.10,
        filter_margin=0.12,
    )
    np.testing.assert_allclose(
        [item.physical_radius for item in specs], [0.45, 0.60],
    )
    np.testing.assert_allclose(
        [item.filter_radius for item in specs], [0.57, 0.72],
    )


def test_filter_uses_each_obstacles_individual_filter_radius(monkeypatch):
    module = load_module()
    calls = []
    real_halfspace = module.hocbf_halfspace

    def capture_halfspace(state, center, radius, *args):
        calls.append(radius)
        return real_halfspace(state, center, radius, *args)

    monkeypatch.setattr(module, "hocbf_halfspace", capture_halfspace)
    specs = module.make_obstacle_specs(
        [np.array([10.0, 0.0]), np.array([0.0, 10.0])],
        [0.20, 0.35], follower_radius=0.15, clearance=0.10,
        filter_margin=0.12,
    )
    module.filter_translation_command(
        np.array([0.0, 0.0, 0.0, 0.1, 0.0, 0.2]), 0.0,
        np.array([0.2, 0.0, 0.3]), specs, 0.43, 2.0, 2.0,
        np.zeros(2), 1.0, 20.0, 0.05,
    )
    np.testing.assert_allclose(calls, [0.57, 0.72])


def test_parse_radius_list_rejects_non_positive_radius():
    module = load_module()
    assert module.parse_radius_list("0.20,0.35") == [0.20, 0.35]
    with pytest.raises(ValueError, match="positive"):
        module.parse_radius_list("0.20,0")
