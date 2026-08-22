import importlib.util
import sys
from pathlib import Path

import numpy as np


PATH = Path(__file__).resolve().parents[1] / "scripts" / "sim_6d_disc_artstein_hocbf_compare.py"


def load_module():
    spec = importlib.util.spec_from_file_location("sim_6d_disc_artstein_hocbf_compare", PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_filter_preserves_yaw_and_nominal_command_when_obstacle_is_far():
    module = load_module()
    cmd, cmd_map, feasible, h = module.filter_translation_command(
        np.array([0.0, 0.0, 0.0, 0.1, 0.0, 0.2]),
        0.0, np.array([0.2, 0.0, 0.3]), np.array([10.0, 0.0]),
        0.8, 0.43, 2.0, 2.0, np.zeros(2), 1.0, 20.0, 0.05)
    np.testing.assert_allclose(cmd, [0.2, 0.0, 0.3])
    np.testing.assert_allclose(cmd_map, [0.2, 0.0])
    assert feasible and h > 0.0


def test_coupled_simulation_records_safe_command_history_fields():
    module = load_module()
    rows = module.simulate_compensated_hocbf(
        Tmax=0.1, h=0.05, tau_v=0.43, tau_w=0.43, Td=0.22,
        obstacle=np.array([2.0, 0.0]), safe_radius=0.8)
    assert len(rows) == 2
    assert rows[0]["cmd_safe_map"].shape == (2,)
    assert "h" in rows[0] and "correction_norm" in rows[0]
