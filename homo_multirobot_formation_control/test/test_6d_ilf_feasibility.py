import importlib.util
import sys
from pathlib import Path

import numpy as np


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "ilf_6d_feasibility.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("ilf_6d_feasibility", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_build_local_model_includes_leader_coupling_and_actuator_poles():
    feasibility = load_module()
    A, B = feasibility.build_local_model(
        rho=np.array([0.4, -0.2, 0.3]),
        tau=np.array([0.5, 0.4, 0.25]),
    )

    assert A.shape == (6, 6)
    assert B.shape == (6, 3)
    np.testing.assert_allclose(A[0, :], [0.0, 0.3, 0.2, 1.0, 0.0, 0.0])
    np.testing.assert_allclose(A[1, :], [-0.3, 0.0, 0.4, 0.0, 1.0, 0.0])
    np.testing.assert_allclose(np.diag(A)[3:], [-2.0, -2.5, -4.0])
    np.testing.assert_allclose(B[3:, :], np.diag([2.0, 2.5, 4.0]))
    np.testing.assert_allclose(B[:3, :], np.zeros((3, 3)))


def test_nominal_actuator_aware_6d_model_is_full_rank_controllable():
    feasibility = load_module()
    A, B = feasibility.build_local_model(
        np.zeros(3),
        np.array([0.43, 0.43, 0.43]),
    )

    result = feasibility.controllability_diagnostics(A, B)

    assert result["rank"] == 6
    assert result["sigma_min"] > 0.0
    assert np.isfinite(result["condition_number"])


def test_scan_envelope_returns_one_full_rank_row_for_single_operating_point():
    feasibility = load_module()

    rows = feasibility.scan_envelope(
        vx_values=[0.0],
        vy_values=[0.0],
        omega_values=[0.0],
        tau_values=[(0.43, 0.43, 0.43)],
    )

    assert len(rows) == 1
    assert rows[0]["rank"] == 6
    assert rows[0]["tau_x"] == 0.43


def test_write_csv_uses_lf_line_endings(tmp_path):
    feasibility = load_module()
    rows = feasibility.scan_envelope(
        vx_values=[0.0],
        vy_values=[0.0],
        omega_values=[0.0],
        tau_values=[(0.43, 0.43, 0.43)],
    )
    output = tmp_path / "scan.csv"

    feasibility.write_csv(rows, output)

    assert b"\r\n" not in output.read_bytes()
