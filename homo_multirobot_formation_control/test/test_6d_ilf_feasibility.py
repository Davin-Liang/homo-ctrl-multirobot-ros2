import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest


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


def test_nominal_input_transformation_gives_three_double_integrators():
    feasibility = load_module()
    tau = np.array([0.43, 0.43, 0.43])
    A, B = feasibility.build_local_model(np.zeros(3), tau)

    A_tilde, B_tilde = feasibility.build_nominal_canonical_model()
    state_feedback = np.hstack((np.zeros((3, 3)), np.eye(3)))

    np.testing.assert_allclose(A + B @ state_feedback, A_tilde)
    np.testing.assert_allclose(B @ np.diag(tau), B_tilde)


def test_canonical_control_maps_back_to_deviation_cmd_vel():
    feasibility = load_module()

    delta_u = feasibility.canonical_to_deviation_input(
        xi=np.array([1.0, 2.0, 3.0, 0.1, -0.2, 0.3]),
        nu=np.array([2.0, -1.0, 0.5]),
        tau=np.array([0.5, 0.4, 0.2]),
    )

    np.testing.assert_allclose(delta_u, [1.1, -0.6, 0.4])


def test_nominal_mimo_ilf_synthesis_satisfies_theorem_10_matrix_identity():
    feasibility = load_module()
    design = feasibility.synthesize_nominal_mimo_ilf(mu=0.5)
    A_tilde, B_tilde = feasibility.build_nominal_canonical_model()
    residual = (
        A_tilde @ design.X
        + design.X @ A_tilde.T
        + B_tilde @ design.Y
        + design.Y.T @ B_tilde.T
        + design.H @ design.X
        + design.X @ design.H
    )

    assert np.linalg.eigvalsh(design.X).min() > 1e-6
    assert np.linalg.eigvalsh(design.X @ design.H + design.H @ design.X).min() > 1e-6
    np.testing.assert_allclose(residual, np.zeros((6, 6)), atol=1e-7)
    assert np.linalg.eigvals(A_tilde + B_tilde @ design.K).real.max() < 0.0


def test_robust_mimo_ilf_design_satisfies_theorem_15_matrix_inequality():
    feasibility = load_module()
    design = feasibility.synthesize_robust_nominal_mimo_ilf(0.5, 1e-3)
    A_tilde, B_tilde = feasibility.build_nominal_canonical_model()
    lmi_left = (
        A_tilde @ design.X
        + design.X @ A_tilde.T
        + B_tilde @ design.Y
        + design.Y.T @ B_tilde.T
        + design.H @ design.X
        + design.X @ design.H
        + design.R
    )

    assert design.R.shape == (6, 6)
    assert np.linalg.eigvalsh(lmi_left).max() <= 1e-7
    assert np.linalg.eigvalsh(design.X).min() > 1e-6


def test_matched_disturbance_ratio_is_zero_without_disturbance():
    feasibility = load_module()
    design = feasibility.synthesize_robust_nominal_mimo_ilf(0.5, 1e-3)

    ratio = feasibility.matched_disturbance_ratio(
        np.array([1.0, -0.7, 0.3, 0.0, 0.0, 0.0]),
        np.zeros(3),
        design,
    )

    assert ratio == 0.0


def test_delayed_ilf_zero_delay_has_zero_history_disturbance():
    feasibility = load_module()
    design = feasibility.synthesize_robust_nominal_mimo_ilf(0.5, 1e-3)
    result = feasibility.simulate_delayed_ilf(
        x0=np.array([0.2, 0.0, 0.0, 0.0, 0.0, 0.0]),
        design=design,
        tau=np.array([0.43, 0.43, 0.43]),
        delay=0.0,
        duration=0.1,
        dt=0.001,
    )

    np.testing.assert_allclose(result["w_d"], 0.0, atol=1e-12)
    np.testing.assert_allclose(result["ratio"], 0.0, atol=1e-12)


def test_delayed_ilf_rejects_delay_not_on_simulation_grid():
    feasibility = load_module()
    design = feasibility.synthesize_robust_nominal_mimo_ilf(0.5, 1e-3)

    with pytest.raises(ValueError, match="integer multiple"):
        feasibility.simulate_delayed_ilf(
            np.zeros(6),
            design,
            np.array([0.43, 0.43, 0.43]),
            delay=0.0005,
            duration=0.1,
            dt=0.001,
        )


def test_implicit_lyapunov_root_and_controller_satisfy_the_nominal_identity():
    feasibility = load_module()
    design = feasibility.synthesize_nominal_mimo_ilf(mu=0.5)
    xi = np.array([1.0, -0.7, 0.3, 0.0, 0.0, 0.0])

    value = feasibility.implicit_lyapunov_value(xi, design)
    nu = feasibility.nominal_ilf_control(xi, design)
    dilation = np.diag(
        [value ** -(1.0 + design.mu)] * 3 + [value**-1.0] * 3
    )
    q_value = xi @ dilation @ design.P @ dilation @ xi - 1.0
    A_tilde, B_tilde = feasibility.build_nominal_canonical_model()
    finite_difference = (
        feasibility.implicit_lyapunov_value(
            xi + 1e-6 * (A_tilde @ xi + B_tilde @ nu), design
        )
        - value
    ) / 1e-6

    assert value > 0.0
    np.testing.assert_allclose(q_value, 0.0, atol=1e-8)
    np.testing.assert_allclose(
        finite_difference,
        -value ** (1.0 - design.mu),
        rtol=2e-3,
        atol=2e-4,
    )


def test_continuous_nominal_simulation_decreases_implicit_lyapunov_value():
    feasibility = load_module()
    design = feasibility.synthesize_nominal_mimo_ilf(mu=0.5)

    result = feasibility.simulate_nominal_ilf(
        x0=np.array([1.0, -0.7, 0.3, 0.0, 0.0, 0.0]),
        design=design,
        duration=4.0,
        max_step=1e-3,
    )

    assert result["lyapunov"][-1] < result["lyapunov"][0] * 1e-3
    assert np.linalg.norm(result["state"][-1]) < 2e-3


def test_write_nominal_ilf_csv_has_expected_header_and_lf_endings(tmp_path):
    feasibility = load_module()
    design = feasibility.synthesize_nominal_mimo_ilf(mu=0.5)
    result = feasibility.simulate_nominal_ilf(
        np.array([0.2, 0.0, 0.0, 0.0, 0.0, 0.0]),
        design,
        duration=0.1,
        max_step=1e-3,
    )
    output = tmp_path / "nominal.csv"

    feasibility.write_nominal_ilf_csv(result, output)

    assert output.read_text(encoding="utf-8").splitlines()[0] == (
        "time,e_x,e_y,e_theta,e_vx,e_vy,e_omega,V,nu_x,nu_y,nu_omega"
    )
    assert b"\r\n" not in output.read_bytes()


def test_write_delayed_ilf_audit_csv_has_expected_header_and_lf_endings(tmp_path):
    feasibility = load_module()
    output = tmp_path / "audit.csv"

    feasibility.write_delayed_ilf_audit_csv(
        [
            {
                "delay": 0.0,
                "final_state_norm": 0.0,
                "final_V": 0.0,
                "max_state_norm": 1.0,
                "max_ratio": 0.0,
                "ratio_samples_below_one": True,
            }
        ],
        output,
    )

    assert output.read_text(encoding="utf-8").splitlines()[0] == (
        "delay,final_state_norm,final_V,max_state_norm,max_ratio,"
        "ratio_samples_below_one"
    )
    assert b"\r\n" not in output.read_bytes()


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
