import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "hocbf_6d_feasibility.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location(
        "hocbf_6d_feasibility", SCRIPT_PATH
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_exact_predictor_matches_future_delayed_zoh_state():
    feasibility = load_module()
    params = feasibility.PlantParams(
        tau=0.4, delay=0.02, integration_dt=0.01, control_dt=0.05
    )
    ad, bd = feasibility.zoh_matrices(params)
    x0 = np.array([0.0, 0.0, 0.2, -0.1])
    queued = [np.array([0.3, 0.0]), np.array([0.1, -0.2])]

    predicted = feasibility.predict_delayed_state(x0, queued, ad, bd)
    expected = ad @ (ad @ x0 + bd @ queued[0]) + bd @ queued[1]

    np.testing.assert_allclose(predicted, expected, atol=1e-12)


def test_two_time_scale_params_represent_220ms_delay_at_20hz_control():
    feasibility = load_module()

    params = feasibility.PlantParams(
        tau=0.43, delay=0.22, integration_dt=0.01, control_dt=0.05
    )

    assert params.delay_steps == 22
    assert params.control_steps == 5


def test_hocbf_halfspace_tightens_head_on_approach():
    feasibility = load_module()

    a, b, h, psi1 = feasibility.hocbf_halfspace(
        x_pred=np.array([2.0, 0.0, -0.5, 0.0]),
        obstacle=np.zeros(2),
        safe_radius=1.0,
        tau=0.5,
        c1=1.0,
        c2=1.0,
    )

    np.testing.assert_allclose(a, [8.0, 0.0])
    assert h == pytest.approx(3.0)
    assert psi1 == pytest.approx(1.0)
    assert a @ np.array([-1.0, 0.0]) < b
    assert a @ np.zeros(2) >= b


def test_hard_qp_returns_nominal_command_when_feasible():
    feasibility = load_module()

    result = feasibility.solve_hocbf_qp(
        u_nom=np.array([0.3, -0.2]),
        u_prev=np.zeros(2),
        halfspaces=[(np.array([1.0, 0.0]), -1.0)],
        vmax=1.0,
        amax=20.0,
        dt=0.05,
    )

    assert result.feasible
    np.testing.assert_allclose(result.command, [0.3, -0.2])


def test_hard_qp_projects_nominal_command_onto_barrier():
    feasibility = load_module()

    result = feasibility.solve_hocbf_qp(
        u_nom=np.array([-1.0, 0.0]),
        u_prev=np.zeros(2),
        halfspaces=[(np.array([1.0, 0.0]), 0.2)],
        vmax=1.0,
        amax=20.0,
        dt=0.05,
    )

    assert result.feasible
    np.testing.assert_allclose(result.command, [0.2, 0.0], atol=1e-12)


def test_hard_qp_reports_conflicting_barriers_as_infeasible():
    feasibility = load_module()

    result = feasibility.solve_hocbf_qp(
        u_nom=np.zeros(2),
        u_prev=np.zeros(2),
        halfspaces=[
            (np.array([1.0, 0.0]), 0.8),
            (np.array([-1.0, 0.0]), 0.8),
        ],
        vmax=1.0,
        amax=20.0,
        dt=0.05,
    )

    assert not result.feasible


def test_no_obstacle_scenario_preserves_nominal_command():
    feasibility = load_module()

    result = feasibility.simulate_scenario(
        feasibility.ScenarioConfig(
            plant=feasibility.PlantParams(
                tau=0.43, delay=0.22, integration_dt=0.01, control_dt=0.05
            ),
            obstacle=np.array([100.0, 0.0]),
            safe_radius=0.5,
            initial_state=np.zeros(4),
            nominal_command=np.array([0.2, 0.0]),
            vmax=1.0,
            amax=20.0,
            c1=1.0,
            c2=1.0,
            duration=0.5,
        )
    )

    assert not result["braking"].any()
    np.testing.assert_allclose(
        result["command"],
        np.tile(np.array([0.2, 0.0]), (len(result["command"]), 1)),
    )


def test_head_on_feasible_case_keeps_h_nonnegative():
    feasibility = load_module()

    result = feasibility.simulate_scenario(
        feasibility.ScenarioConfig(
            plant=feasibility.PlantParams(
                tau=0.43, delay=0.22, integration_dt=0.01, control_dt=0.05
            ),
            obstacle=np.zeros(2),
            safe_radius=0.8,
            initial_state=np.array([2.0, 0.0, -0.1, 0.0]),
            nominal_command=np.array([-0.8, 0.0]),
            vmax=1.0,
            amax=20.0,
            c1=2.0,
            c2=2.0,
            duration=4.0,
        )
    )

    assert result["feasible"].all()
    assert result["h"].min() >= -1e-9


def test_scan_envelope_has_one_row_per_parameter_combination():
    feasibility = load_module()

    rows = feasibility.scan_envelope(
        tau_values=[0.3, 0.5],
        delay_values=[0.0],
        clearances=[1.0],
        delay_mismatches=[0.0, 0.05],
    )

    assert len(rows) == 4
    assert set(rows[0]) >= {
        "tau",
        "delay_model",
        "delay_actual",
        "initial_clearance",
        "min_h",
        "min_distance",
        "min_psi2",
        "infeasible_steps",
        "braking_steps",
    }


def test_metrics_csv_has_lf_and_expected_header(tmp_path):
    feasibility = load_module()
    output = tmp_path / "scan.csv"

    feasibility.write_metrics_csv(
        [
            {
                "tau": 0.43,
                "delay_model": 0.2,
                "delay_actual": 0.2,
                "initial_clearance": 1.0,
                "min_h": 0.1,
                "min_distance": 0.6,
                "min_psi2": 0.0,
                "max_command_norm": 0.5,
                "infeasible_steps": 0,
                "braking_steps": 0,
            }
        ],
        output,
    )

    assert output.read_text(encoding="utf-8").splitlines()[0].startswith(
        "tau,delay_model,delay_actual,initial_clearance"
    )
    assert b"\r\n" not in output.read_bytes()


def test_sampling_rate_comparison_matches_stationary_safe_case():
    feasibility = load_module()
    config = feasibility.ScenarioConfig(
        plant=feasibility.PlantParams(
            tau=0.43, delay=0.22, integration_dt=0.01, control_dt=0.05
        ),
        obstacle=np.array([2.0, 0.0]),
        safe_radius=0.5,
        initial_state=np.zeros(4),
        nominal_command=np.zeros(2),
        vmax=1.0,
        amax=20.0,
        c1=1.0,
        c2=1.0,
        duration=0.5,
    )

    summary = feasibility.compare_sampling_rates(config, reference_dt=0.001)

    assert summary["control_dt"] == pytest.approx(0.05)
    assert summary["reference_dt"] == pytest.approx(0.001)
    assert summary["min_h_20hz"] == pytest.approx(summary["min_h_1khz"])


def test_predictor_tau_changes_the_predicted_actuator_model_only():
    feasibility = load_module()
    config = feasibility.ScenarioConfig(
        plant=feasibility.PlantParams(
            tau=0.5, delay=0.22, integration_dt=0.01, control_dt=0.05
        ),
        predictor_tau=0.3,
        obstacle=np.zeros(2),
        safe_radius=0.8,
        initial_state=np.array([2.0, 0.0, -0.2, 0.0]),
        nominal_command=np.array([-0.6, 0.0]),
        vmax=1.0,
        amax=20.0,
        c1=2.0,
        c2=2.0,
        duration=0.5,
    )

    result = feasibility.simulate_scenario(config)

    assert result["predictor_tau"] == pytest.approx(0.3)
    assert result["plant_tau"] == pytest.approx(0.5)


def test_predictor_tau_must_be_positive():
    feasibility = load_module()

    with pytest.raises(ValueError, match="predictor_tau"):
        feasibility.ScenarioConfig(
            plant=feasibility.PlantParams(
                tau=0.5, delay=0.22, integration_dt=0.01, control_dt=0.05
            ),
            predictor_tau=0.0,
            obstacle=np.zeros(2),
            safe_radius=0.8,
            initial_state=np.zeros(4),
            nominal_command=np.zeros(2),
            vmax=1.0,
            amax=20.0,
            c1=2.0,
            c2=2.0,
            duration=0.5,
        )


def test_robustness_envelope_has_every_parameter_combination_and_margin():
    feasibility = load_module()

    rows = feasibility.run_robustness_envelope(
        tau_actual_values=[0.43],
        tau_ratios=[1.0, 1.2],
        delay_model_values=[0.22],
        delay_mismatches=[0.0],
        clearances=[0.4],
        radial_speeds=[0.1],
        lateral_speeds=[0.0],
        nominal_speeds=[0.4],
    )

    assert len(rows) == 2
    assert rows[0]["sample_distance_gap"] >= 0.0
    assert rows[0]["exact_model"] == 1
    assert rows[1]["exact_model"] == 0


def test_robustness_summary_reports_all_and_exact_feasible_groups():
    feasibility = load_module()
    rows = [
        {
            "exact_model": 1,
            "infeasible_steps": 0,
            "min_h": 0.1,
            "min_distance": 0.9,
            "sample_distance_gap": 0.02,
        },
        {
            "exact_model": 1,
            "infeasible_steps": 1,
            "min_h": -0.1,
            "min_distance": 0.7,
            "sample_distance_gap": 0.03,
        },
        {
            "exact_model": 0,
            "infeasible_steps": 0,
            "min_h": 0.01,
            "min_distance": 0.81,
            "sample_distance_gap": 0.04,
        },
    ]

    summary = feasibility.summarize_robustness_rows(rows)
    exact = next(row for row in summary if row["group"] == "exact_feasible")

    assert exact["scenario_count"] == 1
    assert exact["max_sample_distance_gap"] == pytest.approx(0.02)
