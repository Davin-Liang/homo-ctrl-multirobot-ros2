import importlib.util
import sys
from pathlib import Path

import numpy as np


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "sim_4d_hpc_artstein_compare.py"


def load_module():
    spec = importlib.util.spec_from_file_location("sim_4d_hpc_artstein_compare", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_first_order_prediction_matches_closed_form():
    simulation = load_module()
    state = np.array([1.0, -2.0, 0.4, -0.6])
    command = np.array([1.2, 0.2])
    tau = 0.5
    predicted = simulation.predict_follower_state_first_order(state, command, tau)
    decay = np.exp(-1.0)
    expected_velocity = command + decay * (state[2:4] - command)
    expected_position = state[:2] + tau * command + tau * (1.0 - decay) * (state[2:4] - command)
    np.testing.assert_allclose(predicted, np.r_[expected_position, expected_velocity])


def test_prediction_only_cases_keep_delayed_plant_and_return_samples():
    simulation = load_module()
    delay_rows = simulation.simulate_delay_case("forward_prediction_only", 0.10, 0.01, 0.43, 0.22)
    circle_rows = simulation.simulate_circle_case("forward_prediction_only", 0.10, 0.01, 0.43, 0.22)
    assert len(delay_rows) == 10
    assert len(circle_rows) == 10
    np.testing.assert_allclose(delay_rows[0][3], delay_rows[0][1])
    np.testing.assert_allclose(circle_rows[0][3], circle_rows[0][1])


def test_three_group_summary_and_plot_include_prediction_only(tmp_path):
    simulation = load_module()
    original = simulation.simulate_circle_case("original", 0.10, 0.01, 0.43, 0.22)
    prediction_only = simulation.simulate_circle_case("forward_prediction_only", 0.10, 0.01, 0.43, 0.22)
    compensated = simulation.simulate_circle_case("compensated", 0.10, 0.01, 0.43, 0.22)
    plot = simulation.plot_circle_compare("no noise", original, prediction_only, compensated, tmp_path)
    summary = simulation.write_summary(tmp_path / "summary.csv", {
        "circle_original_delay_clean": original,
        "circle_forward_prediction_only_clean": prediction_only,
        "circle_artstein_prediction_clean": compensated,
    })
    assert plot.exists()
    assert "circle_forward_prediction_only_clean" in summary.read_text(encoding="utf-8")
