import importlib.util
import sys
from pathlib import Path

from matplotlib.axes import Axes
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


def test_real_follower_applies_every_command_after_delay_and_motor_lag():
    simulation = load_module()
    h = 0.01
    tau = 0.43
    Td = 0.22
    cases = (
        (simulation.simulate_delay_case, np.array([5.0, 1.0, 0.0, 0.0])),
        (simulation.simulate_circle_case, np.array([4.5, 0.0, 0.0, 0.0])),
    )

    for simulate, initial_follower in cases:
        rows = simulate("forward_prediction_only", 0.30, h, tau, Td)
        previous_follower = initial_follower.copy()
        delayed_commands = [initial_follower[2:4].copy()] * int(np.ceil(Td / h))
        for row in rows:
            command = row[5]
            delayed_command = delayed_commands.pop(0)
            delayed_commands.append(command)
            expected_follower = previous_follower.copy()
            expected_follower[2:4] += h * (
                delayed_command - expected_follower[2:4]
            ) / tau
            expected_follower[:2] += h * expected_follower[2:4]
            np.testing.assert_allclose(row[2], expected_follower)
            previous_follower = row[2]


def test_prediction_only_uses_each_measured_state_without_artstein_or_td():
    simulation = load_module()
    h = 0.01
    tau = 0.43
    Td = 0.22
    pos_noise = 0.02
    vel_noise = 0.03
    rows = simulation.simulate_circle_case(
        "forward_prediction_only", 0.10, h, tau, Td, pos_noise, vel_noise, seed=11
    )

    rng = np.random.default_rng(11)
    follower = np.array([4.5, 0.0, 0.0, 0.0])
    simulation.add_measurement_noise(simulation.circle_leader_state(0.0), pos_noise, vel_noise, rng)
    simulation.add_measurement_noise(follower, pos_noise, vel_noise, rng)
    last_command = follower[2:4].copy()

    for index, row in enumerate(rows):
        leader_measured = simulation.add_measurement_noise(
            simulation.circle_leader_state(index * h), pos_noise, vel_noise, rng
        )
        follower_measured = simulation.add_measurement_noise(follower, pos_noise, vel_noise, rng)
        np.testing.assert_allclose(row[3], leader_measured)
        np.testing.assert_allclose(
            row[4],
            simulation.predict_follower_state_first_order(follower_measured, last_command, tau),
        )
        follower = row[2]
        last_command = row[5]


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


def test_existing_csv_case_names_and_plot_labels_are_preserved(tmp_path, monkeypatch):
    simulation = load_module()
    labels = []
    original_plot = Axes.plot

    def record_labels(self, *args, **kwargs):
        if "label" in kwargs:
            labels.append(kwargs["label"])
        return original_plot(self, *args, **kwargs)

    monkeypatch.setattr(Axes, "plot", record_labels)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            str(SCRIPT_PATH), "--out-dir", str(tmp_path), "--tmax", "0.01",
            "--circle-tmax", "0.01", "--dt", "0.01",
        ],
    )
    simulation.main()

    csv_lines = (tmp_path / "summary_metrics.csv").read_text(encoding="utf-8").splitlines()
    csv_names = {line.split(",", 1)[0] for line in csv_lines[1:]}
    assert {
        "matlab_leader_original_delay",
        "matlab_leader_artstein_prediction",
        "circle_original_delay_clean",
        "circle_artstein_prediction_clean",
        "circle_original_delay_noise",
        "circle_artstein_prediction_noise",
    } <= csv_names
    assert {
        "original + delay",
        "Artstein + prediction",
        "original 4D + delay",
    } <= set(labels)
    assert {
        "paper_lpc_hpc_distance_square_reproduction.png",
        "delay_original_vs_artstein_prediction.png",
        "circle_original_vs_artstein_clean.png",
        "circle_original_vs_artstein_noise.png",
    } <= {path.name for path in tmp_path.iterdir()}
