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


def test_real_follower_drains_every_command_through_delay_and_motor_lag():
    simulation = load_module()
    h = 0.01
    tau = 0.43
    Td = 0.22
    cases = (
        (simulation.simulate_delay_case, np.array([5.0, 1.0, 0.0, 0.0])),
        (simulation.simulate_circle_case, np.array([4.5, 0.0, 0.0, 0.0])),
    )

    for simulate, initial_follower in cases:
        # Run a 0.30 s command window plus Td so commands can reach the plant.
        # The local queue drain below also accounts for the commands generated in
        # the final Td window, so no command is left as an unverified queue tail.
        rows = simulate("forward_prediction_only", Td + 0.30, h, tau, Td)
        assert len(rows) == int(round((Td + 0.30) / h))
        previous_follower = initial_follower.copy()
        delayed_commands = [initial_follower[2:4].copy()] * int(np.ceil(Td / h))
        applied_commands = []
        for row in rows:
            command = row[5]
            delayed_command = delayed_commands.pop(0)
            delayed_commands.append(command)
            applied_commands.append(delayed_command)
            expected_follower = previous_follower.copy()
            expected_follower[2:4] += h * (
                delayed_command - expected_follower[2:4]
            ) / tau
            expected_follower[:2] += h * expected_follower[2:4]
            np.testing.assert_allclose(row[2], expected_follower)
            previous_follower = row[2]

        # All recorded plant inputs have the correct command order, and flushing
        # the remaining Td queue exposes every final generated command exactly once.
        queued_tail = delayed_commands.copy()
        delivered_commands = applied_commands[int(np.ceil(Td / h)):] + queued_tail
        np.testing.assert_allclose(delivered_commands, [row[5] for row in rows])
        for delayed_command in queued_tail:
            previous_follower[2:4] += h * (
                delayed_command - previous_follower[2:4]
            ) / tau
            previous_follower[:2] += h * previous_follower[2:4]


def test_delay_prediction_only_uses_measurements_and_first_order_follower_prediction():
    simulation = load_module()
    h = 0.01
    tau = 0.43
    Td = 0.22
    rows = simulation.simulate_delay_case("forward_prediction_only", 0.10, h, tau, Td)

    leader = np.array([1.0, 0.0, 0.0, 0.0])
    follower = np.array([5.0, 1.0, 0.0, 0.0])
    A_di = np.block([[np.zeros((2, 2)), np.eye(2)], [np.zeros((2, 2)), np.zeros((2, 2))]])
    B_di = np.vstack([np.zeros((2, 2)), np.eye(2) / 2.0])
    last_command = follower[2:4].copy()

    for index, row in enumerate(rows):
        leader = leader + h * (A_di @ leader + B_di @ simulation.matlab_leader_accel(index * h, leader))
        np.testing.assert_allclose(row[3], leader)
        np.testing.assert_allclose(
            row[4],
            simulation.predict_follower_state_first_order(follower, last_command, tau),
        )
        follower = row[2]
        last_command = row[5]


def test_circle_prediction_only_uses_each_measured_state_without_artstein_or_td():
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


def test_original_and_compensated_short_runs_are_numerically_unchanged():
    simulation = load_module()
    expected_last_samples = {
        ("simulate_delay_case", "original"): np.array([
            5.0, 1.0, 0.0, 0.0, 0.999702998667, 0.000298983763,
            -0.019550533082, 0.019846009585, 5.0, 1.0, 0.0, 0.0,
            -0.055954845729, -0.017927646756,
        ]),
        ("simulate_delay_case", "compensated"): np.array([
            5.0, 1.0, 0.0, 0.0, 0.986995152163, 0.013198889993,
            -0.019550533082, 0.019846009585, 4.980561332032, 0.993725210992,
            -0.073254135942, -0.023636399611, -0.127093391762, -0.040567335164,
        ]),
        ("simulate_circle_case", "original"): np.array([
            4.5, 0.0, 0.0, 0.0, 1.999943750264, 0.014999859375,
            -0.003749964844, 0.499985937566, 4.5, 0.0, 0.0, 0.0,
            -0.027811934065, 0.013868113892,
        ]),
        ("simulate_circle_case", "compensated"): np.array([
            4.5, 0.0, 0.0, 0.0, 1.997506273115, 0.339990718793,
            -0.003749964844, 0.499985937566, 4.490335496424, 0.006859612655,
            -0.036418635743, 0.025851215137, -0.063107106391, 0.044893344218,
        ]),
    }

    for (function_name, kind), expected in expected_last_samples.items():
        rows = getattr(simulation, function_name)(kind, 0.04, 0.01, 0.43, 0.22)
        row = rows[-1]
        np.testing.assert_allclose(np.r_[row[2], row[3], row[4], row[5]], expected, rtol=1e-10, atol=1e-10)


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


def test_existing_csv_case_names_and_plot_legends_are_preserved(tmp_path, monkeypatch):
    simulation = load_module()
    legends = {}
    original_legend = Axes.legend

    def record_legend(self, *args, **kwargs):
        legends.setdefault(self.get_title(), tuple(line.get_label() for line in self.lines))
        return original_legend(self, *args, **kwargs)

    monkeypatch.setattr(Axes, "legend", record_legend)
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
    csv_names = [line.split(",", 1)[0] for line in csv_lines[1:]]
    assert len(csv_names) == len(set(csv_names))
    existing_names = [name for name in csv_names if "forward_prediction_only" not in name]
    assert existing_names == [
        "ideal_4d_hpc_matlab",
        "matlab_leader_original_delay",
        "matlab_leader_artstein_prediction",
        "circle_original_delay_clean",
        "circle_artstein_prediction_clean",
        "circle_original_delay_noise",
        "circle_artstein_prediction_noise",
    ]
    assert legends["MATLAB leader trajectory"] == (
        "leader", "ideal 4D HPC", "original + delay", "Artstein + prediction",
    )
    assert legends["formation error"] == (
        "ideal 4D HPC", "original + delay", "Artstein + prediction",
    )
    assert legends["circle trajectory (no noise)"] == (
        "leader circle", "original 4D + delay", "Artstein + prediction",
    )
    assert legends["velocity command"] == (
        "orig $v_x^{cmd}$", "orig $v_y^{cmd}$",
        "comp $v_x^{cmd}$", "comp $v_y^{cmd}$",
    )
    assert {
        "paper_lpc_hpc_distance_square_reproduction.png",
        "delay_original_vs_artstein_prediction.png",
        "circle_original_vs_artstein_clean.png",
        "circle_original_vs_artstein_noise.png",
    } <= {path.name for path in tmp_path.iterdir()}
