import importlib.util
from pathlib import Path
import sys
import unittest
from collections import deque
import tempfile
from unittest import mock

import numpy as np


SCRIPT_PATH = Path(__file__).parents[1] / "scripts" / "sim_6d_map_hpc_artstein_compare.py"
SPEC = importlib.util.spec_from_file_location("sim_6d_map_hpc_artstein_compare", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class MapFrameModelTest(unittest.TestCase):
    def test_constant_accel_yaw_keeps_map_translation_and_caps_rate(self):
        state = MODULE.constant_accel_yaw_leader_state(60.0, 2.0, 0.45)
        nominal = MODULE.circle_leader_state(60.0, 2.0, 0.45)
        np.testing.assert_allclose(state[:2], nominal[:2], atol=1e-12)
        np.testing.assert_allclose(
            MODULE.rot(state[2]) @ state[3:5],
            MODULE.rot(nominal[2]) @ nominal[3:5],
            atol=1e-12,
        )
        self.assertAlmostEqual(state[5], 0.8, places=12)

    def test_periodic_accel_yaw_has_expected_rate_offset(self):
        time = np.pi / 0.8
        state = MODULE.periodic_accel_yaw_leader_state(time, 2.0, 0.45)
        nominal = MODULE.circle_leader_state(time, 2.0, 0.45)
        self.assertAlmostEqual(state[5] - nominal[5], 0.2, places=12)

    def test_yaw_step_changes_only_leader_yaw_reference(self):
        before = MODULE.yaw_step_leader_state(29.999, 2.0, 0.45)
        after = MODULE.yaw_step_leader_state(30.001, 2.0, 0.45)
        nominal_after = MODULE.circle_leader_state(30.001, 2.0, 0.45)
        np.testing.assert_allclose(after[:2], nominal_after[:2], atol=1e-12)
        np.testing.assert_allclose(
            MODULE.rot(after[2]) @ after[3:5],
            MODULE.rot(nominal_after[2]) @ nominal_after[3:5],
            atol=1e-12,
        )
        self.assertAlmostEqual(
            MODULE.wrap_angle(after[2] - nominal_after[2]), np.pi / 2.0, places=12
        )
        self.assertLess(abs(MODULE.wrap_angle(before[2] - MODULE.circle_leader_state(29.999, 2.0, 0.45)[2])), 1e-12)

    def test_unknown_yaw_step_is_not_visible_before_it_happens(self):
        horizon = 0.65
        before_time = 29.5
        before = MODULE.yaw_step_leader_state(before_time, 2.0, 0.45)
        before_prediction = MODULE.predict_leader_from_observation(
            before, before_time, horizon, 2.0, 0.45
        )
        nominal_future = MODULE.circle_leader_state(before_time + horizon, 2.0, 0.45)
        self.assertLess(abs(MODULE.wrap_angle(before_prediction[2] - nominal_future[2])), 1e-12)

        after_time = 30.05
        after = MODULE.yaw_step_leader_state(after_time, 2.0, 0.45)
        after_prediction = MODULE.predict_leader_from_observation(
            after, after_time, horizon, 2.0, 0.45
        )
        nominal_future = MODULE.circle_leader_state(after_time + horizon, 2.0, 0.45)
        self.assertAlmostEqual(
            MODULE.wrap_angle(after_prediction[2] - nominal_future[2]), np.pi / 2.0, places=12
        )

    def test_map_error_is_zero_at_fixed_map_offset(self):
        leader = np.array([1.0, -2.0, 0.4, 0.2, -0.1, 0.3])
        offset = np.array([-1.0, 0.5])
        follower = np.array([0.0, -1.5, leader[2], 0.0, 0.0, 0.3])
        follower[3:5] = MODULE.rot(follower[2]).T @ (
            MODULE.rot(leader[2]) @ leader[3:5]
        )
        np.testing.assert_allclose(
            MODULE.map_error(leader, follower, offset), np.zeros(6), atol=1e-12
        )

    def test_nominal_homogeneous_identities_are_machine_precision(self):
        values = MODULE.verify_nominal_identities(2.0, 1.0, -0.25, 1.2, 2.0)
        self.assertEqual(values["controllability_rank"], 6)
        self.assertLess(
            max(value for key, value in values.items() if key != "controllability_rank"),
            1e-10,
        )

    def test_regularized_hpc_returns_zero_for_zero_error(self):
        ctrl = MODULE.RegularizedMapHpc(2.0, 1.0, -0.25, 1.2, 2.0, 0.5)
        np.testing.assert_allclose(ctrl.command(np.zeros(6)), np.zeros(3), atol=1e-12)

    def test_predictor_matches_measurement_without_delay_or_lag(self):
        state = np.array([0.2, -0.1, 0.3, 0.4, -0.2, 0.1])
        np.testing.assert_allclose(
            MODULE.predict_map_state(state, deque(), 0.0, 0.0, 0.05),
            state,
            atol=1e-12,
        )

    def test_artstein_predictor_uses_full_delay_window_history(self):
        state = np.array([0.2, -0.1, 0.3, 0.4, -0.2, 0.1])
        recent = np.array([0.3, -0.1, 0.2])
        history_a = deque([recent, np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3)])
        history_b = deque([recent, np.array([0.8, 0.0, 0.0]), np.zeros(3), np.zeros(3), np.zeros(3)])
        prediction_a = MODULE.predict_map_state(state, history_a, 0.22, 0.43, 0.05)
        prediction_b = MODULE.predict_map_state(state, history_b, 0.22, 0.43, 0.05)
        self.assertGreater(np.linalg.norm(prediction_a - prediction_b), 1e-4)

    def test_cases_share_initial_state_and_delayed_plant(self):
        config = MODULE.SimulationConfig(tmax=0.10)
        delayed = MODULE.simulate_case("delayed", config)
        artstein = MODULE.simulate_case("artstein", config)
        np.testing.assert_allclose(delayed.initial_follower, artstein.initial_follower)
        self.assertEqual(delayed.td, artstein.td)
        self.assertEqual(delayed.td, config.td)

    def test_recorded_error_uses_leader_at_the_same_timestamp(self):
        config = MODULE.SimulationConfig(tmax=0.05)
        result = MODULE.simulate_case("ideal", config)
        leader_at_sample = MODULE.circle_leader_state(
            result.time[0], config.leader_radius, config.leader_speed
        )
        expected = MODULE.map_error(leader_at_sample, result.follower[0], np.asarray(config.offset_map))
        np.testing.assert_allclose(result.error[0], expected, atol=1e-12)

    def test_artstein_integrates_force_from_predicted_follower_state(self):
        predicted = np.array([3.8, -0.5, 0.0, 0.7, -0.1, 0.3])

        class ZeroController:
            def command(self, _error):
                return np.zeros(3)

        with mock.patch.object(MODULE, "RegularizedMapHpc", return_value=ZeroController()), \
             mock.patch.object(MODULE, "predict_map_state", return_value=predicted):
            result = MODULE.simulate_case("artstein", MODULE.SimulationConfig(tmax=0.05))
        np.testing.assert_allclose(result.command_map[0], np.array([0.7, -0.1, 0.3]), atol=1e-12)

    def test_run_writes_the_four_required_artifacts(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = MODULE.run_experiment(MODULE.SimulationConfig(
                tmax=0.10, output_dir=Path(directory)
            ))
            self.assertEqual(
                {path.name for path in paths},
                {"comparison.png", "summary_metrics.csv", "timeseries.csv", "diagnostics.txt"},
            )
            self.assertTrue(all(path.exists() for path in paths))

    def test_yaw_step_run_reports_post_step_metrics(self):
        with tempfile.TemporaryDirectory() as directory:
            MODULE.run_experiment(MODULE.SimulationConfig(
                tmax=31.0, output_dir=Path(directory)
            ))
            header = (Path(directory) / "summary_metrics.csv").read_text().splitlines()[0]
        self.assertIn("post_step_peak_yaw_error", header)

    def test_continuous_yaw_runner_writes_two_scenario_summaries(self):
        with tempfile.TemporaryDirectory() as directory:
            outputs = MODULE.run_continuous_yaw_experiments(Path(directory))
            self.assertEqual(set(outputs), {"constant_yaw_accel", "periodic_yaw_accel"})
            self.assertTrue(all(
                (Path(directory) / name / "summary_metrics.csv").exists()
                for name in outputs
            ))


if __name__ == "__main__":
    unittest.main()
