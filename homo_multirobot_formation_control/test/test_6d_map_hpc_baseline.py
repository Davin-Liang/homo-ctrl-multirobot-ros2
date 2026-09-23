import unittest
from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE_DIR / "config" / "formation_single_follower_6d_map_hpc.yaml"
LAUNCH = PACKAGE_DIR / "launch" / "formation_single_follower_6d_map_hpc.launch.py"
SOURCE = PACKAGE_DIR / "src" / "formation_control_node_6d_map_hpc.cpp"

COMPENSATION_PARAMETERS = (
    "tau",
    "tau_yaw",
    "Td",
    "enable_leader_cmd_feedforward",
    "leader_cmd_timeout",
    "leader_cmd_delta_lpf_tau",
)


class MapHpcBaselineTest(unittest.TestCase):
    def test_baseline_config_and_launch_exclude_compensation_parameters(self):
        params = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))["/**"]["ros__parameters"]
        launch_source = LAUNCH.read_text(encoding="utf-8")
        for name in COMPENSATION_PARAMETERS:
            self.assertNotIn(name, params)
            self.assertNotIn(f'LaunchConfiguration("{name}")', launch_source)

    def test_baseline_control_path_uses_measured_states_directly(self):
        source = SOURCE.read_text(encoding="utf-8")
        self.assertIn("ctrl_->command(leader.x, follower.x)", source)
        self.assertNotIn("ArtsteinPredictorNd", source)
        self.assertNotIn("predict_follower_state", source)


if __name__ == "__main__":
    unittest.main()
