import unittest
from pathlib import Path

import yaml


REPO_DIR = Path(__file__).resolve().parents[2]
CONFIG_DIR = REPO_DIR / "homo_multirobot_formation_control" / "config"

LAUNCH_CONFIGS = {
    "sim_rf2o_ekf_two_robots.launch.yaml": {
        "launch": REPO_DIR / "homo_multirobot_localization" / "launch"
        / "sim_rf2o_ekf_two_robots.launch.py",
        "keys": {
            "use_sim_time", "world_name", "gui", "server", "verbose",
            "software_rendering", "use_rviz", "publish_world_tf",
            "robot1_namespace", "robot2_namespace", "robot1_prefix",
            "robot2_prefix", "robot1_x", "robot1_y", "robot1_z",
            "robot1_yaw", "robot2_x", "robot2_y", "robot2_z",
            "robot2_yaw", "planar_publish_odom", "planar_publish_odom_tf",
            "rf2o_publish_tf",
        },
    },
    "slam_toolbox_loc_two_robots.launch.yaml": {
        "launch": REPO_DIR / "homo_multirobot_nav" / "launch"
        / "slam_toolbox_loc_two_robots.launch.py",
        "keys": {
            "use_sim_time", "map_name", "use_rviz", "initialpose_to",
            "global_frame_id", "scan_topic", "robot1_namespace",
            "robot2_namespace", "robot1_prefix", "robot2_prefix",
            "robot1_map_start_x", "robot1_map_start_y", "robot1_map_start_yaw",
            "robot2_map_start_x", "robot2_map_start_y", "robot2_map_start_yaw",
        },
    },
}


class TestTwoRobotLaunchConfigDefaults(unittest.TestCase):
    def test_config_keys_and_launch_yaml_loading(self):
        for config_name, expected in LAUNCH_CONFIGS.items():
            with self.subTest(config=config_name):
                config_path = CONFIG_DIR / config_name
                self.assertTrue(config_path.is_file())
                defaults = yaml.safe_load(config_path.read_text(encoding="utf-8"))["launch_defaults"]
                self.assertEqual(set(defaults), expected["keys"])

                source = expected["launch"].read_text(encoding="utf-8")
                self.assertIn('get_package_share_directory("homo_multirobot_formation_control")', source)
                self.assertIn(config_name, source)
                self.assertIn("yaml.safe_load", source)
                self.assertIn("_launch_default(defaults[name])", source)


if __name__ == "__main__":
    unittest.main()
