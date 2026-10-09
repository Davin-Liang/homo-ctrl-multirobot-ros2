import unittest
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE_DIR / "src" / "formation_control_node_6d_map_hpc_artstein.cpp"
HEADER = (
    PACKAGE_DIR
    / "include"
    / "homo_multirobot_formation_control"
    / "formation_control_node_6d_map_hpc_artstein.hpp"
)


class Test6DMapHpcArtsteinForwardPrediction(unittest.TestCase):
    def test_custom_horizon_is_applied_to_both_follower_channels(self):
        source = SOURCE.read_text(encoding="utf-8")
        header = HEADER.read_text(encoding="utf-8")

        self.assertIn(
            'declare_parameter("forward_prediction_horizon", 0.0)', source
        )
        self.assertIn("std::max(tau_v_, tau_w_)", source)
        self.assertEqual(
            source.count(
                "enable_forward_prediction_, forward_prediction_horizon_"
            ),
            2,
        )
        self.assertIn("double forward_prediction_horizon_", header)


if __name__ == "__main__":
    unittest.main()
