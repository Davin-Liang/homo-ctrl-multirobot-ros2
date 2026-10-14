import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = (
    REPO_ROOT
    / "homo_multirobot_formation_control"
    / "robot_traj"
    / "real"
    / "plot_homogeneous_linear_xy.m"
)


class PlotHomogeneousLinearXYTest(unittest.TestCase):
    def test_script_declares_selected_trials_and_outputs(self):
        source = SCRIPT.read_text(encoding="utf-8")
        for required in (
            "homogeneous_trial_dir",
            "linear_trial_dir",
            "6d_artstein_no_forw_vs_forw_20260922_112130",
            "6d_artstein_lpc_vs_hpc(no_fora)_20260922_150051",
            "leader_x_m",
            "leader_y_m",
            "follower_x_m",
            "follower_y_m",
            "homogeneous_vs_linear_xy.png",
            "homogeneous_vs_linear_xy.pdf",
        ):
            self.assertIn(required, source)


if __name__ == "__main__":
    unittest.main()
