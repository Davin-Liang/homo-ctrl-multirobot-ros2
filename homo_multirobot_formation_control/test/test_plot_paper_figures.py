import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = (
    REPO_ROOT
    / "homo_multirobot_formation_control"
    / "robot_traj"
    / "real"
    / "plot_paper_figures.m"
)


class PlotPaperFiguresTest(unittest.TestCase):
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
            "leader_vx_map_ms",
            "leader_vy_map_ms",
            "follower_vx_map_ms",
            "follower_vy_map_ms",
            "leader_yaw_rad",
            "follower_yaw_rad",
            "ideal_radius_m",
            "plot_sample_interval_s",
            "no_compensation_trial_dir",
            "artstein_only_trial_dir",
            "forward_only_trial_dir",
            "artstein_forward_trial_dir",
            "homogeneous_vs_linear_xy.png",
            "homogeneous_vs_linear_position_error.png",
            "homogeneous_vs_linear_velocity_error.png",
            "homogeneous_vs_linear_yaw_error.png",
            "four_compensation_xy.png",
            "four_compensation_position_error.png",
            "four_compensation_velocity_error.png",
            "four_compensation_yaw_error.png",
        ):
            self.assertIn(required, source)


if __name__ == "__main__":
    unittest.main()
