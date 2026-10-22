import unittest
from pathlib import Path


SCRIPT = (
    Path(__file__).parents[1]
    / 'robot_traj'
    / 'real'
    / 'plot_homogeneity_metrics_table.m'
)


class PlotHomogeneityMetricsTableTest(unittest.TestCase):
    def test_contract(self):
        source = SCRIPT.read_text(encoding='utf-8')
        required = (
            'homogeneity_groups', '-1.0', '-0.9', '-0.8', '-0.7', '-0.6',
            '-0.5', '-0.4', '-0.3', '-0.2', '-0.1', '0.0',
            'full_distance_mae', 'steady_distance_mae',
            'full_yaw_rms', 'steady_yaw_rms',
            'full_velocity_mae', 'steady_velocity_mae',
            'read_trial_metrics', 'wrap_to_pi', 'draw_metrics_table',
            'axes', 'text(', 'rectangle(',
            'exportgraphics', 'homogeneity_metrics_table.png',
            'homogeneity_metrics_table.pdf',
            'time_s', 'distance_m', 'leader_vx_map_ms', 'leader_vy_map_ms',
            'follower_vx_map_ms', 'follower_vy_map_ms', 'leader_yaw_rad',
            'follower_yaw_rad', 'steady_window_s', 'std(', 'summary_table',
            'Full Distance MAE (m)', 'Steady Distance MAE (m)',
            'Full Yaw RMS (rad)', 'Steady Yaw RMS (rad)',
            'Full Velocity MAE (m/s)', 'Steady Velocity MAE (m/s)',
            "'Resolution', 300", 'repmat(struct', 'numel(group.trials) ~= 3',
            'EmptyCsv', 'InvalidData', 'InvalidTime', 'isfinite(values)',
            'header_labels', "sprintf('Full Distance\\nMAE (m)')",
        )
        for token in required:
            self.assertIn(token, source)


if __name__ == '__main__':
    unittest.main()
