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
            'read_trial_metrics', 'wrap_to_pi',
            'time_s', 'distance_m', 'leader_vx_map_ms', 'leader_vy_map_ms',
            'follower_vx_map_ms', 'follower_vy_map_ms', 'leader_yaw_rad',
            'follower_yaw_rad', 'steady_window_s', 'std(', 'summary_table',
            'repmat(struct', 'numel(group.trials) ~= 3',
            'EmptyCsv', 'InvalidData', 'InvalidTime', 'isfinite(values)',
            'homogeneity_metrics_table.xlsx',
            'homogeneity_metrics_table.csv',
            'writecell(export_cells, xlsx_path)',
            "writecell(export_cells, csv_path, 'Encoding', 'UTF-8')",
            '中文指标', '全程距离 MAE (m)', '末 10 s 距离 MAE (m)',
            '全程相对航向 RMS (rad)', '末 10 s 相对航向 RMS (rad)',
            '全程速度差 MAE (m/s)', '末 10 s 速度差 MAE (m/s)',
            "'%.4f ± %.4f'",
        )
        for token in required:
            self.assertIn(token, source)
        self.assertNotIn('exportgraphics', source)
        self.assertNotIn('homogeneity_metrics_table.png', source)
        self.assertNotIn('homogeneity_metrics_table.pdf', source)


if __name__ == '__main__':
    unittest.main()
