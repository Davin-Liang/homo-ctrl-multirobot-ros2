import importlib.util
from pathlib import Path

from PIL import Image
import pytest


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = PACKAGE_DIR / "scripts" / "analyze_real_tracking_report.py"
SPEC = importlib.util.spec_from_file_location("analyze_real_tracking_report", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


def test_distance_error_metrics_use_ideal_radius():
    distances = [0.8, 1.0, 1.2]

    metrics = module.compute_distance_metrics(distances, ideal_radius_m=1.0)

    assert metrics['mean_abs_distance_error_m'] == 0.13333333333333333
    assert metrics['rms_distance_error_m'] == 0.16329931618554522
    assert 'final_distance_error_m' not in metrics


def test_tail_metrics_describe_the_last_ten_seconds_of_a_recording():
    rows = [
        {'time_s': 0.0, 'distance_m': 0.8},
        {'time_s': 5.0, 'distance_m': 1.0},
        {'time_s': 10.0, 'distance_m': 1.2},
        {'time_s': 15.0, 'distance_m': 1.1},
    ]

    metrics = module.compute_tail_distance_metrics(rows, ideal_radius_m=1.0,
                                                   window_s=10.0)

    assert metrics['tail_window_s'] == 10.0
    assert metrics['tail_mean_signed_distance_error_m'] == pytest.approx(0.1)
    assert metrics['tail_mean_abs_distance_error_m'] == 0.1
    assert metrics['tail_distance_error_std_m'] == pytest.approx(0.0816496580927726)
    assert metrics['tail_max_abs_distance_error_m'] == pytest.approx(0.2)


def test_velocity_metrics_measure_leader_follower_velocity_mismatch():
    rows = [
        {'time_s': 0.0, 'leader_vx_ms': 1.0, 'leader_vy_ms': 0.0,
         'follower_vx_ms': 0.0, 'follower_vy_ms': 0.0},
        {'time_s': 5.0, 'leader_vx_ms': 1.0, 'leader_vy_ms': 0.0,
         'follower_vx_ms': 1.0, 'follower_vy_ms': 0.0},
        {'time_s': 10.0, 'leader_vx_ms': 0.0, 'leader_vy_ms': 1.0,
         'follower_vx_ms': 0.0, 'follower_vy_ms': 1.0},
        {'time_s': 15.0, 'leader_vx_ms': 0.0, 'leader_vy_ms': 1.0,
         'follower_vx_ms': 0.0, 'follower_vy_ms': 2.0},
    ]

    metrics = module.compute_velocity_tracking_metrics(rows, window_s=10.0)

    assert metrics == {
        'mean_relative_velocity_error_mps': 0.5,
        'tail_mean_relative_velocity_error_mps': pytest.approx(1.0 / 3.0),
    }


def test_hpc_lpc_stage_uses_the_dedicated_hpc_reference_record():
    assert module.stage_experiment_ids() == (
        'hpc_lpc_reference', 'lpc_lambda_25_v025')


def test_artstein_original_4d_comparison_uses_the_requested_records():
    assert module.artstein_original_4d_ids() == (
        'hpc_lpc_reference', 'original_4d_reference')


def test_lpc_plot_label_omits_leader_speed_from_the_stage_report():
    assert module.report_label({
        'id': 'lpc_lambda_25_v025',
        'controller_family': 'artstein_lpc',
        'initial_min_lambda': 2.5,
        'leader_speed_mps': 0.25,
    }, chinese_labels=False) == 'Artstein-LPC (lambda=2.5)'


def test_hpc_stage_plot_label_exposes_the_common_lambda_setting():
    assert module.report_label({
        'id': 'hpc_lpc_reference',
        'controller_family': 'artstein_hpc',
        'initial_min_lambda': 2.5,
    }, chinese_labels=False) == 'Artstein-HPC (lambda=2.5)'


def test_generate_report_assets_writes_the_five_high_resolution_pngs(tmp_path, capsys):
    manifest = (PACKAGE_DIR / "analysis" / "real_tracking_report" /
                "experiment_manifest.yaml")

    assets = module.generate_report_assets(manifest, tmp_path)

    expected = {
        "feedforward_trajectory.png",
        "feedforward_velocity_components.png",
        "feedforward_x_position.png",
        "feedforward_y_position.png",
        "hpc_lpc_trajectory.png",
        "hpc_lpc_velocity_components.png",
        "hpc_lpc_x_position.png",
        "hpc_lpc_y_position.png",
        "artstein_original_4d_trajectory.png",
        "artstein_original_4d_velocity_components.png",
        "artstein_original_4d_x_position.png",
        "artstein_original_4d_y_position.png",
    }
    assert {path.name for path in assets} == expected
    assert {path.name for path in tmp_path.glob("*.png")} == expected
    assert all(Image.open(path).info["dpi"][0] >= 299 for path in assets)
    assert "Chinese sans-serif font unavailable" in capsys.readouterr().err
