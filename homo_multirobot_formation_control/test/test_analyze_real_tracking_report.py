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
    assert metrics['tail_mean_abs_distance_error_m'] == 0.1
    assert metrics['tail_distance_error_std_m'] == pytest.approx(0.0816496580927726)


def test_generate_report_assets_writes_the_five_high_resolution_pngs(tmp_path, capsys):
    manifest = (PACKAGE_DIR / "analysis" / "real_tracking_report" /
                "experiment_manifest.yaml")

    assets = module.generate_report_assets(manifest, tmp_path)

    expected = {
        "feedforward_comparison.png",
        "feedforward_distance_error.png",
        "hpc_lpc_trajectory.png",
        "hpc_lpc_distance_error.png",
        "control_pipeline.png",
    }
    assert {path.name for path in assets} == expected
    assert {path.name for path in tmp_path.glob("*.png")} == expected
    assert all(Image.open(path).info["dpi"][0] >= 299 for path in assets)
    assert "Chinese sans-serif font unavailable" in capsys.readouterr().err
