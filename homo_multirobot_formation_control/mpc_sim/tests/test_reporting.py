import csv
import sys
from pathlib import Path


PACKAGE_PARENT = Path(__file__).resolve().parents[2]
if str(PACKAGE_PARENT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_PARENT))

from mpc_sim.reporting import compute_metrics, write_case_results
from mpc_sim.run_experiments import run_all
from mpc_sim.scenarios import make_static_scenario
from mpc_sim.simulation import default_config, run_case


def test_metrics_and_csv_include_required_delay_experiment_fields(tmp_path):
    log = run_case(make_static_scenario(duration=0.2), 6, 6, default_config())
    metrics = compute_metrics(log)
    assert {"position_rmse", "position_max", "position_tail_mean", "constraint_violations", "solve_ms_mean"} <= metrics.keys()
    write_case_results(tmp_path, log, metrics)
    with (tmp_path / "timeseries.csv").open(newline="") as stream:
        columns = set(csv.DictReader(stream).fieldnames or [])
    assert {"time", "ex", "ey", "e_theta", "uf_x", "uf_y", "uf_w", "solve_ms", "qp_status"} <= columns
    assert (tmp_path / "trajectory.png").is_file()
    assert (tmp_path / "errors.png").is_file()


def test_short_full_run_writes_both_circle_comparison_cases(tmp_path):
    run_all(tmp_path, duration_scale=0.02)
    for name in ("circle_ordinary_mpc", "circle_delay_aware"):
        assert (tmp_path / name / "trajectory.png").is_file()
        assert (tmp_path / name / "metrics.csv").is_file()
