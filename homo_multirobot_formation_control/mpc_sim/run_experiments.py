#!/usr/bin/env python3
"""Run all requested delay-aware MPC numerical experiments."""

import argparse
import sys
from datetime import datetime
from pathlib import Path

PACKAGE_PARENT = Path(__file__).resolve().parents[1]
if str(PACKAGE_PARENT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_PARENT))

from mpc_sim.reporting import compute_metrics, write_case_results
from mpc_sim.scenarios import (
    make_circle_scenario,
    make_curve_scenario,
    make_static_scenario,
    make_straight_scenario,
)
from mpc_sim.simulation import default_config, run_case, strict_default_config


def run_all(output_dir: Path, duration_scale: float = 1.0) -> Path:
    if duration_scale <= 0.0:
        raise ValueError("duration_scale must be positive")
    output_dir.mkdir(parents=True, exist_ok=True)
    base = default_config()
    cases = [
        ("static_delay_aware", make_static_scenario(duration=12.0 * duration_scale), 6, 6, base),
        ("straight_delay_aware", make_straight_scenario(duration=16.0 * duration_scale), 6, 6, base),
        ("curve_ordinary_mpc", make_curve_scenario(duration=20.0 * duration_scale), 0, 6, base),
        ("curve_delay_aware", make_curve_scenario(duration=20.0 * duration_scale), 6, 6, base),
        ("circle_ordinary_mpc", make_circle_scenario(duration=32.0 * duration_scale), 0, 6, base),
        ("circle_delay_aware", make_circle_scenario(duration=32.0 * duration_scale), 6, 6, base),
        ("strict_constraints", make_straight_scenario(duration=16.0 * duration_scale), 6, 6, strict_default_config()),
        ("mismatch_actual_4", make_curve_scenario(duration=12.0 * duration_scale), 6, 4, base),
        ("mismatch_actual_8", make_curve_scenario(duration=12.0 * duration_scale), 6, 8, base),
    ]
    rows = []
    for name, scenario, controller_delay, plant_delay, config in cases:
        log = run_case(scenario, controller_delay, plant_delay, config)
        metrics = compute_metrics(log)
        write_case_results(output_dir / name, log, metrics)
        rows.append((name, controller_delay, plant_delay, metrics))
    lines = ["# Delay-Aware MPC numerical experiment summary", "", "| Case | MPC delay steps | Plant delay steps | Position RMSE (m) | Max (m) | Tail mean (m) | Violations | Mean solve (ms) | Non-optimal |", "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for name, controller_delay, plant_delay, metrics in rows:
        lines.append(f"| {name} | {controller_delay} | {plant_delay} | {metrics['position_rmse']:.4f} | {metrics['position_max']:.4f} | {metrics['position_tail_mean']:.4f} | {metrics['constraint_violations']:.0f} | {metrics['solve_ms_mean']:.2f} | {metrics['nonoptimal_solves']:.0f} |")
    lines.extend(["", "All rows use the same nominal tau/weights within their indicated constraint configuration. The ordinary-MPC row has no delay in its internal prediction but runs on the same six-step plant as the delay-aware row."])
    (output_dir / "experiment_summary.md").write_text("\n".join(lines) + "\n")
    return output_dir


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--duration-scale", type=float, default=1.0)
    args = parser.parse_args()
    output = args.output_dir or Path(__file__).resolve().parent / "results" / datetime.now().strftime("%Y%m%d_%H%M%S")
    print(run_all(output, args.duration_scale))


if __name__ == "__main__":
    main()
