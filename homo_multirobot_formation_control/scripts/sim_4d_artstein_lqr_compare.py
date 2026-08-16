#!/usr/bin/env python3
"""Numerical comparison for 4D Artstein-HPC and 4D Artstein-LQR."""

from __future__ import annotations

import argparse
from pathlib import Path

from sim_4d_artstein_mpc_compare import (
    SimRow,
    plot_group,
    simulate_circle_case,
    write_summary,
    write_timeseries,
)


def run_lqr_experiments(args) -> dict[str, list[SimRow]]:
    return {
        "artstein_hpc_no_delay": simulate_circle_case(
            "hpc",
            "none",
            args.circle_tmax,
            args.dt,
            args.tau,
            args.Td,
            args.max_linear_vel,
            args.max_linear_accel,
            args=args,
        ),
        "artstein_lqr_no_delay": simulate_circle_case(
            "lqr",
            "none",
            args.circle_tmax,
            args.dt,
            args.tau,
            args.Td,
            args.max_linear_vel,
            args.max_linear_accel,
            args=args,
        ),
        "original_4d_delay": simulate_circle_case(
            "hpc",
            "none_with_delay",
            args.circle_tmax,
            args.dt,
            args.tau,
            args.Td,
            args.max_linear_vel,
            args.max_linear_accel,
            args=args,
        ),
        "artstein_hpc_delay": simulate_circle_case(
            "hpc",
            "artstein",
            args.circle_tmax,
            args.dt,
            args.tau,
            args.Td,
            args.max_linear_vel,
            args.max_linear_accel,
            args=args,
        ),
        "artstein_lqr_delay": simulate_circle_case(
            "lqr",
            "artstein",
            args.circle_tmax,
            args.dt,
            args.tau,
            args.Td,
            args.max_linear_vel,
            args.max_linear_accel,
            args=args,
        ),
    }


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", default="homo_multirobot_formation_control/analysis/results/4d_artstein_lqr")
    parser.add_argument("--circle-tmax", type=float, default=45.0)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--tau", type=float, default=0.43)
    parser.add_argument("--Td", type=float, default=0.22)
    parser.add_argument("--mass", type=float, default=2.0)
    parser.add_argument("--radius", type=float, default=2.0)
    parser.add_argument("--m-p", dest="m_p", type=int, default=4)
    parser.add_argument("--tol", type=float, default=0.1)
    parser.add_argument("--leader-radius", type=float, default=2.0)
    parser.add_argument("--leader-omega", type=float, default=0.1)
    parser.add_argument("--max-linear-vel", type=float, default=0.5)
    parser.add_argument("--max-linear-accel", type=float, default=0.4)
    parser.add_argument("--hpc-c-min", type=float, default=0.1)
    parser.add_argument("--initial-min-lambda", type=float, default=1.5)
    parser.add_argument("--switch-min-lambda", type=float, default=4.0)
    parser.add_argument("--lqr-q-px", type=float, default=40.0)
    parser.add_argument("--lqr-q-py", type=float, default=40.0)
    parser.add_argument("--lqr-q-vx", type=float, default=1.0)
    parser.add_argument("--lqr-q-vy", type=float, default=1.0)
    parser.add_argument("--lqr-r-ux", type=float, default=0.02)
    parser.add_argument("--lqr-r-uy", type=float, default=0.02)
    return parser.parse_args()


def main():
    args = parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    rows_by_name = run_lqr_experiments(args)
    outputs = [
        plot_group(
            "4D Artstein-HPC vs 4D Artstein-LQR, no delay",
            {
                "Artstein-HPC no delay": rows_by_name["artstein_hpc_no_delay"],
                "Artstein-LQR no delay": rows_by_name["artstein_lqr_no_delay"],
            },
            out_dir / "circle_no_delay_hpc_vs_lqr.png",
        ),
        plot_group(
            "4D delay baselines with shared Artstein prediction layer",
            {
                "original 4D + delay": rows_by_name["original_4d_delay"],
                "Artstein-HPC + delay": rows_by_name["artstein_hpc_delay"],
                "Artstein-LQR + delay": rows_by_name["artstein_lqr_delay"],
            },
            out_dir / "circle_delay_hpc_vs_lqr.png",
        ),
        plot_group("4D Artstein-LQR comparison", rows_by_name, out_dir / "circle_lqr_all_compare.png"),
        write_summary(out_dir / "summary_metrics.csv", rows_by_name),
        write_timeseries(out_dir / "timeseries_circle_lqr_compare.csv", rows_by_name),
    ]
    for path in outputs:
        print(path)


if __name__ == "__main__":
    main()
