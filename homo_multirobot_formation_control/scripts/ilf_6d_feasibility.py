#!/usr/bin/env python3
"""Offline feasibility utilities for the local 6D actuator-aware ILF model."""

import argparse
import csv
import itertools
from pathlib import Path

import numpy as np


def build_local_model(rho: np.ndarray, tau: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Build the frozen 6D deviation-input model from proposal equation (2)."""
    rho = np.asarray(rho, dtype=float)
    tau = np.asarray(tau, dtype=float)
    if rho.shape != (3,) or tau.shape != (3,) or np.any(tau <= 0.0):
        raise ValueError("rho and tau must be length-3 arrays; tau must be positive")

    vx_l, vy_l, omega_l = rho
    inv_tau = 1.0 / tau

    A = np.zeros((6, 6))
    A[0] = [0.0, omega_l, -vy_l, 1.0, 0.0, 0.0]
    A[1] = [-omega_l, 0.0, vx_l, 0.0, 1.0, 0.0]
    A[2, 5] = 1.0
    A[3:, 3:] = -np.diag(inv_tau)

    B = np.zeros((6, 3))
    B[3:, :] = np.diag(inv_tau)
    return A, B


def controllability_diagnostics(
    A: np.ndarray,
    B: np.ndarray,
    rank_tol: float = 1e-9,
) -> dict[str, float | int]:
    """Return Kalman controllability rank and singular-value conditioning."""
    n = A.shape[0]
    blocks = []
    power = np.eye(n)
    for _ in range(n):
        blocks.append(power @ B)
        power = A @ power

    controllability = np.hstack(blocks)
    singular_values = np.linalg.svd(controllability, compute_uv=False)
    sigma_max = float(singular_values[0])
    sigma_min = float(singular_values[-1])
    rank = int(np.linalg.matrix_rank(controllability, tol=rank_tol))
    return {
        "rank": rank,
        "sigma_min": sigma_min,
        "sigma_max": sigma_max,
        "condition_number": (
            float("inf") if sigma_min <= rank_tol else sigma_max / sigma_min
        ),
    }


def scan_envelope(
    vx_values: list[float],
    vy_values: list[float],
    omega_values: list[float],
    tau_values: list[tuple[float, float, float]],
) -> list[dict[str, float | int]]:
    """Evaluate controllability diagnostics on a finite operating envelope."""
    rows: list[dict[str, float | int]] = []
    for vx_l, vy_l, omega_l, tau in itertools.product(
        vx_values,
        vy_values,
        omega_values,
        tau_values,
    ):
        tau_array = np.asarray(tau, dtype=float)
        A, B = build_local_model(
            np.array([vx_l, vy_l, omega_l], dtype=float),
            tau_array,
        )
        row = controllability_diagnostics(A, B)
        row.update(
            {
                "vx_leader": float(vx_l),
                "vy_leader": float(vy_l),
                "omega_leader": float(omega_l),
                "tau_x": float(tau_array[0]),
                "tau_y": float(tau_array[1]),
                "tau_omega": float(tau_array[2]),
            }
        )
        rows.append(row)
    return rows


def write_csv(rows: list[dict[str, float | int]], csv_path: Path) -> None:
    """Write feasibility rows with a deterministic column order."""
    fieldnames = [
        "vx_leader",
        "vy_leader",
        "omega_leader",
        "tau_x",
        "tau_y",
        "tau_omega",
        "rank",
        "sigma_min",
        "sigma_max",
        "condition_number",
    ]
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=fieldnames,
            lineterminator="\n",
        )
        writer.writeheader()
        writer.writerows(rows)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Scan controllability of the frozen 6D actuator-aware model."
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=Path("analysis/results/6d_ilf_feasibility/controllability_scan.csv"),
        help="CSV output path.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rows = scan_envelope(
        vx_values=[-0.5, 0.0, 0.5],
        vy_values=[-0.5, 0.0, 0.5],
        omega_values=[-0.5, 0.0, 0.5],
        tau_values=[(0.25, 0.25, 0.25), (0.43, 0.43, 0.43), (0.55, 0.55, 0.55)],
    )
    write_csv(rows, args.csv)
    ranks = [int(row["rank"]) for row in rows]
    print(
        f"wrote {len(rows)} rows to {args.csv}; "
        f"controllability rank range=[{min(ranks)}, {max(ranks)}]"
    )


if __name__ == "__main__":
    main()
