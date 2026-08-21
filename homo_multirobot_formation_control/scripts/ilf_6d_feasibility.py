#!/usr/bin/env python3
"""Offline feasibility utilities for the local 6D actuator-aware ILF model."""

import argparse
import csv
import itertools
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class IlfDesign:
    """Numerical data for the nominal block-controllability ILF controller."""

    mu: float
    X: np.ndarray
    P: np.ndarray
    Y: np.ndarray
    K: np.ndarray
    H: np.ndarray


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


def build_nominal_canonical_model() -> tuple[np.ndarray, np.ndarray]:
    """Return the three-double-integrator model at the zero-twist nominal point."""
    A_tilde = np.zeros((6, 6))
    A_tilde[:3, 3:] = np.eye(3)
    B_tilde = np.zeros((6, 3))
    B_tilde[3:, :] = np.eye(3)
    return A_tilde, B_tilde


def canonical_to_deviation_input(
    xi: np.ndarray,
    nu: np.ndarray,
    tau: np.ndarray,
) -> np.ndarray:
    """Map canonical input nu to the zero-delay deviation command delta_u."""
    xi = np.asarray(xi, dtype=float)
    nu = np.asarray(nu, dtype=float)
    tau = np.asarray(tau, dtype=float)
    if xi.shape != (6,) or nu.shape != (3,) or tau.shape != (3,) or np.any(tau <= 0.0):
        raise ValueError("xi must have 6 entries; nu and positive tau must have 3")
    return xi[3:] + tau * nu


def synthesize_nominal_mimo_ilf(
    mu: float,
    solver: str = "CLARABEL",
) -> IlfDesign:
    """Solve the normalized zero-delay MIMO ILF condition of Theorem 10."""
    if not 0.0 < mu < 1.0:
        raise ValueError("mu must be strictly between 0 and 1")

    try:
        import cvxpy as cp
    except ImportError as exc:
        raise RuntimeError("nominal MIMO ILF synthesis requires cvxpy") from exc

    A_tilde, B_tilde = build_nominal_canonical_model()
    H = np.diag([1.0 + mu] * 3 + [1.0] * 3)
    X = cp.Variable((6, 6), symmetric=True)
    Y = cp.Variable((3, 6))
    epsilon = 1e-5
    identity = (
        A_tilde @ X
        + X @ A_tilde.T
        + B_tilde @ Y
        + Y.T @ B_tilde.T
        + H @ X
        + X @ H
    )
    problem = cp.Problem(
        cp.Minimize(cp.sum_squares(Y)),
        [
            identity == 0.0,
            X >> epsilon * np.eye(6),
            X @ H + H @ X >> epsilon * np.eye(6),
            cp.trace(X) == 1.0,
        ],
    )
    try:
        problem.solve(solver=solver)
    except cp.error.SolverError as exc:
        raise RuntimeError(f"MIMO ILF solver {solver} failed") from exc
    if problem.status not in {cp.OPTIMAL, cp.OPTIMAL_INACCURATE}:
        raise RuntimeError(f"MIMO ILF synthesis status: {problem.status}")

    x_value = 0.5 * (np.asarray(X.value) + np.asarray(X.value).T)
    y_value = np.asarray(Y.value)
    p_value = np.linalg.inv(x_value)
    k_value = y_value @ p_value
    residual = (
        A_tilde @ x_value
        + x_value @ A_tilde.T
        + B_tilde @ y_value
        + y_value.T @ B_tilde.T
        + H @ x_value
        + x_value @ H
    )
    if np.linalg.norm(residual, ord=np.inf) > 1e-7:
        raise RuntimeError("MIMO ILF matrix identity residual is too large")
    if np.linalg.eigvalsh(x_value).min() <= 0.0:
        raise RuntimeError("MIMO ILF synthesis returned nonpositive X")
    if np.linalg.eigvalsh(x_value @ H + H @ x_value).min() <= 0.0:
        raise RuntimeError("MIMO ILF synthesis returned nonpositive dilation metric")

    return IlfDesign(
        mu=mu,
        X=x_value,
        P=p_value,
        Y=y_value,
        K=k_value,
        H=H,
    )


def implicit_lyapunov_value(xi: np.ndarray, design: IlfDesign) -> float:
    """Find the unique positive root of the nominal implicit Lyapunov equation."""
    from scipy.optimize import brentq

    xi = np.asarray(xi, dtype=float)
    if xi.shape != (6,):
        raise ValueError("xi must have 6 entries")
    if np.linalg.norm(xi) == 0.0:
        return 0.0

    weights = np.array([1.0 + design.mu] * 3 + [1.0] * 3)

    def q_of_log_value(log_value: float) -> float:
        value = np.exp(log_value)
        z = xi * value**(-weights)
        return float(z @ design.P @ z - 1.0)

    lower = -30.0
    upper = 30.0
    while q_of_log_value(lower) <= 0.0:
        lower -= 10.0
    while q_of_log_value(upper) >= 0.0:
        upper += 10.0
    return float(np.exp(brentq(q_of_log_value, lower, upper)))


def nominal_ilf_control(xi: np.ndarray, design: IlfDesign) -> np.ndarray:
    """Evaluate the zero-delay canonical ILF feedback nu(V, xi)."""
    xi = np.asarray(xi, dtype=float)
    value = implicit_lyapunov_value(xi, design)
    if value == 0.0:
        return np.zeros(3)
    weights = np.array([1.0 + design.mu] * 3 + [1.0] * 3)
    z = xi * value**(-weights)
    return value ** (1.0 - design.mu) * (design.K @ z)


def simulate_nominal_ilf(
    x0: np.ndarray,
    design: IlfDesign,
    duration: float,
    max_step: float,
) -> dict[str, np.ndarray]:
    """Integrate only the continuous zero-delay canonical ILF closed loop."""
    from scipy.integrate import solve_ivp

    x0 = np.asarray(x0, dtype=float)
    if x0.shape != (6,) or duration <= 0.0 or max_step <= 0.0:
        raise ValueError("x0 must have 6 entries; duration and max_step must be positive")
    A_tilde, B_tilde = build_nominal_canonical_model()

    def closed_loop(_: float, state: np.ndarray) -> np.ndarray:
        return A_tilde @ state + B_tilde @ nominal_ilf_control(state, design)

    solution = solve_ivp(
        closed_loop,
        (0.0, duration),
        x0,
        rtol=1e-8,
        atol=1e-10,
        max_step=max_step,
    )
    if not solution.success:
        raise RuntimeError(f"nominal ILF integration failed: {solution.message}")

    states = solution.y.T
    lyapunov = np.array(
        [implicit_lyapunov_value(state, design) for state in states]
    )
    control = np.array([nominal_ilf_control(state, design) for state in states])
    return {
        "time": solution.t,
        "state": states,
        "lyapunov": lyapunov,
        "control": control,
    }


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


def write_nominal_ilf_csv(result: dict[str, np.ndarray], csv_path: Path) -> None:
    """Write a deterministic record of the continuous zero-delay ILF run."""
    header = [
        "time",
        "e_x",
        "e_y",
        "e_theta",
        "e_vx",
        "e_vy",
        "e_omega",
        "V",
        "nu_x",
        "nu_y",
        "nu_omega",
    ]
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file, lineterminator="\n")
        writer.writerow(header)
        for time, state, value, control in zip(
            result["time"],
            result["state"],
            result["lyapunov"],
            result["control"],
        ):
            writer.writerow([time, *state, value, *control])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Offline controllability and nominal MIMO-ILF checks for the 6D model."
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=Path("analysis/results/6d_ilf_feasibility/controllability_scan.csv"),
        help="CSV output path.",
    )
    parser.add_argument(
        "--run-nominal-ilf",
        action="store_true",
        help="Run the frozen zero-delay continuous MIMO-ILF baseline instead of the scan.",
    )
    parser.add_argument(
        "--nominal-csv",
        type=Path,
        default=Path("analysis/results/6d_ilf_feasibility/nominal_ilf_run.csv"),
        help="CSV output path for --run-nominal-ilf.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.run_nominal_ilf:
        design = synthesize_nominal_mimo_ilf(mu=0.5)
        result = simulate_nominal_ilf(
            x0=np.array([1.0, -0.7, 0.3, 0.0, 0.0, 0.0]),
            design=design,
            duration=4.0,
            max_step=1e-3,
        )
        write_nominal_ilf_csv(result, args.nominal_csv)
        print(
            f"wrote {len(result['time'])} rows to {args.nominal_csv}; "
            f"V_initial={result['lyapunov'][0]:.12g}; "
            f"V_final={result['lyapunov'][-1]:.12g}; "
            f"final_state_norm={np.linalg.norm(result['state'][-1]):.12g}"
        )
        return

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
