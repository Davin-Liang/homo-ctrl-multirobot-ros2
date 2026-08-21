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
    R: np.ndarray | None = None


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


def synthesize_robust_nominal_mimo_ilf(
    mu: float,
    disturbance_weight: float,
    solver: str = "CLARABEL",
) -> IlfDesign:
    """Solve Theorem 15's nominal MIMO ILF inequality for R=weight*I."""
    if not 0.0 < mu < 1.0:
        raise ValueError("mu must be strictly between 0 and 1")
    if disturbance_weight <= 0.0:
        raise ValueError("disturbance_weight must be positive")

    try:
        import cvxpy as cp
    except ImportError as exc:
        raise RuntimeError("robust MIMO ILF synthesis requires cvxpy") from exc

    A_tilde, B_tilde = build_nominal_canonical_model()
    H = np.diag([1.0 + mu] * 3 + [1.0] * 3)
    R = disturbance_weight * np.eye(6)
    X = cp.Variable((6, 6), symmetric=True)
    Y = cp.Variable((3, 6))
    epsilon = 1e-5
    lmi_left = (
        A_tilde @ X
        + X @ A_tilde.T
        + B_tilde @ Y
        + Y.T @ B_tilde.T
        + H @ X
        + X @ H
        + R
    )
    problem = cp.Problem(
        cp.Minimize(cp.sum_squares(Y)),
        [
            lmi_left << 0.0,
            X >> epsilon * np.eye(6),
            X @ H + H @ X >> epsilon * np.eye(6),
            cp.trace(X) == 1.0,
        ],
    )
    try:
        problem.solve(solver=solver)
    except cp.error.SolverError as exc:
        raise RuntimeError(f"robust MIMO ILF solver {solver} failed") from exc
    if problem.status not in {cp.OPTIMAL, cp.OPTIMAL_INACCURATE}:
        raise RuntimeError(f"robust MIMO ILF synthesis status: {problem.status}")

    x_value = 0.5 * (np.asarray(X.value) + np.asarray(X.value).T)
    y_value = np.asarray(Y.value)
    p_value = np.linalg.inv(x_value)
    k_value = y_value @ p_value
    lmi_value = (
        A_tilde @ x_value
        + x_value @ A_tilde.T
        + B_tilde @ y_value
        + y_value.T @ B_tilde.T
        + H @ x_value
        + x_value @ H
        + R
    )
    if np.linalg.eigvalsh(lmi_value).max() > 1e-7:
        raise RuntimeError("robust MIMO ILF matrix inequality residual is too large")
    if np.linalg.eigvalsh(x_value).min() <= 0.0:
        raise RuntimeError("robust MIMO ILF synthesis returned nonpositive X")
    if np.linalg.eigvalsh(x_value @ H + H @ x_value).min() <= 0.0:
        raise RuntimeError("robust MIMO ILF synthesis returned nonpositive dilation metric")

    return IlfDesign(
        mu=mu,
        X=x_value,
        P=p_value,
        Y=y_value,
        K=k_value,
        H=H,
        R=R,
    )


def matched_disturbance_ratio(
    xi: np.ndarray,
    w_d: np.ndarray,
    design: IlfDesign,
) -> float:
    """Evaluate the Theorem-15 matched-disturbance sufficient-condition ratio."""
    xi = np.asarray(xi, dtype=float)
    w_d = np.asarray(w_d, dtype=float)
    if xi.shape != (6,) or w_d.shape != (3,):
        raise ValueError("xi must have 6 entries and w_d must have 3")
    if design.R is None:
        raise ValueError("matched disturbance ratio requires a robust ILF design")
    if np.array_equal(w_d, np.zeros(3)):
        return 0.0

    value = implicit_lyapunov_value(xi, design)
    if value == 0.0:
        raise ValueError("nonzero disturbance has no finite ratio at xi=0")
    weights = np.array([1.0 + design.mu] * 3 + [1.0] * 3)
    z = xi * value**(-weights)
    d_tilde = np.concatenate((np.zeros(3), w_d))
    d_scaled = d_tilde * value**(-weights)
    numerator = float(d_scaled @ np.linalg.solve(design.R, d_scaled))
    denominator = float(
        value ** (-2.0 * design.mu)
        * (z @ (design.H @ design.P + design.P @ design.H) @ z)
    )
    if denominator <= 0.0:
        raise RuntimeError("matched-disturbance denominator must be positive")
    return numerator / denominator


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


def simulate_delayed_ilf(
    x0: np.ndarray,
    design: IlfDesign,
    tau: np.ndarray,
    delay: float,
    duration: float,
    dt: float,
) -> dict[str, np.ndarray]:
    """Run a grid-aligned Euler method-of-steps audit of the delayed actuator."""
    x0 = np.asarray(x0, dtype=float)
    tau = np.asarray(tau, dtype=float)
    if x0.shape != (6,) or tau.shape != (3,) or np.any(tau <= 0.0):
        raise ValueError("x0 must have 6 entries and tau must be a positive length-3 array")
    if design.R is None:
        raise ValueError("delayed ILF audit requires a robust ILF design")
    if delay < 0.0 or duration <= 0.0 or dt <= 0.0:
        raise ValueError("delay must be nonnegative; duration and dt must be positive")

    delay_steps = round(delay / dt)
    if not np.isclose(delay_steps * dt, delay, rtol=0.0, atol=1e-12):
        raise ValueError("delay must be an integer multiple of dt")
    duration_steps = round(duration / dt)
    if not np.isclose(duration_steps * dt, duration, rtol=0.0, atol=1e-12):
        raise ValueError("duration must be an integer multiple of dt")

    xi = x0.copy()
    history = [x0.copy() for _ in range(delay_steps + 1)]
    times: list[float] = []
    states: list[np.ndarray] = []
    lyapunov: list[float] = []
    nu_values: list[np.ndarray] = []
    delayed_commands: list[np.ndarray] = []
    disturbances: list[np.ndarray] = []
    ratios: list[float] = []

    for step in range(duration_steps + 1):
        xi_delayed = history[0]
        nu_now = nominal_ilf_control(xi, design)
        nu_delayed = nominal_ilf_control(xi_delayed, design)
        delta_u_delayed = canonical_to_deviation_input(xi_delayed, nu_delayed, tau)
        w_d = -(xi[3:] - xi_delayed[3:]) / tau + nu_delayed - nu_now

        times.append(step * dt)
        states.append(xi.copy())
        lyapunov.append(implicit_lyapunov_value(xi, design))
        nu_values.append(nu_now)
        delayed_commands.append(delta_u_delayed)
        disturbances.append(w_d)
        ratios.append(matched_disturbance_ratio(xi, w_d, design))

        if step == duration_steps:
            break
        derivative = np.concatenate((xi[3:], (-xi[3:] + delta_u_delayed) / tau))
        xi = xi + dt * derivative
        history.append(xi.copy())
        history.pop(0)

    return {
        "time": np.asarray(times),
        "state": np.asarray(states),
        "lyapunov": np.asarray(lyapunov),
        "nu": np.asarray(nu_values),
        "delta_u_delayed": np.asarray(delayed_commands),
        "w_d": np.asarray(disturbances),
        "ratio": np.asarray(ratios),
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


def delayed_ilf_audit_rows(
    delay_values: list[float],
    design: IlfDesign,
    x0: np.ndarray,
    tau: np.ndarray,
    duration: float,
    dt: float,
) -> list[dict[str, float | bool]]:
    """Summarize delayed-plant DDE traces and their sampled sufficient-condition ratios."""
    rows: list[dict[str, float | bool]] = []
    for delay in delay_values:
        result = simulate_delayed_ilf(x0, design, tau, delay, duration, dt)
        ratios = result["ratio"]
        finite_ratios = ratios[np.isfinite(ratios)]
        final_state_norm = float(np.linalg.norm(result["state"][-1]))
        max_state_norm = float(np.linalg.norm(result["state"], axis=1).max())
        rows.append(
            {
                "delay": float(delay),
                "final_state_norm": final_state_norm,
                "final_V": float(result["lyapunov"][-1]),
                "max_state_norm": max_state_norm,
                "max_ratio": (
                    float(finite_ratios.max()) if finite_ratios.size else 0.0
                ),
                "ratio_samples_below_one": bool(
                    finite_ratios.size == 0 or np.all(finite_ratios < 1.0)
                ),
            }
        )
    return rows


def write_delayed_ilf_audit_csv(
    rows: list[dict[str, float | bool]],
    csv_path: Path,
) -> None:
    """Write delayed ILF audit summaries with a deterministic column order."""
    fieldnames = [
        "delay",
        "final_state_norm",
        "final_V",
        "max_state_norm",
        "max_ratio",
        "ratio_samples_below_one",
    ]
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


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
    parser.add_argument(
        "--run-delayed-ilf-audit",
        action="store_true",
        help="Run the frozen delayed-actuator MIMO-ILF sufficient-condition audit.",
    )
    parser.add_argument(
        "--delayed-audit-csv",
        type=Path,
        default=Path("analysis/results/6d_ilf_feasibility/delayed_ilf_audit.csv"),
        help="CSV output path for --run-delayed-ilf-audit.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.run_delayed_ilf_audit:
        design = synthesize_robust_nominal_mimo_ilf(
            mu=0.5,
            disturbance_weight=1e-3,
        )
        rows = delayed_ilf_audit_rows(
            delay_values=[0.0, 0.05, 0.10, 0.15, 0.22, 0.30],
            design=design,
            x0=np.array([1.0, -0.7, 0.3, 0.0, 0.0, 0.0]),
            tau=np.array([0.43, 0.43, 0.43]),
            duration=8.0,
            dt=0.001,
        )
        write_delayed_ilf_audit_csv(rows, args.delayed_audit_csv)
        print(f"wrote {len(rows)} rows to {args.delayed_audit_csv}")
        return

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
