#!/usr/bin/env python3
"""Map-frame 6D regularized-HPC and Artstein comparison utilities."""

from __future__ import annotations

import numpy as np
from scipy.linalg import expm, solve_continuous_lyapunov


def wrap_angle(angle: float) -> float:
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


def rot(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def build_nominal_model(
    mass: float, inertia: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if mass <= 0.0 or inertia <= 0.0:
        raise ValueError("mass and inertia must be positive")
    a = np.zeros((6, 6))
    a[0, 3] = a[1, 4] = a[2, 5] = 1.0
    d_inv = np.diag([1.0 / mass, 1.0 / mass, 1.0 / inertia])
    b = np.vstack([np.zeros((3, 3)), d_inv])
    g0 = np.diag([-1.0, -1.0, -1.0, 0.0, 0.0, 0.0])
    return a, b, g0, np.diag([mass, mass, inertia])


def map_error(
    leader: np.ndarray,
    follower: np.ndarray,
    offset_map: np.ndarray,
    dtheta: float = 0.0,
) -> np.ndarray:
    return np.r_[
        follower[:2] - leader[:2] - offset_map,
        wrap_angle(follower[2] - leader[2] - dtheta),
        rot(follower[2]) @ follower[3:5] - rot(leader[2]) @ leader[3:5],
        follower[5] - leader[5],
    ]


def hnorm(error: np.ndarray, gd: np.ndarray, p: np.ndarray, nmax: int = 40) -> float:
    if np.linalg.norm(error) < 1e-14:
        return 0.0
    lower, upper = -1.0, 1.0
    while float((expm(-gd * lower) @ error).T @ p @ (expm(-gd * lower) @ error)) < 1.0:
        lower *= 2.0
    while float((expm(-gd * upper) @ error).T @ p @ (expm(-gd * upper) @ error)) > 1.0:
        upper *= 2.0
    for _ in range(nmax):
        midpoint = 0.5 * (lower + upper)
        scaled = expm(-gd * midpoint) @ error
        if float(scaled.T @ p @ scaled) > 1.0:
            lower = midpoint
        else:
            upper = midpoint
    return float(np.exp(0.5 * (lower + upper)))


class RegularizedMapHpc:
    """Existing project's regularized engineering HPC, in map-frame error coordinates."""

    def __init__(self, mass: float, inertia: float, mu: float, kp: float, kv: float,
                 c_min: float):
        if not 0.0 < c_min <= 1.0:
            raise ValueError("c_min must be in (0, 1]")
        self.a, self.b, g0, scale = build_nominal_model(mass, inertia)
        self.mu = mu
        self.gd = np.eye(6) + mu * g0
        self.k = -scale @ np.hstack([kp * np.eye(3), kv * np.eye(3)])
        self.p = solve_continuous_lyapunov((self.a + self.b @ self.k).T, -2.0 * np.eye(6))
        self.c_min = c_min

    def command(self, error: np.ndarray) -> np.ndarray:
        if np.linalg.norm(error) < 1e-14:
            return np.zeros(3)
        c = np.clip(hnorm(error, self.gd, self.p), self.c_min, 1.0)
        return c ** (1.0 + self.mu) * self.k @ expm(
            self.gd * (1.0 - np.log(c))
        ) @ error


def map_to_body(theta: float, velocity_map: np.ndarray) -> np.ndarray:
    return rot(theta).T @ velocity_map


def predict_map_state(state: np.ndarray, history, td: float, tau: float,
                      control_dt: float) -> np.ndarray:
    """Predict with the map-frame first-order command model used by the plant."""
    if td == 0.0 and tau == 0.0:
        return state.copy()
    if td < 0.0 or tau < 0.0 or control_dt <= 0.0:
        raise ValueError("td/tau must be non-negative and control_dt positive")
    predicted = state.copy()
    velocity_map = rot(state[2]) @ state[3:5]
    command = np.asarray(history[-1], dtype=float) if history else np.r_[velocity_map, state[5]]
    horizon = td + tau
    if tau > 0.0:
        decay = np.exp(-horizon / tau)
        predicted_velocity = command[:2] + decay * (velocity_map - command[:2])
        predicted_rate = command[2] + decay * (state[5] - command[2])
        predicted[:2] += horizon * command[:2] + tau * (1.0 - decay) * (velocity_map - command[:2])
        predicted[2] = wrap_angle(state[2] + horizon * command[2] + tau * (1.0 - decay) * (state[5] - command[2]))
    else:
        predicted_velocity, predicted_rate = velocity_map, state[5]
        predicted[:2] += horizon * velocity_map
        predicted[2] = wrap_angle(state[2] + horizon * state[5])
    predicted[3:5] = map_to_body(predicted[2], predicted_velocity)
    predicted[5] = predicted_rate
    return predicted


def verify_nominal_identities(
    mass: float, inertia: float, mu: float, kp: float, kv: float
) -> dict[str, float]:
    if not -1.0 < mu < 0.0:
        raise ValueError("mu must be in (-1, 0)")
    if kp <= 0.0 or kv <= 0.0:
        raise ValueError("kp and kv must be positive")
    a, b, g0, scale = build_nominal_model(mass, inertia)
    gd = np.eye(6) + mu * g0
    k = -scale @ np.hstack([kp * np.eye(3), kv * np.eye(3)])
    acl = a + b @ k
    if np.max(np.real(np.linalg.eigvals(acl))) >= 0.0:
        raise ValueError("linear feedback is not Hurwitz")
    p = solve_continuous_lyapunov(acl.T, -2.0 * np.eye(6))
    if np.min(np.linalg.eigvalsh(p)) <= 0.0:
        raise ValueError("Lyapunov solution is not positive definite")
    if np.min(np.linalg.eigvalsh(p @ gd + gd.T @ p)) <= 0.0:
        raise ValueError("dilation compatibility is not positive definite")
    controllability = np.hstack([b, a @ b])
    return {
        "controllability_rank": int(np.linalg.matrix_rank(controllability, tol=1e-10)),
        "g0_b": float(np.linalg.norm(g0 @ b)),
        "ag0_commutator": float(np.linalg.norm(a @ g0 - g0 @ a - a)),
        "agd_commutator": float(np.linalg.norm(a @ gd - gd @ a - mu * a)),
        "gd_b": float(np.linalg.norm(gd @ b - b)),
    }
