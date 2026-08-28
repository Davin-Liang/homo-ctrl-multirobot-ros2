#!/usr/bin/env python3
"""Map-frame 6D regularized-HPC and Artstein comparison utilities."""

from __future__ import annotations

import numpy as np
from scipy.linalg import solve_continuous_lyapunov


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
