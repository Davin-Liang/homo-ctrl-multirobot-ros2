#!/usr/bin/env python3
"""
Numerical experiments for the proposed 6D Disc + Artstein-prediction controller.

This script intentionally mirrors sim_4d_hpc_artstein_compare.py, but targets the
6D Disc controller architecture:

  - baseline: 6D Disc controller, plant has command dead time + first-order lag
  - compensated: map-frame 4D translation predictor + 2D yaw predictor feeding
    the same 6D Disc HPC core

The compensated case is "direction A" from the design discussion: predictor
compensation stays outside the 6D HPC core, then the predicted state is converted
back to [px, py, theta, vx_body, vy_body, omega].
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import expm, null_space, solve, sqrtm


# -----------------------------------------------------------------------------
# MATLAB/C++ HPC toolbox functions ported to Python
# -----------------------------------------------------------------------------

def matrix_rank(mat: np.ndarray) -> int:
    return int(np.linalg.matrix_rank(mat, tol=1e-10))


def trans_con(A: np.ndarray, B: np.ndarray) -> tuple[np.ndarray, list[int]]:
    n = A.shape[0]
    U = np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(n)])
    if matrix_rank(U) < n:
        raise ValueError("The pair {A,B} is not controllable")

    T = np.eye(n)
    Ak = A.copy()
    Bk = B.copy()
    nt: list[int] = []

    while matrix_rank(Bk) < Ak.shape[0]:
        rank_bk = matrix_rank(Bk)
        nt.insert(0, rank_bk)
        B_ort = null_space(Bk.T).T
        B_p = null_space(B_ort).T
        T_block = np.vstack([B_ort, B_p])

        if Ak.shape[0] < n:
            T_temp = np.eye(n)
            rows = Ak.shape[0]
            T_temp[:rows, :rows] = T_block
            T = T_temp @ T
        else:
            T = T_block

        Ak_old = Ak
        Bk = B_ort @ Ak_old @ B_p.T
        Ak = B_ort @ Ak_old @ B_ort.T

    nt.insert(0, matrix_rank(Bk))
    return T, nt


def block_con(A: np.ndarray, B: np.ndarray) -> tuple[np.ndarray, list[int]]:
    T, nt = trans_con(A, B)
    k = len(nt)
    n = A.shape[0]
    n_ind = [0]
    for size in nt:
        n_ind.append(n_ind[-1] + size)

    Acur = T @ A @ np.linalg.inv(T)
    Phi = np.eye(n)
    for i in range(k - 1):
        r0 = n_ind[i]
        r1 = n_ind[i + 1]
        c0 = n_ind[i + 1]
        c1 = n_ind[i + 2]
        temp_A = Acur[r0:r1, c0:c1]
        left = temp_A.T @ np.linalg.inv(temp_A @ temp_A.T) @ Acur[r0:r1, :c0]
        temp_block = np.hstack([left, np.eye(nt[i + 1]), np.zeros((nt[i + 1], n - c1))])
        temp_T = np.eye(n)
        temp_T[c0:c1, :] = temp_block
        Phi = temp_T @ Phi
        Acur = temp_T @ Acur @ np.linalg.inv(temp_T)
    return Phi @ T, nt


def lpc2hpc(A: np.ndarray, B: np.ndarray, K: np.ndarray, margin: float = 0.01):
    eig_cl = np.linalg.eigvals(A + B @ K)
    max_real = float(np.max(np.real(eig_cl)))
    if max_real >= -margin:
        raise ValueError(f"Closed-loop stability margin too small: max_real={max_real:.4f}")

    T, nt = block_con(A, B)
    n = A.shape[0]
    k = len(nt)
    n_ind = [0]
    for size in nt:
        n_ind.append(n_ind[-1] + size)

    Anew = T @ A @ np.linalg.inv(T)
    Bnew = T @ B
    B0 = Bnew[n_ind[k - 1]:n, :]
    A0 = Anew[n_ind[k - 1]:n, :]
    K0 = -np.linalg.pinv(B0) @ A0 @ T

    vG0 = []
    for i, size in enumerate(nt):
        vG0.extend([float(k - 1 - i)] * size)
    G0 = -np.linalg.inv(T) @ np.diag(vG0) @ T

    Acl = A + B @ K
    W = np.kron(np.eye(n), Acl.T) + np.kron(Acl.T, np.eye(n))
    P = solve(W, -(2.0 * np.eye(n)).reshape(-1, order="F")).reshape((n, n), order="F")

    sqrt_P = sqrtm(P).real
    inv_sqrt_P = np.linalg.inv(sqrt_P)
    M = sqrt_P @ G0 @ inv_sqrt_P
    eig_m = np.real(np.linalg.eigvals(M + M.T))
    lambda_min = float(np.min(eig_m))
    lambda_max = float(np.max(eig_m))
    nu_min = max(-1.0, -1.0 / lambda_max + 1e-5) if lambda_max > 1e-5 else -1.0
    nu_max = min(1.0 / k, -1.0 / lambda_min) if lambda_min < -1e-5 else 1.0 / k
    return K0, G0, P, nu_min, nu_max


def hnorm(x: np.ndarray, Gd: np.ndarray, P: np.ndarray, alpha=None, beta=None, nmax=20) -> float:
    if np.linalg.norm(x) < 1e-16:
        return 0.0

    a = -1.0
    y = expm(-Gd * a) @ x
    while float(y.T @ P @ y) < 1.0 and a > -746.0:
        a *= 2.0
        y = expm(-Gd * a) @ x

    b = 1.0
    y = expm(-Gd * b) @ x
    while float(y.T @ P @ y) > 1.0 and b < 710.0:
        b *= 2.0
        y = expm(-Gd * b) @ x

    c = 0.5 * (a + b)
    for _ in range(nmax):
        y = expm(-Gd * c) @ x
        qf = float(y.T @ P @ y) - 1.0
        if abs(qf) < 1e-6:
            break
        if qf > 0.0:
            a = c
        else:
            b = c
        c = 0.5 * (a + b)

    q = float(np.exp(c))
    if beta is not None:
        q = min(beta, q)
    if alpha is not None:
        q = max(alpha, q)
    return q


# -----------------------------------------------------------------------------
# Geometry helpers
# -----------------------------------------------------------------------------

def wrap_angle(a: float) -> float:
    return float(np.arctan2(np.sin(a), np.cos(a)))


def rot(theta: float) -> np.ndarray:
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s], [s, c]])


def body_to_map(theta: float, v_body: np.ndarray) -> np.ndarray:
    return rot(theta) @ v_body


def map_to_body(theta: float, v_map: np.ndarray) -> np.ndarray:
    return rot(theta).T @ v_map


def se2_predict_constant_twist(x: np.ndarray, horizon: float) -> np.ndarray:
    """Predict [px,py,theta,vx_b,vy_b,omega] with constant body twist."""
    y = x.copy()
    vx, vy, omega = x[3], x[4], x[5]
    if abs(omega) < 1e-8:
        dp_body = np.array([vx * horizon, vy * horizon])
    else:
        wt = omega * horizon
        dp_body = np.array([
            vx * np.sin(wt) / omega + vy * (np.cos(wt) - 1.0) / omega,
            vx * (1.0 - np.cos(wt)) / omega + vy * np.sin(wt) / omega,
        ])
    y[0:2] += rot(x[2]) @ dp_body
    y[2] = wrap_angle(x[2] + omega * horizon)
    return y


# -----------------------------------------------------------------------------
# 6D Disc HPC core
# -----------------------------------------------------------------------------

@dataclass
class Hpc6DDisc:
    mass: float = 2.0
    inertia: float = 1.0
    radius: float = 1.0
    m_p: int = 4
    tol: float = 0.1
    initial_min_lambda: float = 1.0
    switch_min_lambda: float = 4.0
    control_period: float = 0.05
    hpc_c_min: float = 0.5
    hpc_vel_threshold: float = 0.15
    hpc_yaw_threshold: float = 0.2
    stability_margin: float = 0.01
    use_hpc: bool = True
    offset_frame: str = "leader"

    def __post_init__(self):
        self.A = np.zeros((6, 6))
        self.B = np.array([
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0 / self.mass, 0.0, 0.0],
            [0.0, 1.0 / self.mass, 0.0],
            [0.0, 0.0, 1.0 / self.inertia],
        ])
        self.dl = np.column_stack([
            np.array([
                -self.radius * np.cos(2.0 * np.pi * i / self.m_p),
                -self.radius * np.sin(2.0 * np.pi * i / self.m_p),
                0.0, 0.0, 0.0, 0.0,
            ])
            for i in range(self.m_p)
        ])
        self.d = self.dl[:, 0].copy()
        self.K = np.zeros((3, 6))
        self.min_lambda = self.initial_min_lambda
        self.Gd = np.eye(6)
        self.P = np.eye(6)
        self.nu = 0.0
        self.hpc_valid = False
        self.fallback_count = 0
        self.target_idx = 0
        self.last_hpc_leader_vel = np.zeros(3)
        self.last_dtheta = 0.0

    def init(self, x1: np.ndarray, x2: np.ndarray):
        distances = [np.linalg.norm(self.compute_error_with_d(x1, x2, self.dl[:, i]))
                     for i in range(self.m_p)]
        self.target_idx = int(np.argmin(distances))
        self.d = self.dl[:, self.target_idx].copy()
        self._update_A(x1)
        e, dtheta, _, _ = self.compute_error(x1, x2)
        self.K = self.calculate_klin(e)
        self._try_rebuild_hpc()
        self.last_hpc_leader_vel = x1[3:6].copy()
        self.last_dtheta = dtheta

    def command(self, x1: np.ndarray, x2: np.ndarray) -> np.ndarray:
        self._update_A(x1)
        self._switch_if_needed(x1, x2)
        e, dtheta, cos_dt, sin_dt = self.compute_error(x1, x2)
        self.K = self.calculate_klin(e)

        leader_vel = x1[3:6]
        vel_changed = np.linalg.norm(leader_vel - self.last_hpc_leader_vel) > self.hpc_vel_threshold
        yaw_changed = abs(wrap_angle(dtheta - self.last_dtheta)) > self.hpc_yaw_threshold
        if vel_changed or yaw_changed:
            self._try_rebuild_hpc()
            self.last_hpc_leader_vel = leader_vel.copy()
            self.last_dtheta = dtheta

        if self.use_hpc and self.hpc_valid:
            nx = hnorm(e, self.Gd, self.P)
            c = min(1.0, max(self.hpc_c_min, nx))
            u_L = (c ** (1.0 + self.nu)) * self.K @ expm(self.Gd * (1.0 - np.log(c))) @ e
        else:
            u_L = self.K @ e

        if self.offset_frame == "map":
            v2_map = body_to_map(x2[2], x2[3:5])
            goal_v_map = v2_map + self.control_period * u_L[0:2] / self.mass
            goal_v_body = map_to_body(x2[2], goal_v_map)
            return np.array([
                goal_v_body[0],
                goal_v_body[1],
                x2[5] + self.control_period * u_L[2] / self.inertia,
            ])

        ux_f = u_L[0] * cos_dt + u_L[1] * sin_dt
        uy_f = -u_L[0] * sin_dt + u_L[1] * cos_dt

        return np.array([
            x2[3] + self.control_period * ux_f / self.mass,
            x2[4] + self.control_period * uy_f / self.mass,
            x2[5] + self.control_period * u_L[2] / self.inertia,
        ])

    def compute_error(self, x1: np.ndarray, x2: np.ndarray):
        dtheta = wrap_angle(x2[2] - x1[2])
        cos_dt = np.cos(dtheta)
        sin_dt = np.sin(dtheta)

        if self.offset_frame == "map":
            rel_p = x2[0:2] - x1[0:2]
            vf_err = body_to_map(x2[2], x2[3:5]) - body_to_map(x1[2], x1[3:5])
        else:
            rel_p = rot(x1[2]).T @ (x2[0:2] - x1[0:2])
            vf_err = np.array([
                x2[3] * cos_dt - x2[4] * sin_dt - x1[3],
                x2[3] * sin_dt + x2[4] * cos_dt - x1[4],
            ])
        e = np.array([
            rel_p[0] - self.d[0],
            rel_p[1] - self.d[1],
            wrap_angle(dtheta - self.d[2]),
            vf_err[0] - self.d[3],
            vf_err[1] - self.d[4],
            x2[5] - x1[5] - self.d[5],
        ])
        return e, dtheta, cos_dt, sin_dt

    def compute_error_with_d(self, x1: np.ndarray, x2: np.ndarray, d: np.ndarray) -> np.ndarray:
        old_d = self.d
        self.d = d
        e, _, _, _ = self.compute_error(x1, x2)
        self.d = old_d
        return e

    def distance(self, x1: np.ndarray, x2: np.ndarray) -> tuple[float, float, float]:
        distances = [self.compute_error_with_d(x1, x2, self.dl[:, i]) for i in range(self.m_p)]
        norms = [np.linalg.norm(e) for e in distances]
        e = distances[int(np.argmin(norms))]
        return float(np.linalg.norm(e[0:2])), float(abs(wrap_angle(e[2]))), float(np.linalg.norm(e))

    def _update_A(self, x1: np.ndarray):
        if self.offset_frame == "map":
            self.A[:, :] = 0.0
            self.A[0, 3] = 1.0
            self.A[1, 4] = 1.0
            self.A[2, 5] = 1.0
            return

        vx_l, vy_l, omega_l = x1[3], x1[4], x1[5]
        self.A[:, :] = 0.0
        self.A[0, 1] = omega_l
        self.A[0, 2] = -vy_l
        self.A[0, 3] = 1.0
        self.A[1, 0] = -omega_l
        self.A[1, 2] = vx_l
        self.A[1, 4] = 1.0
        self.A[2, 5] = 1.0

    def _switch_if_needed(self, x1: np.ndarray, x2: np.ndarray):
        errors = [self.compute_error_with_d(x1, x2, self.dl[:, i]) for i in range(self.m_p)]
        distances = [np.linalg.norm(e) for e in errors]
        best_idx = int(np.argmin(distances))
        current_dist = np.linalg.norm(self.compute_error_with_d(x1, x2, self.d))
        if distances[best_idx] + self.tol < current_dist:
            self.target_idx = best_idx
            self.d = self.dl[:, best_idx].copy()
            self.min_lambda = self.switch_min_lambda
            e, dtheta, _, _ = self.compute_error(x1, x2)
            self.K = self.calculate_klin(e)
            self._try_rebuild_hpc()
            self.last_hpc_leader_vel = x1[3:6].copy()
            self.last_dtheta = dtheta

    def _try_rebuild_hpc(self):
        if not self.use_hpc:
            self.hpc_valid = False
            return
        try:
            _, G0, P, nu_min, _ = lpc2hpc(self.A, self.B, self.K, self.stability_margin)
            self.P = P
            self.nu = nu_min
            self.Gd = np.eye(6) + self.nu * G0
            self.hpc_valid = True
        except (ValueError, np.linalg.LinAlgError):
            self.fallback_count += 1
            self.hpc_valid = False

    def calculate_klin(self, e: np.ndarray) -> np.ndarray:
        def channel(e_p: float, e_v: float, mass_like: float, wd: float):
            # Match the C++ 6D Disc behavior: critical damping with a lower bound.
            val = -mass_like * e_v / e_p if abs(e_p) > 1e-6 else 0.0
            val = np.clip(val, -wd * mass_like, wd * mass_like)
            a = max(val, wd * mass_like)
            k2 = -2.0 * a
            k1 = a * (k2 + a) / mass_like
            return k1, k2

        k1_x, k2_x = channel(e[0], e[3], self.mass, self.min_lambda)
        k1_y, k2_y = channel(e[1], e[4], self.mass, self.min_lambda)
        k1_t, k2_t = channel(e[2], e[5], self.inertia, self.min_lambda)
        K = np.zeros((3, 6))
        K[0, 0] = k1_x
        K[0, 3] = k2_x
        K[1, 1] = k1_y
        K[1, 4] = k2_y
        K[2, 2] = k1_t
        K[2, 5] = k2_t
        return K


# -----------------------------------------------------------------------------
# Artstein predictors for direction A
# -----------------------------------------------------------------------------

def actuator_matrices_4d(tau: float) -> tuple[np.ndarray, np.ndarray]:
    A = np.array([
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, -1.0 / tau, 0.0],
        [0.0, 0.0, 0.0, -1.0 / tau],
    ])
    B = np.array([
        [0.0, 0.0],
        [0.0, 0.0],
        [1.0 / tau, 0.0],
        [0.0, 1.0 / tau],
    ])
    return A, B


def actuator_matrices_2d(tau: float) -> tuple[np.ndarray, np.ndarray]:
    A = np.array([[0.0, 1.0], [0.0, -1.0 / tau]])
    B = np.array([[0.0], [1.0 / tau]])
    return A, B


def artstein_integral(history: deque[np.ndarray], A: np.ndarray, B: np.ndarray,
                      Td: float, h: float) -> np.ndarray:
    if Td <= 0.0:
        return np.zeros(A.shape[0])
    n = max(1, int(np.ceil(Td / h)))
    integral = np.zeros(A.shape[0])
    for k in range(min(n, len(history))):
        weight = h if k < n - 1 else Td - (n - 1) * h
        integral += expm(A * (k * h - Td)) @ B @ np.atleast_1d(history[k]) * weight
    return integral


def predict_from_artstein(z: np.ndarray, current_cmd: np.ndarray, A: np.ndarray,
                          tau: float, Td: float) -> np.ndarray:
    delay_free = expm(A * Td) @ z
    decay = np.exp(-1.0)
    q = delay_free.shape[0] // 2
    pos = delay_free[:q] + current_cmd * tau + tau * (1.0 - decay) * (delay_free[q:] - current_cmd)
    vel = current_cmd + decay * (delay_free[q:] - current_cmd)
    return np.r_[pos, vel]


def predict_follower_direction_a(x_meas: np.ndarray, vcmd_map_history: deque[np.ndarray],
                                 wcmd_history: deque[np.ndarray], last_vcmd_map: np.ndarray,
                                 last_wcmd: float, tau_v: float, tau_w: float,
                                 Td: float, h: float) -> np.ndarray:
    v_map_meas = body_to_map(x_meas[2], x_meas[3:5])
    x4 = np.r_[x_meas[0:2], v_map_meas]
    A4, B4 = actuator_matrices_4d(tau_v)
    z4 = x4 + artstein_integral(vcmd_map_history, A4, B4, Td, h)
    pred4 = predict_from_artstein(z4, last_vcmd_map, A4, tau_v, Td)

    x2 = np.array([x_meas[2], x_meas[5]])
    A2, B2 = actuator_matrices_2d(tau_w)
    z2 = x2 + artstein_integral(wcmd_history, A2, B2, Td, h)
    pred2 = predict_from_artstein(z2, np.array([last_wcmd]), A2, tau_w, Td)

    theta_pred = wrap_angle(pred2[0])
    v_body_pred = map_to_body(theta_pred, pred4[2:4])
    return np.array([
        pred4[0], pred4[1], theta_pred,
        v_body_pred[0], v_body_pred[1], pred2[1],
    ])


def add_measurement_noise(x: np.ndarray, pos_std: float, yaw_std: float,
                          vel_std: float, omega_std: float,
                          rng: np.random.Generator) -> np.ndarray:
    y = x.copy()
    if pos_std > 0.0:
        y[0:2] += rng.normal(0.0, pos_std, size=2)
    if yaw_std > 0.0:
        y[2] = wrap_angle(y[2] + rng.normal(0.0, yaw_std))
    if vel_std > 0.0:
        y[3:5] += rng.normal(0.0, vel_std, size=2)
    if omega_std > 0.0:
        y[5] += rng.normal(0.0, omega_std)
    return y


# -----------------------------------------------------------------------------
# Leader, plant, simulation cases
# -----------------------------------------------------------------------------

def circle_leader_state(t: float, radius: float = 2.0, speed: float = 0.45,
                        direction: float = 1.0, heading_fixed: bool = False,
                        heading: float = 0.0) -> np.ndarray:
    omega_path = direction * speed / radius
    phi = omega_path * t
    px = radius * np.cos(phi)
    py = radius * np.sin(phi)
    vx_map = -speed * np.sin(phi)
    vy_map = speed * np.cos(phi)
    if heading_fixed:
        theta = heading
        v_body = map_to_body(theta, np.array([vx_map, vy_map]))
        omega_body = 0.0
    else:
        theta = wrap_angle(phi + direction * np.pi / 2.0)
        v_body = np.array([speed, 0.0])
        omega_body = omega_path
    return np.array([px, py, theta, v_body[0], v_body[1], omega_body])


def step_plant(x: np.ndarray, delayed_cmd: np.ndarray, h: float,
               tau_v: float, tau_w: float) -> np.ndarray:
    y = x.copy()
    y[3] += h * ((delayed_cmd[0] - y[3]) / tau_v)
    y[4] += h * ((delayed_cmd[1] - y[4]) / tau_v)
    y[5] += h * ((delayed_cmd[2] - y[5]) / tau_w)
    y[0:2] += h * body_to_map(y[2], y[3:5])
    y[2] = wrap_angle(y[2] + h * y[5])
    return y


def step_direct_plant(x: np.ndarray, cmd: np.ndarray, h: float) -> np.ndarray:
    y = x.copy()
    y[3:6] = cmd
    y[0:2] += h * body_to_map(y[2], y[3:5])
    y[2] = wrap_angle(y[2] + h * y[5])
    return y


def make_delay_line(initial_cmd: np.ndarray, delay: float, plant_dt: float) -> deque[np.ndarray]:
    """Create a ZOH delay line whose first new command acts at exactly delay."""
    if delay < 0.0 or plant_dt <= 0.0:
        raise ValueError("delay must be non-negative and plant_dt positive")
    delay_steps = delay / plant_dt
    if not np.isclose(delay_steps, round(delay_steps), atol=1e-12):
        raise ValueError("delay must be an integer multiple of plant_dt")
    return deque(
        [np.asarray(initial_cmd, dtype=float).copy() for _ in range(int(round(delay_steps)))],
    )


def advance_delay_line(delay_line: deque[np.ndarray], command: np.ndarray) -> np.ndarray:
    """Apply the oldest delayed command and append the current ZOH command."""
    if not delay_line:
        return np.asarray(command, dtype=float).copy()
    applied = delay_line.popleft()
    delay_line.append(np.asarray(command, dtype=float).copy())
    return applied


def simulate_case(kind: str, Tmax: float, h: float, tau_v: float, tau_w: float, Td: float,
                  pos_noise: float = 0.0, yaw_noise: float = 0.0,
                  vel_noise: float = 0.0, omega_noise: float = 0.0,
                  seed: int = 7, follower_yaw0: float = np.pi / 2.0,
                  offset_frame: str = "leader", leader_heading_fixed: bool = False,
                  leader_heading: float = 0.0, mass: float = 2.0,
                  inertia: float = 1.0, hpc_c_min: float = 0.5,
                  initial_min_lambda: float = 1.0,
                  switch_min_lambda: float = 4.0,
                  use_hpc: bool = True, plant_dt: float = 0.01):
    control_substeps = h / plant_dt
    if not np.isclose(control_substeps, round(control_substeps), atol=1e-12):
        raise ValueError("plant_dt must be an integer multiple of control period")
    control_substeps = int(round(control_substeps))
    ctrl = Hpc6DDisc(mass=mass, inertia=inertia, control_period=h,
                     hpc_c_min=hpc_c_min,
                     initial_min_lambda=initial_min_lambda,
                     switch_min_lambda=switch_min_lambda,
                     use_hpc=use_hpc,
                     offset_frame=offset_frame)
    rng = np.random.default_rng(seed)
    x1 = circle_leader_state(0.0, heading_fixed=leader_heading_fixed,
                             heading=leader_heading)
    x2 = np.array([4.2, -0.4, follower_yaw0, 0.0, 0.0, 0.0])

    delay_line = make_delay_line(x2[3:6], Td, plant_dt)
    hist_len = max(1, int(np.ceil(Td / h))) + 2
    vcmd_map_hist = deque([body_to_map(x2[2], x2[3:5]) for _ in range(hist_len)],
                          maxlen=hist_len)
    wcmd_hist = deque([np.array([x2[5]]) for _ in range(hist_len)], maxlen=hist_len)
    last_cmd = x2[3:6].copy()
    last_vcmd_map = body_to_map(x2[2], last_cmd[0:2])
    last_wcmd = float(last_cmd[2])

    x1_meas = add_measurement_noise(x1, pos_noise, yaw_noise, vel_noise, omega_noise, rng)
    x2_meas = add_measurement_noise(x2, pos_noise, yaw_noise, vel_noise, omega_noise, rng)
    if kind == "ideal":
        x1_ctrl = x1_meas
        x2_ctrl = x2_meas
    elif kind == "compensated":
        x1_ctrl = se2_predict_constant_twist(x1_meas, Td + tau_v)
        x2_ctrl = predict_follower_direction_a(x2_meas, vcmd_map_hist, wcmd_hist,
                                               last_vcmd_map, last_wcmd,
                                               tau_v, tau_w, Td, h)
    else:
        x1_ctrl = x1_meas
        x2_ctrl = x2_meas
    ctrl.init(x1_ctrl, x2_ctrl)

    rows = []
    t = 0.0
    while t < Tmax - 1e-12:
        x1 = circle_leader_state(t, heading_fixed=leader_heading_fixed,
                                 heading=leader_heading)
        x1_meas = add_measurement_noise(x1, pos_noise, yaw_noise, vel_noise, omega_noise, rng)
        x2_meas = add_measurement_noise(x2, pos_noise, yaw_noise, vel_noise, omega_noise, rng)

        if kind == "compensated":
            x1_ctrl = se2_predict_constant_twist(x1_meas, Td + tau_v)
            x2_ctrl = predict_follower_direction_a(x2_meas, vcmd_map_hist, wcmd_hist,
                                                   last_vcmd_map, last_wcmd,
                                                   tau_v, tau_w, Td, h)
        else:
            x1_ctrl = x1_meas
            x2_ctrl = x2_meas

        cmd = ctrl.command(x1_ctrl, x2_ctrl)
        cmd = np.array([
            np.clip(cmd[0], -1.0, 1.0),
            np.clip(cmd[1], -1.0, 1.0),
            np.clip(cmd[2], -0.8, 0.8),
        ])

        for _ in range(control_substeps):
            if kind == "ideal":
                x2 = step_direct_plant(x2, cmd, plant_dt)
            else:
                delayed_cmd = advance_delay_line(delay_line, cmd)
                x2 = step_plant(x2, delayed_cmd, plant_dt, tau_v, tau_w)

        last_cmd = cmd.copy()
        last_vcmd_map = body_to_map(x2_meas[2], cmd[0:2])
        last_wcmd = float(cmd[2])
        vcmd_map_hist.appendleft(last_vcmd_map.copy())
        wcmd_hist.appendleft(np.array([last_wcmd]))

        t += h
        pos_dist, yaw_err, sixd_dist = ctrl.distance(x1, x2)
        e_cur, _, _, _ = ctrl.compute_error(x1, x2)
        cmd_map = body_to_map(x2_meas[2], cmd[0:2])
        rows.append((
            t, x1.copy(), x2.copy(), x1_ctrl.copy(), x2_ctrl.copy(), cmd.copy(),
            cmd_map.copy(), e_cur.copy(), pos_dist, yaw_err, sixd_dist,
            ctrl.fallback_count, ctrl.target_idx,
        ))
    return rows


# -----------------------------------------------------------------------------
# Plotting and metrics
# -----------------------------------------------------------------------------

def rows_to_arrays(rows):
    t = np.array([r[0] for r in rows])
    x1 = np.column_stack([r[1] for r in rows])
    x2 = np.column_stack([r[2] for r in rows])
    cmd = np.column_stack([r[5] for r in rows])
    cmd_map = np.column_stack([r[6] for r in rows])
    err = np.column_stack([r[7] for r in rows])
    pos_dist = np.array([r[8] for r in rows])
    yaw_err = np.array([r[9] for r in rows])
    sixd_dist = np.array([r[10] for r in rows])
    fallback = np.array([r[11] for r in rows])
    target_idx = np.array([r[12] for r in rows])
    return t, x1, x2, cmd, cmd_map, err, pos_dist, yaw_err, sixd_dist, fallback, target_idx


def metrics(pos_dist: np.ndarray, yaw_err: np.ndarray) -> tuple[float, float, float, float, float, float]:
    tail = slice(int(0.7 * len(pos_dist)), None)
    return (
        float(np.max(pos_dist)),
        float(np.mean(pos_dist[tail])),
        float(np.std(pos_dist[tail])),
        float(pos_dist[-1]),
        float(np.mean(yaw_err[tail])),
        float(yaw_err[-1]),
    )


def plot_compare(label: str, ideal_rows, original_rows, compensated_rows, out_dir: Path,
                 lpc_rows=None):
    ti, x1i, x2i, cmd_i, cmd_map_i, _, dist_i, yaw_i, _, _, _ = rows_to_arrays(ideal_rows)
    to, _, x2o, cmd_o, cmd_map_o, err_o, dist_o, yaw_o, _, _, _ = rows_to_arrays(original_rows)
    tc, _, x2c, cmd_c, cmd_map_c, err_c, dist_c, yaw_c, _, _, _ = rows_to_arrays(compensated_rows)
    if lpc_rows is not None:
        tl, _, x2l, cmd_l, cmd_map_l, _, dist_l, yaw_l, _, _, _ = rows_to_arrays(lpc_rows)

    fig, axs = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    axs[0, 0].plot(x1i[0], x1i[1], "k--", label="leader")
    axs[0, 0].plot(x2i[0], x2i[1], "0.55", label="ideal 6D Disc")
    if lpc_rows is not None:
        axs[0, 0].plot(x2l[0], x2l[1], "tab:green", label="ideal 6D Disc LPC")
    axs[0, 0].plot(x2o[0], x2o[1], "tab:red", label="6D Disc + delay")
    axs[0, 0].plot(x2c[0], x2c[1], "tab:blue", label="6D Artstein Disc")
    axs[0, 0].axis("equal")
    axs[0, 0].set(xlabel="x (m)", ylabel="y (m)", title=f"trajectory ({label})")

    axs[0, 1].plot(ti, dist_i, "0.55", label="ideal 6D Disc")
    if lpc_rows is not None:
        axs[0, 1].plot(tl, dist_l, "tab:green", label="ideal 6D Disc LPC")
    axs[0, 1].plot(to, dist_o, "tab:red", label="6D Disc + delay")
    axs[0, 1].plot(tc, dist_c, "tab:blue", label="6D Artstein Disc")
    axs[0, 1].set(xlabel="t (s)", ylabel="position formation error (m)", title="position error")

    axs[1, 0].plot(ti, yaw_i, "0.55", label="ideal yaw")
    if lpc_rows is not None:
        axs[1, 0].plot(tl, yaw_l, "tab:green", label="ideal LPC yaw")
    axs[1, 0].plot(to, yaw_o, "tab:red", label="delay yaw")
    axs[1, 0].plot(tc, yaw_c, "tab:blue", label="comp yaw")
    axs[1, 0].set(xlabel="t (s)", ylabel="yaw error (rad)", title="yaw error")

    if lpc_rows is not None:
        axs[1, 1].plot(tl, cmd_map_l[0], "tab:green", label="lpc vx map")
        axs[1, 1].plot(tl, cmd_map_l[1], "limegreen", label="lpc vy map")
        axs[1, 1].plot(tl, cmd_l[2], "darkgreen", label="lpc omega")
    axs[1, 1].plot(to, cmd_map_o[0], "tab:red", label="delay vx map")
    axs[1, 1].plot(to, cmd_map_o[1], "tab:orange", label="delay vy map")
    axs[1, 1].plot(to, cmd_o[2], "tab:pink", label="delay omega")
    axs[1, 1].plot(tc, cmd_map_c[0], "tab:blue", label="comp vx map")
    axs[1, 1].plot(tc, cmd_map_c[1], "tab:cyan", label="comp vy map")
    axs[1, 1].plot(tc, cmd_c[2], "tab:purple", label="comp omega")
    axs[1, 1].set(xlabel="t (s)", ylabel="cmd_vel", title="map-frame commands")

    for ax in axs.ravel():
        ax.grid(True)
        ax.legend(frameon=False)
    suffix = label.replace(" ", "_").replace("=", "").replace(",", "").replace("/", "_")
    path = out_dir / f"6d_disc_artstein_compare_{suffix}.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def write_summary(path: Path, rows_by_name: dict[str, tuple]):
    lines = [
        "case,max_pos_error,tail_mean_pos_error,tail_std_pos_error,"
        "final_pos_error,tail_mean_yaw_error,final_yaw_error,hpc_fallback_count,"
        "final_target_idx"
    ]
    for name, rows in rows_by_name.items():
        *_, pos_dist, yaw_err, _, fallback, target_idx = rows_to_arrays(rows)
        lines.append("{},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{},{}".format(
            name, *metrics(pos_dist, yaw_err), int(fallback[-1]), int(target_idx[-1])
        ))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def write_timeseries(path: Path, rows_by_name: dict[str, tuple]):
    lines = [
        "case,t,leader_x,leader_y,follower_x,follower_y,follower_yaw,"
        "cmd_vx_body,cmd_vy_body,cmd_w,cmd_vx_map,cmd_vy_map,"
        "err_x,err_y,err_yaw,pos_error,yaw_error,sixd_error,"
        "hpc_fallback_count,target_idx"
    ]
    for name, rows in rows_by_name.items():
        t, x1, x2, cmd, cmd_map, err, pos_dist, yaw_err, sixd_dist, fallback, target_idx = rows_to_arrays(rows)
        for i in range(len(t)):
            lines.append(
                "{},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},"
                "{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},"
                "{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{},{}".format(
                    name, t[i], x1[0, i], x1[1, i], x2[0, i], x2[1, i], x2[2, i],
                    cmd[0, i], cmd[1, i], cmd[2, i], cmd_map[0, i], cmd_map[1, i],
                    err[0, i], err[1, i], err[2, i],
                    pos_dist[i], yaw_err[i], sixd_dist[i],
                    int(fallback[i]), int(target_idx[i])
                )
            )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", default="homo_multirobot_formation_control/analysis/results/6d_artstein_disc")
    parser.add_argument("--tmax", type=float, default=60.0)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--tau", type=float, default=0.43)
    parser.add_argument("--tau-yaw", type=float, default=0.43)
    parser.add_argument("--Td", type=float, default=0.22)
    parser.add_argument("--mass", type=float, default=2.0)
    parser.add_argument("--inertia", type=float, default=1.0)
    parser.add_argument("--hpc-c-min", type=float, default=0.5)
    parser.add_argument("--initial-min-lambda", type=float, default=1.0)
    parser.add_argument("--switch-min-lambda", type=float, default=4.0)
    parser.add_argument("--pos-noise", type=float, default=0.02)
    parser.add_argument("--yaw-noise", type=float, default=0.02)
    parser.add_argument("--vel-noise", type=float, default=0.03)
    parser.add_argument("--omega-noise", type=float, default=0.03)
    parser.add_argument("--follower-yaw0", type=float, default=np.pi / 2.0,
                        help="Initial follower yaw in rad. Leader yaw starts at pi/2.")
    parser.add_argument("--offset-frame", choices=["leader", "map"], default="leader",
                        help="Formation offset frame. map gives 4D-like translated circles.")
    parser.add_argument("--leader-heading-fixed", action="store_true",
                        help="Keep leader yaw fixed, matching leader_circle.py in Gazebo.")
    parser.add_argument("--leader-heading", type=float, default=0.0,
                        help="Fixed leader yaw in rad when --leader-heading-fixed is set.")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    ideal = simulate_case("ideal", args.tmax, args.dt, args.tau, args.tau_yaw, 0.0,
                          follower_yaw0=args.follower_yaw0,
                          offset_frame=args.offset_frame,
                          leader_heading_fixed=args.leader_heading_fixed,
                          leader_heading=args.leader_heading,
                          mass=args.mass, inertia=args.inertia,
                          hpc_c_min=args.hpc_c_min,
                          initial_min_lambda=args.initial_min_lambda,
                          switch_min_lambda=args.switch_min_lambda)
    ideal_lpc = simulate_case("ideal", args.tmax, args.dt, args.tau, args.tau_yaw, 0.0,
                              follower_yaw0=args.follower_yaw0,
                              offset_frame=args.offset_frame,
                              leader_heading_fixed=args.leader_heading_fixed,
                              leader_heading=args.leader_heading,
                              mass=args.mass, inertia=args.inertia,
                              hpc_c_min=args.hpc_c_min,
                              initial_min_lambda=args.initial_min_lambda,
                              switch_min_lambda=args.switch_min_lambda,
                              use_hpc=False)
    original = simulate_case("original", args.tmax, args.dt, args.tau, args.tau_yaw, args.Td,
                             follower_yaw0=args.follower_yaw0,
                             offset_frame=args.offset_frame,
                             leader_heading_fixed=args.leader_heading_fixed,
                             leader_heading=args.leader_heading,
                             mass=args.mass, inertia=args.inertia,
                             hpc_c_min=args.hpc_c_min,
                             initial_min_lambda=args.initial_min_lambda,
                             switch_min_lambda=args.switch_min_lambda)
    compensated = simulate_case("compensated", args.tmax, args.dt, args.tau, args.tau_yaw, args.Td,
                                follower_yaw0=args.follower_yaw0,
                                offset_frame=args.offset_frame,
                                leader_heading_fixed=args.leader_heading_fixed,
                                leader_heading=args.leader_heading,
                                mass=args.mass, inertia=args.inertia,
                                hpc_c_min=args.hpc_c_min,
                                initial_min_lambda=args.initial_min_lambda,
                                switch_min_lambda=args.switch_min_lambda)
    noisy_original = simulate_case("original", args.tmax, args.dt, args.tau, args.tau_yaw, args.Td,
                                   args.pos_noise, args.yaw_noise, args.vel_noise, args.omega_noise,
                                   seed=11, follower_yaw0=args.follower_yaw0,
                                   offset_frame=args.offset_frame,
                                   leader_heading_fixed=args.leader_heading_fixed,
                                   leader_heading=args.leader_heading,
                                   mass=args.mass, inertia=args.inertia,
                                   hpc_c_min=args.hpc_c_min,
                                   initial_min_lambda=args.initial_min_lambda,
                                   switch_min_lambda=args.switch_min_lambda)
    noisy_compensated = simulate_case("compensated", args.tmax, args.dt, args.tau, args.tau_yaw, args.Td,
                                      args.pos_noise, args.yaw_noise, args.vel_noise, args.omega_noise,
                                      seed=11, follower_yaw0=args.follower_yaw0,
                                      offset_frame=args.offset_frame,
                                      leader_heading_fixed=args.leader_heading_fixed,
                                      leader_heading=args.leader_heading,
                                      mass=args.mass, inertia=args.inertia,
                                      hpc_c_min=args.hpc_c_min,
                                      initial_min_lambda=args.initial_min_lambda,
                                      switch_min_lambda=args.switch_min_lambda)

    outputs = [
        plot_compare("clean", ideal, original, compensated, out_dir, ideal_lpc),
        plot_compare(
            f"noise pos={args.pos_noise}, yaw={args.yaw_noise}",
            ideal, noisy_original, noisy_compensated, out_dir, ideal_lpc),
        write_summary(out_dir / "summary_metrics.csv", {
            "ideal_6d_disc": ideal,
            "ideal_6d_disc_lpc": ideal_lpc,
            "6d_disc_original_delay_clean": original,
            "6d_artstein_disc_clean": compensated,
            "6d_disc_original_delay_noise": noisy_original,
            "6d_artstein_disc_noise": noisy_compensated,
        }),
        write_timeseries(out_dir / "timeseries_clean.csv", {
            "ideal_6d_disc": ideal,
            "ideal_6d_disc_lpc": ideal_lpc,
            "6d_disc_original_delay_clean": original,
            "6d_artstein_disc_clean": compensated,
        }),
    ]
    for path in outputs:
        print(path)


if __name__ == "__main__":
    main()
