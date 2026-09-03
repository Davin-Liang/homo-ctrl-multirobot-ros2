#!/usr/bin/env python3
"""
Numerical experiments for the 4D double-integrator HPC architecture.

Outputs:
  1. Python reproduction of matlab/source/lpc_hpc_distance_square.m.
  2. Original 4D HPC under command dead time + motor lag.
  3. First-order motor forward prediction without delay compensation.
  4. Proposed Artstein + forward-prediction layer under the same delays.
  5. Circle-leader tests, with and without position/velocity measurement noise.
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
# MATLAB HPC toolbox functions ported to Python
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


def lpc2hpc(A: np.ndarray, B: np.ndarray, K: np.ndarray):
    eig_cl = np.linalg.eigvals(A + B @ K)
    rho = -np.max(np.real(eig_cl)) * 0.001
    if rho < 1e-5:
        raise ValueError("The linear control system has insufficient stability margin")

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
    mu_min = max(-1.0, -1.0 / lambda_max + 1e-5) if lambda_max > 1e-5 else -1.0
    mu_max = min(1.0 / k, -1.0 / lambda_min) if lambda_min < -1e-5 else 1.0 / k
    return K0, G0, P, mu_min, mu_max


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
# 4D double-integrator HPC core
# -----------------------------------------------------------------------------

@dataclass
class Hpc4D:
    mass: float = 2.0
    radius: float = 1.0
    m_p: int = 4
    tol: float = 0.1
    c_min: float = 0.1
    initial_min_lambda: float = 1.0
    switch_min_lambda: float = 4.0
    use_hpc: bool = True

    def __post_init__(self):
        self.A = np.block([[np.zeros((2, 2)), np.eye(2)], [np.zeros((2, 2)), np.zeros((2, 2))]])
        self.B = np.vstack([np.zeros((2, 2)), np.eye(2) / self.mass])
        self.dl = np.column_stack([
            np.array([-self.radius * np.cos(2.0 * np.pi * i / self.m_p),
                      -self.radius * np.sin(2.0 * np.pi * i / self.m_p), 0.0, 0.0])
            for i in range(self.m_p)
        ])
        self.d = self.dl[:, 0].copy()
        self.K = np.zeros((2, 4))
        self.Gd = np.eye(4)
        self.P = np.eye(4)
        self.nu = 0.0

    def init(self, x1: np.ndarray, x2: np.ndarray):
        distances = [np.linalg.norm(x2 - x1 - self.dl[:, i]) for i in range(self.m_p)]
        self.d = self.dl[:, int(np.argmin(distances))].copy()
        self._rebuild(x2 - x1 - self.d, self.initial_min_lambda)

    def accel(self, x1: np.ndarray, x2: np.ndarray) -> np.ndarray:
        self._switch_if_needed(x1, x2)
        e = x2 - x1 - self.d
        if self.use_hpc:
            nx = hnorm(e, self.Gd, self.P)
            c = min(1.0, max(self.c_min, nx))
            return (c ** (1.0 + self.nu)) * self.K @ expm(self.Gd * (1.0 - np.log(c))) @ e
        return self.K @ e

    def distance(self, x1: np.ndarray, x2: np.ndarray) -> float:
        return float(np.linalg.norm(x2 - x1 - self.d))

    def _switch_if_needed(self, x1: np.ndarray, x2: np.ndarray):
        distances = [np.linalg.norm(x2 - x1 - self.dl[:, i]) for i in range(self.m_p)]
        best_idx = int(np.argmin(distances))
        current_dist = np.linalg.norm(x2 - x1 - self.d)
        if distances[best_idx] + self.tol < current_dist:
            self.d = self.dl[:, best_idx].copy()
            self._rebuild(x2 - x1 - self.d, self.switch_min_lambda)

    def _rebuild(self, e: np.ndarray, min_lambda: float):
        self.K = calculate_klin_matlab(e, self.mass, min_lambda)
        if self.use_hpc:
            _, G0, P, nu_min, _ = lpc2hpc(self.A, self.B, self.K)
            self.P = P
            self.nu = nu_min
            self.Gd = np.eye(4) + self.nu * G0


def calculate_klin_matlab(e: np.ndarray, mass: float, min_lambda: float) -> np.ndarray:
    ax = -mass * e[2] / e[0] if abs(e[0]) > 1e-12 else 0.0
    ay = -mass * e[3] / e[1] if abs(e[1]) > 1e-12 else 0.0
    ax = max(ax, min_lambda)
    ay = max(ay, min_lambda)
    K = np.zeros((2, 4))
    K[0, 0] = ax * (-2.0 * ax + ax) / mass
    K[1, 1] = ay * (-2.0 * ay + ay) / mass
    K[0, 2] = -2.0 * ax
    K[1, 3] = -2.0 * ay
    return K


# -----------------------------------------------------------------------------
# Trajectories and actuator prediction
# -----------------------------------------------------------------------------

def matlab_leader_accel(t: float, x1: np.ndarray) -> np.ndarray:
    return -np.array([x1[0] + x1[2], x1[1] + x1[3]]) + np.array([np.sin(t), np.cos(t)])


def circle_leader_state(t: float, radius: float = 2.0, omega: float = 0.25) -> np.ndarray:
    return np.array([
        radius * np.cos(omega * t),
        radius * np.sin(omega * t),
        -radius * omega * np.sin(omega * t),
        radius * omega * np.cos(omega * t),
    ])


def actuator_matrices(tau: float) -> tuple[np.ndarray, np.ndarray]:
    A = np.array([[0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, 0.0, 1.0],
                  [0.0, 0.0, -1.0 / tau, 0.0],
                  [0.0, 0.0, 0.0, -1.0 / tau]])
    B = np.array([[0.0, 0.0], [0.0, 0.0], [1.0 / tau, 0.0], [0.0, 1.0 / tau]])
    return A, B


def artstein_integral(history: deque[np.ndarray], tau: float, Td: float, h: float) -> np.ndarray:
    if Td <= 0.0:
        return np.zeros(4)
    A, B = actuator_matrices(tau)
    n = max(1, int(np.ceil(Td / h)))
    integral = np.zeros(4)
    for k in range(min(n, len(history))):
        weight = h if k < n - 1 else Td - (n - 1) * h
        integral += expm(A * (k * h - Td)) @ B @ history[k] * weight
    return integral


def predict_follower_state_first_order(state: np.ndarray, vcmd: np.ndarray, tau: float) -> np.ndarray:
    decay = np.exp(-1.0)
    velocity = vcmd + decay * (state[2:4] - vcmd)
    position = state[0:2] + vcmd * tau + tau * (1.0 - decay) * (state[2:4] - vcmd)
    return np.r_[position, velocity]


def predict_follower_state_from_artstein(z: np.ndarray, vcmd: np.ndarray, tau: float, Td: float) -> np.ndarray:
    A, _ = actuator_matrices(tau)
    delay_free = expm(A * Td) @ z
    decay = np.exp(-1.0)
    v_pred = vcmd + decay * (delay_free[2:4] - vcmd)
    p_pred = delay_free[0:2] + vcmd * tau + tau * (1.0 - decay) * (delay_free[2:4] - vcmd)
    return np.r_[p_pred, v_pred]


def predict_leader_state(x: np.ndarray, tau: float, Td: float) -> np.ndarray:
    predicted = x.copy()
    predicted[0:2] += x[2:4] * (Td + tau)
    return predicted


def add_measurement_noise(x: np.ndarray, pos_std: float, vel_std: float, rng: np.random.Generator) -> np.ndarray:
    y = x.copy()
    if pos_std > 0.0:
        y[0:2] += rng.normal(0.0, pos_std, size=2)
    if vel_std > 0.0:
        y[2:4] += rng.normal(0.0, vel_std, size=2)
    return y


# -----------------------------------------------------------------------------
# Simulation cases
# -----------------------------------------------------------------------------

def simulate_paper(Tmax: float, h: float):
    mass = 2.0
    A = np.block([[np.zeros((2, 2)), np.eye(2)], [np.zeros((2, 2)), np.zeros((2, 2))]])
    B = np.vstack([np.zeros((2, 2)), np.eye(2) / mass])
    ctrl = Hpc4D(mass=mass, radius=1.0, c_min=0.1)
    x1 = np.array([1.0, 0.0, 0.0, 0.0])
    x2 = np.array([5.0, 1.0, 0.0, 0.0])
    ctrl.init(x1, x2)

    rows = []
    t = 0.0
    while t < Tmax - 1e-12:
        u1 = matlab_leader_accel(t, x1)
        x1 = x1 + h * (A @ x1 + B @ u1)
        u2 = ctrl.accel(x1, x2)
        x2 = x2 + h * (A @ x2 + B @ u2)
        t += h
        e = x2 - x1 - ctrl.d
        rows.append((t, x1.copy(), x2.copy(), u2.copy(), e.copy(), ctrl.distance(x1, x2)))
    return rows


def simulate_delay_case(kind: str, Tmax: float, h: float, tau: float, Td: float):
    mass = 2.0
    A_di = np.block([[np.zeros((2, 2)), np.eye(2)], [np.zeros((2, 2)), np.zeros((2, 2))]])
    B_di = np.vstack([np.zeros((2, 2)), np.eye(2) / mass])
    ctrl = Hpc4D(mass=mass, radius=1.0, c_min=0.1)
    x1 = np.array([1.0, 0.0, 0.0, 0.0])
    x2 = np.array([5.0, 1.0, 0.0, 0.0])

    delay_steps = max(1, int(np.ceil(Td / h)))
    delay_line = deque([x2[2:4].copy() for _ in range(delay_steps + 1)], maxlen=delay_steps + 1)
    hist_len = max(1, int(np.ceil(Td / h))) + 2
    cmd_history = deque([x2[2:4].copy() for _ in range(hist_len)], maxlen=hist_len)
    last_cmd = x2[2:4].copy()

    x1_meas = x1
    x2_meas = x2
    if kind == "compensated":
        z2 = x2_meas + artstein_integral(cmd_history, tau, Td, h)
        x2_ctrl = predict_follower_state_from_artstein(z2, last_cmd, tau, Td)
        x1_ctrl = predict_leader_state(x1_meas, tau, Td)
    elif kind == "forward_prediction_only":
        x2_ctrl = predict_follower_state_first_order(x2_meas, last_cmd, tau)
        x1_ctrl = x1_meas
    else:
        x1_ctrl = x1_meas
        x2_ctrl = x2_meas
    ctrl.init(x1_ctrl, x2_ctrl)

    rows = []
    t = 0.0
    while t < Tmax - 1e-12:
        u1 = matlab_leader_accel(t, x1)
        x1 = x1 + h * (A_di @ x1 + B_di @ u1)

        x1_meas = x1
        x2_meas = x2
        if kind == "compensated":
            z2 = x2_meas + artstein_integral(cmd_history, tau, Td, h)
            x2_ctrl = predict_follower_state_from_artstein(z2, last_cmd, tau, Td)
            x1_ctrl = predict_leader_state(x1_meas, tau, Td)
        elif kind == "forward_prediction_only":
            x2_ctrl = predict_follower_state_first_order(x2_meas, last_cmd, tau)
            x1_ctrl = x1_meas
        else:
            x1_ctrl = x1_meas
            x2_ctrl = x2_meas

        accel = ctrl.accel(x1_ctrl, x2_ctrl)
        vcmd = np.clip(x2_ctrl[2:4] + h * (accel / mass), -1.5, 1.5)

        delay_line.appendleft(vcmd.copy())
        delayed_cmd = delay_line[-1]
        x2[2:4] += h * ((delayed_cmd - x2[2:4]) / tau)
        x2[0:2] += h * x2[2:4]

        last_cmd = vcmd.copy()
        cmd_history.appendleft(vcmd.copy())
        t += h
        e_real = x2 - x1 - ctrl.d
        rows.append((t, x1.copy(), x2.copy(), x1_ctrl.copy(), x2_ctrl.copy(), vcmd.copy(), e_real.copy(), ctrl.distance(x1, x2)))
    return rows


def simulate_circle_case(kind: str, Tmax: float, h: float, tau: float, Td: float,
                         pos_noise: float = 0.0, vel_noise: float = 0.0, seed: int = 7):
    mass = 2.0
    ctrl = Hpc4D(mass=mass, radius=1.0, c_min=0.1)
    rng = np.random.default_rng(seed)
    x1 = circle_leader_state(0.0)
    x2 = np.array([4.5, 0.0, 0.0, 0.0])

    delay_steps = max(1, int(np.ceil(Td / h)))
    delay_line = deque([x2[2:4].copy() for _ in range(delay_steps + 1)], maxlen=delay_steps + 1)
    hist_len = max(1, int(np.ceil(Td / h))) + 2
    cmd_history = deque([x2[2:4].copy() for _ in range(hist_len)], maxlen=hist_len)
    last_cmd = x2[2:4].copy()

    x1_meas = add_measurement_noise(x1, pos_noise, vel_noise, rng)
    x2_meas = add_measurement_noise(x2, pos_noise, vel_noise, rng)
    if kind == "compensated":
        z2 = x2_meas + artstein_integral(cmd_history, tau, Td, h)
        x2_ctrl = predict_follower_state_from_artstein(z2, last_cmd, tau, Td)
        x1_ctrl = predict_leader_state(x1_meas, tau, Td)
    elif kind == "forward_prediction_only":
        x2_ctrl = predict_follower_state_first_order(x2_meas, last_cmd, tau)
        x1_ctrl = x1_meas
    else:
        x1_ctrl = x1_meas
        x2_ctrl = x2_meas
    ctrl.init(x1_ctrl, x2_ctrl)

    rows = []
    t = 0.0
    while t < Tmax - 1e-12:
        x1 = circle_leader_state(t)
        x1_meas = add_measurement_noise(x1, pos_noise, vel_noise, rng)
        x2_meas = add_measurement_noise(x2, pos_noise, vel_noise, rng)

        if kind == "compensated":
            z2 = x2_meas + artstein_integral(cmd_history, tau, Td, h)
            x2_ctrl = predict_follower_state_from_artstein(z2, last_cmd, tau, Td)
            x1_ctrl = predict_leader_state(x1_meas, tau, Td)
        elif kind == "forward_prediction_only":
            x2_ctrl = predict_follower_state_first_order(x2_meas, last_cmd, tau)
            x1_ctrl = x1_meas
        else:
            x1_ctrl = x1_meas
            x2_ctrl = x2_meas

        accel = ctrl.accel(x1_ctrl, x2_ctrl)
        vcmd = np.clip(x2_ctrl[2:4] + h * (accel / mass), -1.5, 1.5)

        delay_line.appendleft(vcmd.copy())
        delayed_cmd = delay_line[-1]
        x2[2:4] += h * ((delayed_cmd - x2[2:4]) / tau)
        x2[0:2] += h * x2[2:4]

        last_cmd = vcmd.copy()
        cmd_history.appendleft(vcmd.copy())
        t += h
        e_real = x2 - x1 - ctrl.d
        rows.append((t, x1.copy(), x2.copy(), x1_ctrl.copy(), x2_ctrl.copy(), vcmd.copy(), e_real.copy(), ctrl.distance(x1, x2)))
    return rows


# -----------------------------------------------------------------------------
# Plotting and metrics
# -----------------------------------------------------------------------------

def rows_to_arrays(rows, delayed=False):
    t = np.array([r[0] for r in rows])
    x1 = np.column_stack([r[1] for r in rows])
    x2 = np.column_stack([r[2] for r in rows])
    if delayed:
        cmd = np.column_stack([r[5] for r in rows])
        err = np.column_stack([r[6] for r in rows])
        dist = np.array([r[7] for r in rows])
        return t, x1, x2, cmd, err, dist
    u2 = np.column_stack([r[3] for r in rows])
    err = np.column_stack([r[4] for r in rows])
    dist = np.array([r[5] for r in rows])
    return t, x1, x2, u2, err, dist


def metrics(dist: np.ndarray) -> tuple[float, float, float, float]:
    tail = dist[int(0.7 * len(dist)):]
    return float(np.max(dist)), float(np.mean(tail)), float(np.std(tail)), float(dist[-1])


def plot_paper(rows, out_dir: Path):
    t, x1, x2, u2, err, dist = rows_to_arrays(rows)
    fig, axs = plt.subplots(3, 2, figsize=(12, 10), constrained_layout=True)
    axs[0, 0].plot(t, x1[0], "r", label="$x_1$")
    axs[0, 0].plot(t, x2[0], "b", label="$x_2$")
    axs[0, 0].set(xlabel="t (s)", ylabel="x")
    axs[0, 1].plot(t, x1[1], "r", label="$y_1$")
    axs[0, 1].plot(t, x2[1], "b", label="$y_2$")
    axs[0, 1].set(xlabel="t (s)", ylabel="y")
    axs[1, 0].plot(x1[0], x1[1], "r", label="$r_1$")
    axs[1, 0].plot(x2[0], x2[1], "b", label="$r_2$")
    axs[1, 0].set(xlabel="x", ylabel="y", ylim=(-0.8, 1.2))
    axs[1, 1].plot(t, u2[0], "r", label="$u_x$")
    axs[1, 1].plot(t, u2[1], "b", label="$u_y$")
    axs[1, 1].set(xlabel="t (s)", ylabel="u")
    axs[2, 0].plot(t, err[0], "r", label="$e_x$")
    axs[2, 0].plot(t, err[1], "b", label="$e_y$")
    axs[2, 0].set(xlabel="t (s)", ylabel="formation error")
    axs[2, 1].plot(t, dist, "k", label="$||e||$")
    axs[2, 1].set(xlabel="t (s)", ylabel="selected target error norm", ylim=(0, 3.5))
    for ax in axs.ravel():
        ax.grid(True)
        ax.legend(frameon=False)
    path = out_dir / "paper_lpc_hpc_distance_square_reproduction.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def plot_delay_compare(ideal_rows, original_rows, prediction_only_rows, compensated_rows, out_dir: Path):
    ti, x1i, x2i, _, _, dist_i = rows_to_arrays(ideal_rows)
    to, _, x2o, cmd_o, err_o, dist_o = rows_to_arrays(original_rows, delayed=True)
    tp, _, x2p, cmd_p, err_p, dist_p = rows_to_arrays(prediction_only_rows, delayed=True)
    tc, _, x2c, cmd_c, err_c, dist_c = rows_to_arrays(compensated_rows, delayed=True)

    fig, axs = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    axs[0, 0].plot(x1i[0], x1i[1], "k--", label="leader")
    axs[0, 0].plot(x2i[0], x2i[1], "0.6", label="ideal 4D HPC")
    axs[0, 0].plot(x2o[0], x2o[1], "tab:red", label="original + delay")
    axs[0, 0].plot(x2p[0], x2p[1], "tab:orange", label="prediction-only + delay")
    axs[0, 0].plot(x2c[0], x2c[1], "tab:blue", label="Artstein + prediction")
    axs[0, 0].set(xlabel="x", ylabel="y", title="MATLAB leader trajectory")
    axs[0, 1].plot(ti, dist_i, "0.6", label="ideal 4D HPC")
    axs[0, 1].plot(to, dist_o, "tab:red", label="original + delay")
    axs[0, 1].plot(tp, dist_p, "tab:orange", label="prediction-only + delay")
    axs[0, 1].plot(tc, dist_c, "tab:blue", label="Artstein + prediction")
    axs[0, 1].set(xlabel="t (s)", ylabel="selected target error norm", title="formation error")
    axs[1, 0].plot(to, err_o[0], "tab:red", label="orig $e_x$")
    axs[1, 0].plot(to, err_o[1], "tab:orange", label="orig $e_y$")
    axs[1, 0].plot(tp, err_p[0], "tab:orange", label="pred $e_x$")
    axs[1, 0].plot(tp, err_p[1], "gold", label="pred $e_y$")
    axs[1, 0].plot(tc, err_c[0], "tab:blue", label="comp $e_x$")
    axs[1, 0].plot(tc, err_c[1], "tab:cyan", label="comp $e_y$")
    axs[1, 0].set(xlabel="t (s)", ylabel="formation error", title="component error")
    axs[1, 1].plot(to, cmd_o[0], "tab:red", label="orig $v_x^{cmd}$")
    axs[1, 1].plot(to, cmd_o[1], "tab:orange", label="orig $v_y^{cmd}$")
    axs[1, 1].plot(tp, cmd_p[0], "tab:orange", label="pred $v_x^{cmd}$")
    axs[1, 1].plot(tp, cmd_p[1], "gold", label="pred $v_y^{cmd}$")
    axs[1, 1].plot(tc, cmd_c[0], "tab:blue", label="comp $v_x^{cmd}$")
    axs[1, 1].plot(tc, cmd_c[1], "tab:cyan", label="comp $v_y^{cmd}$")
    axs[1, 1].set(xlabel="t (s)", ylabel="cmd_vel (m/s)", title="velocity command")
    for ax in axs.ravel():
        ax.grid(True)
        ax.legend(frameon=False)
    path = out_dir / "delay_original_vs_artstein_prediction.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def plot_circle_compare(noise_label: str, original_rows, prediction_only_rows, compensated_rows, out_dir: Path):
    to, x1o, x2o, cmd_o, err_o, dist_o = rows_to_arrays(original_rows, delayed=True)
    tp, _, x2p, cmd_p, err_p, dist_p = rows_to_arrays(prediction_only_rows, delayed=True)
    tc, x1c, x2c, cmd_c, err_c, dist_c = rows_to_arrays(compensated_rows, delayed=True)
    fig, axs = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    axs[0, 0].plot(x1o[0], x1o[1], "k--", label="leader circle")
    axs[0, 0].plot(x2o[0], x2o[1], "tab:red", label="original 4D + delay")
    axs[0, 0].plot(x2p[0], x2p[1], "tab:orange", label="prediction-only 4D + delay")
    axs[0, 0].plot(x2c[0], x2c[1], "tab:blue", label="Artstein + prediction")
    axs[0, 0].axis("equal")
    axs[0, 0].set(xlabel="x", ylabel="y", title=f"circle trajectory ({noise_label})")
    axs[0, 1].plot(to, dist_o, "tab:red", label="original 4D + delay")
    axs[0, 1].plot(tp, dist_p, "tab:orange", label="prediction-only 4D + delay")
    axs[0, 1].plot(tc, dist_c, "tab:blue", label="Artstein + prediction")
    axs[0, 1].set(xlabel="t (s)", ylabel="selected target error norm", title="formation error")
    axs[1, 0].plot(to, err_o[0], "tab:red", label="orig $e_x$")
    axs[1, 0].plot(to, err_o[1], "tab:orange", label="orig $e_y$")
    axs[1, 0].plot(tp, err_p[0], "tab:orange", label="pred $e_x$")
    axs[1, 0].plot(tp, err_p[1], "gold", label="pred $e_y$")
    axs[1, 0].plot(tc, err_c[0], "tab:blue", label="comp $e_x$")
    axs[1, 0].plot(tc, err_c[1], "tab:cyan", label="comp $e_y$")
    axs[1, 0].set(xlabel="t (s)", ylabel="formation error", title="component error")
    axs[1, 1].plot(to, cmd_o[0], "tab:red", label="orig $v_x^{cmd}$")
    axs[1, 1].plot(to, cmd_o[1], "tab:orange", label="orig $v_y^{cmd}$")
    axs[1, 1].plot(tp, cmd_p[0], "tab:orange", label="pred $v_x^{cmd}$")
    axs[1, 1].plot(tp, cmd_p[1], "gold", label="pred $v_y^{cmd}$")
    axs[1, 1].plot(tc, cmd_c[0], "tab:blue", label="comp $v_x^{cmd}$")
    axs[1, 1].plot(tc, cmd_c[1], "tab:cyan", label="comp $v_y^{cmd}$")
    axs[1, 1].set(xlabel="t (s)", ylabel="cmd_vel (m/s)", title="velocity command")
    for ax in axs.ravel():
        ax.grid(True)
        ax.legend(frameon=False)
    suffix = "noise" if noise_label != "no noise" else "clean"
    path = out_dir / f"circle_original_vs_artstein_{suffix}.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def write_summary(path: Path, rows_by_name: dict[str, tuple]):
    lines = ["case,max_distance,tail_mean_distance,tail_std_distance,final_distance"]
    for name, rows in rows_by_name.items():
        *_, dist = rows_to_arrays(rows, delayed=(len(rows[0]) == 8))
        lines.append("{},{:.6f},{:.6f},{:.6f},{:.6f}".format(name, *metrics(dist)))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", default="analysis/results/4d_artstein")
    parser.add_argument("--tmax", type=float, default=30.0)
    parser.add_argument("--circle-tmax", type=float, default=60.0)
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--tau", type=float, default=0.43)
    parser.add_argument("--Td", type=float, default=0.22)
    parser.add_argument("--pos-noise", type=float, default=0.02)
    parser.add_argument("--vel-noise", type=float, default=0.03)
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    paper_rows = simulate_paper(args.tmax, args.dt)
    delay_orig = simulate_delay_case("original", args.tmax, args.dt, args.tau, args.Td)
    delay_prediction_only = simulate_delay_case("forward_prediction_only", args.tmax, args.dt, args.tau, args.Td)
    delay_comp = simulate_delay_case("compensated", args.tmax, args.dt, args.tau, args.Td)

    circle_orig = simulate_circle_case("original", args.circle_tmax, args.dt, args.tau, args.Td)
    circle_prediction_only = simulate_circle_case("forward_prediction_only", args.circle_tmax, args.dt, args.tau, args.Td)
    circle_comp = simulate_circle_case("compensated", args.circle_tmax, args.dt, args.tau, args.Td)
    circle_noise_orig = simulate_circle_case("original", args.circle_tmax, args.dt, args.tau, args.Td,
                                             args.pos_noise, args.vel_noise, seed=11)
    circle_noise_prediction_only = simulate_circle_case("forward_prediction_only", args.circle_tmax, args.dt, args.tau, args.Td,
                                                        args.pos_noise, args.vel_noise, seed=11)
    circle_noise_comp = simulate_circle_case("compensated", args.circle_tmax, args.dt, args.tau, args.Td,
                                             args.pos_noise, args.vel_noise, seed=11)

    outputs = [
        plot_paper(paper_rows, out_dir),
        plot_delay_compare(paper_rows, delay_orig, delay_prediction_only, delay_comp, out_dir),
        plot_circle_compare("no noise", circle_orig, circle_prediction_only, circle_comp, out_dir),
        plot_circle_compare(f"pos σ={args.pos_noise}m, vel σ={args.vel_noise}m/s",
                            circle_noise_orig, circle_noise_prediction_only, circle_noise_comp, out_dir),
        write_summary(out_dir / "summary_metrics.csv", {
            "ideal_4d_hpc_matlab": paper_rows,
            "matlab_leader_original_delay": delay_orig,
            "matlab_leader_forward_prediction_only": delay_prediction_only,
            "matlab_leader_artstein_prediction": delay_comp,
            "circle_original_delay_clean": circle_orig,
            "circle_forward_prediction_only_clean": circle_prediction_only,
            "circle_artstein_prediction_clean": circle_comp,
            "circle_original_delay_noise": circle_noise_orig,
            "circle_forward_prediction_only_noise": circle_noise_prediction_only,
            "circle_artstein_prediction_noise": circle_noise_comp,
        }),
    ]
    for path in outputs:
        print(path)


if __name__ == "__main__":
    main()
