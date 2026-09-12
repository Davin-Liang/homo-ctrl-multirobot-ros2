# 4D Artstein-LQR Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a 4D DARE-LQR numerical baseline that shares the existing Artstein/input-delay and first-order motor prediction layer with the current 4D HPC simulation.

**Architecture:** Extend the existing Python numerical comparison module with a focused `Lqr4D` controller, then add a thin LQR entrypoint script for the requested plots and CSVs. Reuse the existing 4D ZOH model, formation target switching, predictor layer, delay plant, command limits, plotting, summary CSV, and tests. Keep the existing MPC script behavior intact.

**Tech Stack:** Python 3, NumPy, SciPy `solve_discrete_are`, Matplotlib Agg, pytest.

## Global Constraints

- The LQR method is a DARE baseline on the predicted delay-free 4D double-integrator state, not a strict optimal controller for the original delayed plant.
- Delay handling must reuse the current project method: Artstein compensation for `Td` plus first-order motor forward prediction for `tau`.
- Main LQR comparison must include `original_4d_delay`, `artstein_hpc_delay`, and `artstein_lqr_delay`.
- No-delay sanity checks must include `artstein_hpc_no_delay` and `artstein_lqr_no_delay`.
- Do not include `artstein_mpc_delay` in the LQR primary comparison.
- Do not add a ROS C++ LQR node in this plan.
- Do not implement delay-state-augmented LQR in this plan.
- Keep changes surgical and avoid refactoring unrelated MPC/HPC code.

---

## File Structure

- Modify: `homo_multirobot_formation_control/scripts/sim_4d_artstein_mpc_compare.py`
  - Add `Lqr4D`.
  - Extend controller factory and raw command generation.
- Create: `homo_multirobot_formation_control/scripts/sim_4d_artstein_lqr_compare.py`
  - Add LQR CLI parameters.
  - Add LQR-specific experiment grouping and output files.
- Modify: `homo_multirobot_formation_control/CMakeLists.txt`
  - Install the new LQR simulation script with the other Python scripts.
- Modify: `homo_multirobot_formation_control/test/test_sim_4d_artstein_mpc_compare.py`
  - Add tests for DARE stability and short closed-loop error reduction.
  - Keep existing MPC tests passing.
- Create: `homo_multirobot_formation_control/doc/4d_artstein_lqr_simulation.md`
  - Document theory, delay boundary, run command, output files, and default weights.

---

### Task 1: Add Failing Tests for 4D DARE-LQR

**Files:**
- Modify: `homo_multirobot_formation_control/test/test_sim_4d_artstein_mpc_compare.py`

**Interfaces:**
- Consumes: existing `load_module()` helper in the test file.
- Produces: expectations for `sim.Lqr4D`, `sim.make_controller("lqr", dt, args)`, and `sim.simulate_circle_case("lqr", ...)`.

- [ ] **Step 1: Inspect current test file**

Run:

```bash
sed -n '1,220p' homo_multirobot_formation_control/test/test_sim_4d_artstein_mpc_compare.py
```

Expected: existing tests import the simulation module and cover ZOH, MPC command semantics, and short MPC closed-loop behavior.

- [ ] **Step 2: Add LQR tests**

Add this import near the top of `homo_multirobot_formation_control/test/test_sim_4d_artstein_mpc_compare.py`:

```python
import argparse
```

Append these tests to the same file:

```python
def test_lqr_dare_feedback_is_discrete_stable():
    sim = load_module()
    controller = sim.Lqr4D(
        mass=2.0,
        dt=0.05,
        radius=2.0,
        m_p=4,
        tol=0.1,
        q=(40.0, 40.0, 1.0, 1.0),
        r=(0.02, 0.02),
    )

    eigvals = np.linalg.eigvals(controller.Ad - controller.Bd @ controller.K)

    assert controller.K.shape == (2, 4)
    assert np.max(np.abs(eigvals)) < 1.0


def test_lqr_factory_uses_lqr_parameters():
    sim = load_module()
    args = argparse.Namespace(
        mass=2.0,
        radius=2.0,
        m_p=4,
        tol=0.1,
        lqr_q_px=40.0,
        lqr_q_py=30.0,
        lqr_q_vx=2.0,
        lqr_q_vy=1.5,
        lqr_r_ux=0.03,
        lqr_r_uy=0.04,
    )

    controller = sim.make_controller("lqr", 0.05, args)

    assert isinstance(controller, sim.Lqr4D)
    assert np.allclose(np.diag(controller.Q), [40.0, 30.0, 2.0, 1.5])
    assert np.allclose(np.diag(controller.R), [0.03, 0.04])


def test_short_no_delay_lqr_closed_loop_reduces_formation_error():
    sim = load_module()
    args = argparse.Namespace(
        mass=2.0,
        radius=2.0,
        m_p=4,
        tol=0.1,
        hpc_c_min=0.1,
        initial_min_lambda=1.5,
        switch_min_lambda=4.0,
        mpc_horizon=30,
        q_px=40.0,
        q_py=40.0,
        q_vx=1.0,
        q_vy=1.0,
        r_ux=0.02,
        r_uy=0.02,
        terminal_factor=10.0,
        max_linear_vel=0.8,
        max_linear_accel=1.0,
        mpc_max_iter=320,
        mpc_eps_abs=1e-4,
        mpc_eps_rel=1e-3,
        mpc_rho=1.0,
        lqr_q_px=40.0,
        lqr_q_py=40.0,
        lqr_q_vx=1.0,
        lqr_q_vy=1.0,
        lqr_r_ux=0.02,
        lqr_r_uy=0.02,
        leader_radius=2.0,
        leader_omega=0.1,
    )

    rows = sim.simulate_circle_case(
        "lqr",
        "none",
        tmax=8.0,
        dt=0.05,
        tau=0.43,
        Td=0.22,
        max_speed=args.max_linear_vel,
        max_accel=args.max_linear_accel,
        args=args,
    )

    assert rows[-1].distance < rows[0].distance
```

- [ ] **Step 3: Run tests and verify they fail for missing LQR**

Run:

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_sim_4d_artstein_mpc_compare.py -q
```

Expected: FAIL because `sim.Lqr4D` and `"lqr"` controller factory support do not exist yet.

- [ ] **Step 4: Commit failing tests**

Run:

```bash
git add homo_multirobot_formation_control/test/test_sim_4d_artstein_mpc_compare.py
git commit -m "test: 添加4D LQR数值仿真用例"
```

---

### Task 2: Implement `Lqr4D` and Controller Factory Support

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/sim_4d_artstein_mpc_compare.py`

**Interfaces:**
- Consumes: `double_integrator_zoh(mass: float, dt: float) -> tuple[np.ndarray, np.ndarray]`.
- Produces: `class Lqr4D` with `init(x1, x2)`, `command(x1, x2) -> np.ndarray`, `distance(x1, x2) -> float`, `selected_error(x1, x2) -> np.ndarray`, and attributes `Ad`, `Bd`, `Q`, `R`, `K`, `target_idx`, `d`, `dl`.

- [ ] **Step 1: Add SciPy DARE import**

In `sim_4d_artstein_mpc_compare.py`, change:

```python
import numpy as np
```

to:

```python
import numpy as np
from scipy.linalg import solve_discrete_are
```

- [ ] **Step 2: Add `Lqr4D` after `Mpc4D`**

Insert this class after the `Mpc4D` class:

```python
class Lqr4D:
    """Discrete infinite-horizon LQR for the predicted 4D double integrator."""

    def __init__(
        self,
        mass: float = 2.0,
        dt: float = 0.05,
        radius: float = 2.0,
        m_p: int = 4,
        tol: float = 0.1,
        q: tuple[float, float, float, float] = (40.0, 40.0, 1.0, 1.0),
        r: tuple[float, float] = (0.02, 0.02),
    ):
        self.mass = mass
        self.dt = dt
        self.radius = radius
        self.m_p = m_p
        self.tol = tol
        self.Ad, self.Bd = double_integrator_zoh(mass, dt)
        self.Q = np.diag(q)
        self.R = np.diag(r)
        self.P = solve_discrete_are(self.Ad, self.Bd, self.Q, self.R)
        self.K = np.linalg.solve(
            self.R + self.Bd.T @ self.P @ self.Bd,
            self.Bd.T @ self.P @ self.Ad,
        )
        self.dl = np.column_stack(
            [
                np.array(
                    [
                        -radius * np.cos(2.0 * np.pi * i / m_p),
                        -radius * np.sin(2.0 * np.pi * i / m_p),
                        0.0,
                        0.0,
                    ]
                )
                for i in range(m_p)
            ]
        )
        self.target_idx = 0
        self.d = self.dl[:, 0].copy()

    def init(self, x1: np.ndarray, x2: np.ndarray):
        self.target_idx = self._best_target_idx(x1, x2)
        self.d = self.dl[:, self.target_idx].copy()

    def command(self, x1: np.ndarray, x2: np.ndarray) -> np.ndarray:
        self._switch_if_needed(x1, x2)
        error = self.selected_error(x1, x2)
        force_like_u = -self.K @ error
        return x2[2:4] + self.dt * (force_like_u / self.mass)

    def distance(self, x1: np.ndarray, x2: np.ndarray) -> float:
        return float(min(np.linalg.norm(x2 - x1 - self.dl[:, i]) for i in range(self.m_p)))

    def selected_error(self, x1: np.ndarray, x2: np.ndarray) -> np.ndarray:
        return x2 - x1 - self.d

    def _best_target_idx(self, x1: np.ndarray, x2: np.ndarray) -> int:
        distances = [np.linalg.norm(x2 - x1 - self.dl[:, i]) for i in range(self.m_p)]
        return int(np.argmin(distances))

    def _switch_if_needed(self, x1: np.ndarray, x2: np.ndarray):
        best_idx = self._best_target_idx(x1, x2)
        current_dist = np.linalg.norm(x2 - x1 - self.d)
        best_dist = np.linalg.norm(x2 - x1 - self.dl[:, best_idx])
        if best_dist + self.tol < current_dist:
            self.target_idx = best_idx
            self.d = self.dl[:, best_idx].copy()
```

- [ ] **Step 3: Update type unions**

Change:

```python
def selected_error(ctrl: Hpc4D | Mpc4D, x1: np.ndarray, x2: np.ndarray) -> np.ndarray:
    if isinstance(ctrl, Mpc4D):
        return ctrl.selected_error(x1, x2)
    return x2 - x1 - ctrl.d
```

to:

```python
def selected_error(ctrl: Hpc4D | Mpc4D | Lqr4D, x1: np.ndarray, x2: np.ndarray) -> np.ndarray:
    if isinstance(ctrl, (Mpc4D, Lqr4D)):
        return ctrl.selected_error(x1, x2)
    return x2 - x1 - ctrl.d
```

Change:

```python
def make_controller(controller_kind: str, dt: float, args) -> Hpc4D | Mpc4D:
```

to:

```python
def make_controller(controller_kind: str, dt: float, args) -> Hpc4D | Mpc4D | Lqr4D:
```

Change:

```python
def raw_velocity_command(ctrl: Hpc4D | Mpc4D, x1_ctrl: np.ndarray, x2_ctrl: np.ndarray, dt: float, mass: float):
```

to:

```python
def raw_velocity_command(ctrl: Hpc4D | Mpc4D | Lqr4D, x1_ctrl: np.ndarray, x2_ctrl: np.ndarray, dt: float, mass: float):
```

- [ ] **Step 4: Add `make_controller("lqr", ...)` branch**

In `make_controller()`, before the final `raise`, add:

```python
    if controller_kind == "lqr":
        return Lqr4D(
            mass=args.mass,
            dt=dt,
            radius=args.radius,
            m_p=args.m_p,
            tol=args.tol,
            q=(args.lqr_q_px, args.lqr_q_py, args.lqr_q_vx, args.lqr_q_vy),
            r=(args.lqr_r_ux, args.lqr_r_uy),
        )
```

- [ ] **Step 5: Update `raw_velocity_command()`**

Change:

```python
    if isinstance(ctrl, Mpc4D):
        return ctrl.command(x1_ctrl, x2_ctrl)
```

to:

```python
    if isinstance(ctrl, (Mpc4D, Lqr4D)):
        return ctrl.command(x1_ctrl, x2_ctrl)
```

- [ ] **Step 6: Update target index extraction in `simulate_circle_case()`**

Change:

```python
        target_idx = ctrl.target_idx if isinstance(ctrl, Mpc4D) else target_idx_from_hpc(ctrl)
```

to:

```python
        target_idx = ctrl.target_idx if isinstance(ctrl, (Mpc4D, Lqr4D)) else target_idx_from_hpc(ctrl)
```

- [ ] **Step 7: Run LQR tests**

Run:

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_sim_4d_artstein_mpc_compare.py -q
```

Expected: PASS.

- [ ] **Step 8: Commit implementation**

Run:

```bash
git add homo_multirobot_formation_control/scripts/sim_4d_artstein_mpc_compare.py
git commit -m "feat: 添加4D DARE-LQR数值控制器"
```

---

### Task 3: Add LQR Experiment Entrypoint

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/sim_4d_artstein_mpc_compare.py`
- Create: `homo_multirobot_formation_control/scripts/sim_4d_artstein_lqr_compare.py`
- Modify: `homo_multirobot_formation_control/CMakeLists.txt`

**Interfaces:**
- Consumes: `simulate_circle_case("lqr", compensation_kind, ...)`.
- Produces: executable script `sim_4d_artstein_lqr_compare.py`, `run_lqr_experiments(args) -> dict[str, list[SimRow]]`, and LQR output files in `analysis/results/4d_artstein_lqr/`.

- [ ] **Step 1: Add default LQR args to fallback namespace**

In the fallback `argparse.Namespace` inside `simulate_circle_case()`, add:

```python
            lqr_q_px=40.0,
            lqr_q_py=40.0,
            lqr_q_vx=1.0,
            lqr_q_vy=1.0,
            lqr_r_ux=0.02,
            lqr_r_uy=0.02,
```

- [ ] **Step 2: Create the LQR comparison script**

Create `homo_multirobot_formation_control/scripts/sim_4d_artstein_lqr_compare.py` with:

```python
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
            "hpc", "none", args.circle_tmax, args.dt, args.tau, args.Td, args.max_linear_vel, args.max_linear_accel, args=args
        ),
        "artstein_lqr_no_delay": simulate_circle_case(
            "lqr", "none", args.circle_tmax, args.dt, args.tau, args.Td, args.max_linear_vel, args.max_linear_accel, args=args
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
```

- [ ] **Step 3: Add the new script to CMake install list**

In `homo_multirobot_formation_control/CMakeLists.txt`, add this line next to the other simulation scripts:

```cmake
  scripts/sim_4d_artstein_lqr_compare.py
```

- [ ] **Step 4: Do not modify the MPC script `main()`**

Run:

```bash
rg -n "circle_no_delay_hpc_vs_mpc|circle_delay_hpc_vs_mpc|run_experiments\\(args\\)" homo_multirobot_formation_control/scripts/sim_4d_artstein_mpc_compare.py
```

Expected: existing MPC main still references the MPC comparison outputs.

- [ ] **Step 5: Run tests**

Run:

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_sim_4d_artstein_mpc_compare.py -q
```

Expected: PASS.

- [ ] **Step 6: Run LQR numerical simulation**

Run:

```bash
python3 homo_multirobot_formation_control/scripts/sim_4d_artstein_lqr_compare.py
```

Expected output paths include:

```text
homo_multirobot_formation_control/analysis/results/4d_artstein_lqr/circle_no_delay_hpc_vs_lqr.png
homo_multirobot_formation_control/analysis/results/4d_artstein_lqr/circle_delay_hpc_vs_lqr.png
homo_multirobot_formation_control/analysis/results/4d_artstein_lqr/circle_lqr_all_compare.png
homo_multirobot_formation_control/analysis/results/4d_artstein_lqr/summary_metrics.csv
homo_multirobot_formation_control/analysis/results/4d_artstein_lqr/timeseries_circle_lqr_compare.csv
```

- [ ] **Step 7: Inspect summary for non-finite values**

Run:

```bash
python3 - <<'PY'
import csv
from pathlib import Path

path = Path("homo_multirobot_formation_control/analysis/results/4d_artstein_lqr/summary_metrics.csv")
with path.open(newline="", encoding="utf-8") as f:
    rows = list(csv.DictReader(f))

assert {row["case"] for row in rows} == {
    "artstein_hpc_no_delay",
    "artstein_lqr_no_delay",
    "original_4d_delay",
    "artstein_hpc_delay",
    "artstein_lqr_delay",
}
for row in rows:
    for key, value in row.items():
        if key == "case":
            continue
        number = float(value)
        assert number == number and abs(number) < 1e9, (row["case"], key, value)
print("summary ok")
PY
```

Expected: prints `summary ok`.

- [ ] **Step 8: Commit experiment entrypoint**

Run:

```bash
git add homo_multirobot_formation_control/scripts/sim_4d_artstein_mpc_compare.py homo_multirobot_formation_control/scripts/sim_4d_artstein_lqr_compare.py homo_multirobot_formation_control/CMakeLists.txt homo_multirobot_formation_control/analysis/results/4d_artstein_lqr
git commit -m "feat: 添加4D LQR数值对照实验"
```

---

### Task 4: Document the LQR Simulation

**Files:**
- Create: `homo_multirobot_formation_control/doc/4d_artstein_lqr_simulation.md`
- Modify: `homo_multirobot_formation_control/README.md`

**Interfaces:**
- Consumes: final script behavior and output names from Task 3.
- Produces: user-facing instructions and theory notes.

- [ ] **Step 1: Create simulation doc**

Create `homo_multirobot_formation_control/doc/4d_artstein_lqr_simulation.md` with:

```markdown
# 4D Artstein-LQR 数值仿真说明

## 1. 目的

`scripts/sim_4d_artstein_mpc_compare.py` 现在默认输出 4D Artstein-LQR 数值对照组。该对照组用于回答：
在同一 Artstein 输入时延补偿、一阶电机前向预测、速度/加速度限幅和离散编队点切换条件下，标准离散 LQR
相对 4D HPC 的表现如何。

主对照组为：

```text
original_4d_delay
artstein_hpc_delay
artstein_lqr_delay
```

no-delay sanity check 为：

```text
artstein_hpc_no_delay
artstein_lqr_no_delay
```

## 2. 模型

LQR 作用在预测补偿后的 4D 双积分状态：

```math
x=[p_x,p_y,v_x,v_y]^T,\qquad \dot p=v,\qquad \dot v=u/m .
```

脚本使用 ZOH 离散化：

```math
A_d =
\begin{bmatrix}
1&0&h&0\\
0&1&0&h\\
0&0&1&0\\
0&0&0&1
\end{bmatrix},
\quad
B_d =
\begin{bmatrix}
h^2/(2m)&0\\
0&h^2/(2m)\\
h/m&0\\
0&h/m
\end{bmatrix}.
```

DARE-LQR 代价为：

```math
J=\sum_{k=0}^{\infty} e_k^TQe_k+u_k^TRu_k,
\qquad
e_k=x_{2,h,k}-x_{1,h,k}-d .
```

反馈律为：

```math
u_k=-K e_k .
```

脚本输出的 map 系原始速度命令为：

```math
v_{cmd,raw}=v_{2,h}+h u_k/m .
```

## 3. 延迟边界

`artstein_lqr_delay` 不是严格 delayed LQR。它复用当前项目的补偿层：

```text
测量状态 + 历史 cmd_vel
  -> Artstein 输入时延补偿 Td
  -> 一阶电机响应前向预测 tau
  -> 等效 4D 双积分状态
  -> DARE-LQR
```

因此它的论文定位应写成“共享同一预测补偿层的离散 LQR 基线”，不是“原始时滞执行器系统的最优控制器”。

## 4. 运行命令

从源码仓库根目录运行：

```bash
python3 homo_multirobot_formation_control/scripts/sim_4d_artstein_mpc_compare.py
```

默认输出目录：

```text
homo_multirobot_formation_control/analysis/results/4d_artstein_lqr/
```

主要输出：

```text
circle_no_delay_hpc_vs_lqr.png
circle_delay_hpc_vs_lqr.png
circle_lqr_all_compare.png
summary_metrics.csv
timeseries_circle_lqr_compare.csv
```

## 5. 默认参数

```text
dt=0.05
circle_tmax=45.0
tau=0.43
Td=0.22
mass=2.0
radius=2.0
m_p=4
max_linear_vel=0.5
max_linear_accel=0.4
Q_lqr=diag(40,40,1,1)
R_lqr=diag(0.02,0.02)
```

## 6. 验证命令

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_sim_4d_artstein_mpc_compare.py -q
python3 homo_multirobot_formation_control/scripts/sim_4d_artstein_mpc_compare.py
```
```

- [ ] **Step 2: Update README references**

In `homo_multirobot_formation_control/README.md`, add one short sentence near the 4D Artstein-MPC paragraph:

```markdown
**4D Artstein-LQR 数值对照** 复用同一预测补偿层，只把上层控制律替换为基于 DARE 的离散 LQR；第一版仅做离线数值仿真，详见 `doc/4d_artstein_lqr_simulation.md`。
```

- [ ] **Step 3: Run doc path check**

Run:

```bash
test -f homo_multirobot_formation_control/doc/4d_artstein_lqr_simulation.md && rg -n "4D Artstein-LQR|4d_artstein_lqr" homo_multirobot_formation_control/README.md homo_multirobot_formation_control/doc/4d_artstein_lqr_simulation.md
```

Expected: both files contain the new LQR references.

- [ ] **Step 4: Commit docs**

Run:

```bash
git add homo_multirobot_formation_control/doc/4d_artstein_lqr_simulation.md homo_multirobot_formation_control/README.md
git commit -m "docs: 说明4D LQR数值仿真"
```

---

### Task 5: Final Verification

**Files:**
- Verify only.

**Interfaces:**
- Consumes: all prior task outputs.
- Produces: final evidence that tests and simulation run successfully.

- [ ] **Step 1: Run targeted pytest**

Run:

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_sim_4d_artstein_mpc_compare.py -q
```

Expected: PASS.

- [ ] **Step 2: Run numerical simulation**

Run:

```bash
python3 homo_multirobot_formation_control/scripts/sim_4d_artstein_mpc_compare.py
```

Expected: script prints five output paths in `homo_multirobot_formation_control/analysis/results/4d_artstein_lqr/`.

- [ ] **Step 3: Confirm LQR cases in summary**

Run:

```bash
cut -d, -f1 homo_multirobot_formation_control/analysis/results/4d_artstein_lqr/summary_metrics.csv
```

Expected output includes:

```text
case
artstein_hpc_no_delay
artstein_lqr_no_delay
original_4d_delay
artstein_hpc_delay
artstein_lqr_delay
```

- [ ] **Step 4: Review git diff**

Run:

```bash
git status --short
git diff --stat HEAD
```

Expected: only planned files and generated `4d_artstein_lqr` results are changed after the last commit, unless the workspace already had unrelated changes.

- [ ] **Step 5: Final commit if needed**

If Task 5 produced new generated outputs not yet committed, run:

```bash
git add homo_multirobot_formation_control/analysis/results/4d_artstein_lqr
git commit -m "chore: 更新4D LQR数值仿真结果"
```

If no files remain from this work, do not create an empty commit.
