# 6D Map-Frame HPC Artstein Simulation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新增独立的 map-frame 6D 正则化 HPC 数值仿真，公平比较理想、延迟未补偿与 Artstein 预测补偿闭环。

**Architecture:** 新脚本不调用 ROS 2，也不修改旧 `sim_6d_disc_artstein_compare.py`。它将 map-frame 固定偏移误差、正则化 HPC、map-frame 延迟 plant 和预测器分开；unittest 直接验证核心代数和三组对照配置。

**Tech Stack:** Python 3、NumPy、SciPy、Matplotlib、标准库 unittest。

## Global Constraints

- 位置偏移固定在 map 系，默认 `d_p_map=[-1.0, 0.0]`、`d_theta=0`。
- 只使用现有仿真的正则化 `u_impl`，不实现理论 `K0` 控制律。
- 三个对照组共享初值、Leader 轨迹、HPC 参数、限幅和随机种子。
- 默认 `dt=0.05`、`plant_dt=0.01`、`Td=0.22`、`tau=0.43`。

---

### Task 1: Map-frame 误差与齐次代数核心

**Files:**

- Create: `homo_multirobot_formation_control/scripts/sim_6d_map_hpc_artstein_compare.py`
- Create: `homo_multirobot_formation_control/test/test_sim_6d_map_hpc_artstein.py`

**Interfaces:**

- `rot(theta: float) -> np.ndarray`
- `map_error(leader: np.ndarray, follower: np.ndarray, offset_map: np.ndarray, dtheta: float = 0.0) -> np.ndarray`
- `build_nominal_model(mass: float, inertia: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]`
- `verify_nominal_identities(mass: float, inertia: float, mu: float, kp: float, kv: float) -> dict[str, float]`

- [ ] **Step 1: Write the failing tests**

```python
def test_map_error_is_zero_at_fixed_map_offset():
    leader = np.array([1.0, -2.0, 0.4, 0.2, -0.1, 0.3])
    offset = np.array([-1.0, 0.5])
    follower = np.array([0.0, -1.5, -0.7, 0.0, 0.0, 0.3])
    follower[3:5] = rot(follower[2]).T @ (rot(leader[2]) @ leader[3:5])
    np.testing.assert_allclose(map_error(leader, follower, offset), np.zeros(6), atol=1e-12)

def test_nominal_homogeneous_identities_are_machine_precision():
    values = verify_nominal_identities(2.0, 1.0, -0.25, 1.2, 2.0)
    assert values['controllability_rank'] == 6
    assert max(v for k, v in values.items() if k != 'controllability_rank') < 1e-10
```

- [ ] **Step 2: Verify RED**

Run: `python3 -m unittest homo_multirobot_formation_control.test.test_sim_6d_map_hpc_artstein -v`

Expected: FAIL because the module does not exist.

- [ ] **Step 3: Implement the minimum core**

```python
def build_nominal_model(mass, inertia):
    A = np.zeros((6, 6)); A[0, 3] = A[1, 4] = A[2, 5] = 1.0
    B = np.vstack([np.zeros((3, 3)), np.diag([1/mass, 1/mass, 1/inertia])])
    G0 = np.diag([-1., -1., -1., 0., 0., 0.])
    return A, B, G0, np.diag([mass, mass, inertia])

def map_error(leader, follower, offset_map, dtheta=0.0):
    return np.r_[follower[:2] - leader[:2] - offset_map,
                 wrap_angle(follower[2] - leader[2] - dtheta),
                 rot(follower[2]) @ follower[3:5] - rot(leader[2]) @ leader[3:5],
                 follower[5] - leader[5]]
```

`verify_nominal_identities` must report rank and residual norms for `G0@B`, `A@G0-G0@A-A`, `A@Gd-Gd@A-mu*A`, and `Gd@B-B`. It must reject non-Hurwitz `A+B@K` and non-positive `P@Gd+Gd.T@P`.

- [ ] **Step 4: Verify GREEN**

Run: `python3 -m unittest homo_multirobot_formation_control.test.test_sim_6d_map_hpc_artstein -v`

Expected: PASS for both tests.

- [ ] **Step 5: Commit**

Run: `git add homo_multirobot_formation_control/scripts/sim_6d_map_hpc_artstein_compare.py homo_multirobot_formation_control/test/test_sim_6d_map_hpc_artstein.py && git commit -m '新增6D地图系仿真模型校验'`

### Task 2: 工程正则化 HPC 与 Artstein 预测

**Files:**

- Modify: `homo_multirobot_formation_control/scripts/sim_6d_map_hpc_artstein_compare.py`
- Modify: `homo_multirobot_formation_control/test/test_sim_6d_map_hpc_artstein.py`

**Interfaces:**

- `RegularizedMapHpc(...).command(error: np.ndarray) -> np.ndarray`
- `predict_map_state(state: np.ndarray, history: deque[np.ndarray], td: float, tau: float, control_dt: float) -> np.ndarray`
- `step_delayed_plant(state: np.ndarray, delayed_command_map: np.ndarray, dt: float, tau: float) -> np.ndarray`

- [ ] **Step 1: Write the failing tests**

```python
def test_regularized_hpc_returns_zero_for_zero_error():
    ctrl = RegularizedMapHpc(2.0, 1.0, -0.25, 1.2, 2.0, 0.5)
    np.testing.assert_allclose(ctrl.command(np.zeros(6)), np.zeros(3), atol=1e-12)

def test_predictor_matches_measurement_without_delay_or_lag():
    state = np.array([.2, -.1, .3, .4, -.2, .1])
    np.testing.assert_allclose(predict_map_state(state, deque(), 0.0, 0.0, .05), state, atol=1e-12)
```

- [ ] **Step 2: Verify RED**

Run: `python3 -m unittest homo_multirobot_formation_control.test.test_sim_6d_map_hpc_artstein -v`

Expected: FAIL because the controller and predictor do not exist.

- [ ] **Step 3: Implement the minimum controller and plant**

```python
def command(self, error):
    if np.linalg.norm(error) < 1e-14:
        return np.zeros(3)
    c = np.clip(hnorm(error, self.gd, self.p), self.c_min, 1.0)
    return c ** (1 + self.mu) * self.k @ expm(self.gd * (1 - np.log(c))) @ error
```

Use the existing script's binary-search `hnorm`. Model delay and first-order lag in map-frame translation plus scalar yaw-rate; after plant integration, convert map velocity back to follower body velocity. The predictor must use the complete command history and become the identity for zero `td` and `tau`.

- [ ] **Step 4: Verify GREEN**

Run: `python3 -m unittest homo_multirobot_formation_control.test.test_sim_6d_map_hpc_artstein -v`

Expected: PASS for all four tests.

- [ ] **Step 5: Commit**

Run: `git add homo_multirobot_formation_control/scripts/sim_6d_map_hpc_artstein_compare.py homo_multirobot_formation_control/test/test_sim_6d_map_hpc_artstein.py && git commit -m '新增6D地图系Artstein预测仿真核心'`

### Task 3: 三组对照、输出和文档

**Files:**

- Modify: `homo_multirobot_formation_control/scripts/sim_6d_map_hpc_artstein_compare.py`
- Modify: `homo_multirobot_formation_control/test/test_sim_6d_map_hpc_artstein.py`
- Create: `homo_multirobot_formation_control/doc/6d_map_hpc_artstein_simulation.md`

**Interfaces:**

- `SimulationConfig`
- `simulate_case(kind: str, config: SimulationConfig) -> SimulationResult`
- `run_experiment(config: SimulationConfig) -> list[Path]`

- [ ] **Step 1: Write the failing fairness and artifact tests**

```python
def test_cases_share_initial_state_and_delayed_plant():
    config = SimulationConfig(tmax=.10)
    delayed = simulate_case('delayed', config)
    artstein = simulate_case('artstein', config)
    np.testing.assert_allclose(delayed.initial_follower, artstein.initial_follower)
    assert delayed.td == artstein.td == config.td

def test_run_writes_the_four_required_artifacts(tmp_path):
    paths = run_experiment(SimulationConfig(tmax=.10, output_dir=tmp_path))
    assert {p.name for p in paths} == {'comparison.png', 'summary_metrics.csv', 'timeseries.csv', 'diagnostics.txt'}
    assert all(p.exists() for p in paths)
```

- [ ] **Step 2: Verify RED**

Run: `python3 -m unittest homo_multirobot_formation_control.test.test_sim_6d_map_hpc_artstein -v`

Expected: FAIL because simulation orchestration does not exist.

- [ ] **Step 3: Implement fair comparison and reports**

```python
@dataclass(frozen=True)
class SimulationConfig:
    tmax: float = 60.0
    control_dt: float = .05
    plant_dt: float = .01
    td: float = .22
    tau: float = .43
    output_dir: Path = Path('homo_multirobot_formation_control/analysis/results/6d_map_hpc_artstein')
```

Implement `ideal` (no delay/lag), `delayed` (measured-state feedback), and `artstein` (predicted-state feedback). Their leader circle, follower initial state, controller parameters, clipping, and seed must be identical. Write a four-panel PNG (XY, position error, yaw error, map command), summary CSV, timeseries CSV, and diagnostic text. The documentation must give the run command and state that conclusions concern the regularized engineering controller only.

- [ ] **Step 4: Verify GREEN with the default experiment**

Run:

```bash
python3 -m unittest homo_multirobot_formation_control.test.test_sim_6d_map_hpc_artstein -v
python3 homo_multirobot_formation_control/scripts/sim_6d_map_hpc_artstein_compare.py --out-dir /tmp/6d_map_hpc_artstein
```

Expected: tests PASS; CLI prints four paths; diagnostics report rank `6` and algebra residuals below `1e-10`.

- [ ] **Step 5: Commit**

Run: `git add homo_multirobot_formation_control/scripts/sim_6d_map_hpc_artstein_compare.py homo_multirobot_formation_control/test/test_sim_6d_map_hpc_artstein.py homo_multirobot_formation_control/doc/6d_map_hpc_artstein_simulation.md && git commit -m '新增6D地图系Artstein对照仿真'`

## Self-Review

- Spec coverage: Task 1 covers map-frame model and matrix conditions; Task 2 covers the required regularized controller, delayed plant, and predictor; Task 3 covers all fair comparison cases and every requested artifact.
- Placeholder scan: no deferred implementation or undefined interface remains.
- Type consistency: every state has the frozen `[px, py, theta, vx_body, vy_body, omega]` order and every virtual command has three map-force/moment entries.

## Execution Handoff

Plan complete. Choose either task-by-task subagent-driven execution or inline execution with checkpoints.
