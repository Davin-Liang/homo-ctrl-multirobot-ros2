# 6D Map-Frame Continuous Yaw Dynamics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 运行恒定角加速度和周期时变角加速度下的 6D map-frame 三组对照仿真。

**Architecture:** 新增连续 Leader yaw 轨迹函数，始终保持 map 圆轨迹的位置和速度；以轨迹函数的当前状态为预测初值。每个场景复用现有 `simulate_case`，但使用独立结果目录和 CSV/图文件。

**Tech Stack:** Python 3、NumPy、Matplotlib、标准库 unittest。

## Global Constraints

- 恒定角加速度 `0.05 rad/s^2`，总 yaw-rate 上限 `0.8 rad/s`。
- 周期场景 `alpha=0.08*cos(0.4*t) rad/s^2`。
- Leader map 位置和 map 平移速度必须不因 yaw 动态而改变。
- 三组共享连续 yaw 场景、plant、初值、HPC 参数和限幅。

---

### Task 1: 连续 Leader yaw 动态函数

**Files:**

- Modify: `homo_multirobot_formation_control/scripts/sim_6d_map_hpc_artstein_compare.py`
- Modify: `homo_multirobot_formation_control/test/test_sim_6d_map_hpc_artstein.py`

**Interfaces:**

- `constant_accel_yaw_leader_state(t, radius, speed, accel=0.05, max_yaw_rate=0.8) -> np.ndarray`
- `periodic_accel_yaw_leader_state(t, radius, speed, accel_amplitude=0.08, frequency=0.4) -> np.ndarray`

- [ ] **Step 1: Write failing continuous-dynamics tests**

```python
def test_constant_accel_yaw_keeps_map_translation_and_caps_rate():
    state = constant_accel_yaw_leader_state(60.0, 2.0, .45)
    nominal = circle_leader_state(60.0, 2.0, .45)
    np.testing.assert_allclose(state[:2], nominal[:2], atol=1e-12)
    np.testing.assert_allclose(rot(state[2]) @ state[3:5], rot(nominal[2]) @ nominal[3:5], atol=1e-12)
    assert state[5] == .8

def test_periodic_accel_yaw_has_expected_rate_offset():
    state = periodic_accel_yaw_leader_state(np.pi / .8, 2.0, .45)
    nominal = circle_leader_state(np.pi / .8, 2.0, .45)
    assert abs((state[5] - nominal[5]) - .2) < 1e-12
```

- [ ] **Step 2: Verify RED**

Run: `python3 -m unittest homo_multirobot_formation_control.test.test_sim_6d_map_hpc_artstein -v`

Expected: FAIL because continuous yaw Leader functions do not exist.

- [ ] **Step 3: Implement the minimum state constructors**

```python
def _leader_with_yaw_offset(t, radius, speed, yaw_offset, yaw_rate_offset):
    state = circle_leader_state(t, radius, speed)
    velocity_map = rot(state[2]) @ state[3:5]
    state[2] = wrap_angle(state[2] + yaw_offset)
    state[3:5] = map_to_body(state[2], velocity_map)
    state[5] += yaw_rate_offset
    return state
```

Use the clipped integral of constant acceleration for scenario A. For scenario B use `yaw_rate_offset=(0.08/0.4)*sin(0.4*t)` and `yaw_offset=(0.08/0.4**2)*(1-cos(0.4*t))`.

- [ ] **Step 4: Verify GREEN**

Run: `python3 -m unittest homo_multirobot_formation_control.test.test_sim_6d_map_hpc_artstein -v`

Expected: all tests PASS.

- [ ] **Step 5: Commit**

Run: `git add homo_multirobot_formation_control/scripts/sim_6d_map_hpc_artstein_compare.py homo_multirobot_formation_control/test/test_sim_6d_map_hpc_artstein.py && git commit -m '新增6D连续航向动态场景'`

### Task 2: 双场景运行与报告

**Files:**

- Modify: `homo_multirobot_formation_control/scripts/sim_6d_map_hpc_artstein_compare.py`
- Modify: `homo_multirobot_formation_control/doc/6d_map_hpc_artstein_simulation.md`

**Interfaces:**

- `run_continuous_yaw_experiments(base_output_dir: Path) -> dict[str, list[Path]]`
- Results: `constant_yaw_accel/` and `periodic_yaw_accel/`.

- [ ] **Step 1: Write failing output test**

```python
def test_continuous_yaw_runner_writes_two_scenario_summaries(tmp_path):
    outputs = run_continuous_yaw_experiments(tmp_path)
    assert set(outputs) == {'constant_yaw_accel', 'periodic_yaw_accel'}
    assert all((tmp_path / name / 'summary_metrics.csv').exists() for name in outputs)
```

- [ ] **Step 2: Verify RED**

Run: `python3 -m unittest homo_multirobot_formation_control.test.test_sim_6d_map_hpc_artstein -v`

Expected: FAIL because `run_continuous_yaw_experiments` does not exist.

- [ ] **Step 3: Implement scenario execution and documentation**

Create a `leader_mode` field in `SimulationConfig` with `constant_accel` and `periodic_accel`, route Leader state generation through one helper, and use the observed continuous Leader state in Artstein prediction. Run each three-group experiment in its own output directory. Document the two analytical yaw functions and their physical-model boundary.

- [ ] **Step 4: Verify GREEN and run both defaults**

Run:

```bash
python3 -m unittest homo_multirobot_formation_control.test.test_sim_6d_map_hpc_artstein -v
python3 homo_multirobot_formation_control/scripts/sim_6d_map_hpc_artstein_compare.py --continuous-yaw --out-dir /tmp/6d_map_continuous_yaw
```

Expected: tests PASS; each scenario has PNG, summary CSV, timeseries CSV and diagnostics text.

- [ ] **Step 5: Commit**

Run: `git add homo_multirobot_formation_control/scripts/sim_6d_map_hpc_artstein_compare.py homo_multirobot_formation_control/test/test_sim_6d_map_hpc_artstein.py homo_multirobot_formation_control/doc/6d_map_hpc_artstein_simulation.md && git commit -m '新增6D连续航向动态对照实验'`

## Self-Review

- Spec coverage: Task 1 preserves map translation while defining both requested yaw dynamics; Task 2 runs and reports each scenario for all three groups.
- Placeholder scan: all interfaces, equations, tests and commands are explicit.
- Type consistency: every Leader state remains `[px, py, theta, vx_body, vy_body, omega]`.

## Execution Handoff

Plan complete. Choose either task-by-task subagent-driven execution or inline execution with checkpoints.
