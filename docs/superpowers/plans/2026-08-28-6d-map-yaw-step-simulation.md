# 6D Map-Frame Leader Yaw Step Simulation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 map-frame 6D HPC 对照仿真加入 `t=30 s`、`+pi/2` 的 Leader 航向阶跃场景并输出瞬态指标。

**Architecture:** 在现有 `circle_leader_state` 外新增确定性的 yaw-step Leader 状态函数；位置和 map-frame 速度继承圆轨迹，只有 yaw 参考在阶跃时改变。现有三组仿真共用该 Leader 函数，报告额外记录阶跃后的峰值与最终误差。

**Tech Stack:** Python 3、NumPy、Matplotlib、标准库 unittest。

## Global Constraints

- 阶跃时刻固定为 `30.0 s`，幅值固定为 `+pi/2`。
- Leader 的位置和 map-frame 平移速度在阶跃前后连续。
- 三组共享同一阶跃场景、plant、初值、HPC 参数和限幅。
- 该阶跃是参考/测量扰动，不表述为实车可实现的无限角加速度运动。

---

### Task 1: Leader yaw 阶跃状态与回归测试

**Files:**

- Modify: `homo_multirobot_formation_control/scripts/sim_6d_map_hpc_artstein_compare.py`
- Modify: `homo_multirobot_formation_control/test/test_sim_6d_map_hpc_artstein.py`

**Interfaces:**

- `yaw_step_leader_state(t: float, radius: float, speed: float, step_time: float = 30.0, step_angle: float = np.pi / 2.0) -> np.ndarray`
- `SimulationConfig.yaw_step_time: float = 30.0`
- `SimulationConfig.yaw_step_angle: float = np.pi / 2.0`

- [ ] **Step 1: Write failing continuity and yaw-step tests**

```python
def test_yaw_step_changes_only_leader_yaw_reference():
    before = yaw_step_leader_state(29.999, 2.0, .45)
    after = yaw_step_leader_state(30.001, 2.0, .45)
    nominal_after = circle_leader_state(30.001, 2.0, .45)
    np.testing.assert_allclose(after[:2], nominal_after[:2], atol=1e-12)
    np.testing.assert_allclose(rot(after[2]) @ after[3:5], rot(nominal_after[2]) @ nominal_after[3:5], atol=1e-12)
    assert abs(wrap_angle(after[2] - nominal_after[2]) - np.pi / 2) < 1e-12
```

- [ ] **Step 2: Verify RED**

Run: `python3 -m unittest homo_multirobot_formation_control.test.test_sim_6d_map_hpc_artstein -v`

Expected: FAIL because `yaw_step_leader_state` is undefined.

- [ ] **Step 3: Implement the minimal Leader state function**

```python
def yaw_step_leader_state(t, radius, speed, step_time=30.0, step_angle=np.pi / 2.0):
    state = circle_leader_state(t, radius, speed)
    if t >= step_time:
        velocity_map = rot(state[2]) @ state[3:5]
        state[2] = wrap_angle(state[2] + step_angle)
        state[3:5] = map_to_body(state[2], velocity_map)
    return state
```

Use this function for the control state and each recorded sample in `simulate_case`.

- [ ] **Step 4: Verify GREEN**

Run: `python3 -m unittest homo_multirobot_formation_control.test.test_sim_6d_map_hpc_artstein -v`

Expected: all tests PASS.

- [ ] **Step 5: Commit**

Run: `git add homo_multirobot_formation_control/scripts/sim_6d_map_hpc_artstein_compare.py homo_multirobot_formation_control/test/test_sim_6d_map_hpc_artstein.py && git commit -m '新增6D领航航向阶跃仿真场景'`

### Task 2: 阶跃指标、图和说明

**Files:**

- Modify: `homo_multirobot_formation_control/scripts/sim_6d_map_hpc_artstein_compare.py`
- Modify: `homo_multirobot_formation_control/doc/6d_map_hpc_artstein_simulation.md`

**Interfaces:**

- `summary_metrics.csv` adds `post_step_peak_position_error`, `post_step_peak_yaw_error`, and `final_yaw_error`.
- `comparison.png` draws a vertical dashed line at `config.yaw_step_time` on error and command axes.

- [ ] **Step 1: Write failing artifact assertions**

```python
def test_yaw_step_run_reports_post_step_metrics(tmp_path):
    paths = run_experiment(SimulationConfig(tmax=31.0, output_dir=tmp_path))
    header = (tmp_path / 'summary_metrics.csv').read_text().splitlines()[0]
    assert 'post_step_peak_yaw_error' in header
```

- [ ] **Step 2: Verify RED**

Run: `python3 -m unittest homo_multirobot_formation_control.test.test_sim_6d_map_hpc_artstein -v`

Expected: FAIL because the summary header lacks the new metric.

- [ ] **Step 3: Implement reports and documentation**

Compute post-step slices from `result.time >= config.yaw_step_time`; add peak position/yaw values to each CSV row. Draw `axvline(config.yaw_step_time, linestyle='--', color='0.35')` in the error and command subplots. Document the exact yaw-step scenario, run command, and reference-disturbance limitation.

- [ ] **Step 4: Verify GREEN and run the full experiment**

Run:

```bash
python3 -m unittest homo_multirobot_formation_control.test.test_sim_6d_map_hpc_artstein -v
python3 homo_multirobot_formation_control/scripts/sim_6d_map_hpc_artstein_compare.py --out-dir /tmp/6d_map_hpc_yaw_step
```

Expected: tests PASS; CSV contains three new columns; all three groups have finite post-step metrics and the plot marks `30 s`.

- [ ] **Step 5: Commit**

Run: `git add homo_multirobot_formation_control/scripts/sim_6d_map_hpc_artstein_compare.py homo_multirobot_formation_control/test/test_sim_6d_map_hpc_artstein.py homo_multirobot_formation_control/doc/6d_map_hpc_artstein_simulation.md && git commit -m '输出6D航向阶跃瞬态指标'`

## Self-Review

- Spec coverage: Task 1 implements the requested `30 s`/`+90 deg` scenario without disturbing translation; Task 2 records and visualizes its three-group transient response.
- Placeholder scan: no undefined function or deferred step remains.
- Type consistency: all Leader functions return the established six-element state order.

## Execution Handoff

Plan complete. Choose either task-by-task subagent-driven execution or inline execution with checkpoints.
