# 6D Map-Frame Artstein Linear Baseline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 map-frame 三组仿真添加保持 Artstein 预测、仅关闭齐次升级的 `artstein_linear` 对照。

**Architecture:** `RegularizedMapHpc` 增加 `use_hpc` 布尔值；关闭时直接返回同一基础矩阵 `K @ e`。`simulate_case` 为 `artstein_linear` 复用 `artstein` 的预测和 plant 路径，图与 CSV 自动多出该组。

**Tech Stack:** Python 3、NumPy、Matplotlib、unittest。

## Global Constraints

- `artstein_linear` 与 `artstein` 必须使用相同预测状态、延迟、一阶 plant、初值和限幅。
- 唯一区别是 `u_linear=K e` 替代正则化齐次反馈。

---

### Task 1: 线性反馈开关与第四组对照

**Files:**

- Modify: `homo_multirobot_formation_control/scripts/sim_6d_map_hpc_artstein_compare.py`
- Modify: `homo_multirobot_formation_control/test/test_sim_6d_map_hpc_artstein.py`
- Modify: `homo_multirobot_formation_control/doc/6d_map_hpc_artstein_simulation.md`

**Interfaces:**

- `RegularizedMapHpc(..., use_hpc: bool = True)`
- `simulate_case(kind: Literal['ideal', 'delayed', 'artstein', 'artstein_linear'], config)`

- [ ] **Step 1: Write failing tests**

```python
def test_linear_controller_returns_base_linear_feedback():
    ctrl = RegularizedMapHpc(2.0, 1.0, -.25, 1.2, 2.0, .5, use_hpc=False)
    error = np.array([.3, -.2, .1, .4, -.5, .2])
    np.testing.assert_allclose(ctrl.command(error), ctrl.k @ error)

def test_artstein_linear_shares_delayed_plant_and_initial_state():
    config = SimulationConfig(tmax=.1)
    hpc = simulate_case('artstein', config)
    linear = simulate_case('artstein_linear', config)
    assert linear.td == hpc.td == config.td
    np.testing.assert_allclose(linear.initial_follower, hpc.initial_follower)
```

- [ ] **Step 2: Verify RED**

Run: `python3 -m unittest homo_multirobot_formation_control.test.test_sim_6d_map_hpc_artstein -v`

Expected: FAIL because `use_hpc` and `artstein_linear` are unsupported.

- [ ] **Step 3: Implement minimal switch and group**

```python
def command(self, error):
    if not self.use_hpc:
        return self.k @ error
    # retain existing regularized-HPC branch
```

Instantiate `RegularizedMapHpc(..., use_hpc=(kind != 'artstein_linear'))`. Treat `artstein_linear` identically to `artstein` when selecting predicted Leader/Follower states and delayed plant. Add it to `run_experiment`'s case list and legend/CSV output. Document the group definition.

- [ ] **Step 4: Verify GREEN and run default comparison**

Run:

```bash
python3 -m unittest homo_multirobot_formation_control.test.test_sim_6d_map_hpc_artstein -v
python3 homo_multirobot_formation_control/scripts/sim_6d_map_hpc_artstein_compare.py --out-dir /tmp/6d_map_hpc_linear
```

Expected: tests PASS; summary has `artstein_linear`; PNG legend has four groups.

- [ ] **Step 5: Commit**

Run: `git add homo_multirobot_formation_control/scripts/sim_6d_map_hpc_artstein_compare.py homo_multirobot_formation_control/test/test_sim_6d_map_hpc_artstein.py homo_multirobot_formation_control/doc/6d_map_hpc_artstein_simulation.md && git commit -m '新增6D Artstein线性控制对照'`

## Self-Review

- Spec coverage: the sole task holds prediction and plant fixed while varying only homogeneous upgrade.
- Placeholder scan: interfaces, tests, commands, and output expectations are explicit.
- Type consistency: all four groups use the existing six-state vector and three-command interface.

## Execution Handoff

Plan complete. Choose either task-by-task subagent-driven execution or inline execution with checkpoints.
