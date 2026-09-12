# 4D Artstein 理想 Leader 前馈数值仿真实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在纯 Python 4D Artstein-HPC 数值模型中加入理想 Leader 加速度前馈，并输出基线对照结果。

**Architecture:** 仅扩展 `sim_4d_hpc_artstein_compare.py` 的 MATLAB 激励和圆轨迹场景。前馈使用物理 Leader 加速度 `a_leader` 的 `h * a_leader` 转成一个周期的 map 系速度增量，在既有速度限幅之前叠加；MATLAB 力输入先由 `u1 / mass` 转为物理加速度。Artstein 状态预测、死区和一阶电机模型不变。

**Tech Stack:** Python 3、NumPy、SciPy、Matplotlib、pytest。

## Global Constraints

- 不修改 ROS 2 节点、launch 文件或话题接口。
- 理想输入仅来自数值模型中已知的 Leader 加速度。
- 前馈案例必须使用与 `compensated` 相同的 Artstein + prediction 和执行器模型。
- 结果通过显式 `--out-dir` 写入新目录，不覆盖已有结果。

---

### Task 1: 定义并验证理想前馈速度增量

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py`
- Modify: `homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py`

**Interfaces:**
- Produces: `ideal_leader_feedforward_velocity(leader_accel: np.ndarray, h: float) -> np.ndarray`。

- [ ] **Step 1: 写入失败测试**

```python
def test_ideal_leader_feedforward_is_one_step_velocity_increment():
    simulation = load_module()
    np.testing.assert_allclose(
        simulation.ideal_leader_feedforward_velocity(np.array([2.0, -4.0]), h=0.05),
        np.array([0.10, -0.20]),
    )
```

- [ ] **Step 2: 确认测试失败**

Run: `pytest -q homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py -k ideal_leader_feedforward`

Expected: FAIL，提示函数不存在。

- [ ] **Step 3: 最小实现**

```python
def ideal_leader_feedforward_velocity(leader_accel: np.ndarray, h: float) -> np.ndarray:
    return h * leader_accel
```

- [ ] **Step 4: 确认测试通过**

Run: `pytest -q homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py -k ideal_leader_feedforward`

Expected: PASS。

### Task 2: 接入 Artstein 前馈数值案例

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py`
- Modify: `homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py`

**Interfaces:**
- Consumes: `ideal_leader_feedforward_velocity(...)`。
- Produces: `simulate_delay_case("compensated_ideal_leader_feedforward", ...)` 和 `simulate_circle_case("compensated_ideal_leader_feedforward", ...)`，返回既有 delayed-row 格式。

- [ ] **Step 1: 写入失败测试**

```python
def test_ideal_feedforward_changes_command_before_delay():
    simulation = load_module()
    base = simulation.simulate_circle_case("compensated", 0.01, 0.01, 0.43, 0.22)
    ff = simulation.simulate_circle_case(
        "compensated_ideal_leader_feedforward", 0.01, 0.01, 0.43, 0.22
    )
    np.testing.assert_allclose(ff[0][2], base[0][2])
    assert not np.allclose(ff[0][5], base[0][5])
```

- [ ] **Step 2: 确认测试失败**

Run: `pytest -q homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py -k changes_command`

Expected: FAIL，因为新 case 尚未支持。

- [ ] **Step 3: 最小实现**

把新 case 复用 `compensated` 的预测分支，在命令限幅前执行：

```python
if kind == "compensated_ideal_leader_feedforward":
    vcmd += ideal_leader_feedforward_velocity(leader_accel, h)
```

MATLAB 场景的 `leader_accel` 为当前 `u1 / mass`。圆轨迹新增：

```python
def circle_leader_accel(t: float, radius: float = 2.0, omega: float = 0.25) -> np.ndarray:
    return np.array([-radius * omega**2 * np.cos(omega * t),
                     -radius * omega**2 * np.sin(omega * t)])
```

- [ ] **Step 4: 确认测试通过**

Run: `pytest -q homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py -k 'ideal_leader_feedforward or changes_command'`

Expected: PASS。

### Task 3: 输出对照图、指标并执行实验

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py`
- Modify: `homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py`
- Create: `homo_multirobot_formation_control/analysis/results/4d_artstein_ideal_leader_feedforward_20260912/`（运行产物，不提交）

**Interfaces:**
- Produces: CSV 中 `matlab_leader_artstein_prediction_ideal_leader_feedforward`、`circle_artstein_prediction_ideal_leader_feedforward_clean` 和 `_noise` 行；两张比较图的前馈曲线。

- [ ] **Step 1: 写入失败测试**

```python
def test_main_writes_ideal_leader_feedforward_summary_rows(tmp_path, monkeypatch):
    simulation = load_module()
    monkeypatch.setattr(sys, "argv", [str(SCRIPT_PATH), "--out-dir", str(tmp_path),
                                       "--tmax", "0.01", "--circle-tmax", "0.01", "--dt", "0.01"])
    simulation.main()
    assert "circle_artstein_prediction_ideal_leader_feedforward_clean" in (
        tmp_path / "summary_metrics.csv"
    ).read_text(encoding="utf-8")
```

- [ ] **Step 2: 确认测试失败**

Run: `pytest -q homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py -k summary_rows`

Expected: FAIL，因为 `main()` 未汇总新案例。

- [ ] **Step 3: 最小实现和验证**

让 `main()` 生成三个前馈 rows，两个比较图增加标签为 `Artstein + prediction + ideal leader FF` 的曲线，并传给 `write_summary`。随后运行：

```bash
pytest -q homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py
python3 homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py \
  --out-dir homo_multirobot_formation_control/analysis/results/4d_artstein_ideal_leader_feedforward_20260912 \
  --tmax 30 --circle-tmax 60 --dt 0.01 --tau 0.43 --Td 0.22 --leader-speed 0.5
```

Expected: 全部测试 PASS；生成 PNG 和 `summary_metrics.csv`；CSV 的指标全为有限数。

- [ ] **Step 4: 提交源代码和测试**

```bash
git add homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py
git commit -m "增加4D Artstein理想领航前馈仿真"
```
