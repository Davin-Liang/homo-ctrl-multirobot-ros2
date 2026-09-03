# 4D 仅一阶前向预测数值消融实验 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 4D HPC 延迟数值仿真增加“仅一阶前向预测、无 Artstein”对照组，并在图像、CSV 和说明中提供公平的三组对比。

**Architecture:** 保持已有的延迟 plant 和 `original`、`compensated` 两组。脚本增加仅跨越 `tau` 的闭式 Follower 状态预测；仿真按组选择反馈状态，绘图和指标接收三份延迟结果。

**Tech Stack:** Python 3、NumPy、SciPy、Matplotlib、pytest。

## Global Constraints

- 三个组都固定采用 `cmd_vel -> Td -> tau -> v_real` 的真实 plant。
- `forward_prediction_only` 只预测 `tau`，不得计算 Artstein 积分或预推进 Leader/Follower 的 `Td`。
- `original` 和 `compensated` 的数值流程、标签及既有 CSV 行名保持不变。
- 不修改 ROS/C++ 控制器、launch 文件或既有 `analysis/results/` 内容。

---

### Task 1: 为仅预测组建立失败测试

**Files:**

- Create: `homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py`
- Modify: `homo_multirobot_formation_control/CMakeLists.txt:297-307`

**Interfaces:**

- Consumes: `simulate_delay_case(kind, Tmax, h, tau, Td)`、`simulate_circle_case(kind, Tmax, h, tau, Td, pos_noise=0.0, vel_noise=0.0, seed=7)`、`write_summary(path, rows_by_name)`。
- Produces: 新函数 `predict_follower_state_first_order(state, vcmd, tau) -> np.ndarray` 和三组绘图接口的行为约束。

- [ ] **Step 1: 写入失败测试**

```python
import importlib.util
import sys
from pathlib import Path

import numpy as np


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "sim_4d_hpc_artstein_compare.py"


def load_module():
    spec = importlib.util.spec_from_file_location("sim_4d_hpc_artstein_compare", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_first_order_prediction_matches_closed_form():
    simulation = load_module()
    state = np.array([1.0, -2.0, 0.4, -0.6])
    command = np.array([1.2, 0.2])
    tau = 0.5
    predicted = simulation.predict_follower_state_first_order(state, command, tau)
    decay = np.exp(-1.0)
    expected_velocity = command + decay * (state[2:4] - command)
    expected_position = state[:2] + tau * command + tau * (1.0 - decay) * (state[2:4] - command)
    np.testing.assert_allclose(predicted, np.r_[expected_position, expected_velocity])


def test_prediction_only_cases_keep_delayed_plant_and_return_samples():
    simulation = load_module()
    delay_rows = simulation.simulate_delay_case("forward_prediction_only", 0.10, 0.01, 0.43, 0.22)
    circle_rows = simulation.simulate_circle_case("forward_prediction_only", 0.10, 0.01, 0.43, 0.22)
    assert len(delay_rows) == 10
    assert len(circle_rows) == 10
    np.testing.assert_allclose(delay_rows[0][3], delay_rows[0][1])
    np.testing.assert_allclose(circle_rows[0][3], circle_rows[0][1])


def test_three_group_summary_and_plot_include_prediction_only(tmp_path):
    simulation = load_module()
    original = simulation.simulate_circle_case("original", 0.10, 0.01, 0.43, 0.22)
    prediction_only = simulation.simulate_circle_case("forward_prediction_only", 0.10, 0.01, 0.43, 0.22)
    compensated = simulation.simulate_circle_case("compensated", 0.10, 0.01, 0.43, 0.22)
    plot = simulation.plot_circle_compare("no noise", original, prediction_only, compensated, tmp_path)
    summary = simulation.write_summary(tmp_path / "summary.csv", {
        "circle_original_delay_clean": original,
        "circle_forward_prediction_only_clean": prediction_only,
        "circle_artstein_prediction_clean": compensated,
    })
    assert plot.exists()
    assert "circle_forward_prediction_only_clean" in summary.read_text(encoding="utf-8")
```

- [ ] **Step 2: 运行测试，确认其因新接口尚不存在而失败**

Run: `cd /home/l1anggmgo/ros-projects/homo_multirobot_ws/src/homo-ctrl-multirobot-ros2 && pytest -q homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py`

Expected: FAIL，错误指出 `predict_follower_state_first_order` 不存在，或三组绘图函数参数数量不匹配。

- [ ] **Step 3: 将 Python 测试注册到 CTest**

```cmake
  add_test(NAME test_sim_4d_hpc_artstein_compare
    COMMAND ${Python3_EXECUTABLE}
    -m pytest -q
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_sim_4d_hpc_artstein_compare.py)
```

- [ ] **Step 4: 再次运行测试，确认失败原因仍是功能缺失而非测试装载错误**

Run: `pytest -q homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py`

Expected: FAIL，仅因 Task 2 尚未提供的预测函数或三组接口失败。

- [ ] **Step 5: 提交测试基线**

```bash
git add homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py homo_multirobot_formation_control/CMakeLists.txt
git commit -m "新增4D前向预测仿真测试"
```

### Task 2: 实现第三个数值实验组并扩展输出

**Files:**

- Modify: `homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py:282-607`
- Modify: `homo_multirobot_formation_control/doc/4d_artstein_prediction_simulation.md:33-84,151-165`
- Test: `homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py`

**Interfaces:**

- Consumes: `predict_follower_state_first_order(state, vcmd, tau) -> np.ndarray`。
- Produces: `simulate_delay_case("forward_prediction_only", ...)`、`simulate_circle_case("forward_prediction_only", ...)` 与含第三组参数的 `plot_delay_compare`、`plot_circle_compare`。

- [ ] **Step 1: 增加仅一阶预测辅助函数**

在 `predict_follower_state_from_artstein` 之前插入：

```python
def predict_follower_state_first_order(state: np.ndarray, vcmd: np.ndarray, tau: float) -> np.ndarray:
    decay = np.exp(-1.0)
    velocity = vcmd + decay * (state[2:4] - vcmd)
    position = state[0:2] + vcmd * tau + tau * (1.0 - decay) * (state[2:4] - vcmd)
    return np.r_[position, velocity]
```

- [ ] **Step 2: 让两类仿真选择三组反馈状态**

将 `simulate_delay_case` 与 `simulate_circle_case` 的初始化和循环内选择逻辑统一为：

```python
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
```

在 delay 场景中令 `x1_meas = x1`、`x2_meas = x2` 后复用该逻辑；圆轨迹场景继续使用已有噪声测量值。延迟队列和一阶 plant 更新保持原样。

- [ ] **Step 3: 扩展绘图和 CSV 为三组结果**

将函数签名改为：

```python
def plot_delay_compare(ideal_rows, original_rows, prediction_only_rows, compensated_rows, out_dir: Path):
def plot_circle_compare(noise_label: str, original_rows, prediction_only_rows, compensated_rows, out_dir: Path):
```

每张图加入橙色 `prediction-only + delay` 曲线，保留红色 `original + delay` 和蓝色 `Artstein + prediction` 曲线。`main()` 运行新组并传给绘图函数，且在 CSV 增加：

```python
"matlab_leader_forward_prediction_only": delay_prediction_only,
"circle_forward_prediction_only_clean": circle_prediction_only,
"circle_forward_prediction_only_noise": circle_noise_prediction_only,
```

- [ ] **Step 4: 将第三组写入中文说明**

在 `doc/4d_artstein_prediction_simulation.md` 的模型章节新增“原始 4D HPC + 仅一阶前向预测”小节，明确它从测得的 Follower 状态预测 `tau` 后状态、不调用 Artstein 且不预测 `Td`；将现有 Artstein 小节顺延。同步将输出图说明改为三组颜色和标签。

- [ ] **Step 5: 运行测试，确认通过**

Run: `cd /home/l1anggmgo/ros-projects/homo_multirobot_ws/src/homo-ctrl-multirobot-ros2 && pytest -q homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py`

Expected: PASS，3 passed。

- [ ] **Step 6: 用短时参数运行端到端仿真**

Run: `tmp_dir=$(mktemp -d) && python3 homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py --out-dir "$tmp_dir" --tmax 0.1 --circle-tmax 0.1 && test -f "$tmp_dir/summary_metrics.csv" && rg -n "forward_prediction_only" "$tmp_dir/summary_metrics.csv" && find "$tmp_dir" -maxdepth 1 -name '*.png' -type f | wc -l`

Expected: 输出三条包含 `forward_prediction_only` 的 CSV 行，且生成 4 张 PNG 图。

- [ ] **Step 7: 检查改动范围和提交实现**

```bash
git diff --check
git diff -- homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py homo_multirobot_formation_control/doc/4d_artstein_prediction_simulation.md homo_multirobot_formation_control/CMakeLists.txt
git add homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py homo_multirobot_formation_control/doc/4d_artstein_prediction_simulation.md homo_multirobot_formation_control/CMakeLists.txt
git commit -m "新增4D仅一阶前向预测数值对照"
```
