# 动捕速度响应模型测试 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新增可由命令行选择平移或偏航阶跃的实车动捕采集工具，并以一阶和固定加速度模型对每段响应给出可复核的拟合比较。

**Architecture:** 脚本将纯数值处理函数和 ROS 节点分开：纯函数负责标量响应提取、两种模型拟合、指标和文件输出；节点只执行安全的阶跃状态机、订阅 `TwistStamped` 与发布 `Twist`。`move` 以全局动捕线速度的模为观测，`turn` 以 `wz` 为观测；原始样本始终先落盘，再生成拟合汇总与叠加图。

**Tech Stack:** Python 3、ROS 2 Humble `rclpy`、NumPy、Matplotlib、CSV、JSON、pytest。

## Global Constraints

- 默认测试 `/robot2`：读取 `/robot2/mocap/twist`，发布 `/robot2/cmd_vel`；`--namespace` 可覆盖。
- `--mode move|turn` 与 `--target` 是必填 CLI 参数；`move` 单位 m/s，`turn` 单位 rad/s。
- `move` 的观测速度必须为 `hypot(vx_map, vy_map)`，不转换为或解释为车体系分量。
- 每次仅激励一个自由度；所有正常、超时、异常和 Ctrl-C 退出路径必须发布零 `Twist`。
- 默认输出目录为 `robot_traj/real/model_identification/<timestamp>/`；不得自动修改控制器参数。
- 所有 colcon 命令必须在 `/home/l1anggmgo/ros-projects/homo_multirobot_ws` 执行。

---

### Task 1: 建立与 ROS 无关的标量响应和模型比较测试

**Files:**
- Create: `homo_multirobot_formation_control/test/test_measure_mocap_velocity_model.py`
- Create: `homo_multirobot_formation_control/scripts/measure_mocap_velocity_model.py`

**Interfaces:**
- Produces: `scalar_response(mode, vx_map, vy_map, wz) -> float`。
- Produces: `fit_first_order(time_s, speed, target) -> FitResult`、`fit_constant_accel(time_s, speed, target) -> FitResult` 与 `choose_model(first, accel) -> str`。
- `FitResult` 包含 `prediction`、`parameters`、`sse`、`rmse`、`mae`、`aic`。

- [ ] **Step 1: 写入不依赖 ROS 的失败测试**

创建测试文件，通过 `importlib.util.spec_from_file_location` 从 `scripts/measure_mocap_velocity_model.py` 加载模块。写入：

```python
def test_scalar_response_uses_planar_magnitude():
    assert module.scalar_response("move", 0.3, 0.4, 9.0) == pytest.approx(0.5)
    assert module.scalar_response("turn", 0.3, 0.4, -0.7) == pytest.approx(-0.7)

def test_first_order_wins_for_first_order_samples():
    t = np.linspace(0.0, 1.0, 51)
    y = 0.2 * (1.0 - np.exp(-t / 0.25))
    first = module.fit_first_order(t, y, 0.2)
    accel = module.fit_constant_accel(t, y, 0.2)
    assert module.choose_model(first, accel) == "first_order"
```

- [ ] **Step 2: 运行测试确认失败**

```bash
python3 -m pytest -q homo_multirobot_formation_control/test/test_measure_mocap_velocity_model.py
```

预期：失败，指出目标脚本不存在。

- [ ] **Step 3: 实现最小纯函数与 CLI 骨架**

创建带 shebang 的脚本。将 `rclpy`、`Twist` 与 `TwistStamped` 导入放入 ROS 节点创建路径，使纯拟合函数可被未安装 ROS 的 Python 测试导入。实现 `FitResult` dataclass；`scalar_response` 对 `move` 返回 `math.hypot(vx_map, vy_map)`，对 `turn` 返回 `wz`，其他 mode 抛 `ValueError`。

一阶上升/下降模型在候选 `tau` 网格 `np.linspace(0.02, 3.0, 300)` 中选择最小 SSE：

```python
prediction = target + (initial_speed - target) * np.exp(-time_s / tau)
```

固定加速度模型在候选斜率网格中最小化 SSE，并将预测裁剪在初值和目标之间：

```python
prediction = np.clip(initial_speed + accel * time_s,
                     min(initial_speed, target), max(initial_speed, target))
```

计算 `rmse=sqrt(sse/n)`、`mae=mean(abs(error))`、`aic=n*log(max(sse/n, 1e-12))+2`；`choose_model` 返回 AIC 更小的名称。先定义 argparse 的必填 `--mode`、`--target`，但不在此任务实现 ROS 节点。

- [ ] **Step 4: 运行测试确认通过**

```bash
python3 -m pytest -q homo_multirobot_formation_control/test/test_measure_mocap_velocity_model.py
python3 homo_multirobot_formation_control/scripts/measure_mocap_velocity_model.py --help
```

预期：pytest 通过，帮助中显示 `--mode {move,turn}` 与 `--target`。

- [ ] **Step 5: 提交纯函数与测试**

```bash
git add homo_multirobot_formation_control/scripts/measure_mocap_velocity_model.py \
  homo_multirobot_formation_control/test/test_measure_mocap_velocity_model.py
git commit -m "新增动捕速度模型拟合工具"
```

### Task 2: 实现安全的 ROS 阶跃采集和文件输出

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/measure_mocap_velocity_model.py`
- Modify: `homo_multirobot_formation_control/test/test_measure_mocap_velocity_model.py`

**Interfaces:**
- Consumes: Task 1 的 `scalar_response` 与两个拟合函数。
- Produces: `MocapVelocityModelMeter`，参数包括 `namespace`、`mode`、`target`、`trials`、`hold_seconds`、`settle_seconds`、`mocap_timeout`、`out_dir`。
- Produces: `samples.csv`、`fit_summary.csv`、`metadata.json`、`comparison.png`。

- [ ] **Step 1: 写入失败的采样与输出测试**

新增测试，构造两个纯字典样本并调用 `write_outputs`：

```python
def test_write_outputs_creates_replayable_files(tmp_path):
    samples = [{"trial": 1, "phase": "hold", "elapsed_s": 0.0,
                "cmd_vx": 0.15, "cmd_vy": 0.0, "cmd_wz": 0.0,
                "vx_map": 0.0, "vy_map": 0.0, "wz": 0.0,
                "response": 0.0}]
    module.write_outputs(tmp_path, samples, [], {"mode": "move"})
    assert (tmp_path / "samples.csv").is_file()
    assert (tmp_path / "fit_summary.csv").is_file()
    assert json.loads((tmp_path / "metadata.json").read_text())["mode"] == "move"
```

并测试命令生成：`move` 返回仅 `linear.x=target`，`turn` 返回仅 `angular.z=target`。

- [ ] **Step 2: 运行测试确认失败**

```bash
python3 -m pytest -q homo_multirobot_formation_control/test/test_measure_mocap_velocity_model.py
```

预期：失败，指出 `write_outputs` 和命令生成函数不存在。

- [ ] **Step 3: 实现节点状态机与确定性输出**

实现 `make_command(mode, target)`；动捕回调记录 `header.stamp`、`time.monotonic()`、三个原始速度与 `scalar_response`。状态机对每一试次依次执行 `settle`、`hold`、`stop`，在 `hold` 发布目标命令，其余阶段发布零命令；每个循环样本附带 `trial`、`phase`、相对时间与命令。

当动捕陈旧超过 `mocap_timeout` 时抛出可报告错误，`finally` 中调用 `publish_stop()`。将每次运行的时间戳目录创建在 `--out-dir` 或包内默认根目录下。`write_outputs` 必须写入所有指定 CSV 列，即使某些拟合段因样本不足而没有结果；不足样本的段在 `fit_summary.csv` 标为 `invalid`。

从每个 `hold` 段和随后 `stop` 段提取相对时间和响应，调用两模型并将每段参数和指标写入汇总。Matplotlib 使用 `Agg` 后端，为每个段画观测、目标、一阶与固定加速度预测，生成单个 `comparison.png`。

- [ ] **Step 4: 运行测试确认通过**

```bash
python3 -m pytest -q homo_multirobot_formation_control/test/test_measure_mocap_velocity_model.py
python3 homo_multirobot_formation_control/scripts/measure_mocap_velocity_model.py \
  --mode move --target 0.15 --help
```

预期：pytest 通过；帮助显示 `--namespace`、`--trials`、`--hold-seconds`、`--settle-seconds`、`--mocap-timeout` 与 `--out-dir`。

- [ ] **Step 5: 提交采集和输出功能**

```bash
git add homo_multirobot_formation_control/scripts/measure_mocap_velocity_model.py \
  homo_multirobot_formation_control/test/test_measure_mocap_velocity_model.py
git commit -m "支持动捕阶跃速度响应采集"
```

### Task 3: 安装脚本、文档和端到端静态验证

**Files:**
- Modify: `homo_multirobot_formation_control/CMakeLists.txt:244-257`
- Modify: `homo_multirobot_formation_control/README.md:762-804`
- Modify: `homo_multirobot_formation_control/test/test_measure_mocap_velocity_model.py`

**Interfaces:**
- Consumes: 可执行脚本 `scripts/measure_mocap_velocity_model.py`。
- Produces: `ros2 run homo_multirobot_formation_control measure_mocap_velocity_model.py` 和带安全说明的 README 用法。

- [ ] **Step 1: 写入失败的安装与安全契约测试**

在测试中读取 `CMakeLists.txt` 与脚本源码，断言：

```python
assert "scripts/measure_mocap_velocity_model.py" in cmake_source
assert "finally:" in script_source
assert "publish_stop()" in script_source
```

- [ ] **Step 2: 运行测试确认失败**

```bash
python3 -m pytest -q homo_multirobot_formation_control/test/test_measure_mocap_velocity_model.py
```

预期：失败，指出 CMake 尚未安装该脚本。

- [ ] **Step 3: 安装并说明实车操作**

在既有 `install(PROGRAMS ...)` 列表中加入该脚本。README 在 `measure_motor_latency` 后新增小节，说明 `move`/`turn` 示例、默认 `/robot2` 话题、动捕全局速度的合成方式、输出目录与清空场地后操作的要求。说明 `--target` 必填且脚本结束、异常或动捕超时均发零速度。

- [ ] **Step 4: 构建并验证安装后的 CLI**

```bash
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select homo_multirobot_formation_control --symlink-install \
  --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 run homo_multirobot_formation_control measure_mocap_velocity_model.py --help
python3 -m pytest -q src/homo-ctrl-multirobot-ros2/homo_multirobot_formation_control/test/test_measure_mocap_velocity_model.py
```

预期：构建成功，`ros2 run` 显示 CLI 帮助，所有新测试通过；不在未清空的实车场地自动执行运动实验。

- [ ] **Step 5: 提交集成与文档**

```bash
git add homo_multirobot_formation_control/CMakeLists.txt \
  homo_multirobot_formation_control/README.md \
  homo_multirobot_formation_control/test/test_measure_mocap_velocity_model.py
git commit -m "提供动捕速度模型测试命令"
```

### Task 4: 完整验证

**Files:**
- Verify only: 所有 Task 1–3 文件。

- [ ] **Step 1: 执行完整包测试**

```bash
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
ctest --test-dir build/homo_multirobot_formation_control --output-on-failure
```

- [ ] **Step 2: 审查本功能差异**

```bash
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws/src/homo-ctrl-multirobot-ros2
git diff --check bb19e19..HEAD
git status --short
```

预期：差异无空白错误；报告与本功能无关的已有 CTest 失败，但不回退用户实验配置。
