# 6D Map HPC Artstein 独立前向预测时域 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 6D map HPC Artstein 控制器新增独立、可配置且向后兼容的 Follower/Leader 前向预测时域。

**Architecture:** `ArtsteinPredictorNd` 保持 Artstein 的历史积分与 `Td` 外推不变，只将额外一阶响应外推从固定一个 `tau` 推广到调用者指定的 `H`。节点将 `forward_prediction_horizon <= 0` 解析为旧默认 `max(tau, tau_yaw)`，再用同一个有效时域驱动 Follower 与 Leader；YAML 和 launch 只传递原始参数值。

**Tech Stack:** ROS 2 Humble、C++17、Eigen、ament_cmake、Python unittest、YAML launch 配置。

## Global Constraints

- 只修改 6D map HPC Artstein 路径；不改变 4D Artstein、普通 6D Artstein Disc 与 HOCBF。
- `forward_prediction_horizon <= 0` 必须保持旧行为：有效时域为 `max(tau, tau_yaw)`。
- `enable_forward_prediction=false` 时必须完全跳过额外 `H` 前向预测。
- 在工作空间根目录 `/home/l1anggmgo/ros-projects/homo_multirobot_ws` 执行 colcon，不能在源码仓库根目录执行。

---

### Task 1: 为任意前向时域建立预测器回归测试

**Files:**
- Modify: `homo_multirobot_formation_control/test/test_artstein_predictor_nd.cpp`
- Modify: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/artstein_predictor_nd.hpp:61-87`

**Interfaces:**
- Consumes: `ArtsteinPredictorNd::build(A, B, tau, Td, h)`。
- Produces: `predict(state, current_cmd, enable_artstein_compensation, enable_forward_prediction, forward_horizon)`；`forward_horizon` 单位为秒。

- [ ] **Step 1: 写入指定时域的失败测试**

在 `test_artstein_predictor_nd.cpp` 中，在已有 `forward_only` 断言之后加入：

```cpp
  const double H = 0.25;
  const Eigen::VectorXd custom_horizon = predictor.predict(
      z, command, false, true, H);
  const double decay = std::exp(-H);
  Eigen::VectorXd expected_custom(2);
  expected_custom(0) = z(0) + command(0) * H
      + (1.0 - decay) * (z(1) - command(0));
  expected_custom(1) = command(0) + decay * (z(1) - command(0));
  assert((custom_horizon - expected_custom).norm() < 1e-12);
```

- [ ] **Step 2: 运行测试确认失败**

从工作空间根目录运行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select homo_multirobot_formation_control --symlink-install \
  --cmake-args -DBUILD_TESTING=ON --cmake-target test_artstein_predictor_nd
ctest --test-dir build/homo_multirobot_formation_control -R test_artstein_predictor_nd --output-on-failure
```

预期：编译失败，指出 `predict` 不接受第五个 `H` 参数。

- [ ] **Step 3: 以最小接口扩展实现任意时域预测**

将四参数 `predict` 扩展为接收 `double forward_horizon`，并保留三参数重载调用 `tau_`，使旧调用继续等价于预测一个时间常数。额外前向段使用：

```cpp
const double decay = std::exp(-forward_horizon / tau_);
predicted.head(q) = compensated.head(q) + current_cmd * forward_horizon
    + tau_ * (1.0 - decay) * (compensated.tail(q) - current_cmd);
predicted.tail(q) = current_cmd + decay * (compensated.tail(q) - current_cmd);
```

对 `enable_forward_prediction == false` 保留原样返回 `compensated`；不加入未被需求要求的截断或额外策略。

- [ ] **Step 4: 运行测试确认通过**

重复步骤 2 的构建和 CTest 命令。

预期：`test_artstein_predictor_nd` PASS，已有 `Td` 补偿和默认单 `tau` 测试仍 PASS。

- [ ] **Step 5: 提交预测器测试与实现**

```bash
git add homo_multirobot_formation_control/test/test_artstein_predictor_nd.cpp \
  homo_multirobot_formation_control/include/homo_multirobot_formation_control/artstein_predictor_nd.hpp
git commit -m "支持Artstein自定义前向预测时域"
```

### Task 2: 在 6D map 节点解析并应用有效预测时域

**Files:**
- Modify: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_6d_map_hpc_artstein.hpp:63-69`
- Modify: `homo_multirobot_formation_control/src/formation_control_node_6d_map_hpc_artstein.cpp:58-64,250-271,314-316`
- Create: `homo_multirobot_formation_control/test/test_6d_map_hpc_artstein_forward_prediction.py`
- Modify: `homo_multirobot_formation_control/CMakeLists.txt:270-273`

**Interfaces:**
- Consumes: `ArtsteinPredictorNd::predict(..., bool, bool, double)` from Task 1.
- Produces: ROS 参数 `forward_prediction_horizon`，默认 `0.0`；成员 `forward_prediction_horizon_` 保存解析后的有效正时域。

- [ ] **Step 1: 写入节点路径的失败测试**

创建 Python unittest `test_6d_map_hpc_artstein_forward_prediction.py`，读取节点 `.cpp` 与 `.hpp`，并断言：

```python
self.assertIn('declare_parameter("forward_prediction_horizon", 0.0)', source)
self.assertIn('std::max(tau_v_, tau_w_)', source)
self.assertEqual(
    source.count('enable_forward_prediction_, forward_prediction_horizon_'), 2)
self.assertIn('double forward_prediction_horizon_', header)
```

注册为 CTest：

```cmake
add_test(NAME test_6d_map_hpc_artstein_forward_prediction
  COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/test/test_6d_map_hpc_artstein_forward_prediction.py)
```

该契约测试防止参数只影响 Leader 而遗漏 Follower。

- [ ] **Step 2: 运行测试确认失败**

从工作空间根目录运行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select homo_multirobot_formation_control --symlink-install \
  --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/homo_multirobot_formation_control \
  -R test_6d_map_hpc_artstein_forward_prediction --output-on-failure
```

预期：测试失败，因为参数声明和调用尚不存在。

- [ ] **Step 3: 实现有效时域回退和统一应用**

在构造函数读取原始 `forward_prediction_horizon`：

```cpp
const double configured_forward_prediction_horizon =
    declare_parameter("forward_prediction_horizon", 0.0);
forward_prediction_horizon_ = configured_forward_prediction_horizon > 0.0
    ? configured_forward_prediction_horizon
    : std::max(tau_v_, tau_w_);
```

在平移与偏航两个 `predict(...)` 调用中追加 `forward_prediction_horizon_`。将 Leader 的 `prediction_horizon` 改为：

```cpp
(enable_artstein_compensation_ ? Td_ : 0.0) +
(enable_forward_prediction_ ? forward_prediction_horizon_ : 0.0)
```

保留 `enable_forward_prediction_` 为 false 时 Leader 和 Follower 都不额外外推的既有语义。

- [ ] **Step 4: 运行测试确认通过**

重复步骤 2 的命令，并额外构建控制器目标：

```bash
colcon build --packages-select homo_multirobot_formation_control --symlink-install \
  --cmake-args -DBUILD_TESTING=ON --cmake-target formation_control_node_6d_map_hpc_artstein
```

预期：契约测试和节点目标均通过。

- [ ] **Step 5: 提交节点接线**

```bash
git add homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_6d_map_hpc_artstein.hpp \
  homo_multirobot_formation_control/src/formation_control_node_6d_map_hpc_artstein.cpp \
  homo_multirobot_formation_control/test/test_6d_map_hpc_artstein_forward_prediction.py \
  homo_multirobot_formation_control/CMakeLists.txt
git commit -m "解耦6D Map HPC前向预测时域"
```

### Task 3: 公开配置并验证 YAML/launch 契约

**Files:**
- Modify: `homo_multirobot_formation_control/config/formation_single_follower_6d_map_hpc_artstein.yaml:37-46`
- Modify: `homo_multirobot_formation_control/launch/formation_single_follower_6d_map_hpc_artstein.launch.py:56-60,110-114,168-172`
- Modify: `homo_multirobot_formation_control/launch/formation_single_follower_6d_map_hpc_artstein_mocap.launch.py:1-13`
- Modify: `homo_multirobot_formation_control/test/test_launch_yaml_defaults.py:82-92`
- Modify: `homo_multirobot_formation_control/doc/6d_map_hpc_artstein_simulation.md`

**Interfaces:**
- Consumes: Node parameter `forward_prediction_horizon`, default `0.0`.
- Produces: YAML default、普通 launch 覆盖和 mocap include 转发的同名参数；用户可运行 `forward_prediction_horizon:=0.30`。

- [ ] **Step 1: 写入配置传递的失败测试**

将 `test_6d_map_hpc_artstein_forward_prediction_parameter` 扩展为：

```python
self.assertIn("forward_prediction_horizon", names)
self.assertIn('LaunchConfiguration("forward_prediction_horizon")', source)
self.assertIn(
    '"forward_prediction_horizon": forward_prediction_horizon', source)
mocap_source = (
    LAUNCH_DIR / "formation_single_follower_6d_map_hpc_artstein_mocap.launch.py"
).read_text(encoding="utf-8")
self.assertIn('"forward_prediction_horizon": LaunchConfiguration(', mocap_source)
```

- [ ] **Step 2: 运行测试确认失败**

```bash
python3 homo_multirobot_formation_control/test/test_launch_yaml_defaults.py
```

预期：失败并报告缺少 `forward_prediction_horizon`。

- [ ] **Step 3: 最小化公开 YAML 和两个 launch 接口**

在 YAML 的 `enable_forward_prediction` 前添加：

```yaml
# 额外前向预测时域（s）；小于等于 0 时沿用 max(tau, tau_yaw)。
forward_prediction_horizon: 0.0
```

普通 launch 创建、传入参数字典并声明 `forward_prediction_horizon`。mocap launch 声明同名 `LaunchConfiguration`，并在 `IncludeLaunchDescription(..., launch_arguments=...)` 中传递该值，使其可从 mocap 顶层直接覆盖。文档说明 `0.0` 的回退规则、有效的总 Leader 时域 `Td + H`，以及示例：

```bash
ros2 launch homo_multirobot_formation_control \
  formation_single_follower_6d_map_hpc_artstein.launch.py \
  forward_prediction_horizon:=0.30
```

- [ ] **Step 4: 运行配置测试和 show-args 验证**

```bash
python3 homo_multirobot_formation_control/test/test_launch_yaml_defaults.py
source /opt/ros/humble/setup.bash
source /home/l1anggmgo/ros-projects/homo_multirobot_ws/install/setup.bash
ros2 launch homo_multirobot_formation_control \
  formation_single_follower_6d_map_hpc_artstein.launch.py --show-args
```

预期：Python 测试 PASS；输出包含 `forward_prediction_horizon` 且默认值为 `0.0`。

- [ ] **Step 5: 提交配置与文档**

```bash
git add homo_multirobot_formation_control/config/formation_single_follower_6d_map_hpc_artstein.yaml \
  homo_multirobot_formation_control/launch/formation_single_follower_6d_map_hpc_artstein.launch.py \
  homo_multirobot_formation_control/launch/formation_single_follower_6d_map_hpc_artstein_mocap.launch.py \
  homo_multirobot_formation_control/test/test_launch_yaml_defaults.py \
  homo_multirobot_formation_control/doc/6d_map_hpc_artstein_simulation.md
git commit -m "公开6D Map HPC前向预测时域参数"
```

### Task 4: 运行完整受影响验证并收尾

**Files:**
- Verify only: `homo_multirobot_formation_control/CMakeLists.txt`
- Verify only: 已修改的预测器、节点、配置、launch、测试与文档文件。

**Interfaces:**
- Consumes: Tasks 1–3 的完整变更。
- Produces: 经 CTest、Python 配置测试、编译和 Git diff 验证的可交付改动。

- [ ] **Step 1: 全量构建受影响包**

```bash
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select homo_multirobot_formation_control --symlink-install \
  --cmake-args -DBUILD_TESTING=ON
```

- [ ] **Step 2: 执行受影响 CTest 与 Python 测试**

```bash
ctest --test-dir build/homo_multirobot_formation_control \
  -R 'test_(artstein_predictor_nd|6d_map_hpc_artstein_forward_prediction|launch_yaml_defaults)' \
  --output-on-failure
python3 src/homo-ctrl-multirobot-ros2/homo_multirobot_formation_control/test/test_launch_yaml_defaults.py
```

预期：全部 PASS。

- [ ] **Step 3: 审查改动范围与最终提交**

```bash
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws/src/homo-ctrl-multirobot-ros2
git diff --check HEAD~3..HEAD
git status --short
```

预期：无空白错误；除计划文档外没有未提交改动。若任务提交数与实际不同，则以本功能所有提交范围替代 `HEAD~3..HEAD`。
