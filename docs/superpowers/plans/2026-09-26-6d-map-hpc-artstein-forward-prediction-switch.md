# 6D Map HPC Artstein 前向预测开关 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 6D Map HPC Artstein 控制器可独立关闭 `tau/tau_yaw` 前向预测，同时保留 Artstein 的 `Td` 输入延迟补偿。

**Architecture:** 把无 ROS 依赖的 `ArtsteinPredictorNd` 移入独立头文件，并由显式布尔参数选择“仅 Td”或“Td+tau”状态映射。ROS 节点将该参数用于 Follower 预测和 Leader 外推时域；YAML 与 launch 对外透传参数。

**Tech Stack:** ROS 2 Humble、rclcpp、C++17、Eigen、CMake/CTest、Python unittest。

## Global Constraints

- 参数名为 `enable_forward_prediction`，类型 bool，默认 `true`。
- `false` 时保留 Artstein 历史积分和 `Td` 外推，只跳过 `tau/tau_yaw` 前向预测。
- `tau` 和 `tau_yaw` 仍必须大于零。
- 不修改带有用户未提交变更的 `formation_single_follower_6d_map_hpc.yaml`。

---

### Task 1: 为预测器增加可测试的前向预测开关

**Files:**

- Create: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/artstein_predictor_nd.hpp`
- Modify: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_6d_map_hpc_artstein.hpp`
- Create: `homo_multirobot_formation_control/test/test_artstein_predictor_nd.cpp`
- Modify: `homo_multirobot_formation_control/CMakeLists.txt`

**Interfaces:**

- Produces: `formation_control::ArtsteinPredictorNd::predict(const Eigen::VectorXd&, const Eigen::VectorXd&, bool) const`。
- Consumes: `build()` 和 `integral()` 保持已有签名。

- [ ] **Step 1: Write the failing C++ unit test**

创建 `test/test_artstein_predictor_nd.cpp`，以 `tau=1.0`、`Td=0.5` 构建一维一阶预测器，并断言：

```cpp
const Eigen::VectorXd delay_only = predictor.predict(z, command, false);
const Eigen::VectorXd forward = predictor.predict(z, command, true);
assert((delay_only - ((A * 0.5).exp() * z)).norm() < 1e-12);
assert((forward - delay_only).norm() > 1e-6);
```

- [ ] **Step 2: Run test to verify it fails**

在 `CMakeLists.txt` 添加仅链接 Eigen 的 `test_artstein_predictor_nd` 目标与 CTest 注册，然后执行：

```bash
source /opt/ros/humble/setup.bash
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
colcon build --packages-select homo_multirobot_formation_control --symlink-install
source install/setup.bash
ctest --test-dir build/homo_multirobot_formation_control -R test_artstein_predictor_nd --output-on-failure
```

Expected: 编译失败，因为新预测器头文件/API 尚不存在。

- [ ] **Step 3: Write minimal implementation**

将现有 `ArtsteinPredictorNd` 原样移动到 `artstein_predictor_nd.hpp`，只保留 deque/vector/Eigen 依赖。将预测函数改为：

```cpp
Eigen::VectorXd delay_free = (A_ * Td_).exp() * artstein_state;
if (!enable_forward_prediction) return delay_free;
// 保留现有 tau 前向预测公式
```

节点头文件 include 新头文件并删除原内联结构。

- [ ] **Step 4: Run test to verify it passes**

重复 Step 2 的构建与 CTest 命令。

Expected: `test_artstein_predictor_nd` 通过。

- [ ] **Step 5: Commit**

```bash
git add homo_multirobot_formation_control/include/homo_multirobot_formation_control/artstein_predictor_nd.hpp homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_6d_map_hpc_artstein.hpp homo_multirobot_formation_control/test/test_artstein_predictor_nd.cpp homo_multirobot_formation_control/CMakeLists.txt
git commit -m "支持6D Map HPC Artstein前向预测开关"
```

### Task 2: 将参数接入节点和启动接口

**Files:**

- Modify: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_6d_map_hpc_artstein.hpp`
- Modify: `homo_multirobot_formation_control/src/formation_control_node_6d_map_hpc_artstein.cpp`
- Modify: `homo_multirobot_formation_control/config/formation_single_follower_6d_map_hpc_artstein.yaml`
- Modify: `homo_multirobot_formation_control/launch/formation_single_follower_6d_map_hpc_artstein.launch.py`
- Modify: `homo_multirobot_formation_control/test/test_launch_yaml_defaults.py`
- Modify: `homo_multirobot_formation_control/README.md`
- Modify: `homo_multirobot_formation_control/config/README.md`

**Interfaces:**

- Consumes: `predict(..., enable_forward_prediction_)` from Task 1。
- Produces: `enable_forward_prediction:=false` launch 覆盖。

- [ ] **Step 1: Write the failing launch/config test**

在 `test_launch_yaml_defaults.py` 添加：

```python
names = yaml_parameter_names(CONFIG_DIR / "formation_single_follower_6d_map_hpc_artstein.yaml")
source = (LAUNCH_DIR / "formation_single_follower_6d_map_hpc_artstein.launch.py").read_text(encoding="utf-8")
self.assertIn("enable_forward_prediction", names)
self.assertIn('LaunchConfiguration("enable_forward_prediction")', source)
self.assertIn('"enable_forward_prediction": enable_forward_prediction', source)
```

- [ ] **Step 2: Run test to verify it fails**

```bash
python3 homo_multirobot_formation_control/test/test_launch_yaml_defaults.py
```

Expected: 因 YAML 与 launch 尚未公开参数而失败。

- [ ] **Step 3: Write minimal implementation**

声明 `enable_forward_prediction_ = declare_parameter("enable_forward_prediction", true);`，传入两个 predictor `predict` 调用。将所有 Leader 预测时域替换为：

```cpp
const double prediction_horizon = Td_ +
    (enable_forward_prediction_ ? std::max(tau_v_, tau_w_) : 0.0);
```

在 Artstein YAML 的 `Td` 后添加默认 true 和中文注释；在 launch 添加 `LaunchConfiguration`、Node 参数映射和 `DeclareLaunchArgument`。两个 README 明确 `false` 仍保留 `Td`，只关闭 tau 前向预测。

- [ ] **Step 4: Run tests to verify they pass**

```bash
python3 homo_multirobot_formation_control/test/test_launch_yaml_defaults.py
source /opt/ros/humble/setup.bash
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
colcon build --packages-select homo_multirobot_formation_control --symlink-install
source install/setup.bash
ctest --test-dir build/homo_multirobot_formation_control -R 'test_(artstein_predictor_nd|launch_yaml_defaults)' --output-on-failure
```

Expected: 构建成功，两个测试通过。

- [ ] **Step 5: Commit**

```bash
git add homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_6d_map_hpc_artstein.hpp homo_multirobot_formation_control/src/formation_control_node_6d_map_hpc_artstein.cpp homo_multirobot_formation_control/config/formation_single_follower_6d_map_hpc_artstein.yaml homo_multirobot_formation_control/launch/formation_single_follower_6d_map_hpc_artstein.launch.py homo_multirobot_formation_control/test/test_launch_yaml_defaults.py homo_multirobot_formation_control/README.md homo_multirobot_formation_control/config/README.md
git commit -m "配置6D Map HPC Artstein前向预测开关"
```

### Task 3: 完整包验证

**Files:**

- Verify only: Tasks 1-2 的文件。

- [ ] **Step 1: Run the package test suite**

```bash
source /opt/ros/humble/setup.bash
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
colcon build --packages-select homo_multirobot_formation_control --symlink-install
source install/setup.bash
ctest --test-dir build/homo_multirobot_formation_control --output-on-failure
```

Expected: 构建退出码为 0，所有配置的 CTest 通过。

- [ ] **Step 2: Inspect scoped changes**

```bash
git diff --check
git status --short
```

Expected: 无空白错误；无补偿配置文件的预先存在修改仍未被纳入本次变更。
