# 6D Map HPC Artstein 延迟补偿开关 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 6D Map HPC Artstein 控制器提供独立的 Artstein 延迟补偿开关。

**Architecture:** 扩展无 ROS 依赖的 `ArtsteinPredictorNd`，由参数决定是否使用命令历史积分与 `Td` 状态外推；节点的 Leader 时域用同一参数控制。已有前向预测开关保持独立，因此四种组合均可用。

**Tech Stack:** ROS 2 Humble、rclcpp、C++17、Eigen、CMake/CTest、Python unittest。

## Global Constraints

- 新参数为 `enable_artstein_compensation`，类型 bool，默认 `true`。
- 关闭时不得执行 Follower 历史积分或 Leader/Follower 的 `Td` 外推。
- `enable_forward_prediction` 独立工作，保留现有默认值与用户当前 YAML 设置。
- 不修改无补偿 YAML，也不覆盖 Artstein YAML 中现有的 `enable_forward_prediction: false`。

---

### Task 1: 为预测器添加 Artstein 选择路径

**Files:**

- Modify: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/artstein_predictor_nd.hpp`
- Modify: `homo_multirobot_formation_control/test/test_artstein_predictor_nd.cpp`

**Interfaces:**

- Produces: `ArtsteinPredictorNd::predict(const Eigen::VectorXd&, const Eigen::VectorXd&, bool enable_artstein_compensation, bool enable_forward_prediction) const`。

- [ ] **Step 1: Write the failing test**

扩展 `test_artstein_predictor_nd.cpp`，令 `x` 为当前状态并使用非零命令历史；断言关闭 Artstein 且关闭前向预测时预测值正好是 `x`，关闭 Artstein 且开启前向预测时结果不同于 `x`：

```cpp
const Eigen::VectorXd no_compensation = predictor.predict(
    x, command, false, false);
const Eigen::VectorXd forward_only = predictor.predict(
    x, command, false, true);
assert((no_compensation - x).norm() < 1e-12);
assert((forward_only - no_compensation).norm() > 1e-6);
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
source /opt/ros/humble/setup.bash
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
colcon build --base-paths src/homo-ctrl-multirobot-ros2 --packages-select homo_multirobot_formation_control --symlink-install --allow-overriding homo_multirobot_formation_control
source install/setup.bash
ctest --test-dir build/homo_multirobot_formation_control -R test_artstein_predictor_nd --output-on-failure
```

Expected: 编译失败，因为预测器尚未提供四参数接口。

- [ ] **Step 3: Write the minimal implementation**

将 `predict` 签名扩展为四参数。以当前测量/Artstein 状态作为输入；仅当 `enable_artstein_compensation` 为 true 时计算：

```cpp
Eigen::VectorXd compensated = artstein_state;
if (enable_artstein_compensation) {
  compensated = (A_ * Td_).exp() * compensated;
}
if (!enable_forward_prediction) return compensated;
```

随后将已有 `tau` 前向预测公式作用于 `compensated`。

- [ ] **Step 4: Run the test to verify it passes**

重复 Step 2 的命令。

Expected: `test_artstein_predictor_nd` 通过。

- [ ] **Step 5: Commit**

```bash
git add homo_multirobot_formation_control/include/homo_multirobot_formation_control/artstein_predictor_nd.hpp homo_multirobot_formation_control/test/test_artstein_predictor_nd.cpp
git commit -m "支持6D Map HPC Artstein补偿开关"
```

### Task 2: 接入 ROS 参数与启动接口

**Files:**

- Modify: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_6d_map_hpc_artstein.hpp`
- Modify: `homo_multirobot_formation_control/src/formation_control_node_6d_map_hpc_artstein.cpp`
- Modify: `homo_multirobot_formation_control/config/formation_single_follower_6d_map_hpc_artstein.yaml`
- Modify: `homo_multirobot_formation_control/launch/formation_single_follower_6d_map_hpc_artstein.launch.py`
- Modify: `homo_multirobot_formation_control/test/test_launch_yaml_defaults.py`
- Modify: `homo_multirobot_formation_control/README.md`
- Modify: `homo_multirobot_formation_control/config/README.md`

**Interfaces:**

- Consumes: `predict(..., enable_artstein_compensation_, enable_forward_prediction_)`。
- Produces: `enable_artstein_compensation:=false` launch 覆盖。

- [ ] **Step 1: Write the failing interface test**

向 `test_launch_yaml_defaults.py` 添加：

```python
self.assertIn("enable_artstein_compensation", names)
self.assertIn('LaunchConfiguration("enable_artstein_compensation")', source)
self.assertIn('"enable_artstein_compensation": enable_artstein_compensation', source)
```

- [ ] **Step 2: Run test to verify it fails**

```bash
python3 homo_multirobot_formation_control/test/test_launch_yaml_defaults.py
```

Expected: YAML 与 launch 尚未定义该参数，测试失败。

- [ ] **Step 3: Write minimal parameter plumbing**

声明 `enable_artstein_compensation_ = declare_parameter("enable_artstein_compensation", true);`。当其关闭时，`predict_follower_state` 传入当前 `x4/x2` 而不叠加 `integral()`，并传入 false 以跳过 `Td` 外推。Leader 时域替换为：

```cpp
const double prediction_horizon =
    (enable_artstein_compensation_ ? Td_ : 0.0) +
    (enable_forward_prediction_ ? std::max(tau_v_, tau_w_) : 0.0);
```

在 Artstein YAML 中新增 true 的注释参数但保留用户的前向预测值；在 launch 映射、声明并透传；README 描述四种组合。

- [ ] **Step 4: Run focused tests**

```bash
python3 homo_multirobot_formation_control/test/test_launch_yaml_defaults.py
source /opt/ros/humble/setup.bash
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
colcon build --base-paths src/homo-ctrl-multirobot-ros2 --packages-select homo_multirobot_formation_control --symlink-install --allow-overriding homo_multirobot_formation_control
source install/setup.bash
ctest --test-dir build/homo_multirobot_formation_control -R 'test_(artstein_predictor_nd|launch_yaml_defaults)' --output-on-failure
```

Expected: 包构建成功，两个测试通过。

- [ ] **Step 5: Commit**

```bash
git add homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_6d_map_hpc_artstein.hpp homo_multirobot_formation_control/src/formation_control_node_6d_map_hpc_artstein.cpp homo_multirobot_formation_control/config/formation_single_follower_6d_map_hpc_artstein.yaml homo_multirobot_formation_control/launch/formation_single_follower_6d_map_hpc_artstein.launch.py homo_multirobot_formation_control/test/test_launch_yaml_defaults.py homo_multirobot_formation_control/README.md homo_multirobot_formation_control/config/README.md
git commit -m "配置6D Map HPC Artstein补偿开关"
```
