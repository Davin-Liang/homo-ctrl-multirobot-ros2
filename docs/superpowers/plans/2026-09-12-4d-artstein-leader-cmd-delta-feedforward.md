# 4D Artstein Leader 命令速度增量前馈实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 4D Artstein ROS 2 节点增加默认关闭的 Leader `/cmd_vel` 速度增量前馈，并保持 Artstein、预测及安全约束的现有行为。

**Architecture:** 新建与 ROS 消息无关的 `LeaderCommandDeltaFeedforward` 小类，负责新消息判定、map 系速度差分、滤波、模长限幅、超时和一次性消费。节点仅负责订阅、用 Leader yaw 将 body 命令转 map 系，并在 HPC 输出 `out_map` 后、所有安全约束前叠加被消费的增量。

**Tech Stack:** ROS 2 Humble、C++17、rclcpp、geometry_msgs、Eigen、ament_cmake。

## Global Constraints

- 不修改 Leader 节点或新增 `AccelStamped` 接口。
- `enable_leader_cmd_feedforward=false` 必须是默认值，且输出与旧路径一致。
- 单个 Leader 命令差分只能消费一次；过期或无新消息时增量为零。
- 前馈必须在 map 系 `out_map` 上、径向安全和全部限幅之前施加。
- 最终约束后的 Follower 命令必须继续写入既有 Artstein 历史。

---

### Task 1: 创建可独立测试的速度增量状态机

**Files:**
- Create: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/leader_command_delta_feedforward.hpp`
- Create: `homo_multirobot_formation_control/test/test_leader_command_delta_feedforward.cpp`
- Modify: `homo_multirobot_formation_control/CMakeLists.txt`

**Interfaces:**
- Produces: `formation_control::LeaderCommandDeltaFeedforward(bool enabled, double lpf_tau, double delta_max)`，其中 `delta_max=max_linear_accel/control_rate`。
- Produces: `void ingest(const Eigen::Vector2d& command_map, double received_sec)`；`Eigen::Vector2d consume(double now_sec, double timeout_sec)`。

- [ ] **Step 1: 写入失败测试**

```cpp
formation_control::LeaderCommandDeltaFeedforward ff(true, 0.0, 0.6 / 20.0);
ff.ingest(Eigen::Vector2d(0.2, 0.0), 1.0);
assert(ff.consume(1.0, 0.15).isZero(1e-12));  // 首帧
ff.ingest(Eigen::Vector2d(0.22, -0.01), 1.05);
assert((ff.consume(1.05, 0.15) - Eigen::Vector2d(0.02, -0.01)).norm() < 1e-12);
assert(ff.consume(1.06, 0.15).isZero(1e-12)); // 不重复消费
```

- [ ] **Step 2: 确认测试失败**

Run: `cd ../.. && source /opt/ros/humble/setup.bash && colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=ON && ctest --test-dir build/homo_multirobot_formation_control -R leader_command_delta_feedforward --output-on-failure`

Expected: 编译失败，提示头文件或类不存在。

- [ ] **Step 3: 最小实现**

实现仅含 Eigen 的状态机：第一帧只保存；后续 `ingest` 计算 `command_map - previous_command_map`；按 `alpha=dt/(lpf_tau+dt)` 滤波（`lpf_tau<=0` 时不滤波）；按 `delta_max` 缩放二维模长；设置 `pending_`。`consume` 在关闭、无 pending 或 `now_sec-received_sec>timeout_sec` 时返回零，否则返回 pending 并清零。

- [ ] **Step 4: 添加限幅与超时测试并通过**

```cpp
formation_control::LeaderCommandDeltaFeedforward capped(true, 0.0, 0.6 / 20.0);
capped.ingest(Eigen::Vector2d::Zero(), 1.0);
capped.ingest(Eigen::Vector2d(0.3, 0.4), 1.05);
assert(std::abs(capped.consume(1.05, 0.15).norm() - 0.03) < 1e-12);
assert(capped.consume(2.0, 0.15).isZero(1e-12));
```

Run the Task 1 command again. Expected: PASS.

### Task 2: 接入 4D Artstein 节点并暴露配置

**Files:**
- Modify: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_4d_artstein.hpp`
- Modify: `homo_multirobot_formation_control/src/formation_control_node_4d_artstein.cpp`
- Modify: `homo_multirobot_formation_control/config/formation_single_follower_4d_artstein.yaml`
- Modify: `homo_multirobot_formation_control/launch/formation_single_follower_4d_artstein.launch.py`

**Interfaces:**
- Consumes: `LeaderCommandDeltaFeedforward::ingest/consume`。
- Produces: launch/YAML parameters `enable_leader_cmd_feedforward`, `leader_cmd_timeout`, `leader_cmd_delta_lpf_tau`；增量上限由已有 `max_linear_accel/control_rate` 推导。

- [ ] **Step 1: 写入失败测试**

扩展 Task 1 的测试：构造 `LeaderCommandDeltaFeedforward(false, 0.0, 1.0)`，输入两帧命令后断言 `consume` 为零。该测试表达节点开关关闭时不会为 `out_map` 贡献任何速度增量。

- [ ] **Step 2: 确认测试失败**

Run the Task 1 CTest command. Expected: FAIL because disabled mode has not been implemented.

- [ ] **Step 3: 最小节点实现**

在构造函数声明三个新增参数，默认值分别为 `false`、`0.15`、`0.10`；以现有 `max_linear_accel / control_rate` 构造前馈状态机。开关开启时订阅 `leader_ns_ + "/cmd_vel"`。timer 中用当前 `leader_yaw` 将最新 body 速度转换为 map 系：

```cpp
Eigen::Vector2d command_map(
  cmd.linear.x * std::cos(leader_yaw) - cmd.linear.y * std::sin(leader_yaw),
  cmd.linear.x * std::sin(leader_yaw) + cmd.linear.y * std::cos(leader_yaw));
feedforward_.ingest(command_map, get_clock()->now().seconds());
```

仅对尚未处理的新消息调用 `ingest`。在 `ctrl_->lpc_calculate` 后执行：

```cpp
const Eigen::Vector2d leader_delta_v =
    feedforward_.consume(get_clock()->now().seconds(), leader_cmd_timeout_);
out_map += leader_delta_v;
```

随后不改动径向安全、速度/轮速/加速度限制和 Follower 历史回写。

- [ ] **Step 4: 配置与启动参数**

在 YAML 写入三项新增默认值；launch 读取三项 `LaunchConfiguration` 并传给 node，增加带单位和默认关闭语义的 `DeclareLaunchArgument`。不新增速度增量上限参数。

- [ ] **Step 5: 编译与 CTest 验证**

Run: `cd ../.. && source /opt/ros/humble/setup.bash && colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=ON && ctest --test-dir build/homo_multirobot_formation_control -R 'leader_command_delta_feedforward|homo_controller_4d_artstein' --output-on-failure`

Expected: 构建成功且两个测试通过。

### Task 3: 诊断、回归检查和提交

**Files:**
- Modify: `homo_multirobot_formation_control/src/formation_control_node_4d_artstein.cpp`
- Test: `homo_multirobot_formation_control/test/test_leader_command_delta_feedforward.cpp`

- [ ] **Step 1: 增加节流诊断**

每秒输出 `leader_ff=on/off`、最新命令年龄、原始/滤波后增量、当前周期消费的 `delta_v`。开关关闭时输出零增量。

- [ ] **Step 2: 运行完整包测试与 YAML 启动检查**

Run: `cd ../.. && source /opt/ros/humble/setup.bash && colcon test --packages-select homo_multirobot_formation_control --event-handlers console_direct+ && colcon test-result --verbose`

Expected: 所有已构建测试通过。

- [ ] **Step 3: 提交源码、测试、配置和计划**

```bash
git add homo_multirobot_formation_control/include/homo_multirobot_formation_control/leader_command_delta_feedforward.hpp \
  homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_4d_artstein.hpp \
  homo_multirobot_formation_control/src/formation_control_node_4d_artstein.cpp \
  homo_multirobot_formation_control/config/formation_single_follower_4d_artstein.yaml \
  homo_multirobot_formation_control/launch/formation_single_follower_4d_artstein.launch.py \
  homo_multirobot_formation_control/test/test_leader_command_delta_feedforward.cpp \
  homo_multirobot_formation_control/CMakeLists.txt \
  docs/superpowers/plans/2026-09-12-4d-artstein-leader-cmd-delta-feedforward.md
git commit -m "增加4D Artstein领航速度增量前馈"
```
