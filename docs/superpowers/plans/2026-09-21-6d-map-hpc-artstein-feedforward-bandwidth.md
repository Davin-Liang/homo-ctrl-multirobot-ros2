# 6D Map HPC Artstein 前馈与切换带宽 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 6D Map HPC Artstein 增加 Leader 命令增量前馈和目标切换后的独立 HPC 带宽。

**Architecture:** 节点复用既有 `LeaderCommandDeltaFeedforward`，将一次性 Leader 线速度增量叠加在 map 系 HPC 平移输出、且在运动学约束之前。控制器保存初始和切换两个最小极点尺度；每次重建都以本轮选定的尺度同步计算 `K/P/Gd/nu`。

**Tech Stack:** ROS 2 Humble、rclcpp、Eigen、ament_cmake、C++17、Python launch/YAML。

## Global Constraints

- 仅修改 6D Map HPC Artstein 的实现、配置、测试和说明。
- 复用 `LeaderCommandDeltaFeedforward`，不复制其滤波、超时或单次消费逻辑。
- 不叠加 Leader `angular.z`；yaw 仅由 6D Artstein/HPC 通道产生。
- 只暂存本计划涉及的文件，保留用户报告改动。

---

### Task 1: 切换带宽控制器接口与单元测试

**Files:**
- Modify: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/homo_controller_6d_map_hpc_artstein.hpp`
- Modify: `homo_multirobot_formation_control/test/test_6d_map_hpc_artstein_controller.cpp`

**Interfaces:**
- Produces: 构造函数的 `switch_min_lambda`，`initialize(leader, follower, bool use_switch_bandwidth = false)`，以及 `double min_lambda() const`。

- [ ] **Step 1: 写入失败测试**

```cpp
MapHpcController6DArtstein controller(4, 1.0, 0.0, 2.0, 1.0,
    0.5, true, 0.05, 1.0, 4.0);
controller.initialize(leader, follower, false);
assert(std::abs(controller.min_lambda() - 1.0) < 1e-12);
controller.initialize(leader, follower, true);
assert(std::abs(controller.min_lambda() - 4.0) < 1e-12);
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd ../.. && source /opt/ros/humble/setup.bash && colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=ON --cmake-target test_6d_map_hpc_artstein_controller`

Expected: 编译失败，原因是新构造函数、初始化参数和 accessor 尚不存在。

- [ ] **Step 3: 实现最小接口**

```cpp
void initialize(const Eigen::VectorXd& leader, const Eigen::VectorXd& follower,
                bool use_switch_bandwidth = false) {
  min_lambda_ = use_switch_bandwidth ? switch_min_lambda_ : initial_min_lambda_;
  k_ = calculate_klin(error_of(leader, follower));
  auto res = lpc2hpc_nd(a_, b_, k_);
  // 保留已有 P/Gd/nu 同步赋值和失败检查
}
```

- [ ] **Step 4: 运行测试确认通过**

Run: 与 Step 2 相同，并执行 `./build/homo_multirobot_formation_control/test_6d_map_hpc_artstein_controller`。

Expected: 测试退出码为 0。

- [ ] **Step 5: 提交**

```bash
git add homo_multirobot_formation_control/include/homo_multirobot_formation_control/homo_controller_6d_map_hpc_artstein.hpp homo_multirobot_formation_control/test/test_6d_map_hpc_artstein_controller.cpp
git commit -m "增加6D目标切换独立带宽"
```

### Task 2: 节点前馈和参数透传

**Files:**
- Modify: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_6d_map_hpc_artstein.hpp`
- Modify: `homo_multirobot_formation_control/src/formation_control_node_6d_map_hpc_artstein.cpp`
- Modify: `homo_multirobot_formation_control/config/formation_single_follower_6d_map_hpc_artstein.yaml`
- Modify: `homo_multirobot_formation_control/launch/formation_single_follower_6d_map_hpc_artstein.launch.py`

**Interfaces:**
- Consumes: Task 1 的 `initialize(..., bool)`；`LeaderCommandDeltaFeedforward`。
- Produces: `switch_min_lambda`、`enable_leader_cmd_feedforward`、`leader_cmd_timeout`、`leader_cmd_delta_lpf_tau` 四项运行参数。

- [ ] **Step 1: 写入失败的节点编译验证**

在节点构造中传入 `switch_min_lambda` 并引用 `LeaderCommandDeltaFeedforward`；尚未增加头文件与成员，从而使构建因缺少接口失败。

- [ ] **Step 2: 构建确认失败**

Run: `cd ../.. && source /opt/ros/humble/setup.bash && colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=OFF`

Expected: 编译失败，指出缺少前馈头文件、成员或 Task 1 接口。

- [ ] **Step 3: 最小节点实现和配置**

```cpp
enable_leader_cmd_feedforward_ =
    declare_parameter("enable_leader_cmd_feedforward", false);
leader_cmd_timeout_ = declare_parameter("leader_cmd_timeout", 0.15);
leader_cmd_delta_lpf_tau_ =
    declare_parameter("leader_cmd_delta_lpf_tau", 0.10);
// 启用时订阅 leader_ns_ + "/cmd_vel"；新消息以当前 leader yaw 转为 map 系后 ingest。
// map_out.head<2>() += leader_cmd_feedforward_->consume(now, leader_cmd_timeout_);
// 首次 initialize(..., false)，目标切换 initialize(..., true)。
```

YAML 设置上述默认值；launch 为四项参数创建 `LaunchConfiguration`、节点参数项和 `DeclareLaunchArgument`。

- [ ] **Step 4: 构建确认通过**

Run: 与 Step 2 相同。

Expected: 包成功构建。

- [ ] **Step 5: 提交**

```bash
git add homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_6d_map_hpc_artstein.hpp homo_multirobot_formation_control/src/formation_control_node_6d_map_hpc_artstein.cpp homo_multirobot_formation_control/config/formation_single_follower_6d_map_hpc_artstein.yaml homo_multirobot_formation_control/launch/formation_single_follower_6d_map_hpc_artstein.launch.py
git commit -m "增加6D领航增量前馈"
```

### Task 3: 文档和完整回归

**Files:**
- Modify: `homo_multirobot_formation_control/README.md`
- Test: `homo_multirobot_formation_control/test/test_6d_map_hpc_artstein_controller.cpp`
- Test: `homo_multirobot_formation_control/test/test_leader_command_delta_feedforward.cpp`

- [ ] **Step 1: 更新 README**

在 6D Map HPC Artstein 段落说明初始/切换分别使用 `initial_min_lambda`/`switch_min_lambda`；Leader 前馈仅处理线速度变化、默认关闭且不改变 yaw 通道。

- [ ] **Step 2: 运行针对性回归**

Run: `cd ../.. && source /opt/ros/humble/setup.bash && colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=ON && source install/setup.bash && ctest --test-dir build/homo_multirobot_formation_control --output-on-failure -R 'test_(6d_map_hpc_artstein_controller|leader_command_delta_feedforward)'`

Expected: 两个测试通过。

- [ ] **Step 3: 检查 launch 参数完整性**

Run: `source ../../install/setup.bash && ros2 launch homo_multirobot_formation_control formation_single_follower_6d_map_hpc_artstein.launch.py --show-args`

Expected: 输出四个新增参数及 YAML 默认值。

- [ ] **Step 4: 提交**

```bash
git add homo_multirobot_formation_control/README.md
git commit -m "补充6D前馈带宽使用说明"
```
