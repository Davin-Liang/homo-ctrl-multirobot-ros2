# 4D Artstein 齐次度 Override Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 4D Artstein 提供受自动 `[nu_min, nu_max]` 约束的固定 `nu` 实验 override。

**Architecture:** 在 4D `LpcController` 保存自动可行区间并选择自动或指定 `nu`；`LpcController4DArtstein` 只透传构造参数和诊断接口，节点负责 ROS 参数、日志和 launch/YAML 透传。初始化与编队点切换均使用相同校验路径。

**Tech Stack:** ROS 2 Humble、rclcpp、Eigen、ament_cmake、C++17、Python launch/YAML。

## Global Constraints

- 默认自动 `nu_min` 行为不变。
- override 非有限或越界时抛出错误，不回退。
- 不修改用户未提交的 YAML、记录和报告内容。

---

### Task 1: 4D HPC 校验与单元测试

**Files:**
- Modify: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/homo_controller.hpp`
- Modify: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/homo_controller_4d_artstein.hpp`
- Modify: `homo_multirobot_formation_control/test/test_homo_controller_4d_artstein.cpp`

- [ ] 写失败测试：默认实例 `nu()==nu_min()`；自动区间中点 override 被采用；越界和 NaN 在 `controller_initial()` 抛出。
- [ ] 构建 `test_homo_controller_4d_artstein`，确认新接口缺失导致红灯。
- [ ] 在每次 `lpc2hpc()` 重建后保存区间，选择并验证 `nu`，以其构建 `Gd`；Artstein wrapper 透传构造参数和 accessors。
- [ ] 重建并运行单测，确认通过。
- [ ] 提交控制器和测试，提交信息：`增加4D齐次度实验覆盖`。

### Task 2: ROS 参数、配置和说明

**Files:**
- Modify: `homo_multirobot_formation_control/src/formation_control_node_4d_artstein.cpp`
- Modify: `homo_multirobot_formation_control/config/formation_single_follower_4d_artstein.yaml`
- Modify: `homo_multirobot_formation_control/launch/formation_single_follower_4d_artstein.launch.py`
- Modify: `homo_multirobot_formation_control/README.md`

- [ ] 声明 `use_hpc_nu_override` 和 `hpc_nu_override`，构造 wrapper 后传入；初始化及切换重建路径输出模式、使用值和自动区间。
- [ ] YAML 和 launch 加同名默认参数及参数透传；只暂存本功能 hunk，保留用户 YAML 修改。
- [ ] README 说明 4D 的自动/override 行为与越界拒绝。
- [ ] 构建包并通过 launch `--show-args` 确认参数公开。
- [ ] 提交节点、launch、README 和可安全独立暂存的 YAML hunk，提交信息：`开放4D齐次度高级实验参数`。

### Task 3: 回归

**Files:**
- Test: `homo_multirobot_formation_control/test/test_homo_controller_4d_artstein.cpp`

- [ ] 以 `BUILD_TESTING=ON` 构建包。
- [ ] 执行 `ctest --test-dir build/homo_multirobot_formation_control --output-on-failure`；若用户记录 YAML 改动导致既有测试失败，报告该独立失败，并执行 4D 针对性测试。
