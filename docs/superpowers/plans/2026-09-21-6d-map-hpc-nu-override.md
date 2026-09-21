# 6D Map HPC 齐次度 Override Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 6D Map HPC Artstein 增加受自动理论可行区间约束的固定 `nu` 实验 override。

**Architecture:** `lpc2hpc_nd()` 继续计算 `[nu_min, nu_max]`。默认使用 `nu_min`；override 启用时，只有有限且在该区间内的指定值才能构建 `Gd` 和控制律。每次初始化及离散目标切换均重新校验。

**Tech Stack:** ROS 2 Humble、rclcpp、Eigen、ament_cmake、C++17、Python launch/YAML。

## Global Constraints

- 默认行为必须与当前自动 `nu_min` 路径兼容。
- 越界或非有限 override 抛出错误，不静默回退。
- 不修改 4D 或其他 6D 控制器；不覆盖用户未提交的 YAML、记录和报告。

---

### Task 1: 控制器 API、可行性校验与单元测试

**Files:**
- Modify: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/homo_controller_6d_map_hpc_artstein.hpp`
- Modify: `homo_multirobot_formation_control/test/test_6d_map_hpc_artstein_controller.cpp`

**Interfaces:**
- Produces: 构造参数 `bool use_nu_override, double nu_override`；只读 `nu()`, `nu_min()`, `nu_max()`。

- [ ] 写失败测试：默认实例断言 `nu()==nu_min()`；选择自动区间内的值构建 override 实例并断言 `nu()==override`；非有限、低于 `nu_min`、高于 `nu_max` 的值在 `initialize()` 抛出 `std::runtime_error`。
- [ ] 运行 `test_6d_map_hpc_artstein_controller`，确认因新接口不存在而编译失败。
- [ ] 实现：保存 `res.nu_min/res.nu_max`；选择自动或 override `nu`；以 `1e-12` 容差验证有限性和闭区间；以选定 `nu` 构造 `Gd`。
- [ ] 重建并运行该测试，确认通过。
- [ ] 提交控制器和测试，提交信息：`增加6D齐次度实验覆盖`。

### Task 2: 节点、配置、launch 与说明

**Files:**
- Modify: `homo_multirobot_formation_control/src/formation_control_node_6d_map_hpc_artstein.cpp`
- Modify: `homo_multirobot_formation_control/config/formation_single_follower_6d_map_hpc_artstein.yaml`
- Modify: `homo_multirobot_formation_control/launch/formation_single_follower_6d_map_hpc_artstein.launch.py`
- Modify: `homo_multirobot_formation_control/README.md`

**Interfaces:**
- Consumes: Task 1 的构造参数和 `nu()`, `nu_min()`, `nu_max()`。
- Produces: ROS 参数 `use_hpc_nu_override`、`hpc_nu_override`；初始化/切换日志。

- [ ] 节点声明两个参数，传给控制器；首次初始化和目标切换成功后记录 `nu_mode`, `nu_used`, `feasible=[nu_min,nu_max]`。
- [ ] YAML 默认 `false` 和 `-0.30`，launch 创建并透传对应参数；以最小 patch 合并用户现有 YAML 改动。
- [ ] README 说明默认自动、固定 override、可行区间和越界错误。
- [ ] 构建包并运行 launch `--show-args`，确认新增参数可见。
- [ ] 提交节点、配置、launch 和说明，提交信息：`开放6D齐次度高级实验参数`。

### Task 3: 完整回归

**Files:**
- Test: `homo_multirobot_formation_control/test/test_6d_map_hpc_artstein_controller.cpp`

- [ ] 以 `BUILD_TESTING=ON` 构建包。
- [ ] 执行 `ctest --test-dir build/homo_multirobot_formation_control --output-on-failure`，确认全部测试通过。
