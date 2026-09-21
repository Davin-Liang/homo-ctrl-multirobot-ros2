# 双机 Launch 配置默认值 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让双机定位和双机 slam_toolbox 定位 launch 从 formation_control 配置目录读取默认值，同时保留命令行覆盖。

**Architecture:** 两个 launch 在 import 时通过 `get_package_share_directory("homo_multirobot_formation_control")` 找到各自 YAML 的 `launch_defaults` 映射。小型 `_launch_default()` 将 bool 转为 ROS launch 接受的小写字符串；每个 `DeclareLaunchArgument` 继续使用原名，仅将硬编码默认值换为 YAML 值。

**Tech Stack:** ROS 2 Humble launch、launch_ros、Python 3、PyYAML、ament index。

## Global Constraints

- 新 YAML 必须位于 `homo_multirobot_formation_control/config/`。
- 命令行 `name:=value` 覆盖 YAML 默认值。
- 不迁移 EKF/rf2o/slam_toolbox 节点算法配置。
- 不修改用户未提交的实验配置、报告和记录。

---

### Task 1: 双机定位 launch 默认值 YAML

**Files:**
- Create: `homo_multirobot_formation_control/config/sim_rf2o_ekf_two_robots.launch.yaml`
- Modify: `homo_multirobot_localization/launch/sim_rf2o_ekf_two_robots.launch.py`
- Test: `homo_multirobot_localization/test/test_sim_rf2o_ekf_two_robots_launch_defaults.py`

- [ ] 写失败测试：加载 launch 模块，断言 YAML 中的 `robot2_x` 成为 `DeclareLaunchArgument` 默认值；使用 `robot2_x:=2.0` 时该值可被 launch 覆盖。
- [ ] 运行测试，确认 YAML/读取接口不存在导致失败。
- [ ] 创建包含所有当前 launch 参数的 `launch_defaults` YAML；launch 使用 formation_control share 路径加载 YAML，并替换每项硬编码 default。
- [ ] 重跑测试，并运行 `ros2 launch homo_multirobot_localization sim_rf2o_ekf_two_robots.launch.py --show-args`。

### Task 2: 双机 slam_toolbox 定位 launch 默认值 YAML

**Files:**
- Create: `homo_multirobot_formation_control/config/slam_toolbox_loc_two_robots.launch.yaml`
- Modify: `homo_multirobot_nav/launch/slam_toolbox_loc_two_robots.launch.py`
- Test: `homo_multirobot_nav/test/test_slam_toolbox_loc_two_robots_launch_defaults.py`

- [ ] 写失败测试：断言 `map_name`、`robot1_map_start_x` 等默认值来自新的 YAML，并检查命令行覆盖语义。
- [ ] 运行测试确认失败。
- [ ] 创建 YAML 并以同一 helper 读取；保留 `PythonExpression` 形式的 TF frame 默认值，YAML 只覆盖用户可配置 launch 参数。
- [ ] 重跑测试，并运行 `ros2 launch homo_multirobot_nav slam_toolbox_loc_two_robots.launch.py --show-args`。

### Task 3: 文档、构建与提交

**Files:**
- Modify: `homo_multirobot_localization/README.md`
- Modify: `homo_multirobot_nav/README.md`

- [ ] 在两个 README 给出 YAML 默认启动和单参数命令行覆盖示例。
- [ ] 从 workspace 根目录构建 affected packages，并运行新增 launch 默认值测试。
- [ ] 提交 YAML、launch、测试和说明，提交信息：`支持双机启动配置覆盖`。
