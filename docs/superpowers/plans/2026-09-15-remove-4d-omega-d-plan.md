# 移除 4D omega_d 参数 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 从原始 4D 与 4D Artstein 控制器删除无独立作用的 `omega_d` 参数。

**Architecture:** 共用 `LpcController` 直接接受两个 lambda 下界，默认值为 1.5 和 4.0。原始 4D 与 Artstein 的 ROS 接口均删除 `omega_d`；6D Map 保持不变，其他 6D 控制器继续保留其真实生效的 `omega_d`。

**Tech Stack:** C++17、ROS 2 Humble、Python launch、PyYAML、pytest、ament_cmake。

## Global Constraints

- 不修改 6D Map、6D Disc、6D OA 或其 `omega_d` 接口。
- 不改变已显式使用 `initial_min_lambda=1.5`、`switch_min_lambda=4.0` 的 4D 默认控制行为。
- 记录器仅在控制器实际公开 `omega_d` 时才把 `od...` 写入自动目录标签。

---

### Task 1: 移除共用 4D 控制器和 Artstein wrapper 的 omega_d 依赖

**Files:**
- Modify: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/homo_controller.hpp:23-30,263-300`
- Modify: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/homo_controller_4d_artstein.hpp:26-35`
- Modify: `homo_multirobot_formation_control/test/test_lpc_controller_params.cpp:8-15`
- Modify: `homo_multirobot_formation_control/test/test_homo_controller_4d_artstein.cpp:14-20`

**Interfaces:**
- Produces: `LpcController(m_p, radius, tol, mass, use_hpc, hpc_c_min, control_period, initial_min_lambda, switch_min_lambda)`.
- Produces: `LpcController4DArtstein(m_p, radius, tol, mass, tau_nominal, use_hpc, control_period, hpc_c_min, Td, initial_min_lambda, switch_min_lambda)`.

- [ ] **Step 1: 改写 C++ 测试为新构造接口**

从两个测试的控制器构造调用删除原 `omega_d` 实参；保留 `initial_min_lambda=1.7` 与 `switch_min_lambda=3.4` 的断言。

- [ ] **Step 2: 构建并确认因旧签名失败**

Run: `cd ../.. && source /opt/ros/humble/setup.bash && colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=ON`

Expected: C++ 编译失败，提示现有构造函数仍期望旧 `omega_d` 参数位置。

- [ ] **Step 3: 最小实现**

从 `LpcController` 删除 `omega_d` 构造参数和成员变量；将两个 lambda 默认值改为 `1.5`、`4.0`，不再用 `omega_d * mass` 兜底。相应删除 Artstein wrapper 的 `omega_d` 参数及向底层转发。更新两处注释，使其只描述 `min_lambda`。

- [ ] **Step 4: 构建并运行 C++ 构造接口测试**

Run: `cd ../.. && source /opt/ros/humble/setup.bash && colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=ON && source install/setup.bash && ctest --test-dir build/homo_multirobot_formation_control -R 'test_lpc_controller_params|test_homo_controller_4d_artstein' --output-on-failure`

Expected: 两个测试 PASS。

### Task 2: 删除两条 4D ROS 接口和自动标签中的 omega_d

**Files:**
- Modify: `homo_multirobot_formation_control/src/formation_control_node.cpp:42-65`
- Modify: `homo_multirobot_formation_control/src/formation_control_node_4d_artstein.cpp:105-155`
- Modify: `homo_multirobot_formation_control/launch/formation_single_follower.launch.py:39-90,142-145`
- Modify: `homo_multirobot_formation_control/launch/formation_single_follower_4d_artstein.launch.py:41-99,168-169`
- Modify: `homo_multirobot_formation_control/config/formation_single_follower.yaml:17-31`
- Modify: `homo_multirobot_formation_control/config/formation_single_follower_4d_artstein.yaml:21-32`
- Modify: `homo_multirobot_formation_control/scripts/record_trajectory.py:363-375`
- Test: `homo_multirobot_formation_control/test/test_launch_yaml_defaults.py`
- Test: `homo_multirobot_formation_control/test/test_record_trajectory_yaml.py`

**Interfaces:**
- Produces: 原始 4D 与 4D Artstein 的 ROS 参数列表不包含 `omega_d`。
- Produces: 4D 控制器自动标签不包含 `odna`；仍含 `omega_d` 的其他控制器保留 `od...`。

- [ ] **Step 1: 写入失败测试**

添加 Python 测试，读取两个 4D YAML 和 launch 源码，断言 `omega_d` 不在 YAML 参数名、也不在两个 launch 文件。再构造只含 `mass`、`radius`、`control_rate` 的 `ctrl_params`，断言 `_build_auto_tag()` 结果不含 `od`。

- [ ] **Step 2: 运行测试并确认失败**

Run: `source /opt/ros/humble/setup.bash && python3 -m pytest -q homo_multirobot_formation_control/test/test_launch_yaml_defaults.py homo_multirobot_formation_control/test/test_record_trajectory_yaml.py`

Expected: FAIL，因为两条 4D 接口和自动标签仍出现 `omega_d`。

- [ ] **Step 3: 最小实现**

从两个 4D 节点、launch 和 YAML 删除 `omega_d`；节点直接声明 lambda 默认值 1.5 与 4.0。自动标签仅在 `omega_d in ctrl_params` 时添加 `od...`。

- [ ] **Step 4: 运行 Python 接口测试**

Run: `source /opt/ros/humble/setup.bash && python3 -m pytest -q homo_multirobot_formation_control/test/test_launch_yaml_defaults.py homo_multirobot_formation_control/test/test_record_trajectory_yaml.py`

Expected: PASS。

### Task 3: 更新 4D 文档并完整验证

**Files:**
- Modify: `homo_multirobot_formation_control/README.md:261-280,685-692`
- Modify: `homo_multirobot_formation_control/doc/chapter3_experiment_plot_plan.md:48-52,106-137`

**Interfaces:**
- Produces: 4D 说明只将两个 lambda 列为反馈力度参数；6D 文档中的 `omega_d` 保持。

- [ ] **Step 1: 删除 4D 文档和命令中的 omega_d**

从 4D 参数表、原始 4D 启动示例、轨迹自动标签说明和第 3 章 4D 对照参数清单中删除 `omega_d`。保留 README 6D Disc 示例和 6D 专用理论文档的同名参数。

- [ ] **Step 2: 搜索接口边界**

Run: `rg -n "omega_d" homo_multirobot_formation_control -g '*.cpp' -g '*.hpp' -g '*.py' -g '*.yaml' -g '*.md'`

Expected: 不出现原始 4D/4D Artstein 节点、launch、YAML 和 4D 文档中的 `omega_d`；只保留 6D 与 MATLAB 等不在本次范围的引用。

- [ ] **Step 3: 完整构建和回归测试**

Run: `cd ../.. && source /opt/ros/humble/setup.bash && colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=ON && source install/setup.bash && ctest --test-dir build/homo_multirobot_formation_control --output-on-failure`

Expected: 构建成功，注册测试全部 PASS。

- [ ] **Step 4: 提交实现**

Run: `git add docs/superpowers/plans/2026-09-15-remove-4d-omega-d-plan.md homo_multirobot_formation_control && git commit -m "移除4D控制器无效omega参数"`
