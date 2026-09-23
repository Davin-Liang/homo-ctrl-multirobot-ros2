# 6D Map HPC 无补偿基线 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新增独立的 6D map-frame HPC ROS 2 节点，以当前测量状态直接闭环，供实物无 Artstein/无前向预测对照实验。

**Architecture:** 新节点仅保留 Artstein 节点的状态采集、map 变换、HPC 核心和运动学约束路径。它直接用 `leader.x`、`follower.x` 调用既有 `MapHpcController6DArtstein`，不保留预测器、延迟历史或 Leader 命令前馈。独立 YAML 与 launch 暴露共同控制参数和可选仿真延迟注入。

**Tech Stack:** ROS 2 Humble、rclcpp、Eigen、tf2、ament_cmake、Python unittest。

## Global Constraints

- 可执行文件必须名为 `formation_control_node_6d_map_hpc`。
- 原 Artstein 节点、YAML 和 launch 不改变行为。
- 基线节点不得声明或使用 `tau`、`tau_yaw`、`Td`、`enable_leader_cmd_feedforward`、`leader_cmd_timeout`、`leader_cmd_delta_lpf_tau`。
- 每周期以当前 `leader.x`、`follower.x` 调用 `select_target`、`initialize` 和 `command`。
- `use_motor_delay` 仍仅是 launch 级仿真扰动。

---

### Task 1: 基线入口回归测试

**Files:** Create `homo_multirobot_formation_control/test/test_6d_map_hpc_baseline.py`; modify `homo_multirobot_formation_control/CMakeLists.txt`.

**Interfaces:** 新 CTest 名为 `test_6d_map_hpc_baseline`；测试读取新 YAML、launch 和节点源码。

- [ ] **Step 1: 写失败测试。** 使用 `unittest` 和 `yaml.safe_load`：读取 `formation_single_follower_6d_map_hpc.yaml`，逐项断言上述六个参数不存在；读取 launch，断言不存在对应 `LaunchConfiguration`；读取 `formation_control_node_6d_map_hpc.cpp`，断言包含 `ctrl_->command(leader.x, follower.x)`，不含 `ArtsteinPredictorNd` 和 `predict_follower_state`。
- [ ] **Step 2: 验证 RED。** 运行 `python3 homo_multirobot_formation_control/test/test_6d_map_hpc_baseline.py`；预期因新增 YAML、launch、源码不存在而失败。
- [ ] **Step 3: 注册测试。** 在 `if(BUILD_TESTING)` 添加 `add_test(NAME test_6d_map_hpc_baseline COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_6d_map_hpc_baseline.py)`，然后提交测试：`git add homo_multirobot_formation_control/test/test_6d_map_hpc_baseline.py homo_multirobot_formation_control/CMakeLists.txt && git commit -m '添加6D Map HPC基线回归测试'`。

### Task 2: 无补偿 ROS 2 控制节点

**Files:** Create `include/homo_multirobot_formation_control/formation_control_node_6d_map_hpc.hpp`, `src/formation_control_node_6d_map_hpc.cpp`, `src/main_6d_map_hpc.cpp`; modify `CMakeLists.txt` under `homo_multirobot_formation_control`.

**Interfaces:** 产生 `FormationController6DMapHpc` 和 `formation_control_node_6d_map_hpc`；仅接受共同状态源、HPC、轮速和命令限幅参数。

- [ ] **Step 1: 创建最小头文件。** 保留 `State6D`、`timer_cb`、`odom_to_state`、坐标转换、HPC/约束/TF、EKF 与 mocap 订阅、命令发布与诊断字段；不得包含 `<deque>`、`ArtsteinPredictorNd` 或 `leader_command_delta_feedforward.hpp`。
- [ ] **Step 2: 直接测量闭环。** 复制并保留速度限幅、`min_cmd_vel`、`KinematicConstraint`、mocap 超时急停和状态新鲜度诊断；控制核心依次执行 `ctrl_->select_target(leader.x, follower.x)`，首次/切换时 `ctrl_->initialize(leader.x, follower.x[, true])`，然后 `ctrl_->command(leader.x, follower.x)`，并将 map 平移输出用当前 follower yaw 转回 body。日志标签为 `6D Map HPC`，没有预测偏移或预测 yaw。入口构造节点并 `rclcpp::spin`。
- [ ] **Step 3: 注册目标。** 以 Artstein 目标相同的 include 目录、`rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs` 依赖、`Eigen3::Eigen` 链接与 install 规则创建新 CMake target。
- [ ] **Step 4: 验证 GREEN。** 运行 `python3 homo_multirobot_formation_control/test/test_6d_map_hpc_baseline.py`；预期 PASS。接着从 `/home/l1anggmgo/ros-projects/homo_multirobot_ws` 运行 `source /opt/ros/humble/setup.bash && colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=ON`；预期 exit 0 并生成基线可执行文件。
- [ ] **Step 5: 提交节点。** `git add homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_6d_map_hpc.hpp homo_multirobot_formation_control/src/formation_control_node_6d_map_hpc.cpp homo_multirobot_formation_control/src/main_6d_map_hpc.cpp homo_multirobot_formation_control/CMakeLists.txt && git commit -m '新增6D Map HPC无补偿控制节点'`。

### Task 3: 配置、启动入口和说明

**Files:** Create `config/formation_single_follower_6d_map_hpc.yaml`, `launch/formation_single_follower_6d_map_hpc.launch.py`; modify `README.md` under `homo_multirobot_formation_control`.

**Interfaces:** 提供 `ros2 launch homo_multirobot_formation_control formation_single_follower_6d_map_hpc.launch.py`。

- [ ] **Step 1: 创建 YAML 和 launch。** YAML 复制共同状态源、HPC、轮子/速度/加速度和仿真延迟参数，删除六个补偿/前馈参数，并保持 `control_rate: 20.0`、`max_linear_vel: 0.45`、`max_angular_vel: 0.8`。Launch 复制 `cmd_vel_raw`/`cmd_vel` 延迟 remap，删除六个参数的 `LaunchConfiguration`、Node 参数和 `DeclareLaunchArgument`；使用 executable/name `formation_control_node_6d_map_hpc`。
- [ ] **Step 2: 补充 README。** 控制器表新增 `6D Map HPC（无补偿）`；在 Artstein 启动说明之前增加 `ros2 launch homo_multirobot_formation_control formation_single_follower_6d_map_hpc.launch.py leader_ns:=/robot1 follower_ns:=/robot2 use_sim_time:=false`。说明它直接用当前测量状态，不能传 `tau`、`tau_yaw`、`Td`，应在相同定位、速度和限幅条件下和 Artstein 对照。
- [ ] **Step 3: 验证 CTest 与安装入口。** 从 workspace 根执行 `source /opt/ros/humble/setup.bash && source install/setup.bash && ctest --test-dir build/homo_multirobot_formation_control --output-on-failure -R 'test_6d_map_hpc_baseline|test_6d_map_hpc_artstein_controller'`，预期两项通过；再执行 `ros2 pkg executables homo_multirobot_formation_control | rg 'formation_control_node_6d_map_hpc$' && ros2 launch homo_multirobot_formation_control formation_single_follower_6d_map_hpc.launch.py --show-args`，预期发现可执行文件且参数列表无补偿/前馈参数。
- [ ] **Step 4: 提交。** `git add homo_multirobot_formation_control/config/formation_single_follower_6d_map_hpc.yaml homo_multirobot_formation_control/launch/formation_single_follower_6d_map_hpc.launch.py homo_multirobot_formation_control/README.md && git commit -m '补充6D Map HPC无补偿启动配置'`。
