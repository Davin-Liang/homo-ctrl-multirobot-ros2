# 动捕直连状态接入实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不改变现有仿真/EKF 模式的前提下，增加纯动捕启动模式；控制主机可为 Follower 电脑或笔记本，原始 4D 与 4D Artstein 直接使用统一 map 系的位姿与线速度。

**Architecture:** `vrpn_listener` 只提供原始 VRPN 数据。控制主机（Follower 或笔记本）同时运行 bridge、两个 `mocap_state_adapter` 和选定的 4D 控制器，以本机得到两台车的同源动捕状态。适配器完成坐标轴、刚体外参、时间戳、角速度和失效检测，并发布明确的 map 系 `PoseStamped` 与 `TwistStamped`。原始 4D 和 4D Artstein 的 `state_source=mocap` 分支直接读取这两组状态，不查询 `map → robotN_odom`、不将速度再次旋转；默认 `state_source=ekf_tf` 保持当前行为。所有 6D 代码保持不变。

**Tech Stack:** ROS 2 Humble, C++17, rclcpp, tf2_ros, geometry_msgs, nav_msgs, VRPN.

## 全局约束

- 动捕模式和现有 rf2o/EKF/AMCL/slam 模式必须通过不同 launch 启动，不能同时运行。
- `map` 表示动捕校准后的全局世界系；两台机器人必须共享同一个 `map`。
- 已验证 VRPN `linear.x/y` 是动捕全局坐标系速度；4D mocap 分支将其直接作为 `vx_map/vy_map`。
- 动捕频率约为 `120 Hz`；`state_timeout` 默认设为 `0.10 s`，即约连续 12 帧未收到 pose 才判定状态失效。
- 不把 map 系速度伪装为标准 `Odometry.twist` 的 body 系速度。
- 动捕链路与控制器集中运行在同一控制主机，控制主机可为 Follower 电脑或笔记本；Leader 实车不再通过 DDS 向控制主机转发定位状态。
- 若控制主机为笔记本，`/robot2/cmd_vel` 将跨 DDS/Wi-Fi 发送给 Follower；若控制主机为 Follower，该控制命令为本机话题。两种部署均使用同一 launch 接口并记录端到端延迟。
- 任何一台刚体状态超时，Follower 必须发布零 `cmd_vel` 并暂停控制。
- 在当前源码仓库执行修改；colcon 只从 `/home/l1anggmgo/ros-projects/homo_multirobot_ws` 运行。

---

### Task 1: 修正并稳定原始 VRPN bridge

**Files:**
- Modify: `third_party/vrpn_client_ros2/src/vrpn_listener/src/vrpn_listener.cpp`
- Modify: `third_party/vrpn_client_ros2/src/vrpn_listener/config/params.yaml`

**Interfaces:**
- Produces: `/vrpn/<rigid>/pose`、`/twist`、`/accel` 都带 VRPN 原始时间戳；pose/twist 的 `header.frame_id` 为参数 `frame_id`。

- [ ] 将 bridge 默认 `frame_id` 改为 `map`，但不写入实验室服务器 IP、端口或刚体名。
- [ ] 在 twist 和 accel 回调中填充 `header.stamp`、`header.frame_id`；将 `vel_quat` 先转为平面增量 yaw，再除以 `vel_quat_dt` 生成 `angular.z`。`vel_quat_dt <= 0` 时发布零角速度并输出节流告警。
- [ ] 将每帧 INFO 日志降为 DEBUG，避免 100 Hz 动捕导致终端与日志文件膨胀。
- [ ] 用 `homo_multirobot_mocap_tools/vrpn_test_server` 验证 `robot1` 的三类原始话题及 twist 时间戳；保留针对实物动捕的 axis/sign 记录。

### Task 2: 增加每机器人动捕状态适配器

**Files:**
- Create: `homo_multirobot_localization/src/mocap_state_adapter.cpp`
- Create: `homo_multirobot_localization/include/homo_multirobot_localization/mocap_state_adapter.hpp`
- Create: `homo_multirobot_localization/test/test_mocap_state_adapter.cpp`
- Create: `homo_multirobot_localization/config/mocap_robot1.yaml`
- Create: `homo_multirobot_localization/config/mocap_robot2.yaml`
- Modify: `homo_multirobot_localization/CMakeLists.txt`
- Modify: `homo_multirobot_localization/package.xml`

**Interfaces:**
- Consumes: configurable raw VRPN `PoseStamped` 和 `TwistStamped`。
- Produces: `/<robot_ns>/mocap/pose` (`PoseStamped`, `frame_id=map`) 和 `/<robot_ns>/mocap/twist` (`TwistStamped`, `frame_id=map`)，以及 TF。

- [ ] 以 C++17 / rclcpp 实现单实例适配器，参数包含 `input_pose_topic`、`input_twist_topic`、`output_pose_topic`、`output_twist_topic`、`base_frame`、`odom_frame`、`state_timeout`（默认 `0.10 s`）、二维 world 对齐变换与 rigid-body→base 外参。
- [ ] 对 pose 应用固定的 world 轴/原点变换，再应用每台车刚体到 `robotN_base_footprint` 的外参；输出必须为米、右手 ROS `map` 坐标。
- [ ] 对线速度应用同一个 world 轴旋转；4D 模式保留其 map 系语义。姿态角速度由连续 pose 的 unwrap-yaw 差分和低通滤波获得，避免依赖未验证的服务端角速度。
- [ ] 发布 TF：静态 `map → robotN_odom` identity 与动态 `robotN_odom → robotN_base_footprint`。动捕模式中不启动任何 EKF/rf2o/AMCL TF 发布者。
- [ ] 增加状态有效性：未收到 pose、pose/twist 时间戳超时、非有限数值时标记无效；发布诊断并停止更新有效状态。
- [ ] 添加单元测试覆盖二维旋转/平移、外参组合、map→body 速度转换、yaw 跨 `±pi` 的差分和超时判定。

### Task 3: 为原始 4D 与 4D Artstein 增加纯动捕直接状态分支

**Files:**
- Modify: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node.hpp`
- Modify: `homo_multirobot_formation_control/src/formation_control_node.cpp`
- Modify: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_4d_artstein.hpp`
- Modify: `homo_multirobot_formation_control/src/formation_control_node_4d_artstein.cpp`
- Modify: `homo_multirobot_formation_control/config/formation_single_follower.yaml`
- Modify: `homo_multirobot_formation_control/config/formation_single_follower_4d_artstein.yaml`
- Modify: `homo_multirobot_formation_control/launch/formation_single_follower.launch.py`
- Modify: `homo_multirobot_formation_control/launch/formation_single_follower_4d_artstein.launch.py`
- Create: `homo_multirobot_formation_control/test/test_4d_mocap_state.cpp`

**Interfaces:**
- Consumes in `mocap` mode: `/<leader_ns>/mocap/pose`、`/<leader_ns>/mocap/twist`、`/<follower_ns>/mocap/pose`、`/<follower_ns>/mocap/twist`。
- Produces: existing `/<follower_ns>/cmd_vel` without changed 4D controller gains or Artstein history semantics.

- [ ] 为原始 4D 与 4D Artstein 都新增 `state_source`，合法值为 `ekf_tf` 和 `mocap`，默认 `ekf_tf`。
- [ ] 保留两节点在 `ekf_tf` 下的当前 Odometry 订阅和 `map → robotN_odom` 查 TF 代码，作为完全不变的默认路径。
- [ ] 为两节点新增 `mocap` 订阅和独立缓存；直接构造 `[x_map, y_map, vx_map, vy_map]`。不得调用原有 `ekf_to_map()`，不得对 mocap `vx_map/vy_map` 再旋转。
- [ ] 为两节点的 mocap 状态加入双车新鲜度门限；任一状态失效时清空控制器初始化状态、停止历史推进，并发布零速度命令。
- [ ] 编写测试：map pose/twist 直接生成预期 4D 状态；缺少 TF 不影响 mocap 分支；过期 leader 或 follower 产生零命令；默认 `ekf_tf` 行为保持不变。

### Task 4: 创建控制主机无关的互斥双车动捕 launch

**Files:**
- Create: `homo_multirobot_localization/launch/mocap_two_robots.launch.py`
- Create: `homo_multirobot_formation_control/launch/formation_single_follower_mocap.launch.py`
- Create: `homo_multirobot_formation_control/launch/formation_single_follower_4d_artstein_mocap.launch.py`
- Modify: `homo_multirobot_localization/README.md`
- Modify: `homo_multirobot_formation_control/README.md`

**Interfaces:**
- `mocap_two_robots.launch.py` 接受 `server`、`port`、`robot1_rigid_name`、`robot2_rigid_name`、`state_timeout` 和每台车配置文件，并在任意控制主机运行。
- `formation_single_follower_mocap.launch.py` 和 `formation_single_follower_4d_artstein_mocap.launch.py` 分别固定向原始 4D、4D Artstein 传入 `state_source=mocap`，并将控制输出显式 remap 到绝对话题 `/robot2/cmd_vel`；其余参数沿用各自现有 launch 参数。

- [ ] 在控制主机启动一个 `vrpn_listener`，使用 launch 参数指定真实服务器但不提交实验室 IP；启动两个 namespace 化适配器。
- [ ] 动捕控制 launch 只启动控制器，并始终将控制输出显式 remap 到 `/robot2/cmd_vel`。控制器运行在 Follower 电脑时该话题由本机 DDS 交付；运行在笔记本时同一绝对话题由 DDS/Wi-Fi 交付到 Follower。两者都明确不包含 Gazebo、rf2o、EKF、AMCL 或 slam_toolbox。
- [ ] 文档提供两台终端的最小验证顺序、话题/TF 检查和“动捕丢失时 cmd_vel 为零”的检查。

### Task 5: 分阶段验收与 6D 后续接入

**Files:**
- Create: `docs/mocap/2026-09-04-mocap-calibration-and-validation.md`
- Modify: `homo_multirobot_formation_control/README.md`

- [ ] 单车静止：确认 map 位置、yaw、速度接近零、TF 无重复发布。
- [ ] 单车直线及固定 `+90°` 朝向运动：确认 map `vx/vy` 的轴向、刚体外参和 cmd_vel 方向。
- [ ] 双车静止与同步移动：确认两车位置处于同一 map 系，算法接收数据但尚不使能驱动。
- [ ] 低速 4D 编队闭环：先限制 `max_linear_vel` 与 `max_angular_vel`，记录 pose、twist、cmd_vel 与状态年龄。
- [ ] 分别在 Follower 和笔记本运行控制主机 launch，记录动捕状态到达年龄、控制频率、`cmd_vel` 到 Follower 的延迟和轨迹误差；使用相同轨迹、控制参数和网络条件比较两种部署。
- [ ] 遮挡测试：停止任意刚体数据，验证在 `state_timeout` 内输出零 cmd_vel，并需要新鲜双车数据后再重新初始化。
- [ ] 本计划不修改任何 6D 节点、模型、launch 或配置；6D 动捕接入将在独立计划中处理，并在状态入口将 map 速度转换为其所需的 body 速度。
