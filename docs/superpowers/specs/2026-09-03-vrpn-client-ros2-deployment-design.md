# VRPN Client ROS 2 最小部署设计

## 目标

在 ROS 2 Humble 工作空间中部署 `efc-robot/vrpn_client_ros2`，将已存在的 VRPN Tracker 服务转换为 ROS 2 位姿话题，并完成本机可执行、可启动的验证。

## 范围

本阶段仅部署与验证桥接器：

- 将上游源码放在当前仓库的 `third_party/vrpn_client_ros2/`，并纳入本仓库 Git 管理。
- 使用系统的 VRPN 开发依赖和 ROS 2 Humble 环境构建桥接器。
- 启动 `vrpn_listener`，验证 ROS 2 图中出现其发布的刚体 pose 话题。

本阶段明确不包含：

- 修改 `homo_multirobot_*` 任何功能包。
- 将动捕位姿转换、重映射或融合为 `/<robot_ns>/odometry/filtered`。
- 发布或变更 TF；特别是不改变现有 EKF 对 `robot*_odom -> robot*_base_footprint` 的唯一发布权。
- 写入 VRPN 服务器 IP、端口或刚体名等实验室专属配置。

## 架构与数据流

运行在机器人计算机上的 `vrpn_listener` 作为 VRPN 客户端连接动捕服务器。它按 Tracker 名称动态发现刚体，并为每个刚体发布 ROS 2 位姿话题。此阶段 pose 话题仅供 `ros2 topic list`、`ros2 topic echo` 与 RViz 等诊断使用，不连接项目的控制或定位链路。

```text
VRPN server (IP/port supplied at launch)
  -> vrpn_listener
  -> /vrpn/<tracker_name>/pose
```

## 部署策略

选择上游 `efc-robot/vrpn_client_ros2`，与用户提供的参考文章保持一致。将其完整源码（不使用 Git submodule）导入当前仓库的 `third_party/vrpn_client_ros2/`；这样本仓库的提交可精确固定桥接器版本，并允许后续为 Humble 做本地补丁。源码仍由 colcon 工作空间根目录构建：

```bash
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select vrpn_listener --symlink-install
source install/setup.bash
```

依赖安装使用 ROS 2 Humble 提供的 `ros-humble-vrpn`，其中包含 VRPN 头文件、库和 CMake 配置。Ubuntu 22.04 的当前软件源不提供 `libvrpn-dev`，因此不使用该包名。不会在当前源码仓库运行 `colcon build`，避免生成错误的 build/install/log 目录。导入完成后，`third_party/vrpn_client_ros2/` 的完整源码与本次必要的 Humble 兼容补丁一并提交；不保留独立上游 Git 元数据。

## 配置与运行

初次验证使用 `localhost` 默认参数，运行桥接器确认二进制与 ROS 图正常。连通真实服务器时，在桥接器的参数 YAML 中提供服务器 IP 和端口；固定参考坐标系设为 `world`。刚体名将在启动后的 pose 话题路径中由 VRPN 服务端决定。

## 成功标准与失败处理

- `ros2 pkg executables vrpn_listener` 显示可执行程序。
- 启动 `vrpn_listener` 不出现动态库或参数文件错误。
- 在已连通 VRPN 服务端且服务端已发布刚体时，`ros2 topic list` 出现至少一个 VRPN pose 话题，且 `ros2 topic echo <topic> --once` 获得带时间戳与姿态的消息。
- 若构建失败，保留原始构建日志，先确认 `rosdep` 输出、VRPN 开发包和 ROS 2 Humble API 兼容性；不修改现有机器人包来规避失败。

## 后续阶段

在获得服务器 IP、端口、每台机器人的刚体名、动捕坐标轴定义与机器人上刚体安装外参后，另行设计动捕 pose 到多机器人 `Odometry` / EKF 的融合与 TF 所有权方案。
