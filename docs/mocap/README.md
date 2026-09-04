# 动捕系统使用指南

本指南说明如何用 VRPN 动捕系统为两台机器人提供状态，并让原始 4D 或 4D Artstein 编队控制器直接使用 map 系位姿与速度。

## 运行架构

```text
动捕服务器
  -> VRPN
控制主机（Follower 电脑或笔记本）
  -> vrpn_listener
  -> robot1 / robot2 mocap_state_adapter
  -> /robot1/mocap/{pose,twist}
  -> /robot2/mocap/{pose,twist}
  -> 4D 编队控制器
  -> /robot2/cmd_vel
```

控制主机可以是 Follower 电脑，也可以是笔记本。若控制器在笔记本，`/robot2/cmd_vel` 经 DDS/Wi-Fi 发往 Follower；若控制器在 Follower，该话题在本机交付。

动捕模式与 rf2o、EKF、AMCL、slam_toolbox 模式互斥：不要同时启动它们，以免多源 TF 和状态竞争。

## 前提

- ROS 2 Humble 和 `ros-humble-vrpn` 已安装。
- 动捕软件已启用 VRPN Server，已创建两个 rigid body。
- 控制主机能访问动捕服务器 IP 与端口（通常为 TCP 3883）。
- 刚体名称建议为 `robot1`、`robot2`，不要包含空格或中文。

## 构建

从 colcon workspace 根目录构建：

```bash
cd ~/ros-projects/homo_multirobot_ws
source /opt/ros/humble/setup.bash

colcon build --packages-select \
  vrpn_listener homo_multirobot_localization homo_multirobot_formation_control \
  --symlink-install --cmake-args -DBUILD_TESTING=OFF

source install/setup.bash
```

若 workspace 中存在同名的旧资源副本，改为限定当前仓库路径：

```bash
colcon build --paths \
  src/homo-ctrl-multirobot-ros2/third_party/vrpn_client_ros2/src/vrpn_listener \
  src/homo-ctrl-multirobot-ros2/homo_multirobot_localization \
  src/homo-ctrl-multirobot-ros2/homo_multirobot_formation_control \
  --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

## 启动双车动捕状态链路

在控制主机运行：

```bash
source /opt/ros/humble/setup.bash
source ~/ros-projects/homo_multirobot_ws/install/setup.bash

ros2 launch homo_multirobot_localization mocap_two_robots.launch.py \
  server:=<VRPN服务器IP> \
  port:=3883 \
  robot1_rigid_name:=robot1 \
  robot2_rigid_name:=robot2
```

该 launch 启动一个 `vrpn_listener` 和两个 C++ `mocap_state_adapter`。它不会启动编队算法、Gazebo、EKF 或机器人驱动。

验证话题：

```bash
ros2 topic list | grep '/mocap/'
ros2 topic echo /robot1/mocap/pose --once
ros2 topic echo /robot1/mocap/twist --once
ros2 topic echo /robot2/mocap/pose --once
ros2 topic echo /robot2/mocap/twist --once
```

正常结果中，所有输出都应有：

```text
header.frame_id: map
```

## 坐标与外参标定

配置模板：

```text
homo_multirobot_localization/config/mocap_robot1.yaml
homo_multirobot_localization/config/mocap_robot2.yaml
```

默认值全部为零，仅适用于动捕 world 坐标、刚体坐标与机器人车体坐标完全对齐的情况。实际部署需标定：

| 参数 | 作用 |
|---|---|
| `world_x`, `world_y`, `world_yaw` | 动捕 world 到 ROS `map` 的固定平移与偏航对齐。 |
| `rigid_to_base_x`, `rigid_to_base_y`, `rigid_to_base_yaw` | 动捕刚体到机器人 `base_footprint` 的安装偏置。 |

位置的处理为：

```text
p_map = R(world_yaw) * p_vrpn + [world_x, world_y]
p_base = p_rigid + R(yaw_rigid) * [rigid_to_base_x, rigid_to_base_y]
```

线速度只应用 world 坐标轴旋转，不应用原点平移。已验证 VRPN 线速度为动捕全局坐标系速度。

建议将真实实验室参数放入本机私有 YAML 覆盖文件，不提交动捕服务器 IP 或真实刚体名称。

## TF 验证

动捕 adapter 发布：

```text
map -> robotN_odom                 静态 identity
robotN_odom -> robotN_base_footprint  动态动捕位姿
```

检查：

```bash
ros2 run tf2_ros tf2_echo map robot1_base_footprint
ros2 run tf2_ros tf2_echo map robot2_base_footprint
```

## 启动 4D 编队控制

先确认两台机器人的硬件驱动已运行且能够接收 `/robot2/cmd_vel`。不要同时启动 EKF/rf2o 定位。

原始 4D：

```bash
ros2 launch homo_multirobot_formation_control \
  formation_single_follower_mocap.launch.py
```

4D Artstein：

```bash
ros2 launch homo_multirobot_formation_control \
  formation_single_follower_4d_artstein_mocap.launch.py
```

两者都使用：

```text
state_source=mocap
use_sim_time=false
mocap_state_timeout=0.10
```

在 mocap 模式中，4D 状态直接为：

```text
[x_map, y_map, vx_map, vy_map]
```

不会查 `map -> robotN_odom` TF，也不会将动捕 map 系线速度再次旋转。

## 安全与故障处理

动捕频率约 120 Hz，默认超时为 `0.10 s`。若任一刚体超过约 12 帧没有新 pose：

```text
控制器发布 /robot2/cmd_vel = 0
控制器重置内部初始化，等待两台车重新收到新鲜数据
```

首次实车测试建议：

1. 先只启动动捕链路，确认位置、yaw、速度轴向。
2. 让车静止，确认 `vx/vy/wz` 接近零。
3. 固定机器人朝向后直线移动，确认 map 速度方向正确。
4. 先将控制器 `max_linear_vel` 限制到 `0.10 m/s`。
5. 保留独立急停手段，并先让车轮悬空或保持安全距离。
6. 测试遮挡一个刚体，确认 `cmd_vel` 在超时后归零。

## 本机测试服务端

没有真实动捕时可启动仓库内测试服务端：

```bash
ros2 run homo_multirobot_mocap_tools vrpn_test_server -- --rate 50
```

再将 `mocap_two_robots.launch.py` 的 `server` 设为 `localhost`，并暂时令两个刚体名都为 `robot1`，即可验证 bridge、adapter、4D 控制器和超时急停链路。此测试不代表真实双车编队效果。
