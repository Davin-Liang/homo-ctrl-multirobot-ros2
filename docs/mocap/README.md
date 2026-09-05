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

### 机器人侧纯动捕底盘 bringup

每台实车仍需运行底盘串口驱动，以接收 `cmd_vel` 并驱动 STM32；纯动捕模式不需要 ImuProcessor、rf2o 或 EKF。每台车分别运行：

```bash
# Leader
ros2 launch turn_on_wheeltec_robot bringup_mini_omni_mocap.launch.py \
  namespace:=robot1 prefix:=robot1_

# Follower
ros2 launch turn_on_wheeltec_robot bringup_mini_omni_mocap.launch.py \
  namespace:=robot2 prefix:=robot2_
```

默认只启动 `wheeltec_robot_node` 和 `robot_state_publisher`。如果需要激光 scan 显示或后续避障，追加：

```bash
launch_lidar:=true
```

该 launch 的默认参数为 `namespace:=robot1`、`prefix:=robot1_`、`launch_lidar:=false`。Follower 必须显式覆盖为 `namespace:=robot2 prefix:=robot2_`。

首次使用新增的 `bringup_mini_omni_mocap.launch.py` 前，需构建并重新 source 该功能包：

```bash
cd ~/ros-projects/homo_multirobot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select turn_on_wheeltec_robot \
  --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash

ros2 launch turn_on_wheeltec_robot \
  bringup_mini_omni_mocap.launch.py --show-args
```

最后一条命令应显示 `namespace`、`prefix`、`launch_lidar`。使用 `--symlink-install` 时后续修改已有 launch 文件一般可立即生效，但首次新增 launch 文件后仍建议重新构建一次。

这两个机器人侧 launch 不发布定位状态或 `odom -> base_footprint` TF；该 TF 由控制主机上的动捕 adapter 发布。

在控制主机运行：

```bash
source /opt/ros/humble/setup.bash
source ~/ros-projects/homo_multirobot_ws/install/setup.bash

ros2 launch homo_multirobot_localization mocap_two_robots.launch.py \
  server:=<VRPN服务器IP>
```

该 launch 启动一个 `vrpn_listener` 和两个 C++ `mocap_state_adapter`。它不会启动编队算法、Gazebo、EKF 或机器人驱动。

未显式指定时的默认参数：

| 参数 | 默认值 |
|---|---|
| `port` | `3883` |
| `robot1_rigid_name` | `robot1` |
| `robot2_rigid_name` | `robot2` |
| `state_timeout` | `0.10` s |

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

### 刚体与反光标记点

贴在车上的银色反光小球称为 Marker。动捕软件根据多个 Marker 的固定几何布局，将它们识别为一个 rigid body（刚体），并发布该刚体的位置、朝向和速度。建议在动捕软件中将 Leader 与 Follower 的刚体分别命名为 `robot1`、`robot2`。

刚体原点是 Marker 布局的几何参考点，刚体 x 轴也是由动捕软件中的刚体定义决定；它们不一定等于机器人底盘中心和真正车头。例如所有 Marker 都贴在车尾时，刚体原点会相对底盘中心后移；若刚体 x 轴指向车左或车尾，刚体 yaw 会和真实车头相差 `+90°` 或 `180°`。

贴球与建刚体时应注意：

- 使用至少三个不共线 Marker，尽量四个或更多；形状应不对称，避免动捕软件把前后或左右方向识别反。
- Marker 尽量牢固地贴在刚性车体上，避免随线缆、外壳或减震部件相对底盘移动。
- 尽量让 Marker 的几何中心靠近底盘中心，并在动捕软件中让刚体前方对准机器人车头，以减小后续外参量。
- 先让车静止并观察 pose，再沿车头方向直线推行；确认刚体 x 轴、yaw 和速度方向均与实际车头一致。

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

两者的通用默认参数为 `leader_ns:=/robot1`、`follower_ns:=/robot2`、`control_rate:=20.0`、`max_linear_vel:=1.0`、`max_angular_vel:=0.5`。原始 4D 默认 `radius:=2.0`；4D Artstein 默认 `radius:=4.0`，并额外默认 `tau:=0.43`、`Td:=0.22`。

在 mocap 模式中，4D 状态直接为：

```text
[x_map, y_map, vx_map, vy_map]
```

不会查 `map -> robotN_odom` TF，也不会将动捕 map 系线速度再次旋转。

## Leader 动捕闭环绕圈

`leader_circle_closed_loop_map.launch.py` 可让 Leader 使用自己的动捕 map 系位姿和速度闭环跟踪圆轨迹。先启动上文的 `mocap_two_robots.launch.py`，再在控制主机启动：

```bash
ros2 launch homo_multirobot_formation_control \
  leader_circle_closed_loop_map.launch.py \
  namespace:=robot1 \
  use_sim_time:=false \
  state_source:=mocap \
  radius:=1.0 speed:=0.20 heading:=0.0 direction:=ccw
```

该节点订阅：

```text
/robot1/mocap/pose
/robot1/mocap/twist
```

两条消息都必须有 `header.frame_id=map`。节点输出 `/robot1/cmd_vel`。

| 参数 | 含义 | 建议初值 |
|---|---|---|
| `radius` | 圆轨迹半径（m） | `1.0` |
| `speed` | 圆周切向速度（m/s） | `0.20` |
| `heading` | Leader 期望固定车头方向（度，map 系） | `0.0` |
| `direction` | `ccw` 逆时针或 `cw` 顺时针 | `ccw` |
| `Td` | 电机纯滞后补偿时间（s） | `0.22` |
| `tau_v` | 速度一阶响应时间常数（s） | `0.43` |
| `mocap_state_timeout` | pose/twist 超时急停时间（s） | `0.10` |

与开环 `leader_circle.py` 不同，该节点利用动捕当前位置和全局速度做位置/速度闭环与延迟预测；若动捕 pose 或 twist 超时，会发布零 `/robot1/cmd_vel`。

未显式指定时，该 launch 默认 `namespace:=robot1`、`use_sim_time:=true`、`state_source:=odom_tf`、`radius:=2.0`、`speed:=0.2`、`heading:=0.0`、`direction:=ccw`。动捕实车模式必须至少覆盖 `use_sim_time:=false state_source:=mocap`。

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
