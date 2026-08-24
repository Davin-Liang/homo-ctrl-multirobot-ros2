# AGENTS.md

此文件为 Codex (Codex.ai/code) 在此源码仓库中工作时提供指导。

## 项目概述

基于 ROS 2 Humble 的多机器人协同与仿真工作空间。核心模型为 `mini_omni_robot`——三轮全向底盘，在 Gazebo Classic 11 中进行仿真。

主源码是独立的 Git 仓库。在 colcon 工作空间中通常位于
`/home/l1anggmgo/ros-projects/homo_multirobot_ws/src/homo-ctrl-multirobot-ros2/`。
真正的 colcon 构建根目录是上两级 workspace：
`/home/l1anggmgo/ros-projects/homo_multirobot_ws`。

## 编译位置注意事项

即使对话或编辑从本源码仓库目录 `src/homo-ctrl-multirobot-ros2/` 开始，
也不要在该目录直接运行 `colcon build`。否则会在源码仓库内生成独立的
`build/`、`install/`、`log/`，与 workspace 根目录的环境分离，后续容易
`source` 错环境。

从源码仓库目录编译时，先回到 workspace 根目录：

```bash
cd ../..
source /opt/ros/humble/setup.bash
colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

从 workspace 根目录编译时，直接执行下方命令。

## Git / Push 约定

- Codex 可以按需执行 `git status`、`git diff`、`git add`、`git commit` 等本地 Git 操作。
- commit message 使用中文，简洁说明本次修改内容。
- **不要执行 `git push`**。远程推送由用户手动完成。
- 如果用户要求“更新到远程仓库”，先完成本地验证和 commit，然后给出待推送 commit 与手动 `git push origin main` 命令。

## 构建与环境

```bash
source /opt/ros/humble/setup.bash

# 全量构建（跳过测试）
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF

# 选择性构建（最常用）
colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=OFF

# rf2o 激光里程计（补丁后需重新编译——已发布横向速度 lin_speed_y）
colcon build --packages-select rf2o_laser_odometry --symlink-install

source install/setup.bash
```

## 包架构

本仓库根目录下包含以下 ROS 2 包：

| 包名 | 作用 |
|------|------|
| `homo_multirobot_urdf` | Xacro 模型（`mini_omni_robot.xacro`，总质量 2.0 kg）、STL 网格、RViz 单机展示 |
| `homo_multirobot_gazebo` | Gazebo 世界文件、双机/单机 spawn launch、控制器 YAML 配置 |
| `homo_multirobot_localization` | 定位/里程计链路 launch 与配置：rf2o 激光里程计 + EKF（robot_localization） |
| `homo_multirobot_nav` | 已知地图定位（AMCL 或 slam_toolbox 纯定位）：单车/双车 launch + RViz |
| `homo_multirobot_formation_control` | Leader-Follower 编队控制（齐次控制算法，C++/Eigen）。4D、**4D Artstein + prediction**、4D Artstein-LQR、4D Cont、6D、6D Disc、**6D Motor**、旧 6D+OA，以及 **6D Artstein Disc + predictor-HOCBF**。HOCBF 节点从 `/scan` 拟合静态圆柱，在 map 系预测状态上施加多圆柱硬 QP，并将最终命令回写 Artstein 历史。 |
| `homo_multirobot_slam_toolbox` | 对上游 `slam_toolbox` 的多机器人封装，支持选定一台车建图、多车复用同一张地图 |
| `third_party/*` | 上游源码引入副本：`rf2o_laser_odometry`（已补丁：发布横向速度 lin_speed_y，原版只考虑差速车）、`robot_localization`、`omnidirectional_controllers` |

每个包内有独立的 `README.md`（含详细 launch 参数说明）和 `BUG_RECORD.md`（排障记录）。

## 机器人命名与 TF 约定

- **前缀（prefix）**：`robot1_`、`robot2_`——通过 xacro 参数为所有 link/joint 名添加前缀，避免多机 TF 重名
- **命名空间（namespace）**：`/robot1`、`/robot2`——隔离各机器人的话题
- **传感器话题**：`/<ns>/scan`（LaserScan，10Hz，720 采样点）、`/<ns>/imu`（Imu，50Hz）
- **TF 树**：`world → robot*_odom → robot*_base_footprint → robot*_base_link → [传感器 link、轮子 link]`
- **TF 发布权规则**：`odom → base_footprint` 的 TF 只能由一个节点发布。使用 EKF 时，必须禁用 gazebo_planar_move 和 rf2o 的 TF 发布（`planar_publish_odom_tf:=false`、`rf2o_publish_tf:=false`）

## 两种 Gazebo 驱动模式

xacro 模型通过 `use_ros2_control` 参数支持两种驱动方式：

1. **planar_move 模式**（默认，`use_ros2_control:=false`）：使用 `gazebo_ros_planar_move` 插件，订阅 `/<ns>/cmd_vel`，发布 `/<ns>/odom` 和 TF。轮子摩擦系数已设为零，避免对底盘产生额外的偏航力矩。

2. **ros2_control 模式**（`use_ros2_control:=true`）：加载 `gazebo_ros2_control` + 关节速度接口。需要另行启动 controller_manager + spawner。控制器 YAML 配置位于 `homo_multirobot_gazebo/config/`（轮半径 0.03m，底盘半径 0.24m，gamma 60°）。

## 常用 Launch 命令

```bash
# 仅查看模型（无仿真）
ros2 launch homo_multirobot_urdf display.launch.py

# 双机 Gazebo 仿真
ros2 launch homo_multirobot_gazebo sim_two_robots.launch.py
ros2 launch homo_multirobot_gazebo sim_two_robots.launch.py world_name:=test_world.world

# 单机仿真（建图/联调推荐，避免第二台车的激光干扰）
ros2 launch homo_multirobot_gazebo sim_single_robot.launch.py

# 一键定位链路（仿真 + rf2o + EKF，单机）
ros2 launch homo_multirobot_localization sim_rf2o_ekf_single_robot.launch.py

# 单机定位，指定 namespace 和初始位姿（如只起 Follower robot2）
ros2 launch homo_multirobot_localization sim_rf2o_ekf_single_robot.launch.py \
  robot_namespace:=/robot2 robot_prefix:=robot2_ \
  robot_x:=2.0 robot_y:=0.0 robot_yaw:=0.0

# 双机定位链路
ros2 launch homo_multirobot_localization sim_rf2o_ekf_two_robots.launch.py

# 已知地图定位 — slam_toolbox 纯定位（双机）
ros2 launch homo_multirobot_nav slam_toolbox_loc_two_robots.launch.py \
  robot1_map_start_x:=0.0 robot1_map_start_y:=0.0 robot1_map_start_yaw:=0.0 \
  robot2_map_start_x:=2.0 robot2_map_start_y:=0.0 robot2_map_start_yaw:=0.0

# 已知地图定位 — slam_toolbox 纯定位（单机）
ros2 launch homo_multirobot_nav slam_toolbox_loc_single_robot.launch.py \
  namespace:=/robot2 prefix:=robot2_ \
  map_name:=sim_room1_map map_start_x:=2.0 map_start_y:=0.0 map_start_yaw:=0.0

# Leader-Follower 编队控制 — 4D 质点模型（原版论文算法，离散多边形 + tol 切换）
ros2 launch homo_multirobot_formation_control formation_single_follower.launch.py
ros2 launch homo_multirobot_formation_control formation_single_follower.launch.py \
  mass:=8.0 radius:=2.0 Kp_yaw:=4.0 K_ff:=1.0 omega_d:=1.5

# Leader-Follower 编队控制 — 4D Artstein + prediction（输入延迟补偿 + 电机前向预测）
ros2 launch homo_multirobot_formation_control formation_single_follower_4d_artstein.launch.py \
  leader_ns:=/robot1 follower_ns:=/robot2 \
  tau:=0.43 Td:=0.22 control_rate:=20.0 \
  mass:=2.0 hpc_c_min:=0.1

# Leader-Follower 编队控制 — 4D Cont 质点模型（连续边界投影，无 tol/m_p）
ros2 launch homo_multirobot_formation_control formation_single_follower_4d_cont.launch.py
ros2 launch homo_multirobot_formation_control formation_single_follower_4d_cont.launch.py \
  radius:=2.0 mass:=8.0 omega_d:=1.5

# Leader-Follower 编队控制 — 6D 运动学模型（车身朝向 + 全向轮约束 + 边界投影）
ros2 launch homo_multirobot_formation_control formation_single_follower_6d.launch.py
ros2 launch homo_multirobot_formation_control formation_single_follower_6d.launch.py \
  radius:=1.0 mass:=8.0 I:=1.0 wheel_max_omega:=10.0

# Leader-Follower 编队控制 — 6D Disc 离散多边形编队（6D 模型 + 离散编队点 + tol 切换）
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_disc.launch.py
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_disc.launch.py \
  m_p:=4 tol:=0.1 radius:=1.0 mass:=8.0 I:=1.0

# Leader-Follower 编队控制 — 6D+OA 运动学 + 避障（基于 /scan 激光雷达）
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_oa.launch.py
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_oa.launch.py

# Leader-Follower 编队控制 — 6D Motor 电机感知模型（执行器一阶滞后增广，实物大延迟场景）
# 状态: [px, py, vx_cmd, vy_cmd, vx_real, vy_real] (map 系), v_cmd 为内部积分状态
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_motor.launch.py \
  leader_ns:=/robot1 follower_ns:=/robot2 use_motor_delay:=true
# 注入仿实物电机延迟(motor_tau=0.43)+加速度限幅(0.25)，与模型对齐
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_motor.launch.py \
  leader_ns:=/robot1 follower_ns:=/robot2 use_motor_delay:=true \
  tau:=0.43 omega_d:=0.7 mass:=2.0 hpc_c_min:=0.9
# 实物: 启用自适应 τ + Smith 预估器(死区补偿) + 最小速度
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_motor.launch.py \
  leader_ns:=/virtual_leader follower_ns:=/robot2 \
  use_motor_delay:=false tau:=0.43 mass:=2.0 omega_d:=0.7 max_linear_accel:=0.25 \
  m_p:=1 min_cmd_vel:=0.03 use_smith_predictor:=true smith_tau:=0.43 smith_Td:=0.22
# 轨迹记录（需指定 controller_node_name 以读取 6D Motor 专属参数）
ros2 run homo_multirobot_formation_control record_trajectory.py \
  --ros-args -p mode:=sim -p duration:=45.0 \
  -p controller_node_name:=formation_control_node_6d_motor

# 设计文档
# 6D Motor 电机感知模型: doc/motor_homogeneous_control_full.md
# 原始设计草稿:          doc/6d_motor_model_design.md

# 领航者轨迹（开环 cmd_vel，依赖 Gazebo/EKF 提供里程计）
ros2 run homo_multirobot_formation_control leader_circle.py --ros-args -r __ns:=/robot1
ros2 run homo_multirobot_formation_control leader_eight.py --ros-args -r __ns:=/robot1

# 虚拟 Leader 绕圈（直接发布 odometry/filtered + 静态 TF，不依赖仿真/实车）
ros2 run homo_multirobot_formation_control virtual_leader_circle.py \
  --ros-args -r __ns:=/virtual_leader
ros2 run homo_multirobot_formation_control virtual_leader_circle.py \
  --ros-args -r __ns:=/virtual_leader \
  -p center_x:=0.0 -p center_y:=0.0 -p radius:=2.0 -p speed:=0.5 -p direction:=ccw

# 轨迹记录与画图（支持自定义 leader/follower namespace）
ros2 run homo_multirobot_formation_control record_trajectory.py \
  --ros-args -p mode:=sim -p duration:=30.0

# 轨迹记录（实物，自动从控制器读取参数生成标签）
ros2 run homo_multirobot_formation_control record_trajectory.py \
  --ros-args -p mode:=real -p leader_ns:=/virtual_leader -p follower_ns:=/robot2 \
  -p radius:=2.0 -p duration:=30.0

# 诊断：电机响应延迟（实物）
python3 measure_motor_latency.py --ns /robot2 --raw-odom-topic /odom --trials 10

# 诊断：跨机器话题延迟（实物 Follower 侧）
python3 measure_cross_machine_delay.py --topic /robot1/odometry/filtered --duration 60 --csv /tmp/delay.csv

# 诊断：查看话题发布频率
ros2 topic hz /robot1/odom
ros2 topic hz /robot1/odometry/filtered

# 键盘控制（planar_move 模式）
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel

# 虚拟 Leader + Follower 编队（不跑 leader 仿真/实车）
# 终端1: 只启动 Follower 仿真
ros2 launch homo_multirobot_localization sim_rf2o_ekf_single_robot.launch.py \
  robot_namespace:=/robot2 robot_prefix:=robot2_ \
  robot_x:=2.0 robot_y:=0.0 robot_yaw:=0.0
# 终端2: Follower 已知地图定位
ros2 launch homo_multirobot_nav slam_toolbox_loc_single_robot.launch.py \
  namespace:=/robot2 prefix:=robot2_ \
  map_name:=sim_room1_map map_start_x:=2.0 map_start_y:=0.0 map_start_yaw:=0.0
# 终端3: 虚拟 Leader 绕圈
ros2 run homo_multirobot_formation_control virtual_leader_circle.py \
  --ros-args -r __ns:=/virtual_leader -p radius:=2.0 -p speed:=0.5
# 终端4: 6D 编队控制
ros2 launch homo_multirobot_formation_control formation_single_follower_6d.launch.py \
  leader_ns:=/virtual_leader follower_ns:=/robot2

# 或使用 6D Artstein Disc + predictor-HOCBF（仅 scan 感知静态圆柱）
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_artstein_disc_hocbf.launch.py \
  leader_ns:=/virtual_leader follower_ns:=/robot2 \
  tau:=0.43 Td:=0.22 control_rate:=20.0 \
  follower_radius:=0.15 clearance:=0.10 perception_margin:=0.15

# 或使用 6D Motor 电机感知模型 + 虚拟 Leader
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_motor.launch.py \
  leader_ns:=/virtual_leader follower_ns:=/robot2 use_motor_delay:=true
```

## 实车 Bringup（mini_omni）

实车硬件驱动包 `turn_on_wheeltec_robot`，依赖 SDK 内的 `serial`、`wheeltec_robot_msg`、`lslidar_ros2`。

```bash
# 实车编译
colcon build --packages-select serial wheeltec_robot_msg lslidar_msgs lslidar_driver turn_on_wheeltec_robot --symlink-install --cmake-args -DBUILD_TESTING=OFF --parallel-workers 1

# 轮式里程计模式（默认，推荐）
ros2 launch turn_on_wheeltec_robot bringup_mini_omni.launch.py namespace:=robot1 prefix:=robot1_

# rf2o 激光里程计模式（实验性）
ros2 launch turn_on_wheeltec_robot bringup_mini_omni.launch.py namespace:=robot1 prefix:=robot1_ odom_source:=rf2o

# 双机
ros2 launch turn_on_wheeltec_robot bringup_mini_omni.launch.py namespace:=robot2 prefix:=robot2_
```

**关键参数**：`namespace`（命名空间隔离）、`prefix`（TF 前缀）、`odom_source`（`wheel`/`rf2o`）。

**数据流**：STM32 串口 → `/odom` + `/imu/data_raw` → ImuProcessor → `/imu/data_filtered` → EKF → `/odometry/filtered` + TF。

**频率限制**：`/odom` 发布频率 = 20Hz，由 STM32 固件决定，`wheeltec_robot` 驱动无频率设置参数。
EKF 和控制器频率不应超过此硬件上限（目前全部 20Hz，最优配置）。

### 实物 ARM 编译注意事项

实车 ARM 内存有限（通常 4GB），编译 `homo_multirobot_formation_control` 时模板实例化
（Eigen + 大量 `expm` 调用）可能导致 OOM。处理方式：

**1. 开启 swap（编译前一次性操作）**
```bash
sudo fallocate -l 4G /swapfile && sudo chmod 600 /swapfile
sudo mkswap /swapfile && sudo swapon /swapfile
```

**2. 实物编译命令**

先清 build 缓存（避免 clock skew 等增量编译问题），再单 worker 编译：
```bash
rm -rf ~/homo_multirobot_ws/build/homo_multirobot_formation_control
cd ~/homo_multirobot_ws
MAKEFLAGS="-j1" colcon build --packages-select homo_multirobot_formation_control \
  --symlink-install --executor sequential --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O2" -DBUILD_TESTING=OFF
```

> 关键参数: `MAKEFLAGS="-j1"` + `--executor sequential` + `--parallel-workers 1`
> 三重保证单线程编译，避免并行内存尖峰。`-O2` 优化级别在 ARM 上编译时间和
> 内存占用比 `-O3` 温和。

**3. 目标选择**

> CMakeLists.txt 中默认注释掉部分旧 6D/Motor/OA 目标；当前包含 4D、4D Artstein、
> 4D Artstein-LQR、6D Artstein Disc 与 6D Artstein Disc + HOCBF。
> 如需编译 6D Motor 等目标，先取消对应 `add_executable` 的注释再编译，
> 编译完成后建议重新注释以减轻后续编译内存压力。

**4. 编译后确认可执行文件**
```bash
ls install/homo_multirobot_formation_control/lib/homo_multirobot_formation_control/
# 当前至少包含: formation_control_node, formation_control_node_4d_artstein,
# formation_control_node_4d_artstein_lqr, formation_control_node_6d_artstein_disc,
# formation_control_node_6d_artstein_disc_hocbf
```

**5. 编译后关闭 swap（可选，释放磁盘空间）**
```bash
sudo swapoff /swapfile && sudo rm /swapfile
```

## 系统延迟诊断

### 延迟链路

```
实物: 编码器 → STM32(20Hz) → 串口 → /odom(20Hz) → EKF(20Hz实际) → /odometry/filtered → 控制器(20Hz)
                                                                         ↑ ekf_age
                                                                                    → cmd_vel → 串口 → STM32 → 电机
                                                                                                    ↑ motor_latency
Leader: .../odometry/filtered → DDS → WiFi → Follower回调 → timer取用
                                                 ↑ leader_age
```

### 测量工具

| 延迟段 | 工具 | 说明 |
|--------|------|------|
| 网络延迟 | `ros2 topic delay /odometry/filtered` | 需时钟同步 |
| 电机响应 | `measure_motor_latency.py` | 阶跃响应法 |
| 数据新鲜度 | 控制器 DIAG 输出 | `avg_leader_age` / `avg_ekf_age` |

### 控制器在线诊断输出

4D 控制器内置两层诊断日志：

```bash
# 每秒: raw=算法原始 clamped=硬限幅后 final=约束后 scale=轮速缩放
raw=(+0.523,-0.187) clamped=(+0.523,-0.187) final=(+0.412,-0.147) scale=0.79

# 每5秒: 实际控制频率 + 数据新鲜度（窗口平均）
DIAG: freq=34.8Hz avg_leader_age=7ms avg_ekf_age=6ms
```

### 双机时钟同步（跨机延迟测量前提）

```bash
# Leader(/etc/chrony/chrony.conf): allow 192.168.3.0/24 + local stratum 10
# Follower(/etc/chrony/chrony.conf): server <leader-ip> iburst + local stratum 15
# 两台车: sudo systemctl restart chrony && sudo chronyc makestep
# 验证: chronyc tracking | grep Leap  # 应显示 Normal
```

`leader_age` 依赖此同步，否则测量值无意义。

## 环境说明

- **系统**：Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11
- **WSL 注意事项**：Gazebo 黑屏时可尝试 `software_rendering:=true`（CPU 软渲染）。ALSA 声卡告警可忽略（launch 已设 `ALSOFT_DRIVERS=null`）。`~/.ros/log` 权限错误时设置 `ROS_LOG_DIR` 到可写路径。
- **强制关闭 Gazebo**：`pkill -9 gzserver; pkill -9 gzclient`

## 验证命令

```bash
# 检查关键话题是否存在
ros2 topic list | egrep 'robot(1|2)/(scan|imu|rf2o/odom|odometry/filtered)'

# 检查话题发布频率
ros2 topic hz /robot1/scan
ros2 topic hz /robot1/imu

# 验证 TF 链
ros2 run tf2_ros tf2_echo robot1_odom robot1_base_footprint
ros2 run tf2_tools view_frames   # 生成 frames.pdf

# 确认当前使用的驱动模式
ros2 topic echo /robot1/robot_description --once --full-length | grep -E "gazebo_ros_planar_move|gazebo_ros2_control"

# 验证 rf2o 横向速度已补丁生效（非零——全向底盘必需）
ros2 topic echo /robot1/rf2o/odom --field twist.twist.linear

# 验证 EKF 横向速度正常（非零——受 rf2o vy 驱动）
ros2 topic echo /robot1/odometry/filtered --field twist.twist.linear
```
