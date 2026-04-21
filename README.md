# homo-ctrl-multirobot-ros2

```text
 _   _  ____  __  __  ____        __  __       _ _   _           _           _     ____   ___  ____  
| | | |/ __ \|  \/  |/ __ \      |  \/  |     | | | (_)         | |         | |   |  _ \ / _ \|___ \ 
| |_| | |  | | \  / | |  | |_____| \  / |_   _| | |_ _ _ __ ___ | |__   ___ | |_  | |_) | | | | __) |
|  _  | |  | | |\/| | |  | |_____| |\/| | | | | | __| | '__/ _ \| '_ \ / _ \| __| |  _ <| | | ||__ < 
| | | | |__| | |  | | |__| |     | |  | | |_| | | |_| | | | (_) | |_) | (_) | |_  | |_) | |_| |___) |
|_| |_|\____/|_|  |_|\____/      |_|  |_|\__,_|_|\__|_|_|  \___/|_.__/ \___/ \__| |____/ \___/|____/ 
```

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)
[![Gazebo Classic 11](https://img.shields.io/badge/Gazebo-Classic%2011-4E4E4E?logo=gazebo&logoColor=white)](https://classic.gazebosim.org/)

面向 **ROS 2 Humble** 的多机器人协同与仿真相关代码：以 **`mini_omni_robot`** 全向底盘模型为核心，提供 **URDF/Xacro + mesh**、**Gazebo Classic 双机仿真** 与 **RViz** 可视化。

---

## 📖 快速理解（推荐先看）

### 最短上手路径

- **只看模型（不跑仿真）**：编译 `homo_multirobot_urdf` → `ros2 launch homo_multirobot_urdf display.launch.py`
- **双机仿真（默认 planar_move，可直接 `/cmd_vel`）**：编译 `homo_multirobot_gazebo` + `homo_multirobot_urdf` → `ros2 launch homo_multirobot_gazebo sim_two_robots.launch.py`
- **切到 ros2_control（后续接控制器）**：`use_ros2_control:=true` 后，需启动 controller_manager/spawner（见 `homo_multirobot_gazebo/README.md` 的说明与 YAML）

---

## 🧱 仓库结构

| 包名 | 说明 |
|------|------|
| **`homo_multirobot_urdf`** | `mini_omni_robot.xacro`、STL mesh、单机 RViz 展示 launch |
| **`homo_multirobot_gazebo`** | 空世界、双机 spawn、可选 RViz 配置与 `world` 静态 TF |
| **`rf2o_laser_odometry`** | 2D 激光里程计（rf2o，源码引入，ROS 2 分支），订阅 `/robot*/scan` 输出 `/robot*/rf2o/odom`（可选发布 TF） |
| **`homo_multirobot_localization`** | 多机定位/里程计链路启动与配置：双机/单机 rf2o、双机/单机 EKF（`robot_localization`），以及仿真一键链路（Gazebo + rf2o + EKF） |
| **`omnidirectional_controllers`** | 引入的上游 ros2_control 控制器（订阅 `cmd_vel`，输出轮速，发布里程计等），用于后续三轮全向底盘轮子级控制 |

各包内另有 **`README.md`** 与 **`BUG_RECORD.md`**，用于细节与排障。

---

## ⚠️ 环境依赖

- **Ubuntu 22.04**（与 ROS 2 Humble 官方配套）
- **ROS 2 Humble**（`desktop` 或 `ros-base` + 按需组件）
- **Gazebo Classic 11**（`gazebo_ros` / `gazebo_plugins`）
- **ROS 包**（示例）：`xacro`、`robot_state_publisher`、`joint_state_publisher`、`rviz2`、`tf2_ros`
- **robot_localization（EKF 融合）**：
  - 简化方案：直接安装二进制包 `ros-humble-robot-localization`
  - 源码方案：若你把 `robot_localization` 以源码形式放进工作空间（本仓库当前做法），还需要额外系统依赖（见下方安装示例）

安装示例（按需调整）：

```bash
sudo apt update
sudo apt install -y ros-humble-gazebo-ros ros-humble-gazebo-plugins \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher ros-humble-rviz2

# robot_localization（推荐直接用二进制包；若用源码编译，也可先装这个，能补齐大部分依赖）
sudo apt install -y ros-humble-robot-localization

# robot_localization 源码编译常见缺失依赖（按需；若编译报缺包再补）
sudo apt install -y ros-humble-diagnostic-updater libeigen3-dev libgeographic-dev
```

---

## 🚀 部署步骤

### 1. 获取代码

推荐直接在工作空间 `src/` 下克隆本仓库（路径约定与本文后续命令一致）：

```bash
mkdir -p <your_ws>/src
cd <your_ws>/src
git clone git@github.com:Davin-Liang/homo-ctrl-multirobot-ros2.git
```

目录结构示例：

```text
<your_ws>/
  src/
    homo-ctrl-multirobot-ros2/    # 本仓库（git clone 到此路径）
      homo_multirobot_urdf/
      homo_multirobot_gazebo/
      omnidirectional_controllers/
      README.md
```

### 2. 编译

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
# ros2_control 与控制器基座依赖（提供 controller_interface 等）
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install -y ros-humble-robot-localization

# 编译本仓库所有包
colcon build --symlink-install
source install/setup.bash
```

若你此前在同一工作空间里编译过 **已删除的第三方包**（如旧的 `wheeltec_*`），`install/` 里可能仍残留同名目录；可手动删除这些目录，或在工作空间根目录 **清空 `build/`、`install/`、`log/` 后做一次全量重编**，避免 `ament` 与路径混淆。

### 3. 运行

**单机 RViz（无仿真）**：

```bash
ros2 launch homo_multirobot_urdf display.launch.py
```

**Gazebo 双机仿真（含 Gazebo 客户端与 RViz，可选）**：

```bash
ros2 launch homo_multirobot_gazebo sim_two_robots.launch.py
```

常用参数见 **`homo_multirobot_gazebo/README.md`**（如 `use_rviz`、`publish_world_tf`、`gui` 等）。

**双机 rf2o（激光里程计）**（需要仿真已在发布 `/robot1/scan`、`/robot2/scan`）：

> 若你在 WSL/受限环境下看到 `PermissionError: ... ~/.ros/log/...`，请把 `ROS_LOG_DIR` 指向工作空间可写目录：
>
> `export ROS_LOG_DIR=~/ros-projects/homo_multirobot_ws/log/ros`

```bash
export ROS_LOG_DIR=~/ros-projects/homo_multirobot_ws/log/ros
ros2 launch homo_multirobot_localization rf2o_two_robots.launch.py
```

**双机 EKF（IMU + rf2o 融合）**（需已在发布 `/robot*/imu` 与 `/robot*/rf2o/odom`）：

```bash
export ROS_LOG_DIR=~/ros-projects/homo_multirobot_ws/log/ros
ros2 launch homo_multirobot_localization ekf_two_robots.launch.py
```

**一键启动整条链路（仿真 + rf2o + EKF）**：

```bash
export ROS_LOG_DIR=~/ros-projects/homo_multirobot_ws/log/ros
ros2 launch homo_multirobot_localization sim_rf2o_ekf_two_robots.launch.py
```

**单机 rf2o（实机部署）**（每台机器人各启动一次）：

```bash
ros2 launch homo_multirobot_localization rf2o_single_robot.launch.py \
  namespace:=/robot1 prefix:=robot1_
```

**单机 rf2o + EKF（实机部署，一条命令）**（每台机器人各启动一次）：

```bash
export ROS_LOG_DIR=~/ros-projects/homo_multirobot_ws/log/ros
ros2 launch homo_multirobot_localization rf2o_ekf_single_robot.launch.py \
  namespace:=/robot1 prefix:=robot1_
```

该组合 launch 默认 `ekf_yaml_only:=true`，EKF 参数以 `config_file`(YAML) 为准；如需临时用命令行覆盖 EKF 的 frame/topic/frequency/publish_tf 等参数：

```bash
ros2 launch homo_multirobot_localization rf2o_ekf_single_robot.launch.py \
  namespace:=/robot1 prefix:=robot1_ \
  ekf_yaml_only:=false
```

**单机 EKF（实机部署，IMU + rf2o 融合）**（每台机器人各启动一次）：

```bash
export ROS_LOG_DIR=~/ros-projects/homo_multirobot_ws/log/ros
ros2 launch homo_multirobot_localization ekf_single_robot.launch.py \
  namespace:=/robot1 prefix:=robot1_
```

若你希望零参数启动（frame 固定为 `robot1_*` / `robot2_*`），可直接指定本包内置配置：

```bash
ros2 launch homo_multirobot_localization ekf_single_robot.launch.py \
  namespace:=/robot1 \
  config_file:=$(ros2 pkg prefix homo_multirobot_localization)/share/homo_multirobot_localization/config/ekf_robot1_real.yaml
```

**双机 rf2o + EKF（仿真/回放，一条命令）**：

```bash
export ROS_LOG_DIR=~/ros-projects/homo_multirobot_ws/log/ros
ros2 launch homo_multirobot_localization rf2o_ekf_two_robots.launch.py
```

### 4. 验证话题（仿真）

在仿真运行后：

```bash
ros2 topic list | egrep 'robot(1|2)/(scan|imu|rf2o/odom|odometry/filtered)'
```

可进一步确认各链路是否在“持续发布”：

```bash
ros2 topic hz /robot1/scan
ros2 topic hz /robot1/imu
ros2 topic hz /robot1/rf2o/odom
ros2 topic hz /robot1/odometry/filtered
```

TF 冲突排查（推荐做一次）：

```bash
ros2 run tf2_ros tf2_echo robot1_odom robot1_base_footprint
ros2 run tf2_ros tf2_echo robot2_odom robot2_base_footprint
```

---

## 📌 与上层工作空间的关系

本目录是 **独立 Git 仓库**，可单独 `git clone`；也可作为 **`homo_fleet_ws/src/homo-ctrl-multirobot-ros2`** 等大工作空间中的一份子。  
无论哪种方式，**编译与 `source install/setup.bash` 始终在 Colcon 工作空间根目录**进行。

---

## 📄 许可

各子包以各自 `package.xml` 声明为准（Apache-2.0 等）。
