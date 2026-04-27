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
| **`homo_multirobot_slam_toolbox`** | 多机器人建图封装：支持选定 `mapper_robot` 单机建图（两车复用同一张地图），并支持将 `/map` 重映射进机器人 namespace，便于在 `/robot1` 下调用 `save_map` |
| **`homo_multirobot_nav`** | 已知地图定位（Nav2 `map_server` + `amcl`）：单车/双车共图定位 launch，并提供“开箱即用”的 RViz 配置（Map QoS、初始位姿话题等） |
| **`rf2o_laser_odometry`（third_party）** | 2D 激光里程计（rf2o，上游源码引入，ROS 2 分支），订阅 `/robot*/scan` 输出 `/robot*/rf2o/odom`（可选发布 TF） |
| **`homo_multirobot_localization`** | 多机定位/里程计链路启动与配置：双机/单机 rf2o、双机/单机 EKF（`robot_localization`），以及仿真一键链路（Gazebo + rf2o + EKF） |
| **`omnidirectional_controllers`（third_party）** | 上游 ros2_control 控制器（订阅 `cmd_vel`，输出轮速，发布里程计等），用于后续三轮全向底盘轮子级控制 |
| **`robot_localization`（third_party，可选）** | 上游融合包（EKF/UKF）。推荐直接装 `ros-humble-robot-localization`；若需源码联调可放入工作空间 third_party |

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

# slam_toolbox（建图）
sudo apt install -y ros-humble-slam-toolbox

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
      homo_multirobot_slam_toolbox/
      README.md
      third_party/
        omnidirectional_controllers/
        rf2o_laser_odometry/
        robot_localization/
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
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

> third_party 目录下是上游源码的“引入副本”，用于补齐你工作空间的依赖或方便联调。  
> 如果你更倾向使用 apt 二进制（例如 `robot_localization`），可以保留 third_party 但不使用；或自行移除对应上游包。

若你此前在同一工作空间里编译过 **已删除的第三方包**（如旧的 `wheeltec_*`），`install/` 里可能仍残留同名目录；可手动删除这些目录，或在工作空间根目录 **清空 `build/`、`install/`、`log/` 后做一次全量重编**，避免 `ament` 与路径混淆。

### 3. 运行

下面仅列出“复现项目能力”的最短入口；每个能力的完整参数解释与排障请直接看对应包内 README / BUG_RECORD。

#### 3.1 单机 RViz（无仿真）

```bash
ros2 launch homo_multirobot_urdf display.launch.py
```

更多：见 `homo_multirobot_urdf/README.md`。

#### 3.2 Gazebo 仿真

双机：

```bash
ros2 launch homo_multirobot_gazebo sim_two_robots.launch.py
```

> 建议：若你要跑 `rf2o`/EKF，优先使用带墙体/结构的世界（例如 `sim_room1.world` / `test_world.world`），避免 `empty.world` 特征不足导致 rf2o 漂移：
>
> `ros2 launch homo_multirobot_gazebo sim_two_robots.launch.py world_name:=sim_room1.world`

单机（推荐用于建图/联调，避免第二台车动态影响激光与建图）：

```bash
ros2 launch homo_multirobot_gazebo sim_single_robot.launch.py
```

更多：见 `homo_multirobot_gazebo/README.md` 与 `homo_multirobot_gazebo/BUG_RECORD.md`。

#### 3.3 定位/里程计链路（rf2o + EKF）

一键启动整条链路（仿真单车 + rf2o + EKF）：

```bash
ros2 launch homo_multirobot_localization sim_rf2o_ekf_single_robot.launch.py
```

说明：该仿真总 launch 默认 `use_rviz:=true`（会同时启动 RViz）；如不需要可设置 `use_rviz:=false`。
说明：该仿真总 launch 默认 `world_name:=sim_room1.world`；如需切换可设置 `world_name:=test_world.world` 或 `world_name:=empty.world`。

更多：见 `homo_multirobot_localization/README.md` 与 `homo_multirobot_localization/BUG_RECORD.md`。

#### 3.4 已知地图定位（AMCL + map_server，共图）

单车 AMCL（默认 `/robot1`，并启动 RViz）：

```bash
ros2 launch homo_multirobot_nav amcl_single_robot.launch.py
```

双车 AMCL（共用同一张地图，并提供可分别设置 `/robot1/initialpose` 与 `/robot2/initialpose` 的 RViz 工具）：

```bash
ros2 launch homo_multirobot_nav amcl_two_robots.launch.py
```

选择地图（示例）：

```bash
ros2 launch homo_multirobot_nav amcl_single_robot.launch.py map:=/abs/path/to/my_map.yaml
```

说明：仓库内置示例地图位于 `homo_multirobot_slam_toolbox/maps/`，安装后路径为：

- `$(ros2 pkg prefix homo_multirobot_slam_toolbox)/share/homo_multirobot_slam_toolbox/maps/*.yaml`

更多：见 `homo_multirobot_nav/README.md` 与 `homo_multirobot_nav/BUG_RECORD.md`。

#### 3.4 建图（slam_toolbox）

单机建图（选定 mapper_robot）：

```bash
ros2 launch homo_multirobot_slam_toolbox single_robot_mapping.launch.py mapper_robot:=robot1 use_sim_time:=true
```

保存地图（mapper_robot=robot1）：

```bash
ros2 service call /robot1/slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/abs/path/to/my_map'}}"
```
更多：见 `homo_multirobot_slam_toolbox/README.md` 与 `homo_multirobot_slam_toolbox/BUG_RECORD.md`。

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
