# homo_multirobot_localization

本包用于放置多机“定位/里程计链路”的 **launch 与配置**（与 `homo_multirobot_gazebo` + `homo_multirobot_urdf` 配合）。

当前已提供：

- 双机 **rf2o 激光里程计** 启动（仿真/回放）
- 单机 **rf2o 激光里程计** 启动（实机部署）
- 双机 **EKF（robot_localization）** 启动与参数（融合 IMU + rf2o odom）
- 单机 **EKF（robot_localization）** 启动（实机部署，支持 config 文件约定话题）
- 一键启动整条链路：**Gazebo 双机仿真 + 双机 rf2o + 双机 EKF**
- 组合启动：
  - 单机：**rf2o + EKF**（实机部署）
  - 双机：**rf2o + EKF**（仿真/回放）

---

## 🛠️ 编译

在工作空间根目录：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y ros-humble-robot-localization
colcon build --packages-select homo_multirobot_localization rf2o_laser_odometry --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

---

## 🧠 EKF 融合（robot_localization）

本包提供双机 EKF 启动与参数（融合 `/robot*/imu` + `/robot*/rf2o/odom`，输出 `/robot*/odometry/filtered`，并由 EKF 发布 TF `robot*_odom -> robot*_base_footprint`）：

```bash
ros2 launch homo_multirobot_localization ekf_two_robots.launch.py
```

### 🚀 启动（单机 EKF，实机）

每台真实机器人上各启动一次（默认 `use_sim_time:=false`），推荐显式传入 `namespace` 与 `prefix`：

```bash
ros2 launch homo_multirobot_localization ekf_single_robot.launch.py \
  namespace:=/robot1 prefix:=robot1_
```

该 launch 默认会加载 `config/ekf_single_robot.yaml` 来约定 EKF 的融合项与默认话题（如 `imu`、`rf2o/odom`）；如需自定义，直接覆盖：

```bash
ros2 launch homo_multirobot_localization ekf_single_robot.launch.py \
  namespace:=/robot1 prefix:=robot1_ \
  config_file:=/abs/path/to/my_ekf.yaml
```

#### 🧩 零参数启动（固定 robot1/robot2 的 frame）

如果你希望“每台机器人上直接一条命令启动”，可以用本包内置的实机配置（已把 `map/odom/base_link/world_frame` 固定成 `robot1_*` / `robot2_*`）：

```bash
ros2 launch homo_multirobot_localization ekf_single_robot.launch.py \
  namespace:=/robot1 \
  config_file:=$(ros2 pkg prefix homo_multirobot_localization)/share/homo_multirobot_localization/config/ekf_robot1_real.yaml
```

```bash
ros2 launch homo_multirobot_localization ekf_single_robot.launch.py \
  namespace:=/robot2 \
  config_file:=$(ros2 pkg prefix homo_multirobot_localization)/share/homo_multirobot_localization/config/ekf_robot2_real.yaml
```

话题约定（在 `/robot1` 命名空间下）：

- **输入**：`/robot1/imu`、`/robot1/rf2o/odom`
- **输出**：`/robot1/odometry/filtered`
- **TF**：默认由 EKF 发布 `robot1_odom -> robot1_base_footprint`

---

## 🧪 一键启动整条链路（仿真 + rf2o + EKF）

将 **Gazebo 双机仿真**、**双机 rf2o**、**双机 EKF** 串联启动：

```bash
ros2 launch homo_multirobot_localization sim_rf2o_ekf_two_robots.launch.py
```

默认 `world_name:=test_world.world`（带结构的世界更利于 rf2o 稳定；如需空世界可设为 `empty.world`）。

### 单机版（只起一台车，联调/建图更稳）

当你希望仿真环境中只有一台车（例如只让 `robot1` 建图，避免 `robot2` 的实体进入激光视野影响建图）：

```bash
ros2 launch homo_multirobot_localization sim_rf2o_ekf_single_robot.launch.py
```

默认参数：

- `world_name:=test_world.world`（默认使用带墙体/结构的世界，rf2o 更稳定；如需空世界可设为 `empty.world`）
- `robot_namespace:=/robot1`
- `robot_prefix:=robot1_`
- `use_rviz:=true`（默认会启动 RViz；如不需要可设为 `use_rviz:=false`）
- `planar_publish_odom_tf:=false`
- `rf2o_publish_tf:=false`

也就是：**只让 EKF 发布** `robot1_odom -> robot1_base_footprint` 的 TF，避免多源 TF 冲突；并确保后续 `slam_toolbox` 能拿到完整 TF 链从而发布 `map -> robot1_odom`。

### ⚠️ TF 策略（关键）

该总 launch 默认：

- `planar_publish_odom:=false`
- `planar_publish_odom_tf:=false`
- `rf2o_publish_tf:=false`

也就是：**只让 EKF 发布** `robot*_odom -> robot*_base_footprint` 的 TF，避免 Gazebo planar_move / rf2o 与 EKF 同时发布导致 TF 冲突。

## 🚀 启动（双机 rf2o）

> 在某些受限/自动化环境下（例如无写权限的 home 目录）如果遇到 `PermissionError: ... ~/.ros/log/...`，再临时设置 `ROS_LOG_DIR` 到可写目录即可。

启动前请确保仿真（或实机）已在发布：

- `/robot1/scan`、`/robot2/scan`（`sensor_msgs/LaserScan`）
- TF：`robot*_base_footprint -> robot*_laser_link` 可用（rf2o 用于推算激光位姿）

启动：

```bash
ros2 launch homo_multirobot_localization rf2o_two_robots.launch.py
```

说明：rf2o 的关键参数（scan/odom 话题、frame、publish_tf 等）已在 `rf2o_two_robots.launch.py` 中**显式传入**，本包不再维护单独的 rf2o 参数 YAML，避免“YAML 顶层 key 与节点名不匹配导致参数不生效”的坑。

### 📡 话题约定

- **输入**：`/robot1/scan`、`/robot2/scan`
- **输出**：`/robot1/rf2o/odom`、`/robot2/rf2o/odom`（`nav_msgs/Odometry`）

### ⚠️ TF 策略（关键）

rf2o 是否发布 `odom -> base_footprint` 的 TF 由参数控制：

- **默认**：`rf2o_publish_tf:=false`（只发布里程计话题，不发布 TF）
- **临时调试**：`rf2o_publish_tf:=true`（rf2o 发布 TF）

```bash
ros2 launch homo_multirobot_localization rf2o_two_robots.launch.py rf2o_publish_tf:=true
```

建议：与 EKF（`robot_localization`）二选一，避免多源 TF 冲突。

---

## 🚀 启动（单机 rf2o，实机）

每台真实机器人上各启动一次（默认 `use_sim_time:=false`），推荐显式传入 `namespace` 与 `prefix`：

```bash
ros2 launch homo_multirobot_localization rf2o_single_robot.launch.py \
  namespace:=/robot1 prefix:=robot1_
```

常用覆盖参数：

- `scan_topic`：默认 `scan`（在 `/robot1` 下即 `/robot1/scan`）
- `odom_topic`：默认 `rf2o/odom`（在 `/robot1` 下即 `/robot1/rf2o/odom`）
- `publish_tf`：默认 `false`（建议与 EKF 二选一）
- `freq`：默认 `20.0`
- `base_frame_id` / `odom_frame_id`：默认分别为 `<prefix>base_footprint` / `<prefix>odom`

示例：激光话题不在命名空间下时，直接用绝对话题名：

```bash
ros2 launch homo_multirobot_localization rf2o_single_robot.launch.py \
  namespace:=/robot1 prefix:=robot1_ \
  scan_topic:=/scan
```

---

## 🚀 启动（单机 rf2o + EKF，实机）

每台真实机器人上各启动一次（默认 `use_sim_time:=false`），推荐显式传入 `namespace` 与 `prefix`：

```bash
ros2 launch homo_multirobot_localization rf2o_ekf_single_robot.launch.py \
  namespace:=/robot1 prefix:=robot1_
```

该组合 launch 默认 `ekf_yaml_only:=true`，也就是 **EKF 完全以 `config_file`(YAML) 为准**。  
如果你希望临时用命令行覆盖 EKF 的 frame/topic/frequency/publish_tf 等参数，可显式设置：

```bash
ros2 launch homo_multirobot_localization rf2o_ekf_single_robot.launch.py \
  namespace:=/robot1 prefix:=robot1_ \
  ekf_yaml_only:=false
```

关键默认策略：

- `rf2o_publish_tf:=false`
- `ekf_publish_tf:=true`

也就是：**只由 EKF 发布** `odom -> base_footprint` TF，避免 TF 冲突。

---

## 🚀 启动（双机 rf2o + EKF）

```bash
ros2 launch homo_multirobot_localization rf2o_ekf_two_robots.launch.py
```

