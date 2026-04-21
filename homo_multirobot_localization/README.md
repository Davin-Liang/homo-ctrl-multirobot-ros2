# homo_multirobot_localization

本包用于放置多机“定位/里程计链路”的 **launch 与配置**（与 `homo_multirobot_gazebo` + `homo_multirobot_urdf` 配合）。

当前已提供：

- 双机 **rf2o 激光里程计** 启动（仿真/回放）
- 单机 **rf2o 激光里程计** 启动（实机部署）

---

## 编译

在工作空间根目录：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select homo_multirobot_localization rf2o_laser_odometry --symlink-install
source install/setup.bash
```

---

## 启动（双机 rf2o）

> 在 WSL/受限环境下如果遇到 `PermissionError: ... ~/.ros/log/...`，可把日志目录指向工作空间可写路径：
>
> `export ROS_LOG_DIR=~/ros-projects/homo_multirobot_ws/log/ros`

启动前请确保仿真（或实机）已在发布：

- `/robot1/scan`、`/robot2/scan`（`sensor_msgs/LaserScan`）
- TF：`robot*_base_footprint -> robot*_laser_link` 可用（rf2o 用于推算激光位姿）

启动：

```bash
export ROS_LOG_DIR=~/ros-projects/homo_multirobot_ws/log/ros
ros2 launch homo_multirobot_localization rf2o_two_robots.launch.py
```

说明：rf2o 的关键参数（scan/odom 话题、frame、publish_tf 等）已在 `rf2o_two_robots.launch.py` 中**显式传入**，本包不再维护单独的 rf2o 参数 YAML，避免“YAML 顶层 key 与节点名不匹配导致参数不生效”的坑。

### 话题约定

- **输入**：`/robot1/scan`、`/robot2/scan`
- **输出**：`/robot1/rf2o/odom`、`/robot2/rf2o/odom`（`nav_msgs/Odometry`）

### TF 策略（关键）

rf2o 是否发布 `odom -> base_footprint` 的 TF 由参数控制：

- **默认**：`rf2o_publish_tf:=false`（只发布里程计话题，不发布 TF）
- **临时调试**：`rf2o_publish_tf:=true`（rf2o 发布 TF）

```bash
ros2 launch homo_multirobot_localization rf2o_two_robots.launch.py rf2o_publish_tf:=true
```

建议：与 EKF（`robot_localization`）二选一，避免多源 TF 冲突。

---

## 启动（单机 rf2o，实机）

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

