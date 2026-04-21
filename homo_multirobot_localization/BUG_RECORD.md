# BUG_RECORD — `homo_multirobot_localization` 联调记录

本文档记录在接入 rf2o 等定位/里程计链路过程中遇到的问题与处理方式。

---

## 1. rf2o 一直提示 `Waiting for laser_scans....`

**现象**  
启动 `rf2o_two_robots.launch.py` 后终端持续输出：

- `Waiting for laser_scans....`

**排查要点**  
- 确认 `/robot1/scan`、`/robot2/scan` 存在且确实有数据：

```bash
ros2 topic list | egrep 'robot(1|2)/scan'
ros2 topic echo /robot1/scan --once
ros2 topic echo /robot2/scan --once
```

**常见原因**  
- 话题名未对齐（rf2o 订阅的不是 `/robot*/scan`）。本仓库已在 launch 中显式传入 `/robot1/scan`、`/robot2/scan`，避免命名空间/相对话题歧义。

---

## 2. `ros2 launch ... --show-args` 报 `PermissionError: ~/.ros/log/...`

**现象**  
在 WSL/受限环境下，`ros2 launch` 甚至在加载 launch 文件时就失败，提示对 `~/.ros/log/...` 无写权限。

**处理**  
把 `ROS_LOG_DIR` 指向工作空间可写目录：

```bash
export ROS_LOG_DIR=~/ros-projects/homo_multirobot_ws/log/ros
```

---

## 3. EKF 启动但 `/robot*/odometry/filtered` 没有消息（或节点未订阅 imu/odom）

**现象**  
启动 `ekf_two_robots.launch.py` / `ekf_single_robot.launch.py` 后：

- `/robot1/odometry/filtered` 话题存在但 `ros2 topic echo` 无输出
- `ros2 node info /robot1/ekf_filter_node` 的 Subscribers 中**没有** `/robot1/imu`、`/robot1/rf2o/odom`

**原因**  
`robot_localization` 的参数文件若按“节点名匹配 YAML 顶层 key”的方式写，配合命名空间（`/robot1`）后容易出现**参数未生效但不报错**的情况。

**处理（推荐）**  
参数 YAML 顶层使用 `/**:` 让其在任意命名空间下都生效，或在 launch 中显式覆盖关键输入参数（`imu0`、`odom0` 等）。  
本仓库的 `ekf_*` 参数文件已使用 `/**:` 以规避该坑。

验证：

```bash
ros2 node info /robot1/ekf_filter_node | egrep "/robot1/(imu|rf2o/odom)"
ros2 topic hz /robot1/odometry/filtered
```

---

## 4. rf2o 参数 YAML 未生效（仍显示默认 `/scan`、`/odom_rf2o`）

**现象**  
明明 `/robot1/scan`、`/robot2/scan` 有数据（`ros2 topic echo ... --once` 可看到），但 rf2o 仍持续输出：

- `Waiting for laser_scans....`

并且查询参数发现仍是默认值：

```bash
ros2 param get /robot1/<node_name> laser_scan_topic
# String value is: /scan
```

**原因**  
rf2o 节点确实启动了，但**参数文件没有被实际应用**（常见于“按节点名匹配 YAML 顶层 key”的坑、或命名空间/节点名变更后 YAML key 未同步）。

**处理（推荐）**  
在 launch 中**显式传参**（用字典传入 `laser_scan_topic`、`odom_topic`、`publish_tf` 等），避免依赖 YAML 顶层 key 与节点名的匹配规则。

本仓库已在 `rf2o_two_robots.launch.py` 中采用显式传参，参数应可直接验证：

```bash
ros2 param get /robot1/rf2o_laser_odometry_robot1 laser_scan_topic
ros2 param get /robot2/rf2o_laser_odometry_robot2 laser_scan_topic
```

期望分别为 `/robot1/scan` 与 `/robot2/scan`。

---

## 5. rf2o 静止仍“漂移”（导致 EKF 也漂）

**现象**  
机器人不动时，`/robot*/rf2o/odom` 的 pose 仍持续变化；EKF 融合后 `/robot*/odometry/filtered` 同样漂移。

**常见原因**  
- 仿真使用 `empty.world`：世界里缺少足够的几何特征，2D 激光 scan matching 天然不可观/病态，容易随机游走；
- 激光帧率与 rf2o `freq` 不匹配：例如激光 `update_rate=10Hz`，rf2o 配 `freq=20Hz`，会出现 `Waiting for laser_scans....`、处理间隔不稳定，数值更容易漂。

**处理建议**  
- 在仿真中优先使用带墙体/结构的世界（例如 `homo_multirobot_gazebo/worlds/test_world.world`）：

```bash
ros2 launch homo_multirobot_gazebo sim_two_robots.launch.py world_name:=test_world.world
```

- 将 rf2o `freq` 设为不高于实际 `scan` 频率（建议等于 scan rate）。

验证：

```bash
ros2 topic hz /robot1/scan
ros2 topic hz /robot1/rf2o/odom
```

