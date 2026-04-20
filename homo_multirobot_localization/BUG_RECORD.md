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

## 3. rf2o 参数 YAML 未生效（仍显示默认 `/scan`、`/odom_rf2o`）

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

