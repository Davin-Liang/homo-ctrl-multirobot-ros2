# homo_multirobot_nav

本包用于放置“已知地图定位（AMCL）+ 静态地图加载（map_server）+ 多机器人命名空间/TF 组织 + RViz 便捷配置”的启动文件。

## 目标与约定

- **地图**：使用已保存的 2D 栅格地图（YAML+PGM），由 `nav2_map_server` 加载并发布全局 `/map`。
- **共图多车**：单全局 `map`，每台车各自发布 `map -> robotX_odom`。
- **TF 约定**（与仓库一致）：
  - EKF 发布：`robotX_odom -> robotX_base_footprint`
  - AMCL 发布：`map -> robotX_odom`
- **话题约定**：
  - 激光：`/robotX/scan`
  - 初始位姿：`/robotX/initialpose`

> 注意：由于 `map_server` 发布的是全局 `/map`，而 AMCL 在命名空间下默认会订阅 `/<ns>/map`，本包已在 launch 中显式将 AMCL 的 `map` 订阅 remap 到 `/map`，避免 “Waiting for map”。

---

## 🛠️ 依赖

- `nav2_map_server`
- `nav2_amcl`
- `nav2_lifecycle_manager`
- `rviz2`

（通常安装 `ros-humble-navigation2` 或至少安装上面几个组件即可。）

---

## 🚀 启动（单车 AMCL）

```bash
ros2 launch homo_multirobot_nav amcl_single_robot.launch.py
```

常用参数：

- `namespace`：默认 `/robot1`
- `prefix`：默认 `robot1_`
- `map`：地图 YAML 路径（建议绝对路径；默认指向 `homo_multirobot_slam_toolbox/maps/my_map1.yaml`）
- `use_rviz`：默认 `true`
- `rviz_config`：默认 `homo_multirobot_nav/rviz/amcl_single_robot.rviz`

示例：选择加载另一张地图：

```bash
ros2 launch homo_multirobot_nav amcl_single_robot.launch.py \
  map:=/abs/path/to/my_map.yaml
```

---

## 🚀 启动（双车 AMCL，共用同一张地图）

```bash
ros2 launch homo_multirobot_nav amcl_two_robots.launch.py
```

默认：

- `robot1_namespace:=/robot1`、`robot2_namespace:=/robot2`
- `robot1_prefix:=robot1_`、`robot2_prefix:=robot2_`
- `rviz_config:=homo_multirobot_nav/rviz/amcl_two_robots.rviz`

### 在 RViz 中分别设置两台车的初始位姿

双车 RViz 配置内置了 **两套** “2D Pose Estimate” 工具：

- `/robot1/initialpose`
- `/robot2/initialpose`

在顶部工具栏切换对应工具后，在地图上点击并拖拽即可给对应机器人设置初始位姿。

---

## ✅ 最小验证

```bash
# 地图话题
ros2 topic echo /map --once

# AMCL 输出（示例 robot1）
ros2 topic list | grep -E "/robot1/(amcl_pose|particle_cloud)"

# TF 链
ros2 run tf2_ros tf2_echo map robot1_odom
ros2 run tf2_ros tf2_echo robot1_odom robot1_base_footprint
```

