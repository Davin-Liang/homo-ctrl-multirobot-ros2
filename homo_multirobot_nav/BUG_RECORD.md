# BUG_RECORD — `homo_multirobot_nav` 联调记录

本文档记录在接入 Nav2 `map_server` + `amcl`（单车/双车共图定位）过程中遇到的问题与处理方式。

---

## 1) lifecycle_manager 报 bond 超时：`Server ... unable to be reached ... by bond`

**现象**

- `lifecycle_manager_localization`: `Server /map_server was unable to be reached ... by bond`
- 或 `Server robot1/amcl was unable to be reached ... by bond`

**原因（常见）**

- `node_names` 填写了带前导 `/` 的绝对名，导致生命周期管理器内部名字拼接与 bond 不匹配；
- 或 AMCL 节点没真正进入可工作状态（例如一直在等地图/等 TF），也会导致 bond 失败。

**处理**

- `node_names` 使用**相对名**（如 `map_server`、`amcl`、`robot1/amcl`），不要带前导 `/`；
- 并建议按“同命名空间管理同命名空间节点”的方式拆分 lifecycle_manager：
  - 全局 lifecycle_manager 管 `map_server`
  - `/robotX` 下 lifecycle_manager 管 `/robotX/amcl`

本仓库 `amcl_single_robot.launch.py` / `amcl_two_robots.launch.py` 已按上述方式修复。

---

## 2) AMCL 一直 `Waiting for map....`

**现象**

- `nav2_amcl` 输出：`Waiting for map....`
- 同时 RViz 中 Map 可能显示正常（因为你能 `ros2 topic echo /map --once`）

**原因**

`map_server` 默认发布全局 `/map`，但 AMCL 运行在命名空间下时，默认订阅 `/<ns>/map`（例如 `/robot1/map`），因此订阅不到地图。

**处理**

在 AMCL 节点上做 remap：

- `("map", "/map")`

本仓库 launch 已内置该 remap。

---

## 3) RViz 显示 “No map received”，但命令行能 `echo /map`

**现象**

- RViz Map 显示器状态：`No map received`
- 但 `ros2 topic echo /map --once` 能看到数据

**原因**

QoS 不匹配：地图话题通常需要 RViz 以 `Durability=Transient Local` 订阅才能接到 latched 的地图。

**处理**

- RViz 的 Map 显示器 QoS 设置：
  - `Durability Policy: Transient Local`
  - `Reliability Policy: Reliable`

本仓库 `rviz/*.rviz` 已内置上述 QoS。

---

## 4) RViz 里 “2D Pose Estimate” 不生效，但命令行发 `/robotX/initialpose` 能生效

**现象**

- 在 RViz 点 2D Pose Estimate，看起来 AMCL 没收到初始位姿
- 但用命令行 `ros2 topic pub /robot1/initialpose ...` 能生效

**原因**

RViz 的 2D Pose Estimate 工具默认发布 `/initialpose`，而 AMCL 在命名空间下通常监听 `/<ns>/initialpose`（如 `/robot1/initialpose`）。

**处理**

- 在 RViz 的 Tool Properties 里把 Topic 改成 `/<ns>/initialpose`；
- 或启动 RViz 时 remap：`/initialpose:=/<ns>/initialpose`；
- 本仓库采用“更省事”的方式：
  - RViz 配置文件内将 SetInitialPose 的 Topic 固定为 `/robot1/initialpose`（单机）
  - 双机 RViz 配置内置两套工具：`/robot1/initialpose` 与 `/robot2/initialpose`
  - launch 仍额外提供 remap，保证 namespace 改动时无需改 RViz 文件

