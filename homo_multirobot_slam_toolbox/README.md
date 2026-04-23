# homo_multirobot_slam_toolbox

本包是对上游 `slam_toolbox` 的**多机器人封装**：提供统一的参数与 launch，使同一套配置可在仿真/实机切换，并支持“只让一台车建图、多车复用同一张地图”的用法。

---

## ✅ 关键能力

- **单机建图（多机器人复用地图）**：通过 `mapper_robot:=robot1|robot2` 选择由哪台机器人运行 `slam_toolbox`
- **仿真时间**：支持 `use_sim_time:=true`
- **话题与 TF 对齐约定**：
  - `map_frame`: `map`（全局共享）
  - `odom_frame`: `<mapper_prefix>odom`（如 `robot1_odom`）
  - `base_frame`: `<mapper_prefix>base_footprint`（如 `robot1_base_footprint`）
  - `scan_topic`: `scan`（相对名，落在 `/<ns>/scan`）
- **地图保存**：在 mapper namespace 下提供 `/robot1/slam_toolbox/save_map` 等服务

---

## 🚀 启动（推荐：单机建图）

先确保你的仿真/定位链路已提供：

- `/robot1/scan`（或 mapper 对应 namespace 下的 scan）
- TF：`robot1_odom -> robot1_base_footprint` 与 `robot1_base_footprint -> robot1_laser_link`

启动：

```bash
ros2 launch homo_multirobot_slam_toolbox single_robot_mapping.launch.py mapper_robot:=robot1 use_sim_time:=true
```

---

## 💾 保存地图

查看服务：

```bash
ros2 service list | grep save_map
```

保存（mapper_robot=robot1）：

```bash
ros2 service call /robot1/slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/abs/path/to/my_map'}}"
```

会生成 `/abs/path/to/my_map.yaml` 与 `/abs/path/to/my_map.pgm`。

---

## ⚠️ 注意事项

- `.pgm` 是图片文件，在编辑器里按文本打开会“乱码”；用 `explorer.exe` 或图片查看器打开即可。
- 若 TF 树里没有 `map` frame，请优先检查 `robot1_odom -> robot1_base_footprint` 是否存在（一般是 EKF frame 配置未对齐导致）。

