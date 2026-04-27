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

### 🗺️ RViz 里看不到地图（/map）？

默认情况下，本仓库的 `single_robot_mapping.launch.py` 会将 slam_toolbox 的地图话题固定到**全局**：

- `/map`、`/map_updates`

因此 RViz 直接订阅 `/map` 即可。

同时，为了不影响 `save_map`（其内部 `map_saver` 会在命名空间下订阅相对话题 `map`），本仓库会在 `map_in_namespace:=false` 时自动把 `/map` 转发到 `/<ns>/map`，保证保存地图仍然可用。

如果你确实需要把地图话题放进命名空间（例如希望出现 `/robot1/map`），可显式开启：

```bash
ros2 launch homo_multirobot_slam_toolbox single_robot_mapping.launch.py \
  mapper_robot:=robot1 use_sim_time:=true map_in_namespace:=true
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
- 若使用 `nav2_map_server` 加载本包 `maps/*.yaml`，请确保这些地图文件被正确安装到 `install/.../share/.../maps/`。若 `map_server` 报 `Failed processing YAML file ... bad file`，通常是 YAML 中 `image: xxx.pgm` 相对路径指向的图片在 install 目录下不存在。

