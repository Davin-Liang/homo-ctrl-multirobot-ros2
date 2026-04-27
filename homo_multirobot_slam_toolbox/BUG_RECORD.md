# BUG_RECORD — `homo_multirobot_slam_toolbox` 联调记录

---

## 1. 调用 `/robot1/slam_toolbox/save_map` 返回 `255`，日志提示 `Failed to spin map subscription`

**现象**  
服务调用返回：

- `slam_toolbox.srv.SaveMap_Response(result=255)`

并且 slam_toolbox 终端出现：

- `Failed to spin map subscription`

**原因**  
`map_saver` 在 mapper 的命名空间下订阅 `map`（即 `/robot1/map`），但 `slam_toolbox` 实际发布成了全局 `/map`（绝对话题），导致订阅不到地图而超时失败。

**处理**  
在 `single_robot_mapping.launch.py` 中将绝对话题重映射进命名空间（示例）：

- `("/map", "map")`
- `("/map_metadata", "map_metadata")`
- `("/map_updates", "map_updates")`

重启后确认存在 `/robot1/map` 再保存即可。

验证：

```bash
ros2 topic list | grep -E '^/robot1/map$'
ros2 service call /robot1/slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/abs/path/to/my_map'}}"
```

---

## 3. 运行 `single_robot_mapping.launch.py` 后 RViz 看不到地图（但 slam_toolbox 在工作）

**现象**

- 运行：
  - `ros2 launch homo_multirobot_slam_toolbox single_robot_mapping.launch.py mapper_robot:=robot1 use_sim_time:=true`
- `slam_toolbox` 正常输出、服务存在，但 RViz 的 Map 显示器提示 `No map received`

**原因**

历史上该 launch 曾把地图话题重映射进命名空间（`/robot1/map`），从而导致 RViz 订阅 `/map` 时看不到。
当前仓库版本默认已改为发布全局 `/map`，并提供开关 `map_in_namespace` 来兼容需要 `/<ns>/map` 的场景。

**处理**

- 默认（推荐）：在 RViz 中订阅 `/map` 即可。
- 如需 `/<ns>/map`：启动时设置 `map_in_namespace:=true`，并在 RViz 中订阅 `/<mapper_robot>/map`。
- 命令行快速验证（示例）：

```bash
ros2 topic echo /map --once
```

---

## 2. `SaveMap` 请求格式报错：`name` 期望字典但传入了字符串

**现象**  
调用类似：

- `{name: '/abs/path/to/my_map'}`

报错：

- `Failed to populate field ... expected to be a dictionary but is a str`

**原因**  
当前 `slam_toolbox/srv/SaveMap` 的 `name` 类型为 `std_msgs/String`（不是 `string`）。

**处理**  
用 `data` 字段传入：

```bash
ros2 service call /robot1/slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/abs/path/to/my_map'}}"
```

---

## 4. `nav2_map_server` 加载本包地图时报 `bad file` / `Failed to load map yaml file`

**现象**

- 启动 `nav2_map_server` 时提示：
  - `Failed processing YAML file ... for reason: bad file: .../maps/<map>.yaml`

**原因**

地图 YAML 中的 `image: xxx.pgm` 是**相对路径**，需要在 **同一目录** 下能找到对应的图像文件。  
如果功能包没有把 `maps/` 目录安装到 `install/.../share/.../maps/`，即使源码目录里有 `.pgm`，在运行时也会找不到，导致 `map_server` 读取失败。

**处理**

- 确认 `install/.../share/.../maps/` 下同时存在 `.yaml` 与 `.pgm`；
- 本仓库已在 `homo_multirobot_slam_toolbox/CMakeLists.txt` 中安装 `maps/` 目录（`install(DIRECTORY ... maps ...)`）。


