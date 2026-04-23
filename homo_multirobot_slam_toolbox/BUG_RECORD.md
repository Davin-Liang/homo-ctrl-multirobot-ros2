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

