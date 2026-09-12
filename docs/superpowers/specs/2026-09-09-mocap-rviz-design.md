# 动捕双车启动 RViz2 设计

## 目标

运行 `mocap_two_robots.launch.py` 时自动启动 RViz2，并加载本包维护的双车动捕可视化配置。

## 设计

- 在 `homo_multirobot_localization/rviz/mocap_two_robots.rviz` 保存配置。
- Launch 通过 `FindPackageShare` 解析该配置的安装后路径。
- Launch 无条件启动 `rviz2`，以 `-d` 参数加载该配置；不提供 `use_rviz` 开关。
- 确认 Python 安装规则会把 `rviz/` 目录安装至包的 share 路径。

## 范围外

- 不修改 VRPN bridge、动捕状态适配器、话题和 TF 命名。
- 不改变既有 launch 参数。

## 验收

- `ros2 launch homo_multirobot_localization mocap_two_robots.launch.py ...` 会同时启动 RViz2。
- RViz2 从包安装目录读取 `mocap_two_robots.rviz`。
- Python launch 文件可通过语法检查，包可在工作空间根目录完成构建。
