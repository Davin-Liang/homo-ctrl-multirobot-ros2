# 双机 Launch 配置默认值设计

## 目标

使 `sim_rf2o_ekf_two_robots.launch.py` 与 `slam_toolbox_loc_two_robots.launch.py` 从 YAML 读取启动默认值，同时保留 ROS 2 launch 命令行参数覆盖能力。

## 配置位置与边界

两个新增 YAML 统一放在 `homo_multirobot_formation_control/config/`：

- `sim_rf2o_ekf_two_robots.launch.yaml`
- `slam_toolbox_loc_two_robots.launch.yaml`

它们只保存跨包联调的 launch 默认值，不搬迁 EKF、rf2o 或 slam_toolbox 的节点算法参数；这些参数继续在所属 localization/nav 包的现有配置中维护。

## 格式与覆盖规则

每个文件采用 `launch_defaults` 映射。对应 launch 在生成描述时以 `FindPackageShare("homo_multirobot_formation_control")` 找到 YAML，用其值作为每个 `DeclareLaunchArgument` 的默认值。ROS 2 launch 的既有优先级保持不变：命令行 `name:=value` 覆盖 YAML 默认值。

## 范围

定位 YAML 覆盖仿真、命名空间、TF/odom 发布策略和双机初始位姿。导航 YAML 覆盖地图、RViz、命名空间/TF frame、scan topic 和双机 map 初始位姿。两个 launch 的参数名称不变。

## 验证

测试读取 YAML 的必填键；以 `ros2 launch ... --show-args` 验证显示 YAML 默认值；为两个 launch 使用一个命令行覆盖参数，确认覆盖不影响其他默认值。
