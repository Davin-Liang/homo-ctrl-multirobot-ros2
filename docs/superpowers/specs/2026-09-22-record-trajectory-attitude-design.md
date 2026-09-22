# 轨迹记录器姿态与独立图表设计

## 目标

扩展 `homo_multirobot_formation_control/scripts/record_trajectory.py`，记录并展示 Leader 与 Follower 的航向角和角速度；将原先合并的检查图拆成数据类别独立的 PNG 文件。

## 数据采集与坐标约定

- 由每个状态消息的姿态四元数提取平面 yaw，原始值保持在 ROS 标准的 `[-pi, pi]` 范围。
- 记录 `twist.twist.angular.z` 为 `omega`，单位 `rad/s`。它是绕 z 轴的标量，不随平面坐标旋转而改变。
- 对 `ekf_tf` 状态源，里程计 twist 的 `vx`、`vy` 是车体坐标系速度；用同一时刻 yaw 转换为 map-frame 速度再记录。
- 对 `mocap` 状态源，现有 `TwistStamped` 的 `vx`、`vy` 已是 map-frame 值；yaw 从 `PoseStamped` 提取，`omega` 从 `TwistStamped.twist.angular.z` 读取。

## CSV 契约

保留既有位置、时间与线速度数据，并明确线速度列表示 map-frame。新增四列：

```text
leader_yaw_rad, leader_omega_rads,
follower_yaw_rad, follower_omega_rads
```

CSV 中保存未展开的原始 yaw，供 MATLAB 或其他后处理工具按需使用。

## 图表输出

不再生成单一 `check.png`。一次记录将在实验输出目录生成独立图片：

- `trajectory.png`：Leader/Follower 的 map 平面轨迹与可选理想半径参考线。
- `distance.png`：两车间距离与可选理想编队半径。
- `speed.png`：两车线速度模长。
- `map_velocity.png`：Leader/Follower 的 map-frame `vx`、`vy` 和 `omega` 跟踪曲线。
- `yaw.png`：Leader/Follower yaw 跟踪曲线。

`yaw.png` 在绘图阶段对两条曲线独立执行 unwrap，以消除跨越 `+pi/-pi` 边界产生的视觉跳变；这不改变 CSV 原始 yaw。

## 实现边界

改动限于记录器及其测试。不会改动控制器、launch 文件、既有实验配置或已有 `robot_traj` 数据。

## 验证

- 单元测试覆盖四元数 yaw 提取、车体速度到 map-frame 的转换，以及 unwrap 绘图数据的行为。
- 单元测试覆盖 CSV 表头含新增角度与角速度字段。
- 运行脚本语法检查与该包相关 Python 测试，确认未破坏现有状态源行为。
