# 轨迹记录器动捕状态源设计

## 目标

让 `record_trajectory.py` 通过 `state_source:=mocap` 直接记录动捕适配器的 map 系状态，同时保留既有 EKF+TF 记录路径。

## 状态源

- 新增 YAML/ROS 参数 `state_source`，默认 `ekf_tf`。
- `ekf_tf`：保持订阅 `/<ns>/odometry/filtered` 并通过 `map -> <ns>_odom` TF 转为 map 系的现有行为。
- `mocap`：订阅 `/<ns>/mocap/pose` (`PoseStamped`) 和 `/<ns>/mocap/twist` (`TwistStamped`)；直接使用其中的 map 系位置与线速度。
- 仅当每台机器人均已收到 pose 和 twist 后，动捕模式才开始计时与采样。
- 其他值会报告明确错误并退出。

## 输出与兼容性

- CSV、绘图和既有元数据字段不变；新增状态源和实际记录话题元数据。
- 时间轴沿用当前墙钟接收时间；不改变控制器参数等待与延迟节点查询。

## 验证

- 测试 `state_source` 的 YAML 加载、非法值拒绝，以及动捕 pose/twist 到记录数组的映射。
- README 提供动捕启动顺序和 `state_source:=mocap` 示例。
