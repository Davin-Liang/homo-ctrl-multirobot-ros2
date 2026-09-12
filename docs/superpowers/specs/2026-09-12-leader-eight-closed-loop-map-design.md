# Map 系闭环 8 字领航轨迹设计

## 目标

新增可在 Gazebo 定位链路和动捕环境共用的闭环 8 字领航节点。节点在 `map_frame`
中跟踪参考轨迹，在定位无效或动捕状态超时后发布零速度。

## 范围

新增 `scripts/leader_eight_closed_loop_map.py` 和对应 launch 文件，并补充 README
启动说明。现有开环 `leader_eight.py` 与圆轨迹节点不改动。

## 状态输入

复用 `leader_circle_closed_loop_map.py` 的接口和保护逻辑：

- `state_source:=odom_tf`：订阅 `odom_topic`，通过 `map_frame -> odom` TF 将状态转到 map 系；
- `state_source:=mocap`：订阅 `mocap_pose_topic` 与 `mocap_twist_topic`；两者必须为 `map_frame`，并由 `mocap_state_timeout` 检查新鲜度。

第一帧完整、有效的 map 系状态固定参考原点 `p0` 并开始计时。

## 参考与控制

8 字参考以 `p0` 为交点：

```
p_ref(t) = p0 + [Ax sin(w t), Ay sin(2 w t)]
v_ref(t) = [Ax w cos(w t), 2 Ay w cos(2 w t)]
```

节点公开 `speed`，而不公开 `period`。`speed` 定义为沿曲线的恒定参考线速度。
初始化时在相位区间 `[0, 2 pi]` 对参数曲线采样并累计弧长；以
`s = speed * elapsed` 推进、按总弧长取模，并通过弧长表插值得到相位。参考速度为
曲线导数归一化后的切向量乘以 `speed`。周期自动为总弧长除以 `speed`；`speed` 为零时
轨迹保持静止。

当 `max_linear_vel >= speed` 且加速度限幅未触发时，参考速度严格恒为 `speed`。若硬件或
控制器限幅触发，实际发布命令仍可低于该值，以保证可执行性和安全性。
参考时间沿用圆轨迹节点的 `Td + tau_v` 前瞻和初始相位补偿，使启动时参考位置为
`p0`。保留既有 Artstein 积分预测、PD map 系反馈、线速度/加速度限幅、固定
`heading` 的 yaw 闭环以及 map/body 指令转换。

新参数为 `amplitude_x`、`amplitude_y` 和 `speed`；其余控制、状态源和安全参数与
`leader_circle_closed_loop_map.py` 保持同名及默认值。

## 验证

为参考函数添加独立 Python 单元测试：验证零时刻位置为原点、推导出的周期后回到原点、
解析速度模长等于 `speed`、周期后回到原点、零速度保持静止，并验证弧长反查在周期边界连续。
随后运行测试与 Python 语法编译检查。ROS/Gazebo 与动捕运行需由具备相应环境和设备时进行
集成验证。
