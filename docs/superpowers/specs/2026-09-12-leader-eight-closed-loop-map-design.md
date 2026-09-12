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

节点公开 `speed`，而不公开 `period`。`speed` 定义为参考线速度峰值；根据幅值自动计算：

```
w = speed / sqrt(Ax^2 + 4 Ay^2)
period = 2 pi / w  # speed 为零时轨迹保持静止
```

这样任一参考时刻的线速度均不超过 `speed`，且语义与闭环圆轨迹的 `speed` 一致。
参考时间沿用圆轨迹节点的 `Td + tau_v` 前瞻和初始相位补偿，使启动时参考位置为
`p0`。保留既有 Artstein 积分预测、PD map 系反馈、线速度/加速度限幅、固定
`heading` 的 yaw 闭环以及 map/body 指令转换。

新参数为 `amplitude_x`、`amplitude_y` 和 `speed`；其余控制、状态源和安全参数与
`leader_circle_closed_loop_map.py` 保持同名及默认值。

## 验证

为参考函数添加独立 Python 单元测试：验证零时刻位置为原点、推导出的周期后回到原点、
解析速度正确且其模长不超过 `speed`。随后运行测试与 Python 语法编译检查。ROS/Gazebo
与动捕运行需由具备相应环境和设备时进行集成验证。
