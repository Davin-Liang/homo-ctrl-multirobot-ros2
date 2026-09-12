# 4D Artstein Leader 命令前馈设计

## 目标

为 `formation_control_node_4d_artstein` 加入可开关的 Leader 命令前馈。Follower 订阅 Leader 的既有 `/cmd_vel`，不改 Leader 节点，也不引入 `AccelStamped` 接口。

## 输入与估计

启用时，Follower 订阅 `<leader_ns>/cmd_vel` 的 `geometry_msgs/msg/Twist`。该消息是 Leader 车体系速度命令；控制周期中使用最新 Leader yaw 转换为 map 系速度。仅在收到新消息时，以实际接收时间间隔差分得到命令加速度；首帧加速度为零。

原始差分加速度经过一阶低通和二维模长限幅。消息在 `leader_cmd_timeout` 内未更新时，前馈加速度置零。`Twist` 没有时间戳，因此超时和差分间隔都使用 Follower 节点的 ROS 时钟接收时间。

## 控制链路

当 `enable_leader_cmd_feedforward=false`（默认）时，所有新增状态不影响原控制输出。

当开关开启且命令新鲜时，在 map 系、径向安全约束之前叠加一个控制周期的速度增量：

```
out_map += a_leader_cmd_map_filtered * (1 / control_rate)
```

随后沿用既有径向安全、车体系速度限幅、最小速度补偿、轮速/加速度约束；最终发布的 Follower 命令仍回写 Artstein 历史缓冲。

## 参数

- `enable_leader_cmd_feedforward`：默认 `false`。
- `leader_cmd_timeout`：默认 `0.15 s`，过期时前馈归零。
- `leader_cmd_accel_lpf_tau`：默认 `0.10 s`；`0` 关闭滤波。
- `leader_cmd_accel_max`：默认 `0.60 m/s²`，map 系二维加速度模长上限。

这些参数写入 YAML，并作为 launch 参数暴露。诊断每秒输出前馈开关、命令年龄、估计加速度和前馈速度增量。

## 验证

添加控制节点的 C++ 单元测试，覆盖：开关关闭时无前馈、车体系 x 速度在 yaw=pi/2 时转换到 map-y、新鲜两帧命令生成正确差分并限幅、命令超时后前馈为零。随后在 workspace 根目录构建 `homo_multirobot_formation_control`，运行对应测试。Gazebo 联调使用 `enable_leader_cmd_feedforward:=true`，并观察诊断与 `/robot2/cmd_vel`。
