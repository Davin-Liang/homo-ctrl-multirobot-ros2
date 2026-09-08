# 6D Artstein Disc 动捕接入设计

## 目标

让现有 `formation_control_node_6d_artstein_disc` 支持纯动捕状态输入，同时保持默认 EKF/TF 定位路径、6D Artstein 预测器、HPC 核心、轮速约束和 HOCBF 节点不变。

## 接口

节点新增参数：

```text
state_source: ekf_tf | mocap     默认 ekf_tf
mocap_state_timeout: 0.10        默认 0.10 s
```

`ekf_tf` 保留当前输入：

```text
/<robot_ns>/odometry/filtered
TF map -> robotN_odom
```

`mocap` 输入为：

```text
/<robot_ns>/mocap/pose   PoseStamped, frame_id=map
/<robot_ns>/mocap/twist  TwistStamped, frame_id=map
```

## 状态映射

6D Disc 的内部状态保持：

```text
[x_map, y_map, yaw, vx_body, vy_body, wz]
```

动捕 adapter 输出：

```text
[x_map, y_map, yaw, vx_map, vy_map, wz]
```

在 `mocap` 分支中只进行模型必需的速度映射：

```text
vx_body =  cos(yaw) * vx_map + sin(yaw) * vy_map
vy_body = -sin(yaw) * vx_map + cos(yaw) * vy_map
```

之后继续使用现有 `state.v_map`、平移 Artstein 预测、偏航 Artstein 预测、6D Disc HPC 与 body-frame `cmd_vel` 输出流程。

## 安全

控制器分别缓存两台车的最新 mocap pose/twist 本机接收时间。任一机器人没有 pose 或 twist，或任一消息距当前时间超过 `mocap_state_timeout`：

```text
发布零 /robot2/cmd_vel
清除 controller_initialized_ 和预测器历史
等待两台车重新收到新鲜数据后重新初始化
```

纯动捕模式不依赖 `map -> robotN_odom` TF；但 adapter 仍可发布 TF 供 RViz 和 robot_state_publisher 使用。

## Launch 与范围

更新现有 `formation_single_follower_6d_artstein_disc.launch.py`，增加 `state_source` 与 `mocap_state_timeout` 参数，默认仍为 `ekf_tf`。新增 `formation_single_follower_6d_artstein_disc_mocap.launch.py`，固定：

```text
state_source=mocap
use_sim_time=false
use_motor_delay=false
```

本次不修改 `formation_control_node_6d_artstein_disc_hocbf`、其他 6D 节点或 4D 节点。

## 验证

1. 编译 6D Artstein Disc 节点，确认默认 `ekf_tf` 路径仍可启动。
2. 使用 `vrpn_test_server -> vrpn_listener -> 两个 mocap_state_adapter`，让两个 adapter 都映射测试刚体，确认 6D mocap 节点发布非零 `/robot2/cmd_vel`。
3. 停止动捕状态链路，确认最多 0.10 s 后发布零 cmd_vel。
4. 检查 HOCBF target、其他 6D target 没有源文件改动。
