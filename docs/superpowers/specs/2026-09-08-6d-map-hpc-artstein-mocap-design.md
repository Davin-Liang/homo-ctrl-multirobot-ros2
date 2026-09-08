# 6D Map-HPC Artstein 动捕接入设计

## 目标

为 `formation_control_node_6d_map_hpc_artstein` 增加纯动捕状态输入，并将该 target 改为默认构建。保留现有 6D Artstein Disc 的动捕适配，不修改 HOCBF 或其他节点。

## 输入模式

节点新增：

```text
state_source: ekf_tf | mocap     默认 ekf_tf
mocap_state_timeout: 0.10        默认 0.10 s
```

`ekf_tf` 保持当前 `/odometry/filtered + map -> robotN_odom TF` 路径。

`mocap` 订阅：

```text
/<robot_ns>/mocap/pose   PoseStamped, frame_id=map
/<robot_ns>/mocap/twist  TwistStamped, frame_id=map
```

## 状态映射

Map-HPC Artstein 内部维持：

```text
[x_map, y_map, yaw, vx_body, vy_body, wz]
```

动捕的 map 线速度只在节点入口变换一次：

```text
vx_body =  cos(yaw) * vx_map + sin(yaw) * vy_map
vy_body = -sin(yaw) * vx_map + cos(yaw) * vy_map
```

控制器输出仍先由 Map-HPC 产生 map 命令，再按 Follower yaw 转为 body-frame `/robot2/cmd_vel`。Artstein 平移/偏航预测器、HPC 核心和轮速约束不改。

## 安全

若任一机器人缺少 pose/twist 或本机接收时间超过 `mocap_state_timeout`：

```text
发布零 /robot2/cmd_vel
清空 controller_initialized_ 与平移/偏航命令历史
等待两台车的新鲜状态后重新初始化
```

诊断在 mocap 模式使用本机 mocap 接收时间，不使用 EKF header 时间戳。

## 构建与启动

移除 `BUILD_6D_MAP_HPC_ARTSTEIN` opt-in，Map-HPC target 默认构建。

更新现有 `formation_single_follower_6d_map_hpc_artstein.launch.py`，增加 `state_source` 和 `mocap_state_timeout`；新增 `formation_single_follower_6d_map_hpc_artstein_mocap.launch.py`，固定：

```text
state_source=mocap
use_sim_time=false
use_motor_delay=false
```

## 验证

1. 常规编译产生 `formation_control_node_6d_map_hpc_artstein`。
2. `vrpn_test_server -> mocap_two_robots.launch.py -> Map-HPC mocap launch` 产生非零 `/robot2/cmd_vel`。
3. 停止 mocap 状态链路后，最多 0.10 s 内 `/robot2/cmd_vel` 为零。
