# 6D Map-HPC Artstein 动捕接入设计

## 目标

将 `formation_control_node_6d_map_hpc_artstein` 的单一固定 `offset_map_x/y` 编队方案替换为 map 固定离散多边形，并增加纯动捕状态输入。该 target 改为默认构建。保留现有 6D Artstein Disc 的动捕适配，不修改 HOCBF 或其他节点。

## map 固定离散多边形

移除 `offset_map_x`、`offset_map_y` 参数，改用：

```text
m_p       候选顶点数，必须 >= 1
radius    Leader 到 Follower 的期望半径，必须 > 0
tol       顶点切换迟滞阈值，必须 >= 0
```

候选偏置固定在全局 map 坐标系：

```text
d_j = radius * [cos(2*pi*j/m_p), sin(2*pi*j/m_p)]
```

每个控制周期用当前 Leader map 位置加 `d_j` 形成候选 Follower 目标点。控制器选择当前位置误差最小的候选点；仅当新点的误差比当前点至少小 `tol` 时切换，避免临界位置反复跳点。Leader yaw 不影响候选点方向。

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

控制器把当前选择的 map 固定偏置送入 Map-HPC 误差计算，输出 map 命令后再按 Follower yaw 转为 body-frame `/robot2/cmd_vel`。Artstein 平移/偏航预测器、HPC 核心和轮速约束不改。

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

1. 单元测试验证 `m_p=4/radius=2` 生成四个 map 固定顶点，且 `tol` 阻止不充分收益的切换。
2. 常规编译产生 `formation_control_node_6d_map_hpc_artstein`。
3. `vrpn_test_server -> mocap_two_robots.launch.py -> Map-HPC mocap launch` 产生非零 `/robot2/cmd_vel`。
4. 停止 mocap 状态链路后，最多 0.10 s 内 `/robot2/cmd_vel` 为零。
