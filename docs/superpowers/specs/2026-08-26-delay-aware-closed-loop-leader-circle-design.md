# 延迟感知闭环 Leader 圆轨迹设计

## 目标

新增一个实物/仿真均可使用的闭环 Leader 圆轨迹节点。节点以 EKF/odom 为反馈，保持指定固定航向，并通过纯传输死区和一阶速度响应预测降低开环速度指令引起的轨迹漂移。

## 节点边界

- 新节点：`leader_circle_closed_loop.py`。
- 输入：同一命名空间下的 `odometry/filtered`，默认相对话题名为 `odometry/filtered`。
- 输出：同一命名空间下的 `cmd_vel`。
- 参考轨迹坐标与 odometry 消息的 `header.frame_id` 相同；节点不混用 `map` 与 `odom`。
- `heading` 表示固定目标 yaw，单位为度。
- `start_side` 指定第一帧 odometry 位姿作为圆最上端（`top`）或最下端（`bottom`）。
- launch 文件 `leader_circle_closed_loop_with_delay.launch.py` 同时启动闭环 Leader 和 `sim_motor_delay.py`，形成 `cmd_vel_raw → cmd_vel` 延迟链路。

## 参考轨迹

收到第一帧有效 odometry 时记录 $p_0$。令初始相位 $\phi_0=\pi/2$ 对应 `top`，$\phi_0=3\pi/2$ 对应 `bottom`，圆心为：

```math
c=p_0-R[\cos\phi_0,\sin\phi_0]^T.
```

取 $\omega_d=\pm v/R$、$\phi(t)=\phi_0+\omega_dt$，则：

```math
p_d(t)=c+R[\cos\phi(t),\sin\phi(t)]^T,
```

```math
v_d(t)=\operatorname{sgn}(\omega_d)v[-\sin\phi(t),\cos\phi(t)]^T.
```

参考轨迹从 $p_0$ 连续起步；起始切向方向由 `direction` 与 `start_side` 共同确定。

## 延迟感知外环

使用参数 `Td` 与 `tau_v`，定义：

```math
T_{\mathrm{look}}=T_d+\tau_v.
```

节点维护发布后的最终 map 系速度命令历史。为保证第一个前瞻目标仍位于指定的起点，参考相位预先回退 $\omega_dT_{\mathrm{look}}$；因此在 $t=0$ 时，$p_d(t+T_{\mathrm{look}})=p_0$。采用当前测得 map 系速度 $v^m$、历史命令和一阶执行器模型构造预测状态 $\hat p,\hat v$，并与前瞻参考 $p_d(t+T_{\mathrm{look}}),v_d(t+T_{\mathrm{look}})$ 比较：

```math
v_{\mathrm{cmd}}^m=
v_d(t+T_{\mathrm{look}})
-K_p(\hat p-p_d(t+T_{\mathrm{look}}))
-K_v(\hat v-v_d(t+T_{\mathrm{look}})).
```

这不是 Follower 编队控制器，也不使用其 HPC；它是用于产生可重复实物 Leader 参考运动的延迟感知轨迹跟踪外环。

## 固定航向与约束

使用实际测量 yaw 把 map 系线速度转换到 body 系：

```math
v_{\mathrm{cmd}}^b=R(-\theta)v_{\mathrm{cmd}}^m.
```

固定航向外环：

```math
\omega_{\mathrm{cmd}}=
\operatorname{sat}(K_\theta\operatorname{wrap}(\theta_d-\theta)).
```

在发布前对 map 系线速度模长、body 系角速度以及命令变化率施加限制。发布后的最终 body 命令需旋转回 map 系并写入历史缓冲，保证预测器使用真实已发布命令。

## 参数与默认值

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `radius` | 2.0 m | 圆轨迹半径 |
| `speed` | 0.2 m/s | 切向参考速度 |
| `heading` | 0.0 deg | 固定目标航向 |
| `direction` | `ccw` | 圆周方向 |
| `start_side` | `top` | `top`=圆最上端，`bottom`=圆最下端 |
| `rate` | 20 Hz | 控制频率 |
| `odom_topic` | `odometry/filtered` | 反馈里程计话题 |
| `Td` | 0.22 s | 等效纯输入死区 |
| `tau_v` | 0.43 s | 等效平移速度响应时间常数 |
| `kp` | 0.8 | 位置反馈增益 |
| `kv` | 0.2 | 速度反馈增益 |
| `k_yaw` | 1.5 | 固定航向反馈增益 |
| `max_linear_vel` | 0.4 m/s | map 系线速度模长上限 |
| `max_linear_accel` | 0.25 m/s² | 线速度变化率上限 |
| `max_angular_vel` | 0.8 rad/s | 角速度上限 |
| `max_angular_accel` | 1.0 rad/s² | 角速度变化率上限 |

## 验证

1. 单元级验证参考轨迹在 $t=0$ 从记录的 $p_0$ 起步。
2. 验证 map/body 旋转的正反变换互逆。
3. 验证固定 yaw 误差产生正确符号的 `angular.z` 命令。
4. 验证发布后的限幅命令被写回预测历史。
5. 在仿真中记录实际轨迹、参考轨迹、位置误差、yaw 误差、命令与实际速度。
6. 在实物低速大半径工况下比较开环 `leader_circle.py` 与新节点的圆度和闭环误差。
