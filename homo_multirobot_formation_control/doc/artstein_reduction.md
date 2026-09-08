# 4D 双积分 HPC + Artstein 等效速度执行器预测层

## 最终架构

当前实现采用两层结构：

```text
measured follower [p, v_real]
    ↓
Artstein input-delay compensation (Td)
    ↓
e^{A_a Td} back-mapping
    ↓
first-order equivalent velocity-actuator forward prediction (tau)
    ↓
x_h=[p_pred, v_pred]
    ↓
original 4D double-integrator HPC
    ↓
cmd_vel
```

HPC 核心保持论文/MATLAB 中的 4D 双积分器：

```text
p_dot = v
v_dot = a
A_h^2 = 0
```

`-1/tau` 不进入 HPC 的 `A_h`，只留在预测层，因此不会破坏齐次权重 `[2, 1]` 和幂零结构。

## 名义等效速度执行器模型

Follower 平移执行器在控制器工作速度区间内近似为：

```text
p_dot(t)      = v_real(t)
v_real_dot(t) = (v_cmd(t - Td) - v_real(t)) / tau
```

矩阵形式：

```text
x_a = [px, py, vx_real, vy_real]^T
x_a_dot = A_a x_a + B_a u(t - Td)
```

其中：

```text
A_a = [0 0  1      0
       0 0  0      1
       0 0 -1/tau  0
       0 0  0     -1/tau]

B_a = [0      0
       0      0
       1/tau  0
       0      1/tau]
```

这里的 `tau` 是**等效速度响应时间常数**，而非电机本体的机电时间常数。该低阶输入输出模型将
STM32 速度闭环、电机驱动、减速机构、轮地接触和底盘运动等综合效应近似为“纯输入死区 + 一阶
速度响应”，用于构造预测补偿层。

当供电、摩擦、负载或速度工作点变化时，等效响应可能变化；当加速度/电流/速度约束主导时，系统
更接近速率饱和而非纯指数响应。因此 `tau` 应按主要工作区间由阶跃响应辨识或调参，预测模型不应
被解释为真实电机的完整物理模型。

## Artstein + 预测映射

Artstein 变换：

```text
z(t) = x_a(t) + ∫[t-Td,t] exp(A_a(t-s-Td)) B_a u(s) ds
```

注意：`z` 不是直接可送给双积分 HPC 的物理状态。对常值输入近似，有：

```text
z(t) = exp(-A_a Td) x_a(t + Td)
```

所以实现中先做：

```text
x_bar = exp(A_a Td) z
```

再按当前命令 `u` 向前预测一个等效响应时间常数 `tau`：

```text
v_pred = u + exp(-1) (v_bar - u)
p_pred = p_bar + u tau + tau (1 - exp(-1)) (v_bar - u)
```

最终：

```text
x_h = [p_pred, v_pred]
```

Leader 当前没有可观测 `v_cmd` 历史，采用匀速外推：

```text
x_h_leader = [p_leader + (Td + tau) v_leader, v_leader]
```

## 已做数值验证

Python 数值脚本：

```bash
python3 scripts/sim_4d_hpc_artstein_compare.py --out-dir analysis/results/4d_artstein
```

已额外验证：

- `20Hz` 全离散仿真。
- 位置噪声 `sigma=0.05m`。
- 速度噪声 `sigma=0.08m/s`。
- 执行器加速度限幅 `0.25m/s^2`。
- 圆轨迹 `0.2m/s`、`0.5m/s`。
- 8 字轨迹最大速度约 `0.2m/s`、`0.5m/s`。
- C++ 4D HPC 的 `initial_min_lambda=1.0`、`switch_min_lambda=4.0` 与 Python 数值仿真对齐。

典型结果：8 字轨迹、最大速度约 `0.5m/s`、大噪声、加速度限幅 `0.25m/s^2` 时：

```text
original 4D + delay:        tail_mean_distance ≈ 1.263m
Artstein + prediction:      tail_mean_distance ≈ 0.196m
```

## ROS 入口

新架构入口：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_4d_artstein.launch.py \
  leader_ns:=/virtual_leader follower_ns:=/robot2 \
  tau:=0.43 Td:=0.22 control_rate:=20.0 \
  mass:=2.0 hpc_c_min:=0.1 initial_min_lambda:=1.0 switch_min_lambda:=4.0 \
  use_motor_delay:=true motor_tau:=0.43 transport_delay:=0.22 delay_max_accel:=0.25
```

对照原始 4D + 延迟：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower.launch.py \
  leader_ns:=/virtual_leader follower_ns:=/robot2 \
  hpc_c_min:=0.5 initial_min_lambda:=12.0 switch_min_lambda:=12.0 \
  use_motor_delay:=true motor_tau:=0.43 transport_delay:=0.22 delay_max_accel:=0.25
```
