# 4D 双积分齐次编队控制器的 Artstein-预测补偿架构

## 摘要

针对全向移动机器人编队控制中存在的指令传输时延和电机一阶响应滞后问题，本文档给出一种保持原始 4D 双积分齐次预测控制器结构不变的双层补偿架构。底层执行器补偿层通过 Artstein 等价变换处理输入死区时延，并通过一阶执行器模型进行未来状态预测；上层控制层仍采用论文与 MATLAB 程序中的 4D 双积分齐次控制器。该方法避免将电机状态直接增广进 HPC 系统矩阵，从而保持双积分系统的幂零结构和齐次权重。

对应实现：

```text
include/homo_multirobot_formation_control/homo_controller.hpp
include/homo_multirobot_formation_control/homo_controller_4d_artstein.hpp
src/formation_control_node_4d_artstein.cpp
launch/formation_single_follower_4d_artstein.launch.py
```

## 1. 原始 4D 双积分模型

原始 4D 质点状态定义为

```math
x_i = [p_{x,i}, p_{y,i}, v_{x,i}, v_{y,i}]^T
    = [p_i^T, v_i^T]^T \in \mathbb{R}^4 .
```

双积分动力学为

```math
\dot{x}_i = A_h x_i + B_h u_i,
```

其中

```math
A_h =
\begin{bmatrix}
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0
\end{bmatrix},
\qquad
B_h =
\begin{bmatrix}
0 & 0 \\
0 & 0 \\
1/m & 0 \\
0 & 1/m
\end{bmatrix}.
```

因此

```math
A_h^2=0.
```

数学控制输入 `u_i=[u_x,u_y]^T` 是力或等效控制输入，实际加速度为

```math
a_i = u_i/m.
```

ROS 全向底盘最终接收的是速度命令 `cmd_vel`，因此等效力需要经过离散积分映射为速度命令。

## 2. Leader-Follower 编队误差

Leader 状态和 Follower 状态分别为

```math
x_1=[p_1^T,v_1^T]^T, \qquad x_2=[p_2^T,v_2^T]^T.
```

给定期望编队偏移

```math
d=[d_x,d_y,0,0]^T,
```

定义误差

```math
e=x_2-x_1-d.
```

原始离散多边形编队在 Leader 周围半径 `radius` 的圆上生成 `m_p` 个候选编队点，并根据最近距离选择当前编队偏移。参数 `tol` 用于抑制频繁切换。

## 3. 线性反馈与齐次化控制律

首先构造线性反馈矩阵

```math
K \in \mathbb{R}^{2\times4}
```

使

```math
A_c=A_h+B_hK
```

为 Hurwitz。当前实现中，极点尺度由 `initial_min_lambda` 和 `switch_min_lambda` 控制。

线性控制为

```math
u_{lin}=Ke.
```

HPC 通过 `lpc2hpc` 由线性闭环得到齐次扩张矩阵 `G_0`、Lyapunov 矩阵 `P` 和齐次度 `nu`，并定义

```math
G_d=I+\nu G_0.
```

齐次范数为

```math
n_x=\operatorname{hnorm}(e,G_d,P).
```

实际控制中使用下界限制

```math
c=\operatorname{clamp}(n_x,c_{min},1),
```

其中 `c_min` 对应参数 `hpc_c_min`。

HPC 控制律为

```math
u_{hpc}=c^{1+\nu}K\exp(G_d(1-\ln c))e.
```

离散周期 `h=1/f_c` 下，速度命令由

```math
v_{cmd,k+1}=v_{base,k}+h\nu_{hpc,k}/m
```

生成。当前实现支持两种积分基准：

```text
cmd_integrator_base:=pred  # v_base = v_pred
cmd_integrator_base:=cmd   # v_base = v_cmd_prev
```

当前实验中 `pred` 模式更稳定，因此作为默认模式。

## 4. 非理想执行器模型

实物全向车速度执行器建模为输入死区和一阶滞后：

```math
\dot{p}(t)=v(t),
```

```math
\dot{v}(t)=-\frac{1}{\tau}v(t)+\frac{1}{\tau}u_c(t-T_d),
```

其中 `u_c` 是发送给底盘的速度命令。矩阵形式为

```math
\dot{x}_a(t)=A_ax_a(t)+B_au_c(t-T_d),
```

```math
A_a=
\begin{bmatrix}
0&0&1&0\\
0&0&0&1\\
0&0&-1/\tau&0\\
0&0&0&-1/\tau
\end{bmatrix},
\qquad
B_a=
\begin{bmatrix}
0&0\\
0&0\\
1/\tau&0\\
0&1/\tau
\end{bmatrix}.
```

如果直接把 `A_a` 增广到 HPC 中，则 `-1/tau` 会破坏原始双积分幂零结构。新架构因此不修改 HPC 核心，而是将执行器补偿作为状态映射层。

这里的 `tau` 是一阶近似模型参数，不等价于固定加速度限制。真实底盘受电机控制器、摩擦、
电池电压和速度指令幅值影响，可能表现为随速度指令变化的 `tau_eff`；因此 `tau` 应按主要工作区间辨识或调参。

## 5. Artstein 输入时延补偿

对输入时延系统定义 Artstein 状态

```math
z(t)=x_a(t)+\int_{t-T_d}^{t}\exp(A_a(t-s-T_d))B_au_c(s)\,ds.
```

该变换把输入时延系统转化为无显式输入时延系统。实现中使用历史 `cmd_vel` 缓冲进行离散积分：

```math
z_k\approx x_{a,k}+\sum_{j=0}^{N-1}w_j\exp(A_a(jh-T_d))B_au_{c,k-j},
```

其中

```math
N=\lceil T_d/h\rceil.
```

若 `T_d=0`，积分项为零。

## 6. 电机响应滞后的未来状态预测补偿

本节处理的不是纯输入时延，而是由一阶电机响应时间常数 `tau` 引起的速度滞后。Artstein 变换主要消除 `T_d`，而电机响应滞后通过对无死区一阶执行器模型进行解析前向预测来补偿。

Artstein 状态不直接送入双积分 HPC。对常值输入近似，有

```math
z(t)=\exp(-A_aT_d)x_a(t+T_d).
```

因此先做反映射：

```math
\bar{x}_a=\exp(A_aT_d)z(t).
```

记

```math
\bar{x}_a=[\bar{p}^T,\bar{v}^T]^T.
```

在当前命令 `u_c(t)` 保持常值的近似下，继续向前预测一个电机时间常数 `tau`。这一预测步骤就是电机响应延迟补偿的核心：它把当前真实速度 `v` 尚未追上 `u_c` 的滞后状态，映射到未来更接近命令速度的等效双积分状态。

```math
\hat{v}=u_c+e^{-1}(\bar{v}-u_c),
```

```math
\hat{p}=\bar{p}+u_c\tau+\tau(1-e^{-1})(\bar{v}-u_c).
```

完成电机响应预测后，最终送入原始 HPC 的状态为

```math
x_h=[\hat{p}^T,\hat{v}^T]^T.
```

此时上层 HPC 仍然面对双积分结构：

```math
\dot{p}_h=v_h, \qquad \dot{v}_h=a_h, \qquad A_h^2=0.
```

## 7. Leader 预测

Leader 无可用速度命令历史，当前采用匀速外推：

```math
x_{h,1}(t)=
\begin{bmatrix}
p_1(t)+(T_d+\tau)v_1(t)\\
v_1(t)
\end{bmatrix}.
```

参数 `leader_vel_lpf_tau` 可对 Leader 测量速度做低通滤波。但圆轨迹测试表明，该参数过大会引入相位滞后，因此当前推荐值为 `0.0`。

## 8. 完整控制结构

```text
Follower odom x_a=[p, v_real]
        + historical cmd_vel buffer
        ↓
Artstein transform z
        ↓
back mapping exp(A_a Td) z
        ↓
motor response compensation: forward prediction over tau
        ↓
x_h=[p_pred, v_pred]
        ↓
original 4D double-integrator HPC
        ↓
force-like input u_hpc
        ↓
Euler integration through B_h
        ↓
map-frame velocity command
        ↓
body-frame cmd_vel
```

## 9. 稳定性说明

理想情况下，若 `tau` 和 `T_d` 与实际执行器一致、历史命令缓冲完整、速度命令在采样周期内近似常值，Artstein 变换可以补偿输入时延，一阶解析预测可以补偿主要电机滞后相位。此时 HPC 作用于映射后的等效双积分状态，原始齐次控制理论可保留在上层 HPC 中。

实际 ROS/Gazebo/实车中存在采样、限幅、EKF 噪声、未建模摩擦和参数误差，因此更准确的结论是有界扰动下的实用稳定性。实验中若速度和加速度限幅长期饱和，则预测补偿不能完全抵消延迟。

## 10. 参数含义

| 参数 | 含义 |
| --- | --- |
| `tau` | 控制器内部用于电机响应滞后预测补偿的一阶时间常数 |
| `Td` | 控制器内部使用的输入死区/传输时延 |
| `motor_tau` | 仿真延迟节点施加的真实电机响应时间常数 |
| `transport_delay` | 仿真延迟节点施加的真实纯传输时延 |
| `delay_max_accel` | 仿真延迟节点速度变化率限制 |
| `max_linear_accel` | 控制器发布前速度变化率限制 |
| `hpc_c_min` | HPC 齐次范数下界 |
| `initial_min_lambda` | 初始化反馈极点下界 |
| `switch_min_lambda` | 编队点切换后反馈极点下界 |
| `cmd_integrator_base` | 速度命令积分基准 |
| `leader_vel_lpf_tau` | Leader 测量速度低通时间常数 |

## 11. 推荐实验参数

当前 Gazebo 圆轨迹测试中较稳的一组参数为：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_4d_artstein.launch.py \
  leader_ns:=/robot1 follower_ns:=/robot2 \
  tau:=0.43 Td:=0.22 control_rate:=20.0 \
  mass:=2.0 hpc_c_min:=0.1 \
  initial_min_lambda:=1.5 switch_min_lambda:=4.0 \
  min_cmd_vel:=0.0 max_linear_accel:=0.5 \
  use_motor_delay:=true motor_tau:=0.43 transport_delay:=0.22 delay_max_accel:=0.5 \
  cmd_integrator_base:=pred leader_vel_lpf_tau:=0.0
```

若只测试电机一阶滞后、不测试纯指令时延，应同时设置：

```text
Td:=0.0
transport_delay:=0.0
```

## 12. 与 6D Motor HPC 的区别

6D Motor HPC 将 `v_cmd` 和 `v_real` 一起增广为状态，物理意义直接，但系统矩阵包含 `-1/tau`，破坏原始 4D 双积分链式幂零结构。本文架构将非理想执行器作为预测映射层处理，使上层 HPC 保持：

```math
A_h=\begin{bmatrix}0&I\\0&0\end{bmatrix},\qquad A_h^2=0.
```

因此该架构更适合在保留原始论文理论结构的前提下处理实物执行器延迟。

## 13. 当前实验观察

1. `cmd_integrator_base:=pred` 通常优于 `cmd`。
2. `leader_vel_lpf_tau` 会引入 Leader 预测相位滞后，当前推荐为 `0.0`。
3. 单独提高 `hpc_c_min` 从 `0.1` 到 `0.2` 对 `cmd_vel_raw` 波动改善不明显。
4. 降低 `initial_min_lambda` 和 `switch_min_lambda` 可明显压缩 `cmd_vel_raw` 的速度幅值波动。
5. 当 `transport_delay=0` 而 `motor_tau=0.43` 时效果较好是合理的，因为一阶滞后比纯时间延迟更容易补偿。

