# 4D 双积分齐次编队控制器的 Artstein-预测补偿架构

## 摘要

针对全向移动机器人编队控制中存在的指令传输时延和速度执行器响应滞后，本文档给出一种保持原始 4D 双积分齐次预测控制器结构不变的双层补偿架构。底层补偿层通过 Artstein 等价变换处理输入死区时延，并基于低阶等效速度执行器模型进行未来状态预测；上层控制层仍采用论文与 MATLAB 程序中的 4D 双积分齐次控制器。该方法避免将执行器动态直接增广进 HPC 系统矩阵，从而保持双积分系统的幂零结构和齐次权重。

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

## 4. 非理想执行器的低阶等效模型

在控制器的主要工作速度区间内，将全向底盘的速度执行器近似为“输入死区 + 一阶等效速度响应”：

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

这里的 `tau` 是**等效速度响应时间常数**，不等价于单个电机的机电时间常数，也不等价于固定加速度限制。它将底盘内部速度闭环、电机驱动、减速机构、轮地接触和车体负载等综合效应压缩为低阶输入输出近似。

真实底盘的响应会随速度工作点、供电状态、摩擦和负载改变；在加速度、电流或速度约束长期激活时，系统更接近速率饱和而非纯一阶指数响应。因此 `tau` 只应按主要工作区间由阶跃响应辨识或调参。代码参数名为 `tau`，本文以下将其解释为等效响应参数，而非真实电机物理常数。

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

## 6. 等效速度响应的未来状态预测补偿

本节处理的不是纯输入时延，而是名义等效速度执行器响应引起的速度滞后。Artstein 变换主要消除 `T_d`，而剩余相位滞后通过对无死区一阶等效执行器模型进行解析前向预测来减小。

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

在当前命令 `u_c(t)` 保持常值的近似下，继续向前预测一个等效响应时间常数 `tau`。这一预测步骤将当前速度 `v` 尚未追上 `u_c` 的名义滞后状态，映射到未来更接近命令速度的等效双积分状态。

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
radial braking safety layer (optional, default enabled)
        ↓
body-frame cmd_vel
```

## 9. 径向制动安全层

Artstein 变换和一阶前向预测用于补偿名义输入死区与执行器响应；它们不直接约束“到达编队圆前的剩余距离是否足以消除当前实际相对速度”。因此，当 Leader 静止、Follower 从圆外向内接近时，即使上层 HPC 已开始减速或给出反向命令，延迟链路和执行器仍可能维持一段向内实际速度，使轨迹短暂穿过期望半径后再恢复。

为处理这个物理制动约束，4D Artstein 节点在 map 系速度命令生成后加入可选的径向后处理层，默认由 `enable_radial_safety:=true` 启用。令

```math
r=\frac{p_f-p_l}{\lVert p_f-p_l\rVert},\qquad
d=\lVert p_f-p_l\rVert-r_{\mathrm{form}},
```

其中 `r` 是从 Leader 指向 Follower 的单位径向量，`d` 是圆外余量。相对 Leader 的向内速度定义为

```math
v_{\mathrm{in}}=\max\left(0,-(v_f-v_l)^Tr\right).
```

设保守可用制动加速度为 `a_brake`，有效延迟为 `T_eff`。未启用仿真延迟节点时：

```math
T_{\mathrm{eff}}=T_d+\tau.
```

用“延迟滑行 + 匀减速刹停”近似，安全条件为

```math
v_{\mathrm{in}}T_{\mathrm{eff}}+
\frac{v_{\mathrm{in}}^2}{2a_{\mathrm{brake}}}\le d.
```

解得允许的最大向内相对速度：

```math
v_{\mathrm{in,safe}}=
\max\left(0,-a_{\mathrm{brake}}T_{\mathrm{eff}}+
\sqrt{(a_{\mathrm{brake}}T_{\mathrm{eff}})^2+2a_{\mathrm{brake}}d}\right).
```

安全层先限制候选命令中的径向内切分量；然后检查 EKF 测得的 `v_f-v_l`。若实际向内速度已经超过 `v_in,safe`，则完全移除候选命令中剩余的径向内切分量，避免控制器在底盘已进入不可安全刹停包络后继续推向 Leader。切向分量保留，因此 Leader 绕圆时合理的切向随动不会被当作径向靠近。最小速度补偿、轮速约束和加速度约束完成后，节点把最终 body 命令旋转回 map 系并再次执行同一检查；若二次检查改写了命令，也会同步运动学约束器的上一帧命令，防止下一周期由旧的向内速度继续受限过渡。

该层不是控制障碍函数，也不改变 Artstein/HPC 的理论闭环；它是针对延迟、速率饱和和未建模制动能力的保守命令后处理。它降低越过编队圆的风险，但不能在状态估计严重滞后、制动能力估计过高、轮胎打滑或外部扰动下保证严格不越界。

`use_motor_delay:=true` 时，launch 把 `delay_max_accel` 传给内部参数 `radial_safety_max_decel`，并把实际延迟节点参数 `transport_delay + motor_tau` 传给内部的有效延迟。实际采用：

```math
a_{\mathrm{brake}}=\min(\texttt{max\_linear\_accel},
                         \texttt{delay\_max\_accel}).
```

因此延迟仿真中：

```math
T_{\mathrm{eff}}=\texttt{transport\_delay}+\texttt{motor\_tau}.
```

`use_motor_delay:=false` 时没有仿真延迟节点，使用 `T_d+\tau` 和 `max_linear_accel`。实物部署必须将它们设置为经过阶跃制动试验确认的保守值。控制器在安全层实际修改最终命令时输出 `RADIAL_SAFE` 诊断日志。

## 10. 稳定性说明

理想情况下，若 `tau` 和 `T_d` 与主要工作区间内的等效执行器响应一致、历史命令缓冲完整、速度命令在采样周期内近似常值，Artstein 变换可以补偿输入时延，一阶解析预测可以减小主要执行器相位滞后。此时 HPC 作用于映射后的等效双积分状态，原始齐次控制理论可保留在上层 HPC 中。

实际 ROS/Gazebo/实车中存在采样、速度/加速度限幅、EKF 噪声、未建模摩擦和参数误差，因此更准确的结论是有界扰动下的实用稳定性。实验中若速率饱和长期主导，执行器不再符合纯一阶响应，预测补偿不能完全抵消延迟与约束引起的相位损失。

## 11. 参数含义

| 参数 | 含义 |
| --- | --- |
| `tau` | 控制器内部用于等效速度响应预测补偿的局部时间常数 |
| `Td` | 控制器内部使用的输入死区/传输时延 |
| `motor_tau` | 仿真延迟节点施加的一阶等效速度响应时间常数 |
| `transport_delay` | 仿真延迟节点施加的名义纯传输时延 |
| `delay_max_accel` | 仿真延迟节点速度变化率限制 |
| `max_linear_accel` | 控制器发布前速度变化率限制 |
| `enable_radial_safety` | 是否启用 map 系径向制动安全层，默认 `true` |
| `radial_safety_max_decel` | 内部制动能力参数；launch 在延迟仿真时自动设为 `delay_max_accel`，通常无需手动传入 |
| `hpc_c_min` | HPC 齐次范数下界 |
| `initial_min_lambda` | 初始化反馈极点下界 |
| `switch_min_lambda` | 编队点切换后反馈极点下界 |
| `cmd_integrator_base` | 速度命令积分基准 |
| `leader_vel_lpf_tau` | Leader 测量速度低通时间常数 |

## 12. 推荐实验参数

当前 Gazebo 圆轨迹测试中较稳的一组参数为：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_4d_artstein.launch.py \
  leader_ns:=/robot1 follower_ns:=/robot2 \
  tau:=0.43 Td:=0.22 control_rate:=20.0 \
  mass:=2.0 hpc_c_min:=0.1 \
  initial_min_lambda:=1.5 switch_min_lambda:=4.0 \
  min_cmd_vel:=0.0 max_linear_accel:=0.4 \
  use_motor_delay:=true motor_tau:=0.43 transport_delay:=0.22 delay_max_accel:=0.4 \
  cmd_integrator_base:=pred leader_vel_lpf_tau:=0.0 \
  enable_radial_safety:=true
```

若只测试名义一阶等效速度响应、不测试纯指令时延，应同时设置：

```text
Td:=0.0
transport_delay:=0.0
```

## 13. 与 6D Motor HPC 的区别

6D Motor HPC 将 `v_cmd` 和 `v_real` 一起增广为状态，物理意义直接，但系统矩阵包含 `-1/tau`，破坏原始 4D 双积分链式幂零结构。本文架构将非理想执行器作为预测映射层处理，使上层 HPC 保持：

```math
A_h=\begin{bmatrix}0&I\\0&0\end{bmatrix},\qquad A_h^2=0.
```

因此该架构更适合在保留原始论文理论结构的前提下处理实物执行器延迟。

## 14. 当前实验观察

1. `cmd_integrator_base:=pred` 通常优于 `cmd`。
2. `leader_vel_lpf_tau` 会引入 Leader 预测相位滞后，当前推荐为 `0.0`。
3. 单独提高 `hpc_c_min` 从 `0.1` 到 `0.2` 对 `cmd_vel_raw` 波动改善不明显。
4. 降低 `initial_min_lambda` 和 `switch_min_lambda` 可明显压缩 `cmd_vel_raw` 的速度幅值波动。
5. 当 `transport_delay=0` 而 `motor_tau=0.43` 时效果较好是合理的，因为名义一阶等效响应比纯时间延迟更容易预测补偿。
6. 静止 Leader、`Td=0.22 s`、`tau=0.43 s`、保守制动能力约 `0.4 m/s^2` 的场景中，原始命令链可能出现约 `0.15 m` 的径向过冲；径向制动安全层应在进入编队圆前开始撤销向内命令。仍需在完整 Gazebo/实物闭环中记录最大径向侵入量，作为最终验证。

