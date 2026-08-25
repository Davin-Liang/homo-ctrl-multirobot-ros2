# 4D 双积分齐次编队控制器的 Artstein-预测补偿架构

## 摘要

针对全向移动机器人编队控制中存在的指令传输时延和速度执行器响应滞后，本文档给出一种保留原始 4D 双积分齐次控制器计算结构的双层补偿架构。底层补偿层通过 Artstein 等价变换处理名义输入死区时延，并基于低阶等效速度执行器模型进行未来状态预测；上层控制层仍采用论文与 MATLAB 程序中的 4D 双积分齐次控制器。该方法不将执行器动态直接增广进 HPC 系统矩阵，因而保留 HPC 内核的双积分幂零结构和齐次权重；这不表示包含执行器的完整物理闭环严格等价于双积分系统。

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

### 1.1 记号说明

- 下标 $i\in\{1,2\}$ 分别表示 Leader 与 Follower，$p_i,v_i,u_i\in\mathbb{R}^2$ 分别为 map 系位置、速度与等效控制输入；
- $m>0$ 是将 HPC 输出缩放为等效加速度的**虚拟惯性/控制增益参数**，并非本文辨识的真实车体质量；
- $I$ 表示与所在矩阵维数匹配的单位矩阵，$A_c=A_h+B_hK$ 为线性闭环矩阵；
- $G_0,P,\nu$ 分别为 `lpc2hpc` 得到的齐次扩张矩阵、Lyapunov 矩阵和齐次度，具体构造沿用第 2 章的基础齐次控制方法；
- $h=1/f_c$ 为采样周期，$f_c$ 为控制频率，$k\in\mathbb{Z}_{\ge0}$ 为离散采样时刻索引。

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

### 2.1 名义分析假设

为区分可严格推导的名义部分和实际实现的近似部分，以下 Artstein 推导采用：

1. `T_d` 为已知常值，且控制器保存完整的过去 `T_d` 秒输入历史；
2. `tau` 为主要工作区间内已辨识的常值等效时间常数；
3. 在每个采样周期内速度命令零阶保持，且分析的预测窗口内未触发速度、加速度或轮速饱和；
4. 当前编队偏移在相邻切换时刻之间固定，即 $d(t)=d_q$；
5. Leader 加速度、状态估计误差和未建模扰动有界。

第 4 项意味着下面的连续时间名义分析只在固定编队点的区间内成立。`tol` 逻辑导致的编队点切换属于混杂事件，切换后需重新选择 $d_q$ 并重新计算 HPC 参数。

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
v_{cmd,k+1}=v_{base,k}+h u_{hpc,k}/m
```

生成。当前实现支持两种积分基准：

预测器直接输出当前周期的 map 系速度命令，不再提供基于上一帧命令的显式欧拉积分模式。

当前实验中 `pred` 模式更稳定，因此作为默认模式。

## 4. 非理想执行器的低阶等效模型

在控制器的主要工作速度区间内，将全向底盘的速度执行器近似为“输入死区 + 一阶等效速度响应”：

```math
\dot{p}(t)=v(t),
```

```math
\dot{v}(t)=-\frac{1}{\tau}v(t)+\frac{1}{\tau}u_c(t-T_d),
```

其中 $p,v,u_c\in\mathbb{R}^2$，$x_a=[p^T,v^T]^T\in\mathbb{R}^4$；`u_c` 是发送给底盘的 map 系速度命令。矩阵形式为

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

本节模型还采用 map 系近似：将由 body 系 `cmd_vel` 旋转得到的 map 系命令视为执行器输入。上标 $m,b$ 分别表示 map 系和 body 系，$\theta$、$\omega=\dot\theta$ 分别为 Follower 的 yaw 与 yaw 角速度，$R(\theta)$ 为 body 系到 map 系的二维旋转矩阵。若 yaw 在延迟和预测窗口内变化明显，则严格的 map 系速度动力学会额外包含姿态旋转耦合项。令 $S=\begin{bmatrix}0&-1\\1&0\end{bmatrix}$，有

```math
\dot v^m=\omega S v^m-\frac{1}{\tau}v^m+
\frac{1}{\tau}R(\theta(t))u_c^b(t-T_d).
```

因此，4D 模型适用于固定航向或缓慢转向工况；在一般转向工况中，上式与简化 map 系模型之间的差异被归入有界模型扰动。这也是第 4 章引入 yaw 与 body-frame 速度的 6D 模型的原因。

## 5. Artstein 输入时延补偿

对输入时延系统定义 Artstein 状态

```math
z(t)=x_a(t)+\int_{t-T_d}^{t}\exp(A_a(t-s-T_d))B_au_c(s)\,ds.
```

在名义连续模型、已知常值 `T_d` 和完整输入历史下，该变换精确地把输入时延系统转化为无显式输入时延系统：

```math
\dot z(t)=A_a z(t)+e^{-A_aT_d}B_a u_c(t).
```

注意，变换后系统仍包含执行器矩阵 `A_a` 中的 `-1/tau` 模态，并非原始 4D 双积分器。实现中使用历史 `cmd_vel` 缓冲进行离散积分：

```math
z_k\approx x_{a,k}+\sum_{j=0}^{N-1}w_j\exp(A_a(jh-T_d))B_au_{c,k-j},
```

其中 $w_j>0$ 是第 $j$ 个历史命令片段对应的求积权重，满足 $\sum_{j=0}^{N-1}w_j=T_d$；

```math
N=\lceil T_d/h\rceil.
```

若 `T_d=0`，积分项为零。

令积分项为 $I(t)$。由 Leibniz 求导公式，

```math
\dot I(t)=A_aI(t)+e^{-A_aT_d}B_a u_c(t)-B_a u_c(t-T_d).
```

与 $\dot{x}_a=A_ax_a+B_au_c(t-T_d)$ 相加后，延迟输入项严格抵消，从而得到上式的 $\dot z$。因此不能把该结果误写成 $\dot z=A_az+B_au_c(t)$：该式只有在 $e^{-A_aT_d}B_a=B_a$ 的特殊情形下才成立。

离散实现采用右端点矩形求积，并且 `T_d/h` 通常不是整数（例如 `0.22/0.05=4.4`）。因此 $z_k$ 是连续 Artstein 状态的数值近似；在输入有界、核函数连续且采样周期减小时，求积误差随 $h$ 减小。启动时以当前测得速度填充历史缓冲，同样只是一项启动瞬态近似。

## 6. 名义等效速度响应的未来状态预测补偿

本节处理的不是纯输入时延，而是名义等效速度执行器响应引起的速度滞后。Artstein 变换主要消除 `T_d`，而剩余相位滞后通过对无死区一阶等效执行器模型进行解析前向预测来减小。

Artstein 状态不直接送入双积分 HPC。由名义系统的解可得

```math
z(t)=\exp(-A_aT_d)x_a(t+T_d).
```

该关系对名义连续模型精确成立，不要求输入为常值。因此先做反映射：

```math
\bar{x}_a=\exp(A_aT_d)z(t).
```

进一步有

```math
\dot{\bar{x}}_a=A_a\bar{x}_a+B_au_c(t),
```

即反映射后的状态才满足输入矩阵恢复为 $B_a$ 的无显式时延名义执行器模型；它仍保留 $-1/tau$ 的一阶执行器模态。

记

```math
\bar{x}_a=[\bar{p}^T,\bar{v}^T]^T.
```

在名义连续模型下，$ar{x}_a(t)=x_a(t+T_d)$。因此 $ar v$ 的物理含义是：已经把
纯死区内排队命令的影响向前补偿后，在 $t+T_d$ 时刻的执行器速度；它不是当前 EKF 直接测到的
$v_{\mathrm{real}}(t)$，但仍保留一阶执行器尚未追上命令的滞后。

若从该预测时刻起，命令 $u_c(t)$ 在长度为一般预测窗口 $T_p>0$ 内保持常值，则无显式死区
一阶模型的解析解为

```math
\hat v(T_p)=u_c+e^{-T_p/\tau}(\bar v-u_c),
```

```math
\hat p(T_p)=\bar p+u_cT_p+
\tau(1-e^{-T_p/\tau})(\bar v-u_c).
```

第二式来自 $\hat p=\bar p+\int_0^{T_p}v(s)\,ds$，因此不能简单写成
$\bar p+T_p\hat v$：预测窗口内速度在指数变化。当前实现选择工程预测窗口
$T_p=\tau$，即预测一个主要执行器响应时间常数；这不是 Artstein 变换强制给出的唯一选择。
取 $T_p=\tau$ 后得到

```math
\hat{v}=u_c+e^{-1}(\bar{v}-u_c),
```

```math
\hat{p}=\bar{p}+u_c\tau+\tau(1-e^{-1})(\bar{v}-u_c).
```

在假设 1--3 下，反映射状态和预测状态对应的名义时标分别为

```math
\bar{x}_a(t)=x_a(t+T_d),\qquad
\hat{x}_a(t)=x_a(t+T_d+\tau).
```

因此，Artstein 层补偿纯死区 `T_d`，而本节的一阶解析解在附加的 `tau` 预测窗口内补偿名义速度响应。

完成电机响应预测后，最终送入原始 HPC 的状态为

```math
x_h=[\hat{p}^T,\hat{v}^T]^T.
```

上层 HPC 使用该预测状态，并保持原始双积分控制律的计算形式。当前实现固定由预测状态
上的 HPC 直接生成当前周期的 map 系速度命令；不再提供基于上一帧命令的额外欧拉积分模式。

连续公式中的 $u_c(t)$ 表示预测窗口内保持的命令。在线实现计算新命令前尚不知道该命令，因此以
上一控制周期最终发布、已通过速度/加速度/轮速约束并写入历史缓冲的 map 系命令近似 $u_c(t)$。
这是因果零阶保持近似；该近似以及预测窗口内的饱和都属于预测误差项，而不是连续 Artstein 等式的
严格部分。

其连续名义内部模型写为：

```math
\dot{p}_h=v_h, \qquad \dot{v}_h=a_h, \qquad A_h^2=0.
```

这里的双积分结构是 **HPC 的名义内部模型**，而不是对完整预测闭环的严格状态空间等价。实际预测状态由一阶执行器、采样保持、命令限幅和预测映射共同决定，通常不严格满足上述双积分动力学。该架构的目标是利用预测减少 HPC 所见的执行器相位滞后，而非重新证明真实执行器闭环仍为幂零双积分系统。

### 6.1 预测器失配组与原始 4D 基线的区别

将 4D Artstein 节点设为 `tau` 很小且 `Td=0`，并不等价于原始 4D 控制器。该节点仍会使用
历史中最终发布的命令作为预测式中的 $u_c$，故仍构造
$\hat x_h=[\hat p^T,\hat v^T]^T$；即使 $T_d=0$，其速度预测仍含
$e^{-T_p/\tau}$ 项。真正的无预测基线应使用 `formation_control_node`，在相同延迟 plant 下直接将
当前测量状态 $[p,v_{\mathrm{real}}]$ 送入原始 4D HPC。论文消融实验应据此区分：

```text
原始 4D + 同一延迟 plant：无预测基线
Artstein, tau/Td 匹配 plant：预测补偿组
Artstein, tau/Td 失配：预测器失配敏感性组，不是原始基线
```

### 6.2 预测误差与实用稳定性表述

定义实际未来执行器状态与预测状态之差为

```math
\tilde{x}_h(t)=x_{a,\mathrm{actual}}(t+T_d+\tau)-\hat{x}_a(t).
```

其来源可概括为

```math
\tilde{x}_h=\tilde{x}_{\tau}+\tilde{x}_{T_d}+\tilde{x}_{\mathrm{ZOH}}
+\tilde{x}_{\mathrm{sat}}+\tilde{x}_{\mathrm{frame}}+\tilde{x}_{\mathrm{obs}},
```

分别表示时间常数失配、死区时延失配或抖动、预测窗内非恒定命令、速度/加速度/轮速饱和、yaw 坐标耦合以及状态估计误差。该式是便于归因的概念性误差分解，并不声称各分量唯一或彼此正交。若这些项有界，则原始 4D HPC 面对的是名义编队误差加有界预测扰动；本文据此评价实用跟踪误差和稳定运行范围，而不把该式直接当作完整闭环的严格 ISS 证明。

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

固定编队点区间内，若将 Leader 加速度记为 $a_1=\dot v_1$，并将 $u_2$ 记为 Follower 的等效加速度控制输入，则原始名义误差还满足

```math
\dot e=A_h e+B_hu_2+\begin{bmatrix}0\\-a_1\end{bmatrix}.
```

故仅在静止或匀速 Leader 时该项为零。圆轨迹和 8 字轨迹中，匀速外推误差与 $a_1$ 一并构成有界参考扰动；实验中的稳态跟踪误差不应全部归因于 Follower 控制器。

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
\delta_r=\lVert p_f-p_l\rVert-r_{\mathrm{form}},
```

其中 $p_f,p_l,v_f,v_l$ 分别为 Follower/Leader 的 map 系位置和速度，$r$ 是从 Leader 指向 Follower 的单位径向量，$r_{\mathrm{form}}$ 是期望编队半径，$\delta_r$ 是圆外径向余量。该符号与第 2 节的编队偏移向量 $d$ 不同。相对 Leader 的向内速度定义为

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
\frac{v_{\mathrm{in}}^2}{2a_{\mathrm{brake}}}\le \delta_r.
```

解得允许的最大向内相对速度：

```math
v_{\mathrm{in,safe}}=
\max\left(0,-a_{\mathrm{brake}}T_{\mathrm{eff}}+
\sqrt{(a_{\mathrm{brake}}T_{\mathrm{eff}})^2+2a_{\mathrm{brake}}\delta_r}\right).
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

理想情况下，若 `tau` 和 `T_d` 与主要工作区间内的等效执行器响应一致、历史命令缓冲完整，Artstein 变换可精确消除名义模型中的纯输入时延。若命令在长度为 `tau` 的预测窗口内近似保持常值，则一阶解析预测给出该名义模型的未来执行器状态。HPC 以此预测状态构造控制输入，保留原始双积分 HPC 作为名义控制律。

但完整闭环不直接满足原始 4D HPC 的连续无约束双积分假设：名义 Artstein 系统仍有 `-1/tau` 执行器模态，前向预测依赖零阶保持近似，且实际实现还包含采样、速度/加速度限幅、编队点切换、径向安全层、EKF 噪声、Leader 非匀速运动、未建模摩擦和参数误差。因此本文不宣称完整闭环严格继承原始 HPC 的有限时间稳定性定理；更准确的工程结论是，在有界模型失配和外部扰动下考察闭环的实用稳定性与跟踪误差。实验中若速率饱和长期主导，执行器不再符合纯一阶响应，预测补偿不能完全抵消延迟与约束引起的相位损失。

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
  leader_vel_lpf_tau:=0.0 \
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

因此该架构适合在保留原始 4D HPC 计算结构的前提下处理实物执行器延迟；完整物理闭环的稳定性仍应按前述预测误差与实用稳定性边界解释。

## 14. 当前实验观察

1. `leader_vel_lpf_tau` 会引入 Leader 预测相位滞后，当前推荐为 `0.0`。
2. 单独提高 `hpc_c_min` 从 `0.1` 到 `0.2` 对 `cmd_vel_raw` 波动改善不明显。
4. 降低 `initial_min_lambda` 和 `switch_min_lambda` 可明显压缩 `cmd_vel_raw` 的速度幅值波动。
5. 当 `transport_delay=0` 而 `motor_tau=0.43` 时效果较好是合理的，因为名义一阶等效响应比纯时间延迟更容易预测补偿。
6. 静止 Leader、`Td=0.22 s`、`tau=0.43 s`、保守制动能力约 `0.4 m/s^2` 的场景中，原始命令链可能出现约 `0.15 m` 的径向过冲；径向制动安全层应在进入编队圆前开始撤销向内命令。仍需在完整 Gazebo/实物闭环中记录最大径向侵入量，作为最终验证。

