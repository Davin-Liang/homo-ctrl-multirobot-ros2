# 6D Artstein Disc 的理论说明与扩维模型证明模板

## 摘要

本文档整理拟新增 `6D Artstein Disc` 控制器的理论依据。目标控制器采用
“平移 4D Artstein 预测 + 偏航 2D Artstein 预测 + 6D Disc HPC”的方向 A
架构：

```text
measured state
  -> map-frame translation Artstein + forward prediction
  -> yaw Artstein + forward prediction
  -> predicted 6D Disc feedback state
  -> 6D Disc generalized homogeneous controller
  -> cmd_vel
```

该架构的关键原则是：执行器死区和低阶等效速度响应只作为 HPC 外层的状态预测补偿，
不把执行器动态直接增广进 6D HPC 核心。HPC 层仍使用 6D Disc 的离散编队点、
leader 车体系误差和 `lpc2hpc_nd` 齐次升级。

对应数值仿真脚本：

```text
scripts/sim_6d_disc_artstein_compare.py
```

默认结果输出：

```text
homo_multirobot_formation_control/analysis/results/6d_artstein_disc/
```

## 1. 为什么不直接做 6x3 Artstein kernel

6D Disc 的状态为

```math
x=[p_x,p_y,\theta,v_x^b,v_y^b,\omega]^T.
```

其中位置和偏航角在 map 系，平移速度在车体系。位置动力学为

```math
\dot p =
R(\theta)
\begin{bmatrix}
v_x^b\\
v_y^b
\end{bmatrix}.
```

展开为

```math
\dot p_x=\cos\theta\,v_x^b-\sin\theta\,v_y^b,
```

```math
\dot p_y=\sin\theta\,v_x^b+\cos\theta\,v_y^b.
```

因此若直接把执行器模型写在
`[p_x,p_y,\theta,v_x^b,v_y^b,\omega]` 上，系统矩阵会依赖当前姿态：

```math
\dot x = A(\theta)x+B u(t-T_d).
```

而标准 Artstein reduction 针对的是常值矩阵线性输入时延系统：

```math
\dot x = Ax+B u(t-T_d).
```

所以机械扩成 `6x3` kernel 只有在短时间内冻结 `theta` 时才是局部近似。
它工程上可能可用，但理论表述不能写成严格的全局 Artstein 等价变换。

方向 A 避开这个问题：平移补偿在 map 系完成，偏航补偿单独作为 2D 标量系统完成，
最后再将预测后的 map 系速度旋转回 6D Disc 所需的车体系速度。

## 2. 平移 Artstein 预测层

平移补偿层使用 map 系状态

```math
x_p=[p_x,p_y,v_x^m,v_y^m]^T.
```

执行器命令为 map 系速度指令

```math
u_p=[v_{x,cmd}^m,v_{y,cmd}^m]^T.
```

在主要工作区间内，采用带输入死区和一阶等效速度响应的名义执行器模型：

```math
\dot x_p(t)=A_p x_p(t)+B_p u_p(t-T_d),
```

其中

```math
A_p=
\begin{bmatrix}
0&0&1&0\\
0&0&0&1\\
0&0&-1/\tau_v&0\\
0&0&0&-1/\tau_v
\end{bmatrix},
\qquad
B_p=
\begin{bmatrix}
0&0\\
0&0\\
1/\tau_v&0\\
0&1/\tau_v
\end{bmatrix}.
```

这是常值 LTI 输入时延系统，因此可使用 Artstein 变换。

定义

```math
z_p(t)=x_p(t)+
\int_{t-T_d}^{t}
e^{A_p(t-s-T_d)}B_pu_p(s)\,ds.
```

对 `z_p` 求导：

```math
\dot z_p(t)
=\dot x_p(t)+B_pu_p(t)-e^{-A_pT_d}B_pu_p(t-T_d)
+A_p\int_{t-T_d}^{t}e^{A_p(t-s-T_d)}B_pu_p(s)\,ds.
```

代入

```math
\dot x_p(t)=A_px_p(t)+B_pu_p(t-T_d)
```

后，延迟输入项抵消，得到

```math
\dot z_p(t)=A_pz_p(t)+B_pu_p(t).
```

因此 Artstein 状态满足无显式输入时延的 LTI 系统。

对无死区的一阶等效速度响应

```math
\dot v=-\frac{1}{\tau_v}v+\frac{1}{\tau_v}u
```

若在预测窗口内把 `u` 视为常值，则解析解为

```math
v(t+\tau_v)=u+e^{-1}(v(t)-u),
```

位置预测为

```math
p(t+\tau_v)
=p(t)+\tau_v u+\tau_v(1-e^{-1})(v(t)-u).
```

所以平移预测层输出

```math
\hat x_p=[\hat p_x,\hat p_y,\hat v_x^m,\hat v_y^m]^T.
```

## 3. 偏航 Artstein 预测层

偏航状态定义为

```math
x_\theta=[\theta,\omega]^T.
```

角速度命令为

```math
u_\theta=\omega_{cmd}.
```

在主要工作区间内，采用带输入死区和一阶等效角速度响应的名义模型：

```math
\dot x_\theta(t)=A_\theta x_\theta(t)+B_\theta u_\theta(t-T_d),
```

其中

```math
A_\theta=
\begin{bmatrix}
0&1\\
0&-1/\tau_\omega
\end{bmatrix},
\qquad
B_\theta=
\begin{bmatrix}
0\\
1/\tau_\omega
\end{bmatrix}.
```

同理定义

```math
z_\theta(t)=x_\theta(t)+
\int_{t-T_d}^{t}
e^{A_\theta(t-s-T_d)}B_\theta u_\theta(s)\,ds.
```

可得

```math
\dot z_\theta(t)=A_\theta z_\theta(t)+B_\theta u_\theta(t).
```

再通过一阶等效角速度模型进行前向预测，得到

```math
\hat x_\theta=[\hat\theta,\hat\omega]^T.
```

实现时需对角度误差使用 wrap 到 `[-pi, pi]` 的归一化，避免跨越 `pi` 时产生
虚假的大角度误差。

## 4. 预测状态到 6D Disc 状态

平移预测层得到 map 系速度

```math
\hat v^m=[\hat v_x^m,\hat v_y^m]^T.
```

6D Disc 需要车体系速度，因此使用预测偏航角旋转：

```math
\hat v^b=R(-\hat\theta)\hat v^m.
```

最终送入 6D Disc HPC 的状态为

```math
\hat x=
[\hat p_x,\hat p_y,\hat\theta,\hat v_x^b,\hat v_y^b,\hat\omega]^T.
```

此时补偿层的作用是让 HPC 看到近似无输入死区、相位滞后更小的等效反馈状态。
在 `T_d`、`tau_v`、`tau_\omega` 与主要工作区间内的等效执行器响应匹配，命令历史完整，
采样周期足够小且命令在预测窗口内近似常值时，该状态预测与名义执行器模型一致。`tau_v`
和 `tau_\omega` 是局部等效响应参数，而非真实电机物理时间常数；当速率饱和或轮地接触非线性
主导时，该低阶近似的误差会增大。

## 5. 6D Disc 误差系统与齐次升级条件

### 5.1 车体级 6D 状态方程

当前 6D Disc / 6D Artstein Disc 的 HPC 状态为

```math
x=[p_x,p_y,\theta,v_x^b,v_y^b,\omega]^T.
```

其中 `p_x,p_y,theta` 在 map 系，`v_x^b,v_y^b,omega` 是车体系速度。
这是一种 6D 车体级混合坐标二阶模型：位姿部分满足平面刚体运动学，
速度部分采用广义力/力矩输入的二阶积分近似。它不包含三轮全向轮的轮速状态，
也不是纯一阶运动学模型。

车体级位姿运动学为

```math
\dot p_x=\cos\theta\,v_x^b-\sin\theta\,v_y^b,
```

```math
\dot p_y=\sin\theta\,v_x^b+\cos\theta\,v_y^b,
```

```math
\dot\theta=\omega.
```

HPC 内部不直接以 `cmd_vel` 作为控制输入，而是先计算广义力/力矩

```math
u=[F_x,F_y,M_z]^T.
```

在控制器内部采用二阶近似：

```math
\dot v_x^b=\frac{1}{m}F_x,
```

```math
\dot v_y^b=\frac{1}{m}F_y,
```

```math
\dot\omega=\frac{1}{I}M_z.
```

因此完整车体级混合状态方程可写为

```math
\begin{cases}
\dot p_x=\cos\theta\,v_x^b-\sin\theta\,v_y^b,\\
\dot p_y=\sin\theta\,v_x^b+\cos\theta\,v_y^b,\\
\dot\theta=\omega,\\
\dot v_x^b=F_x/m,\\
\dot v_y^b=F_y/m,\\
\dot\omega=M_z/I.
\end{cases}
```

离散实现中，控制器输出的广义力/力矩会通过前向 Euler 积分转成 `cmd_vel`：

```math
v_{x,cmd}^b=v_x^b+hF_x/m,
```

```math
v_{y,cmd}^b=v_y^b+hF_y/m,
```

```math
\omega_{cmd}=\omega+hM_z/I.
```

随后节点发布

```text
cmd_vel.linear.x  = v_{x,cmd}^b
cmd_vel.linear.y  = v_{y,cmd}^b
cmd_vel.angular.z = omega_cmd
```

轮级运动学没有进入上述状态方程。轮半径、底盘半径和最大轮速只在
`KinematicConstraint` 中作为输出约束，对最终 `cmd_vel` 做缩放和加速度限幅。

### 5.2 Leader 车体系误差标量方程

6D Disc 使用 leader 车体系误差。固定离散编队点 `d_i` 后，定义

```math
e=
\begin{bmatrix}
e_x^L\\
e_y^L\\
e_\theta\\
e_{v_x}^L\\
e_{v_y}^L\\
e_\omega
\end{bmatrix}.
```

其中位置误差在 leader 车体系：

```math
e_p^L=R(-\theta_l)(p_f-p_l)-d_p.
```

更具体地，令

```math
d_i=[d_x,d_y,d_\theta,d_{v_x},d_{v_y},d_\omega]^T,
```

则误差分量为

```math
e_x=(p_{x,f}-p_{x,l})\cos\theta_l+(p_{y,f}-p_{y,l})\sin\theta_l-d_x,
```

```math
e_y=-(p_{x,f}-p_{x,l})\sin\theta_l+(p_{y,f}-p_{y,l})\cos\theta_l-d_y,
```

```math
e_\theta=\operatorname{wrap}(\theta_f-\theta_l-d_\theta),
```

```math
e_{v_x}=v_{x,f}^b\cos e_\theta-v_{y,f}^b\sin e_\theta-v_{x,l}^b-d_{v_x},
```

```math
e_{v_y}=v_{x,f}^b\sin e_\theta+v_{y,f}^b\cos e_\theta-v_{y,l}^b-d_{v_y},
```

```math
e_\omega=\omega_f-\omega_l-d_\omega.
```

对该误差求导时，`R(-theta_l)` 随 leader 偏航变化，引入 leader twist 耦合。
若当前离散编队偏移 `d_i` 非零，leader 系旋转还会产生与 `omega_l d_i` 相关的附加项；
本文将这类项归入扰动 `w`，nominal 矩阵只保留关于误差 `e` 的线性部分。
若在一次 HPC 参数计算内冻结当前 leader 速度

```math
v_{x,l},v_{y,l},\omega_l=\text{const},
```

则局部误差系统可写成

```math
\dot e=A_L e+B_6u+w.
```

无扰动的 nominal 部分为

```math
\dot e=A_L e+B_6u.
```

展开成标量形式为

```math
\dot e_x=\omega_l e_y-v_{y,l}e_\theta+e_{v_x},
```

```math
\dot e_y=-\omega_l e_x+v_{x,l}e_\theta+e_{v_y},
```

```math
\dot e_\theta=e_\omega,
```

```math
\dot e_{v_x}=F_x^L/m,
```

```math
\dot e_{v_y}=F_y^L/m,
```

```math
\dot e_\omega=M_z/I.
```

其中 `F_x^L,F_y^L` 是在 leader 车体系下计算出的平移广义力。实际发布前，
控制器会把平移控制从 leader 车体系旋转到 follower 车体系，再积分为
`v_x^b,v_y^b` 命令。

当前 6D Disc 使用的冻结矩阵为

```math
A_L=
\begin{bmatrix}
0 & \omega_l & -v_{y,l} & 1 & 0 & 0 \\
-\omega_l & 0 & v_{x,l} & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix},
```

```math
B_6=
\begin{bmatrix}
0&0&0\\
0&0&0\\
0&0&0\\
1/m&0&0\\
0&1/m&0\\
0&0&1/I
\end{bmatrix}.
```

齐次升级前必须满足两个条件：

```math
\operatorname{rank}[B_6,A_LB_6,\ldots,A_L^{5}B_6]=6,
```

以及存在当前线性反馈 `K_lin` 使

```math
A_L+B_6K_{lin}
```

为 Hurwitz。工程实现中应检查

```math
\max_i \operatorname{Re}\lambda_i(A_L+B_6K_{lin})<-\varepsilon,
```

其中 `epsilon` 是稳定裕度。若检查失败，应退回上一组稳定 HPC 参数或退回线性控制。

当上述条件成立时，可调用 generalized homogeneous control 的标准构造：

1. 对 `(A_L,B_6)` 做块可控分解。
2. 构造齐次生成元 `G0`。
3. 对线性闭环解 Lyapunov 方程得到 `P`。
4. 计算可用齐次度 `nu`。
5. 构造

```math
G_d=I+\nu G_0.
```

齐次控制律可写为

```math
u_h
=c^{1+\nu}K_{lin}\exp(G_d(1-\ln c))e,
```

其中

```math
c=\operatorname{clamp}(\|e\|_d,c_{min},1).
```

从向量场角度看，冻结 nominal 闭环为

```math
\dot e=f_h(e)=A_L e+B_6u_h(e).
```

令线性膨胀为

```math
d(s)=\exp(sG_d),\qquad G_d=I+\nu G_0.
```

根据 generalized homogeneous control 的构造，`G0` 与齐次反馈项使闭环 nominal
向量场满足

```math
f_h(d(s)e)=\exp(\nu s)d(s)f_h(e),
```

即 `f_h` 是关于膨胀 `d(s)` 的 `d`-齐次向量场，齐次度为 `nu`。这一步只对
固定 `A_L`、固定 `K_lin`、无扰动、无饱和的 nominal 系统成立；真实系统中的
Artstein 参数误差、leader twist 慢变化、目标点切换和输出约束仍归入扰动项。

在无扰动、无饱和、固定 `A_L` 和固定 `d_i` 条件下，标准齐次 Lyapunov 结论给出

```math
\dot V\le -\rho V^{1+\nu}.
```

若

```math
\nu<0,
```

则 nominal 局部系统有限时间收敛。

## 6. 方向 A 的稳定性结论

方向 A 可以表述为以下命题。

**命题**：假设：

1. 平移和偏航执行器分别满足上述 LTI 输入时延模型。
2. 补偿层使用的 `T_d`、`tau_v`、`tau_\omega` 与真实执行器匹配。
3. 命令历史缓冲完整，采样周期足够小，预测窗口内命令近似常值。
4. 固定离散编队点 `d_i`，并在一次 HPC 参数计算内冻结 leader twist。
5. `(A_L,B_6)` 可控，且 `A_L+B_6K_lin` Hurwitz。
6. 不考虑速度饱和、轮速约束、加速度限幅和目标点切换。

则 Artstein 预测层将输入死区系统映射为等效无显式输入时延的反馈状态；
该状态送入 6D Disc HPC 后，nominal 局部误差系统继承 generalized homogeneous
control 的有限时间收敛性质。

真实系统中存在如下非理想项：

```text
tau/Td 参数误差
采样和离散积分误差
速度饱和、轮速约束、加速度限幅
leader twist 慢变化
非零编队偏移在旋转 leader 系中的附加项
离散编队点切换
传感器噪声
map/body 坐标转换误差
```

这些因素可合并为扰动项：

```math
\dot e=A_L e+B_6u_h+\delta(t).
```

因此真实系统不应宣称严格全局有限时间收敛。更稳妥的结论是：

```text
模型匹配和无约束的 nominal 情况下有限时间收敛；
存在采样、饱和、切换、参数失配和噪声时，系统按 ISS/实用稳定解释，
误差最终进入与扰动上界相关的有界邻域。
```

## 7. 扩维状态方程的通用证明模板

后续每新增一个扩维模型，都建议至少补充以下检查。扩维包括但不限于：

```text
6D Disc
6D Bearing
6D Motor
6D Artstein Disc
8D Pade / dead-time augmentation
6D+OA QP fusion
MPC 6D
```

### 7.1 状态定义

明确状态向量每一维的物理含义、坐标系和可测性：

```text
位置是 map 系还是 leader 车体系
速度是 map 系还是 body 系
命令速度是否为内部积分状态
电机状态是否可测
角度是否需要 wrap
```

### 7.2 动力学方程

写出完整状态方程：

```math
\dot x=f(x,u,t)
```

并说明它属于哪一类：

```text
全局 LTI:        dot x = A x + B u
冻结参数 LTI:    dot x = A(q_k)x + B(q_k)u
控制仿射非线性: dot x = f(x)+g(x)u
近似离散模型:   x_{k+1}=F(x_k,u_k)
```

只有前两类能直接使用当前 `lpc2hpc_nd` 的线性齐次升级逻辑。

### 7.3 误差系统是否闭合

定义编队误差 `e` 后，检查是否能写成

```math
\dot e=A_e e+B_eu+w.
```

如果误差方程还依赖没有纳入状态的变量，则必须：

```text
增广状态
冻结该变量
或把它归入扰动项
```

### 7.4 可控性

检查

```math
\operatorname{rank}[B,AB,\ldots,A^{n-1}B]=n.
```

如果不可控，当前 `lpc2hpc_nd` 不适用。

### 7.5 线性反馈稳定性

给出 `K_lin` 的构造，并检查

```math
\max_i\operatorname{Re}\lambda_i(A+BK_{lin})<-\varepsilon.
```

分块二阶或三阶增益在有耦合的扩维系统中不自动保证 Hurwitz。若不能解析证明，
至少应在实现中做在线特征值检查和 fallback。

### 7.6 齐次升级适用性

说明 `lpc2hpc_nd` 的使用前提：

```text
(A,B) 可控
A+B*K_lin Hurwitz
块可控分解成功
Lyapunov 方程有正定解 P
计算得到可用的负齐次度 nu
```

若 `nu` 需要缩放或 `G0` 需要缩放，应说明这是数值温和化措施，不是原始理论结论。

### 7.7 非理想因素与结论降级

最后列出实际系统中没有纳入 nominal 模型的因素：

```text
输入延迟
执行器滞后
参数误差
传感器噪声
采样
饱和
切换
避障 QP 修正
网络延迟
```

若这些因素存在，应将结论写成：

```text
nominal 模型有限时间稳定；
实际系统 ISS/实用稳定；
性能通过数值仿真、Gazebo 仿真和实物实验验证。
```

## 8. 为什么一阶执行器滞后不直接并入 6D HPC

原 6D HPC 的名义积分器链为

$$
\dot p=R(\theta)v^b,\qquad
\dot\theta=\omega,\qquad
\dot v_x^b=\frac{u_x}{m},\qquad
\dot v_y^b=\frac{u_y}{m},\qquad
\dot\omega=\frac{u_\omega}{I}.
$$

其局部冻结误差模型可按积分器链选择递减齐次权重，例如位置、速度和输入分别具有 2、1、0 阶。加入一阶执行器滞后后，平移通道变为

$$
\dot v=-\frac{1}{\tau}v+\frac{1}{\tau}u.
$$

在原权重下，$\dot v$ 与 $u$ 的缩放阶为 0，而衰减项 $-v/\tau$ 的缩放阶为 1。右端出现不同缩放阶的项，故不再满足原积分器链的齐次结构。

若增广真实加速度 $a$：

$$
\dot v=a,\qquad
\dot a=-\frac{1}{\tau}a+\frac{1}{\tau}u,
$$

即使采用 $\deg(p)=3,\deg(v)=2,\deg(a)=1,\deg(u)=0$，仍有 $\deg(\dot a)=0$ 而 $\deg(-a/\tau)=1$。因此问题不在于常数 $\tau$ 本身，而在于一阶衰减漂移项破坏了原齐次缩放。

该增广系统仍是线性、可控且可通过线性反馈稳定的，但不能直接复用原 6D HPC 的齐次权重、齐次范数、LPC 到 HPC 升级和有限时间结论。将执行器滞后直接纳入 HPC 需要重新构造 9D（或更高维含死区）模型的齐次控制器与理论证明。

本项目采用方向 A：

$$
\text{map 系平移 Artstein/执行器预测}
\;+\;
\text{yaw Artstein/执行器预测}
\;\longrightarrow\;
\text{原 6D Disc HPC}.
$$

该做法将滞后和死区留在预测补偿外层，使 HPC 核心仍工作在原 6D 名义积分器链上；完整系统的模型误差和坐标耦合作为预测残差进行数值评估，而不宣称直接 9D 齐次有限时间证明。

## 9. 本项目后续实现建议

第一版 `6D Artstein Disc` 建议按以下方式落地：

```text
include/homo_multirobot_formation_control/homo_controller_6d_artstein_disc.hpp
src/formation_control_node_6d_artstein_disc.cpp
include/homo_multirobot_formation_control/formation_control_node_6d_artstein_disc.hpp
src/main_6d_artstein_disc.cpp
launch/formation_single_follower_6d_artstein_disc.launch.py
```

实现保护：

```text
1. 平移 cmd history 使用 Vector2d(map 系)。
2. 偏航 cmd history 使用 omega_cmd 标量。
3. 预测后再转换为 6D Disc 的 body-frame velocity。
4. 每次重算 A_L/K_lin 后检查闭环 Hurwitz。
5. 若检查失败，复用上一组稳定 HPC 参数；若没有上一组稳定参数，退回线性控制。
6. 输出 summary/diagnostic: pred error, hpc fallback count, max eig real part。
```

和 4D Artstein 做公平对比时，6D 实现还需满足以下一致性约定：

```text
1. initial_min_lambda / switch_min_lambda 直接表示闭环极点尺度下界，
   不再额外乘以 mass 或 inertia。
2. HPC 模式下，K_lin 与 G0/P/Gd 必须在同一个冻结线性化条件下生成；
   若 leader twist 或相对 yaw 未触发 HPC 重建，则复用上一组 K_lin。
3. use_motor_delay:=true 时，sim_motor_delay.py 的运行频率与 4D Artstein
   保持为 100 Hz，避免延迟注入离散化频率成为额外变量。
```

当前数值仿真已验证无延迟、延迟、Artstein 预测补偿三种情况：

```text
原始 6D Disc 无延迟:       tail_mean_pos_error ≈ 0.046 m
原始 6D Disc + 延迟:       tail_mean_pos_error ≈ 0.448 m
6D Artstein Disc + 延迟:   tail_mean_pos_error ≈ 0.053 m
```

注意：上述默认数值仿真使用切向航向的 virtual leader，即 leader yaw 随圆轨迹旋转。
若要复现 Gazebo 中 `leader_circle.py` 的常用现象，应使用固定航向 leader：

```bash
python3 scripts/sim_6d_disc_artstein_compare.py \
  --leader-heading-fixed --leader-heading 0.0 --follower-yaw0 0.0
```

固定航向时，虽然 6D Disc 的编队偏移仍定义在 leader 车体系中，但
`R(theta_l)d` 变成 map 系常值偏移，因此 follower 轨迹表现为 leader 圆轨迹的
平移版本，与 4D 图像更接近。对应结果：

```text
原始 6D Disc 无延迟:       tail_mean_pos_error ≈ 0.022 m
原始 6D Disc + 延迟:       tail_mean_pos_error ≈ 0.051 m
6D Artstein Disc + 延迟:   tail_mean_pos_error ≈ 0.050 m
```

在初始 follower 航向与 leader 航向相差约 `pi/2` 的情况下，也补充验证了两组：

```text
follower_yaw0=0:
  原始 6D Disc + 延迟:       tail_mean_pos_error ≈ 0.395 m
  6D Artstein Disc + 延迟:   tail_mean_pos_error ≈ 0.053 m

follower_yaw0=pi:
  原始 6D Disc + 延迟:       tail_mean_pos_error ≈ 0.484 m
  6D Artstein Disc + 延迟:   tail_mean_pos_error ≈ 0.081 m
```

该结果说明方向 A 的补偿层能在数值仿真中把延迟系统的性能拉回接近无延迟
6D Disc 的水平，但最终结论仍需 Gazebo 和实物实验验证。

### 8.1 Gazebo/实物调参中的约束主导现象

后续 Gazebo 测试中需要单独区分两类加速度约束：

```text
max_linear_accel / max_angular_accel:
  控制器侧约束，作用在 6D Artstein Disc 输出后的 body x/y/omega 命令变化率。

delay_max_accel:
  sim_motor_delay.py 延迟注入节点的约束，只用于模拟执行器响应。
```

因此 `delay_max_accel` 调得更理想，只会让仿真底盘响应更快，不会放宽控制器内部的
命令变化率限制。若要模拟实物约 0.25-0.30 m/s^2 的加速度能力，应同时检查控制器侧
`max_linear_accel` 与延迟节点侧 `delay_max_accel`，并确认两者和实验目的一致。

当前实现中的速度和加速度约束均按分量限制：

```text
|v_x^b| <= max_linear_vel
|v_y^b| <= max_linear_vel
|omega| <= max_angular_vel
|Delta v_x^b / h| <= max_linear_accel
|Delta v_y^b / h| <= max_linear_accel
|Delta omega / h| <= max_angular_accel
```

这不是二维线速度模长限制。若后续论文实验需要严格对齐实物“合成加速度上限”，
应新增 vector-norm 限幅版本并重新跑对照实验。

当 leader 速度提高到 0.5 m/s 且 follower 最大线加速度仍接近 0.25-0.30 m/s^2 时，
跟踪误差可能主要由执行器能力和饱和相位滞后决定。此时 Artstein 预测可以减少延迟造成的
额外相位损失，但不能让不可达的加速度/速度变为可达。论文表述应写成：

```text
nominal 预测补偿模型下恢复接近无延迟 6D Disc 的性能；
受饱和和实物执行器能力约束时，只保证实用稳定和可实验验证的误差界。
```
