---
title: "6D Leader–Follower 齐次编队控制：建模、广义齐次升级与有限时间稳定性分析"
author: ""
date: ""
lang: zh-CN
---

# 1 研究对象与基本假设

本文仅讨论六状态（6D）Leader–Follower 编队控制问题，不涉及 4D 模型、Artstein 延迟补偿或 HOCBF 安全过滤。目标是回答：当全向移动机器人的航向角、车体系平移速度和角速度均显式进入状态后，是否仍能在严格的名义模型下构造广义齐次控制器，并获得有限时间稳定性结论。

对机器人 $i\in\{l,f\}$（分别表示 Leader 与 Follower），定义状态

$$
x_i=\begin{bmatrix}p_{x,i}&p_{y,i}&\theta_i&v_{x,i}^b&v_{y,i}^b&\omega_i\end{bmatrix}^{\!T}\in\mathbb R^6.
$$

其中 $p_i=[p_{x,i},p_{y,i}]^T$ 为地图坐标系位置，$\theta_i$ 为航向角，$v_i^b=[v_{x,i}^b,v_{y,i}^b]^T$ 为车体系平移速度，$\omega_i$ 为角速度。定义二维旋转矩阵与反对称矩阵

$$
R(\theta)=\begin{bmatrix}\cos\theta&-\sin\theta\\\sin\theta&\cos\theta\end{bmatrix},
\qquad
J=\begin{bmatrix}0&-1\\1&0\end{bmatrix}.
$$

满足

$$
\dot R(\theta)=\omega R(\theta)J,
\qquad
R(-\theta)=R^T(\theta).
$$

采用名义刚体运动模型

$$
\dot p_i=R(\theta_i)v_i^b,
\qquad
\dot\theta_i=\omega_i,
$$

$$
\dot v_i^b=\frac{1}{m}\begin{bmatrix}F_{x,i}\\F_{y,i}\end{bmatrix},
\qquad
\dot\omega_i=\frac{1}{I}M_{z,i}.
$$

对 Follower 定义控制输入

$$
u=\begin{bmatrix}F_x&F_y&M_z\end{bmatrix}^{T},
\qquad
D=\operatorname{diag}\!\left(\frac1m,\frac1m,\frac1I\right).
$$

为得到清晰的齐次理论，采用以下名义假设：

1. 期望位置偏移 $d_p=[d_x,d_y]^T$ 固定在 Leader 坐标系中；
2. 理论主结果取期望航向偏移 $d_\theta=0$，即 Leader 与 Follower 名义航向一致；
3. 在每个局部分析区间 $[t_k,t_{k+1})$ 内冻结 Leader twist，即 $v_l^b$ 与 $\omega_l$ 视为常值；
4. 航向误差位于局部连续分支内，$|e_\theta|<\bar\theta<\pi$；
5. 名义理论暂不包含执行器饱和、离散采样、轮地滑移、测量噪声及通信时延。

> 注：若控制输入直接定义为归一化平移/角加速度，则可令 $D=I_3$，后续推导不变。

# 2 运动学一致的 Leader 系误差定义

## 2.1 位置误差

将 Follower 相对位置变换至 Leader 坐标系：

$$
r=R(-\theta_l)(p_f-p_l).
$$

期望编队点在 Leader 坐标系中固定为 $d_p$，定义位置误差

$$
\boxed{e_p=r-d_p=R(-\theta_l)(p_f-p_l)-d_p.}
$$

对 $r$ 求导：

$$
\begin{aligned}
\dot r
&=-\omega_lJr+R(-\theta_l)(\dot p_f-\dot p_l)\\
&=-\omega_lJr+R(\theta_f-\theta_l)v_f^b-v_l^b.
\end{aligned}
$$

因此

$$
\boxed{
\dot e_p=-\omega_lJ(e_p+d_p)+R(\theta_f-\theta_l)v_f^b-v_l^b.
}
$$

## 2.2 航向误差

在 $d_\theta=0$ 的名义条件下，定义

$$
\boxed{e_\theta=\theta_f-\theta_l,}
\qquad
\boxed{e_\omega=\omega_f-\omega_l.}
$$

于是有精确关系

$$
\boxed{\dot e_\theta=e_\omega.}
$$

## 2.3 运动学一致的期望车体系速度

当 Leader 存在角速度 $\omega_l\neq0$ 且 $d_p\neq0$ 时，固定在 Leader 坐标系中的编队点本身具有切向速度。若在理想编队状态下要求 $e_p=0$ 且 $e_\theta=0$，由 $\dot e_p=0$ 可得 Follower 的名义车体系速度必须满足

$$
0=-\omega_lJd_p+v_f^b-v_l^b.
$$

因此定义运动学一致的期望 Follower 车体系速度

$$
\boxed{v_d^b=v_l^b+\omega_lJd_p.}
$$

进一步定义平移速度误差

$$
\boxed{e_v=v_f^b-v_d^b.}
$$

这一速度误差定义的关键作用是：当 $e_p=e_\theta=e_v=0$ 时，名义编队状态确实构成误差系统的平衡点，而无需把 $-\omega_lJd_p$ 作为人为的常值扰动保留在误差动力学中。

# 3 精确误差动力学与局部线性化

由

$$
v_f^b=e_v+v_d^b,
\qquad
\theta_f-\theta_l=e_\theta,
$$

代入位置误差动力学，得到

$$
\begin{aligned}
\dot e_p
&=-\omega_lJ(e_p+d_p)+R(e_\theta)(e_v+v_d^b)-v_l^b\\
&=-\omega_lJe_p+R(e_\theta)e_v+[R(e_\theta)-I_2]v_d^b.
\end{aligned}
$$

故精确位置误差方程为

$$
\boxed{
\dot e_p=-\omega_lJe_p+R(e_\theta)e_v+[R(e_\theta)-I_2]v_d^b.
}
$$

在 $e_\theta=0$ 附近，利用

$$
R(e_\theta)=I_2+Je_\theta+O(e_\theta^2),
$$

得到

$$
\dot e_p=-\omega_lJe_p+e_v+Jv_d^b e_\theta+w_p(e),
$$

其中非线性余项可显式写为

$$
\boxed{
w_p(e)=[R(e_\theta)-I_2]e_v+[R(e_\theta)-I_2-Je_\theta]v_d^b.
}
$$

在任意固定局部区域 $|e_\theta|\le\bar\theta<\pi$ 内，存在常数 $c_R>0$ 使

$$
\|R(e_\theta)-I_2\|\le |e_\theta|,
$$

$$
\|R(e_\theta)-I_2-Je_\theta\|\le c_Re_\theta^2.
$$

从而

$$
\|w_p(e)\|
\le |e_\theta|\|e_v\|+c_R\|v_d^b\|e_\theta^2
=O(\|e\|^2).
$$

因此，冻结 Leader twist 后的线性模型是原非线性误差系统在零误差邻域内的一阶精确近似。

# 4 冻结 6D 名义误差模型

定义

$$
q=\begin{bmatrix}e_x&e_y&e_\theta\end{bmatrix}^T,
\qquad
\eta=\begin{bmatrix}e_{v_x}&e_{v_y}&e_\omega\end{bmatrix}^T,
$$

以及总误差

$$
\boxed{e=\begin{bmatrix}q^T&\eta^T\end{bmatrix}^T\in\mathbb R^6.}
$$

记

$$
v_d^b=\begin{bmatrix}v_{d,x}&v_{d,y}\end{bmatrix}^T.
$$

根据上一节的一阶线性化，有

$$
\dot q=F_Lq+\eta+w_q(e),
$$

其中

$$
\boxed{
F_L=
\begin{bmatrix}
0&\omega_l&-v_{d,y}\\
-\omega_l&0&v_{d,x}\\
0&0&0
\end{bmatrix},
}
$$

且

$$
w_q(e)=\begin{bmatrix}w_p^T(e)&0\end{bmatrix}^T.
$$

对速度误差，有

$$
\dot e_v=\frac1m\begin{bmatrix}F_x\\F_y\end{bmatrix}-\dot v_d^b,
\qquad
\dot e_\omega=\frac1I M_z-\dot\omega_l.
$$

在冻结 Leader twist 的名义模型中，$\dot v_d^b=0$、$\dot\omega_l=0$，故

$$
\boxed{\dot\eta=Du.}
$$

于是冻结 6D 名义误差系统为

$$
\boxed{\dot e=A_Fe+Bu,}
$$

其中

$$
\boxed{
A_F=
\begin{bmatrix}
F_L&I_3\\
0&I_3\!\cdot0
\end{bmatrix}
=
\begin{bmatrix}
F_L&I_3\\
0&0
\end{bmatrix},
\qquad
B=
\begin{bmatrix}
0\\D
\end{bmatrix}.
}
$$

为避免歧义，上式中的零块均为 $3\times3$ 零矩阵。

实际非线性、时变系统可统一写成

$$
\boxed{\dot e=A_F(t_k)e+Bu+w(e,t),}
$$

其中 $w$ 汇总：二阶运动学余项 $w_p$、Leader twist 在冻结区间内的变化 $-\dot v_d^b$ 与 $-\dot\omega_l$，以及未建模扰动。后续严格有限时间结论仅针对 $w\equiv0$ 的冻结名义系统。

# 5 冻结 6D 系统的完全可控性

**引理 1（冻结 6D 名义系统完全可控）**  若 $m>0$ 且 $I>0$，则对任意有限的 $v_d^b$ 与 $\omega_l$，矩阵对 $(A_F,B)$ 完全可控。

**证明：**

由

$$
A_F=
\begin{bmatrix}F_L&I_3\\0&0\end{bmatrix},
\qquad
B=\begin{bmatrix}0\\D\end{bmatrix},
$$

可得

$$
A_FB=
\begin{bmatrix}D\\0\end{bmatrix}.
$$

因此只取前两阶可控矩阵即可得到

$$
\mathcal C_2=[B,A_FB]
=
\begin{bmatrix}
0&D\\
D&0
\end{bmatrix}.
$$

由于

$$
D=\operatorname{diag}\!\left(\frac1m,\frac1m,\frac1I\right)
$$

在 $m>0,I>0$ 时可逆，故

$$
\operatorname{rank}(\mathcal C_2)=6.
$$

因此 $(A_F,B)$ 完全可控。证毕。

该结论表明，Leader twist 所引入的 $F_L$ 耦合并不会破坏冻结系统的完全可控性。

# 6 广义齐次升级的解析构造

广义齐次升级需要寻找矩阵 $G_0$ 与 $Y_0$，使

$$
\boxed{A_FG_0-G_0A_F+BY_0=A_F,}
$$

$$
\boxed{G_0B=0.}
$$

对于本 6D 冻结模型，上述矩阵可直接解析构造，而不必依赖数值块可控分解。

**引理 2（齐次生成元的解析构造）**  取

$$
\boxed{
G_0=
\begin{bmatrix}
-I_3&0\\
F_L&0
\end{bmatrix},
}
$$

以及

$$
\boxed{
Y_0=
\begin{bmatrix}
D^{-1}F_L^2&D^{-1}F_L
\end{bmatrix}.
}
$$

则 $G_0,Y_0$ 满足广义齐次升级代数方程。

**证明：**

首先

$$
G_0B
=
\begin{bmatrix}-I_3&0\\F_L&0\end{bmatrix}
\begin{bmatrix}0\\D\end{bmatrix}
=0.
$$

另一方面，直接计算得

$$
A_FG_0-G_0A_F
=
\begin{bmatrix}
F_L&I_3\\
-F_L^2&-F_L
\end{bmatrix},
$$

而

$$
BY_0
=
\begin{bmatrix}
0&0\\
F_L^2&F_L
\end{bmatrix}.
$$

因此

$$
A_FG_0-G_0A_F+BY_0
=
\begin{bmatrix}F_L&I_3\\0&0\end{bmatrix}
=A_F.
$$

证毕。

# 7 基反馈、幂零结构与齐次伸缩生成元

定义广义齐次基反馈

$$
K_0=Y_0(G_0-I_6)^{-1}.
$$

由于

$$
G_0-I_6=
\begin{bmatrix}
-2I_3&0\\
F_L&-I_3
\end{bmatrix},
$$

其逆矩阵为

$$
(G_0-I_6)^{-1}
=
\begin{bmatrix}
-\frac12I_3&0\\
-\frac12F_L&-I_3
\end{bmatrix}.
$$

故得到解析基反馈

$$
\boxed{
K_0=
\begin{bmatrix}
-D^{-1}F_L^2&-D^{-1}F_L
\end{bmatrix}.
}
$$

定义

$$
A_0=A_F+BK_0,
$$

则

$$
\boxed{
A_0=
\begin{bmatrix}
F_L&I_3\\
-F_L^2&-F_L
\end{bmatrix}.
}
$$

直接相乘可得

$$
\boxed{A_0^2=0.}
$$

因此，虽然原冻结模型含有 $F_L$ 引起的航向—平移耦合，在施加基反馈 $K_0$ 后，等效矩阵 $A_0$ 恢复为二阶幂零结构。这一性质为后续负度齐次有限时间控制提供了清晰的代数基础。

进一步可验证

$$
\boxed{A_0G_0-G_0A_0=A_0.}
$$

选取齐次度

$$
\boxed{-1<\nu<0,}
$$

并定义伸缩生成元

$$
\boxed{G_d=I_6+\nu G_0.}
$$

显式写为

$$
\boxed{
G_d=
\begin{bmatrix}
(1-\nu)I_3&0\\
\nu F_L&I_3
\end{bmatrix}.
}
$$

$G_0$ 的特征值为 $-1$（三重）与 $0$（三重），因此 $G_d$ 的特征值为 $1-\nu$（三重）与 $1$（三重）。当 $-1<\nu<0$ 时全部严格为正，故 $G_d$ 可作为合法的伸缩生成元。

同时

$$
\boxed{G_dB=B,}
$$

以及

$$
\boxed{A_0G_d-G_dA_0=\nu A_0.}
$$

等价地

$$
\boxed{A_0G_d=(G_d+\nu I_6)A_0.}
$$

上述关系表明，经过基反馈后的名义动力学与 $G_d$ 所生成的伸缩具有所需的广义齐次代数兼容性。

# 8 稳定线性反馈与 LMI 兼容条件

由于 $(A_F,B)$ 完全可控，可任选极点配置、LQR 或其他线性设计得到 $K$，使

$$
\boxed{A_c=A_F+BK}
$$

为 Hurwitz 矩阵。

对 Hurwitz 矩阵 $A_c$，存在 $P=P^T>0$ 使

$$
\boxed{PA_c+A_c^TP<0.}
$$

为了将该稳定线性反馈升级为广义齐次有限时间反馈，还需满足伸缩与 Lyapunov 度量之间的兼容条件

$$
\boxed{PG_d+G_d^TP>0.}
$$

因此，仅证明 $(A_F,B)$ 可控和 $A_c$ Hurwitz 并不足以直接推出负度齐次有限时间稳定；必须同时检查所采用广义齐次定理要求的完整矩阵条件。

## 8.1 负齐次度存在性的一个保守充分条件

由于

$$
G_d=I_6+\nu G_0,
$$

有

$$
PG_d+G_d^TP
=2P+\nu(PG_0+G_0^TP).
$$

记

$$
S=PG_0+G_0^TP.
$$

由最小特征值估计

$$
\lambda_{\min}(2P+\nu S)
\ge2\lambda_{\min}(P)-|\nu|\|S\|_2.
$$

因此，只要

$$
|\nu|<\frac{2\lambda_{\min}(P)}{\|PG_0+G_0^TP\|_2},
$$

即可保证

$$
PG_d+G_d^TP>0.
$$

结合 $-1<\nu<0$，可给出一个便于数值选参的保守区间：

$$
\boxed{
\nu\in\left(
-\min\left\{1,\frac{2\lambda_{\min}(P)}{\|PG_0+G_0^TP\|_2}\right\},
0
\right).
}
$$

该条件的意义是：只要先选取 $K$ 使线性冻结闭环 Hurwitz，则至少存在一段足够接近零的负齐次度区间，使 Lyapunov 度量与伸缩生成元保持正定兼容。该区间是充分条件而非最优范围，实际可通过直接求解 LMI 得到更宽的可行域。

# 9 齐次范数与 6D 理论控制律

给定 $P>0$ 与合法生成元 $G_d$，定义由 $G_d$ 诱导的 canonical homogeneous norm。对任意 $e\neq0$，令 $s\in\mathbb R$ 为方程

$$
\left(e^{-sG_d}e\right)^TP\left(e^{-sG_d}e\right)=1
$$

的唯一解，并定义

$$
\boxed{\|e\|_d=e^s.}
$$

同时规定 $\|0\|_d=0$。

基于 $K_0$、稳定线性反馈 $K$ 和负齐次度 $\nu$，定义理论 6D 广义齐次控制律

$$
\boxed{
u_h(e)=K_0e+\|e\|_d^{1+\nu}(K-K_0)e^{-\ln(\|e\|_d)G_d}e,
\qquad e\neq0,
}
$$

并令

$$
\boxed{u_h(0)=0.}
$$

该结构由基反馈 $K_0e$ 与尺度相关的齐次修正项组成。远离或接近原点时，控制增益按 $G_d$ 所定义的非欧式伸缩规律变化，从而形成负度齐次闭环。

# 10 冻结 6D 名义系统的有限时间稳定性

**定理 1（冻结 6D 名义误差系统的有限时间稳定性）**  考虑

$$
\dot e=A_Fe+Bu_h(e),
$$

其中 $(A_F,B)$ 由第 4 节给出。若满足：

1. $m>0,I>0$；
2. Leader twist 在所分析区间冻结，使 $A_F$ 为常矩阵；
3. 选择 $K$ 使 $A_F+BK$ Hurwitz；
4. 存在 $P=P^T>0$ 与 $-1<\nu<0$，满足所采用广义齐次控制定理的完整矩阵条件，特别包括
   $$PA_c+A_c^TP<0,$$
   $$PG_d+G_d^TP>0;$$
5. $G_0,Y_0,K_0,G_d$ 按第 6–7 节构造；
6. 使用第 9 节的理论控制律 $u_h(e)$，且不引入饱和、正则化截断或离散化修改；

则冻结名义闭环在原点具有负度广义齐次结构。根据负度齐次系统的有限时间稳定性定理，若闭环原点渐近稳定，则原点进一步为有限时间稳定。

在相应齐次 Lyapunov/范数估计成立时，可写成

$$
\dot{\|e\|}_d\le-c\|e\|_d^{1+\nu},
\qquad c>0,
$$

从而得到收敛时间上界

$$
\boxed{
T(e_0)\le\frac{\|e_0\|_d^{-\nu}}{c(-\nu)}.
}
$$

**证明思路：**

首先，引理 1 保证冻结系统完全可控；引理 2 给出满足广义齐次升级代数关系的解析 $G_0,Y_0$。由 $K_0$ 构造得到的 $A_0$ 满足 $A_0^2=0$，且

$$
A_0G_d-G_dA_0=\nu A_0,
\qquad
G_dB=B.
$$

上述关系建立了状态向量场与伸缩 $G_d$ 的齐次兼容性。再结合稳定线性反馈 $K$、正定矩阵 $P$ 以及所采用广义齐次控制定理的 LMI 条件，可得到理论控制律下闭环的广义齐次性与渐近稳定性。由于齐次度 $\nu<0$，负度齐次渐近稳定系统具有有限时间收敛性质，故结论成立。

> 说明：正式论文中应将本定理与实际采用的广义齐次控制参考文献中的定理逐项对应，不能只用“可控 + Hurwitz”代替完整假设。

# 11 非线性、时变 6D 系统与名义理论的边界

前述有限时间定理仅针对冻结、连续、无扰动的名义误差系统。真实 6D 系统满足

$$
\dot e=A_F(t_k)e+Bu+w(e,t),
$$

其中主要非理想项包括：

- $w_p(e)=O(\|e\|^2)$ 的姿态—速度高阶运动学耦合；
- Leader 平移速度和角速度随时间变化导致的 $-\dot v_d^b$、$-\dot\omega_l$；
- 航向角 wrap 在 $\pm\pi$ 处的非光滑性；
- 实际执行器动力学、速度/加速度饱和与轮地滑移；
- 离散采样和数值计算误差；
- 若工程实现对 canonical norm 进行截断或正则化，则实际控制律不再与理论 $u_h$ 代数等价。

因此，不应将冻结名义模型的有限时间定理直接表述为“完整实车 6D 系统全局有限时间稳定”。更稳妥的论文表述是：

> 对固定 Leader twist 的 6D 局部名义误差模型，本文证明其完全可控，并给出广义齐次生成元和基反馈的解析构造；在稳定线性反馈及相应 LMI 条件成立时，理论齐次闭环具有有限时间稳定性。对于 Leader twist 时变、非线性高阶项、离散实现和执行器非理想因素，本文通过数值仿真与实验评价实际稳定运行范围，而不将名义有限时间结论无条件外推至完整工程系统。

需要特别指出：由“$w$ 有界”本身不能自动推出“误差最终进入有界邻域”。若论文希望进一步声称 practical stability、ISS 或 ultimate boundedness，则需要额外的鲁棒稳定性证明；否则应将该部分保持为实验和敏感性分析结论。

# 12 理论控制律与工程实现的对应建议

若实际代码采用类似

$$
u_{\mathrm{impl}}
=c^{1+\nu}K\exp\!\bigl(G_d(1-\ln c)\bigr)e,
$$

并对 $c$ 使用

$$
c=\operatorname{clamp}(\|e\|_d,c_{\min},1),
$$

则该实现与第 9 节理论控制律 $u_h$ 一般不代数等价，固定阈值截断还会破坏严格的全局缩放关系。因此建议在论文中明确区分：

- **理论控制律 $u_h$：** 用于名义齐次性与有限时间稳定性证明；
- **正则化工程控制律 $u_{\mathrm{impl}}$：** 用于避免零点附近数值病态并满足实际计算需求，其性能通过数值实验验证。

为避免答辩时出现“证明的是一套、运行的是另一套”的质疑，可增加一组纯数值对照实验，比较线性反馈、理论 $u_h$ 和正则化 $u_{\mathrm{impl}}$ 的收敛轨迹、控制输入峰值和近零区域行为。

# 13 可直接用于论文的工作点表述

基于以上推导，6D 部分可概括为如下独立工作点：

> 针对全向移动机器人 Leader–Follower 编队中航向角、车体系速度和 Leader 转动引起的耦合问题，构造运动学一致的 6D Leader 系误差模型。通过引入 $v_d^b=v_l^b+\omega_lJd_p$，使固定 Leader twist 下的零编队误差成为名义平衡点，并将非线性高阶耦合显式分离为局部二阶余项。在此基础上证明冻结 6D 线性误差系统完全可控，进一步给出广义齐次生成元 $G_0$、辅助矩阵 $Y_0$ 及基反馈 $K_0$ 的解析构造，并证明基反馈后的等效矩阵具有二阶幂零结构。结合稳定线性反馈与 dilation-compatible LMI 条件，可在冻结名义模型下建立负度广义齐次闭环的有限时间稳定性结论。

该工作点的理论核心不是简单“增加两个状态”，而是回答：在显式保留航向—车体系速度耦合后，原有积分链结构不再直接成立时，是否仍能构造满足广义齐次升级条件的 6D 局部模型与有限时间控制律。上述推导给出了肯定答案，并明确了其局部、冻结和名义适用边界。

# 14 建议在正式论文中进一步补充的验证

为使本章形成完整的“建模—证明—验证”闭环，建议至少增加以下三类实验：

1. **线性化有效性验证：** 扫描初始航向误差，比较原始非线性 6D 模型与冻结线性模型的预测误差，验证 $w_p=O(\|e\|^2)$ 的局部特征；
2. **齐次升级有效性验证：** 比较稳定线性反馈 $K$ 与理论齐次反馈 $u_h$ 的收敛时间，特别观察零点附近负度齐次控制的有限时间收敛特征；
3. **冻结模型适用范围验证：** 扫描 $v_l^b$、$\omega_l$ 及 Leader twist 变化率，评价冻结参数更新频率与跟踪误差之间的关系。

若上述实验完成，则 6D 部分可以形成相对独立、理论边界清楚且适合硕士论文答辩的完整章节。
