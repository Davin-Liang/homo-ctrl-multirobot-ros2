# 6维编队误差系统的广义齐次化及有限时间稳定性（小论文正文草稿 v2）

> **写作说明（投稿前删除）**

> 本文在已有4维 Leader–Follower 齐次控制工作的基础上[1]，进一步研究6维位姿编队误差系统。本文不重新提出广义齐次控制理论，而是证明前文构造的6维位姿编队误差系统仍属于同一类二阶积分系统，从而可在已有广义齐次化（generalized homogenization）框架下进行统一设计与稳定性分析[4]。

> 为保持与已有广义齐次控制文献的一致性，本节采用 $G_0,Y_0,K_0,K_{\rm lin},P,G_d,\rho$ 等常用符号[1,3-4]，并将4维与6维系统统一写成一般 $2r$ 维形式。有限时间稳定性仅保留本文所需的 Lyapunov 推导。

---

## X.X 6维编队误差系统的广义齐次控制与有限时间稳定性

由前文推导，本文6维Leader–Follower编队误差状态定义为

$$
x=

\begin{bmatrix}

e_x&

e_y&

e_\theta&

e_{v_x}&

e_{v_y}&

e_\omega

\end{bmatrix}^{T},
$$

其名义误差动力学可写为

$$
\dot x=A_3x+B_3u,
$$

其中

$$
A_3=

\begin{bmatrix}

0_{3\times3}&I_3\\

0_{3\times3}&0_{3\times3}

\end{bmatrix},

\qquad

B_3=

\begin{bmatrix}

0_{3\times3}\\

D_3

\end{bmatrix},
$$

$$
D_3=

\operatorname{diag}

\left(

\frac1m,\frac1m,\frac1I

\right).
$$

可以看出，该系统由位置 $x$、位置 $y$ 和偏航角 $\theta$ 三个二阶通道构成。已有4维平移编队误差系统[1]与上述6维系统具有相同的二阶积分结构。为统一分析，将两者写为一般 $2r$ 维系统

$$
\dot x=A_rx+B_ru

\tag{1}
$$

其中

$$
A_r=

\begin{bmatrix}

0&I_r\\

0&0

\end{bmatrix},

\qquad

B_r=

\begin{bmatrix}

0\\

D_r

\end{bmatrix},

\tag{2}
$$

$D_r\in\mathbb R^{r\times r}$ 为非奇异矩阵。式（1）在 $r=2$ 和 $r=3$ 时分别对应已有4维平移编队误差模型[1]和本文6维位姿编队误差模型。下面仅需验证一般 $2r$ 维系统满足广义齐次化方法所要求的结构条件。

### X.X.1 可控性

由式（2）有

$$
A_rB_r=

\begin{bmatrix}

D_r\\

0

\end{bmatrix}.
$$

又因为 $A_r^2=0$，故可控性矩阵为

$$
\mathcal C_r

=

\begin{bmatrix}

B_r&A_rB_r

\end{bmatrix}

=

\begin{bmatrix}

0&D_r\\

D_r&0

\end{bmatrix}.
$$

当 $D_r$ 非奇异时，

$$
\operatorname{rank}(\mathcal C_r)=2r,
$$

因此 $(A_r,B_r)$ 完全可控。

对于本文 $r=3$ 的系统，由 $m>0$、$I>0$ 可知 $D_3$ 非奇异，因此 $(A_3,B_3)$ 完全可控。

---

### X.X.2 广义齐次化条件

根据广义齐次化方法的构造条件[4]，需要求取矩阵 $G_0$ 和 $Y_0$，使其满足

$$
A_rG_0-G_0A_r+B_rY_0=A_r,

\qquad

G_0B_r=0.

\tag{3}
$$

对于式（2）的 $2r$ 维二阶积分系统，取

$$
G_0=

\begin{bmatrix}

-I_r&0\\

0&0

\end{bmatrix},

\qquad

Y_0=0.


\tag{4}
$$

代入可得

$$
G_0B_r=0,

\qquad

A_rG_0-G_0A_r=A_r.
$$

因此式（3）成立。进一步，

$$
K_0=Y_0(G_0-I_{2r})^{-1}=0.

\tag{5}
$$

定义膨胀生成矩阵（dilation generator）

$$
G_d=I_{2r}+\mu G_0

=

\begin{bmatrix}

(1-\mu)I_r&0\\

0&I_r

\end{bmatrix}.


\tag{6}
$$

其中，$\mu$ 为期望的齐次度，本文取 $-1<\mu<0$。负齐次度是获得有限时间收敛的关键，同时 $1+\mu>0$ 使控制律中的标度因子 $\|x\|_d^{1+\mu}$ 在原点附近不发散。此时 $G_d$ 的全部特征值均具有正实部，因此能够生成线性膨胀映射。进一步可验证

$$
A_rG_d=(G_d+\mu I_{2r})A_r,

\qquad

G_dB_r=B_r.

\tag{7}
$$

因此，一般 $2r$ 维二阶积分系统满足广义齐次化所需的代数条件[4]。

---

### X.X.3 广义齐次控制器

为构造广义齐次反馈，首先选取线性稳定反馈 $u=K_{\rm lin}x$。对于式（2），取

$$
K_{\rm lin}

=

\begin{bmatrix}

-D_r^{-1}k_pI_r&

-D_r^{-1}k_vI_r

\end{bmatrix},

\tag{8}
$$

其中，$k_p>0,\ k_v>0$。此时闭环特征多项式为 $(s^2+k_vs+k_p)^r$，因此 $A_r+B_rK_{\rm lin}$ 为 Hurwitz 矩阵。当 $r=2$ 且 $D_2=(1/m)I_2$ 时，通过适当选取 $k_p$ 和 $k_v$，式（8）可退化为文献[1]所采用的线性反馈形式。

根据广义齐次化理论[4]，进一步求取同一个对称正定矩阵 $P=P^T>0$，使其满足

$$
PG_d+G_d^TP>0,

\tag{9}
$$

以及

$$
P(A_r+B_rK_{\rm lin})

+

(A_r+B_rK_{\rm lin})^TP<0.

\tag{10}
$$

由式（9）和式（10），可取充分小的 $\rho>0$，使

$$
P(A_r+B_rK_{\rm lin})

+

(A_r+B_rK_{\rm lin})^TP

+

\rho

\left(

PG_d+G_d^TP

\right)

\le0.

\tag{11}
$$

上述条件将在后续用于建立典范齐次范数的衰减不等式。

由 $G_d$ 定义线性膨胀映射 $d(s)=\exp(sG_d)$，并定义 $\|z\|_P=\sqrt{z^TPz}$。按照典范齐次范数的定义[3]，对任意 $x\neq0$，$\|x\|_d$ 满足

$$
\left\|

d(-\ln\|x\|_d)x

\right\|_P=1.
$$

由于 $K_0=0$，根据广义齐次反馈的构造形式[1,4]，控制器可写为

$$
u_h(x)

=

\|x\|_d^{1+\mu}

K_{\rm lin}

d(-\ln\|x\|_d)x,

\qquad x\neq0.

\tag{12}
$$

对于本文6维编队误差系统，$u_h=[F_x,F_y,M_z]^T$，其中 $F_x,F_y$ 为 map 坐标系下的等效相对编队控制力，$M_z$ 为偏航等效控制力矩。

---

## X.X.4 有限时间稳定性分析

下面对式（1）所描述的不含输入时延与执行器动态的理想误差系统进行有限时间稳定性分析。

**推论 1**

对于式（1）所描述的 $2r$ 维理想误差系统，若 $D_r$ 非奇异、$-1<\mu<0$，且所选 $K_{\rm lin}$、$P$ 和 $\rho$ 满足式（9）—（11），则在控制律（12）作用下系统原点有限时间稳定。

**证明：**

将控制律（12）代入一般 $2r$ 维理想误差系统（1），得到闭环系统

$$
\dot x

=

A_rx

+

B_r\|x\|_d^{1+\mu}

K_{\rm lin}

d(-\ln\|x\|_d)x,

\qquad

x\in\mathbb R^{2r}.

\tag{13}
$$

记上述闭环向量场为

$$
\dot x=f_r(x).
$$

由式（7）可得

$$
A_rd(s)=e^{\mu s}d(s)A_r,

\qquad

d(s)B_r=e^sB_r.
$$

另一方面，典范齐次范数满足

$$
\|d(s)x\|_d=e^s\|x\|_d.
$$

因此，由控制律（12）有

$$
u_h(d(s)x)=e^{(1+\mu)s}u_h(x).
$$

进而

$$
\begin{aligned}

f_r(d(s)x)

&=A_rd(s)x+B_ru_h(d(s)x)\\

&=e^{\mu s}d(s)\left[A_rx+B_ru_h(x)\right]

=e^{\mu s}d(s)f_r(x).

\end{aligned}
$$

根据齐次向量场的定义[2-4]，上述关系表明闭环向量场 $f_r(x)$ 关于线性膨胀映射 $d(s)=\exp(sG_d)$ 具有齐次度 $\mu$。因此，对任意 $r$，有限时间稳定性分析均可在同一 $2r$ 维框架下进行。

取典范齐次范数

$$
V(x)=\|x\|_d

\tag{14}
$$

作为 Lyapunov 函数。根据典范齐次范数的导数性质[3]及式（11），沿闭环系统轨迹有

$$
\dot V(x)

\le

-\rho V^{1+\mu}(x),

\qquad x\neq0.


\tag{15}
$$

由于 $-1<\mu<0$，故 $0<1+\mu<1$。由式（15）可得

$$
V^{-1-\mu}\dot V

\le

-\rho.
$$

又有

$$
\frac{d}{dt}V^{-\mu}

=

-\mu V^{-1-\mu}\dot V.
$$

由于 $-\mu>0$，因此

$$
\frac{d}{dt}V^{-\mu}

\le

\mu\rho.
$$

从初始时刻 $0$ 积分至 $t$，得到

$$
V^{-\mu}(x(t))

-

V^{-\mu}(x_0)

\le

\mu\rho t.

\tag{16}
$$

因此闭环系统的收敛时间满足

$$
T(x_0)

\le

\frac{

V^{-\mu}(x_0)

}{

-\mu\rho

}.


\tag{17}
$$

即存在有限的 $T(x_0)>0$，使得对任意 $t\ge T(x_0)$ 均有 $x(t)=0$。当 $r=3$ 时，上述结论直接对应本文6维 Leader–Follower编队误差系统，因此其位置、航向及相应速度误差均在有限时间内收敛至零。证毕。

上述结论针对不含输入时延与执行器动态的理想误差模型。考虑实际全向移动机器人通过 `cmd_vel` 接收速度指令时存在的输入纯延迟与速度响应滞后，下一节进一步建立执行器模型，并采用 Artstein reduction 与前向状态预测进行补偿。

---

## 参考文献

[1] Yuan W, Dong C, Duan X, Polyakov A, Zimenko K, Ping X. Leader-Follower Tracking with Collision Avoidance for Omni-directional Mobile Robots: Linear vs Homogeneous Controller. 43rd Chinese Control Conference (CCC), 2024: 124-129.

[2] Polyakov A, Krstic M. Finite- and fixed-time nonovershooting stabilizers and safety filters by homogeneous feedback. IEEE Transactions on Automatic Control, 2023, 68(11): 6434-6449.

[3] Polyakov A. Sliding mode control design using canonical homogeneous norm. International Journal of Robust and Nonlinear Control, 2019.

[4] Wang S, Polyakov A, Zheng G. Generalized homogenization of linear controllers: Theory and experiment. International Journal of Robust and Nonlinear Control, 2021, 31(9): 3455-3479.
