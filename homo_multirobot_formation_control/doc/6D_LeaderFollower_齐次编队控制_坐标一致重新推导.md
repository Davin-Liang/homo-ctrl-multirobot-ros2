# 6D Leader–Follower 齐次编队控制：坐标一致的重新推导

> **用途**：供 `homo_multirobot_formation_control` 的 6D 控制器理论—代码一致性重构与 Codex 工程优化使用。  
> **版本定位**：仅讨论 **6D 无延迟名义核心**。第一版实现建议固定 Leader-frame 编队偏移 `d_p`、固定 `d_theta = 0`、无 Artstein、无离散多边形切换、无 HOCBF。  
> **重要修正**：本文不再使用 `e_v = v_f^b - v_d^b` 这种跨坐标系直接相减的定义，而统一采用 **Leader-frame velocity error**。

---

## 1. 坐标系与符号

定义三个坐标系：

- `m`：map / world 坐标系；
- `L`：Leader 本体坐标系；
- `F`：Follower 本体坐标系。

二维旋转矩阵定义为

\[
R(\theta)=
\begin{bmatrix}
\cos\theta & -\sin\theta\\
\sin\theta & \cos\theta
\end{bmatrix},
\]

以及

\[
J=
\begin{bmatrix}
0 & -1\\
1 & 0
\end{bmatrix}.
\]

满足

\[
\frac{d}{dt}R(\theta)=\dot\theta R(\theta)J
= \dot\theta J R(\theta).
\]

机器人 \(i\in\{l,f\}\) 的状态采用

\[
x_i=
\begin{bmatrix}
p_i^m\\
\theta_i\\
v_i^b\\
\omega_i
\end{bmatrix},
\]

其中

\[
p_i^m=
\begin{bmatrix}
p_{x,i}^m\\
p_{y,i}^m
\end{bmatrix},
\qquad
v_i^b=
\begin{bmatrix}
v_{x,i}^b\\
v_{y,i}^b
\end{bmatrix}.
\]

原始 6D 运动学/动力学模型写为

\[
\dot p_i^m=R(\theta_i)v_i^b,
\]

\[
\dot\theta_i=\omega_i,
\]

\[
\dot v_i^b=a_i^b,
\]

\[
\dot\omega_i=\alpha_i.
\]

其中 \(a_i^b\) 与 \(\alpha_i\) 分别表示机器人本体系下的平移加速度输入与角加速度输入。

> 说明：以上是本文采用的名义模型。若真实底盘的 body-frame 速度动力学还包含轮胎、摩擦、一阶电机响应等项，这些属于后续工程模型或扰动，不在本节有限时间理论中直接覆盖。

---

## 2. Leader-frame 固定编队偏移

第一版理论一致控制器采用固定的 Leader-frame 位置偏移

\[
d_p=
\begin{bmatrix}
d_x\\
d_y
\end{bmatrix},
\]

并暂取

\[
d_\theta=0.
\]

这意味着期望 Follower 始终处于 Leader 自身坐标系中的固定位置。

对应的 map-frame 期望位置是

\[
p_{f,d}^m
=
p_l^m+R(\theta_l)d_p.
\]

因此当 Leader 转弯时，期望编队点会随 Leader 一起旋转。

---

## 3. 位置误差的严格定义

先定义 Follower 相对 Leader 的位置，并将其旋转到 Leader frame：

\[
r^L
=
R(-\theta_l)(p_f^m-p_l^m).
\]

位置误差定义为

\[
\boxed{
e_p
=
r^L-d_p
=
R(-\theta_l)(p_f^m-p_l^m)-d_p.
}
\]

姿态误差定义为

\[
\boxed{
e_\theta
=
\operatorname{wrap}(\theta_f-\theta_l).
}
\]

严格理论分析限定在不跨越 wrap 分支的局部区域，例如

\[
|e_\theta|<\pi.
\]

角速度误差为

\[
\boxed{
e_\omega=\omega_f-\omega_l.
}
\]

---

## 4. 转弯时固定编队点的期望速度

这是旧 6D 实现中最容易遗漏的一项。

因为

\[
p_{f,d}^m
=
p_l^m+R(\theta_l)d_p,
\]

对时间求导：

\[
\dot p_{f,d}^m
=
R(\theta_l)v_l^b
+
\omega_lR(\theta_l)Jd_p.
\]

左乘 \(R(-\theta_l)\)，得到 Leader frame 下的期望 Follower 平移速度

\[
\boxed{
v_d^L
=
v_l^b+\omega_lJd_p.
}
\]

因此当

\[
\omega_l\neq0,\qquad d_p\neq0
\]

时，即使 Leader 自身平移速度不变，固定编队点仍然具有切向速度

\[
\omega_lJd_p.
\]

如果忽略这一项，则在 Leader 转弯时，`e = 0` 一般不会成为真正的编队平衡点。

---

## 5. 坐标一致的 Leader-frame 速度误差

Follower 的速度 \(v_f^b\) 位于 Follower body frame。

相对姿态为

\[
e_\theta=\theta_f-\theta_l.
\]

因此

\[
R(e_\theta)v_f^b
\]

表示将 Follower 本体系速度旋转到 Leader frame。

定义速度误差

\[
\boxed{
e_v
=
R(e_\theta)v_f^b-v_d^L.
}
\]

即

\[
\boxed{
e_v
=
R(e_\theta)v_f^b
-
v_l^b
-
\omega_lJd_p.
}
\]

注意：此时所有相减项均位于 Leader frame，不存在 follower frame 与 leader frame 混用的问题。

---

## 6. 位置误差动力学：精确推导

由

\[
e_p
=
R(-\theta_l)(p_f^m-p_l^m)-d_p
\]

求导：

\[
\dot e_p
=
-\omega_lJ
R(-\theta_l)(p_f^m-p_l^m)
+
R(-\theta_l)
(\dot p_f^m-\dot p_l^m).
\]

利用

\[
R(-\theta_l)(p_f^m-p_l^m)=e_p+d_p
\]

和

\[
R(-\theta_l)\dot p_f^m
=
R(\theta_f-\theta_l)v_f^b
=
R(e_\theta)v_f^b,
\]

以及

\[
R(-\theta_l)\dot p_l^m=v_l^b,
\]

得到

\[
\dot e_p
=
-\omega_lJ(e_p+d_p)
+
R(e_\theta)v_f^b-v_l^b.
\]

由速度误差定义

\[
R(e_\theta)v_f^b-v_l^b
=
e_v+\omega_lJd_p,
\]

因此得到精确关系

\[
\boxed{
\dot e_p
=
-\omega_lJe_p+e_v.
}
\]

这里没有做小角度近似。

---

## 7. 姿态误差动力学

由

\[
e_\theta=\theta_f-\theta_l
\]

在局部不跨 wrap 分支时有

\[
\boxed{
\dot e_\theta=e_\omega.
}
\]

---

## 8. 速度误差动力学：精确推导

定义

\[
R_e=R(e_\theta).
\]

速度误差为

\[
e_v=R_ev_f^b-v_d^L.
\]

首先，

\[
\dot R_e
=
e_\omega R_eJ.
\]

因此

\[
\frac{d}{dt}(R_ev_f^b)
=
e_\omega R_eJv_f^b
+
R_ea_f^b.
\]

由于二维旋转矩阵与 \(J\) 可交换，

\[
R_eJv_f^b
=
J(R_ev_f^b).
\]

而

\[
R_ev_f^b=e_v+v_d^L.
\]

所以

\[
\frac{d}{dt}(R_ev_f^b)
=
e_\omega J(e_v+v_d^L)
+
R_ea_f^b.
\]

另一方面，

\[
v_d^L=v_l^b+\omega_lJd_p,
\]

因此在 \(d_p\) 为常量时

\[
\dot v_d^L
=
a_l^b+\alpha_lJd_p.
\]

最终得到精确速度误差动力学

\[
\boxed{
\dot e_v
=
e_\omega J(e_v+v_d^L)
+
R_ea_f^b
-
a_l^b
-
\alpha_lJd_p.
}
\]

整理为

\[
\dot e_v
=
e_\omega Je_v
+
e_\omega Jv_d^L
+
R_ea_f^b
-
a_l^b
-
\alpha_lJd_p.
\]

其中：

- \(e_\omega Jv_d^L\)：一阶 yaw-rate error 与期望平移速度耦合；
- \(e_\omega Je_v\)：二阶误差耦合项；
- \(a_l^b+\alpha_lJd_p\)：Leader 及旋转编队点的加速度前馈。

---

## 9. 名义输入变换

为了得到适合广义齐次升级的标准 6D 名义结构，定义 Leader-frame 虚拟平移控制输入

\[
u_v^L\in\mathbb R^2
\]

并令实际 Follower body-frame 加速度满足

\[
\boxed{
R(e_\theta)a_f^b
=
a_l^b
+
\alpha_lJd_p
-
e_\omega Jv_d^L
+
u_v^L.
}
\]

即

\[
\boxed{
a_f^b
=
R(-e_\theta)
\left(
a_l^b
+
\alpha_lJd_p
-
e_\omega Jv_d^L
+
u_v^L
\right).
}
\]

代回精确速度误差动力学：

\[
\boxed{
\dot e_v
=
u_v^L+e_\omega Je_v.
}
\]

定义虚拟角加速度控制

\[
\boxed{
u_\omega
=
\alpha_f-\alpha_l,
}
\]

则

\[
\boxed{
\dot e_\omega=u_\omega.
}
\]

因此完整误差系统为

\[
\boxed{
\begin{aligned}
\dot e_p &= -\omega_lJe_p+e_v,\\
\dot e_\theta &= e_\omega,\\
\dot e_v &= u_v^L+e_\omega Je_v,\\
\dot e_\omega &= u_\omega.
\end{aligned}
}
\]

其中唯一显式保留下来的非线性误差耦合项为

\[
e_\omega Je_v.
\]

该项满足

\[
\|e_\omega Je_v\|
\le
|e_\omega|\,\|e_v\|
=
O(\|e\|^2).
\]

因此它在原点附近是二阶小量。

---

## 10. 冻结 Leader twist 的名义 6D 线性模型

定义

\[
q=
\begin{bmatrix}
e_{p,x}\\
e_{p,y}\\
e_\theta
\end{bmatrix},
\qquad
\eta=
\begin{bmatrix}
e_{v,x}\\
e_{v,y}\\
e_\omega
\end{bmatrix},
\]

以及总误差状态

\[
\boxed{
e=
\begin{bmatrix}
q\\
\eta
\end{bmatrix}
\in\mathbb R^6.
}
\]

固定/冻结当前 Leader yaw rate \(\omega_l\)，定义

\[
\boxed{
F_L=
\begin{bmatrix}
0 & \omega_l & 0\\
-\omega_l & 0 & 0\\
0 & 0 & 0
\end{bmatrix}.
}
\]

则

\[
\dot q
=
F_Lq+\eta.
\]

忽略二阶项 \(e_\omega Je_v\) 后，

\[
\dot\eta=u,
\]

其中

\[
u=
\begin{bmatrix}
u_v^L\\
u_\omega
\end{bmatrix}.
\]

于是得到冻结名义线性系统

\[
\boxed{
\dot e=A_Fe+B_0u,
}
\]

其中

\[
\boxed{
A_F=
\begin{bmatrix}
F_L&I_3\\
0&0
\end{bmatrix},
\qquad
B_0=
\begin{bmatrix}
0\\
I_3
\end{bmatrix}.
}
\]

完整非线性误差系统可写成

\[
\dot e=A_Fe+B_0u+w_{\rm nl}(e),
\]

其中

\[
\boxed{
w_{\rm nl}(e)
=
\begin{bmatrix}
0\\
0\\
0\\
e_\omega Je_v\\
0
\end{bmatrix}
}
\]

按 3+3 分块更紧凑地写为

\[
\boxed{
w_{\rm nl}(e)
=
\begin{bmatrix}
0_3\\
e_\omega Je_v\\
0
\end{bmatrix},
}
\]

且

\[
w_{\rm nl}(e)=O(\|e\|^2).
\]

> 工程含义：名义齐次有限时间定理首先针对 \(w_{\rm nl}=0\) 的冻结线性模型；完整非线性系统的局部行为需通过小扰动/数值验证讨论，不能自动继承全局有限时间结论。

---

## 11. 若保留质量和转动惯量缩放

若理论输入仍使用广义力

\[
\tau=
\begin{bmatrix}
F_x\\
F_y\\
M_z
\end{bmatrix},
\]

定义

\[
D=
\operatorname{diag}
\left(
\frac1m,\frac1m,\frac1I
\right).
\]

则冻结名义系统可写成

\[
\boxed{
\dot e=A_Fe+B\tau,
}
\]

其中

\[
\boxed{
B=
\begin{bmatrix}
0\\
D
\end{bmatrix}.
}
\]

以下推导同时给出带 \(D\) 的一般形式。若工程内部直接使用归一化加速度虚拟输入，只需令

\[
D=I_3.
\]

---

## 12. 可控性证明

计算

\[
A_FB
=
\begin{bmatrix}
D\\
0
\end{bmatrix}.
\]

因此只需前两阶可控矩阵

\[
\mathcal C_2
=
[B,\ A_FB]
=
\begin{bmatrix}
0&D\\
D&0
\end{bmatrix}.
\]

只要

\[
m>0,\qquad I>0,
\]

则 \(D\) 可逆，因此

\[
\boxed{
\operatorname{rank}\mathcal C_2=6.
}
\]

故

\[
\boxed{
(A_F,B)\ \text{完全可控}.
}
\]

该结论不依赖 \(\omega_l\) 的具体数值。

---

## 13. 广义齐次生成元的解析构造

考虑广义齐次升级所需代数关系

\[
A_FG_0-G_0A_F+BY_0=A_F,
\]

\[
G_0B=0.
\]

对

\[
A_F=
\begin{bmatrix}
F_L&I\\
0&0
\end{bmatrix},
\qquad
B=
\begin{bmatrix}
0\\
D
\end{bmatrix},
\]

取

\[
\boxed{
G_0=
\begin{bmatrix}
-I_3&0\\
F_L&0
\end{bmatrix}.
}
\]

再取

\[
\boxed{
Y_0=
\begin{bmatrix}
D^{-1}F_L^2&
D^{-1}F_L
\end{bmatrix}.
}
\]

首先，

\[
G_0B
=
\begin{bmatrix}
-I&0\\
F_L&0
\end{bmatrix}
\begin{bmatrix}
0\\
D
\end{bmatrix}
=0.
\]

其次，

\[
A_FG_0-G_0A_F
=
\begin{bmatrix}
F_L&I\\
-F_L^2&-F_L
\end{bmatrix},
\]

而

\[
BY_0
=
\begin{bmatrix}
0&0\\
F_L^2&F_L
\end{bmatrix}.
\]

故

\[
\boxed{
A_FG_0-G_0A_F+BY_0=A_F.
}
\]

所以 \(G_0,Y_0\) 是广义齐次升级代数方程的一组解析解。

---

## 14. 基反馈 \(K_0\) 的解析形式

定义

\[
K_0=Y_0(G_0-I_6)^{-1}.
\]

有

\[
G_0-I_6
=
\begin{bmatrix}
-2I&0\\
F_L&-I
\end{bmatrix},
\]

其逆为

\[
(G_0-I_6)^{-1}
=
\begin{bmatrix}
-\frac12I&0\\
-\frac12F_L&-I
\end{bmatrix}.
\]

因此

\[
\boxed{
K_0
=
\begin{bmatrix}
-D^{-1}F_L^2&
-D^{-1}F_L
\end{bmatrix}.
}
\]

---

## 15. 基闭环的二阶幂零结构

定义

\[
A_0=A_F+BK_0.
\]

代入上式：

\[
\boxed{
A_0=
\begin{bmatrix}
F_L&I\\
-F_L^2&-F_L
\end{bmatrix}.
}
\]

直接计算：

\[
\boxed{
A_0^2=0.
}
\]

这表明虽然 6D 冻结模型包含 Leader-frame 转动项 \(F_L\)，通过基反馈 \(K_0\) 后仍可恢复二阶幂零结构，与双积分链的齐次构造具有直接联系。

---

## 16. 齐次 dilation

选择负齐次度

\[
\boxed{
-1<\nu<0.
}
\]

定义

\[
\boxed{
G_d=I_6+\nu G_0.
}
\]

即

\[
\boxed{
G_d=
\begin{bmatrix}
(1-\nu)I_3&0\\
\nu F_L&I_3
\end{bmatrix}.
}
\]

由于 \(G_0\) 的特征值为

\[
\{-1,-1,-1,0,0,0\},
\]

所以 \(G_d\) 的特征值为

\[
\{1-\nu,1-\nu,1-\nu,1,1,1\}.
\]

当 \(-1<\nu<0\) 时全部为正，因此可作为 dilation generator。

此外，

\[
G_dB=B,
\]

并且由

\[
A_0G_0-G_0A_0=A_0
\]

可得

\[
\boxed{
A_0G_d-G_dA_0=\nu A_0.
}
\]

这是后续广义齐次闭环构造的核心代数关系。

---

## 17. 稳定线性反馈与 Lyapunov 条件

选取一个线性反馈矩阵

\[
K\in\mathbb R^{3\times6}
\]

使

\[
\boxed{
A_K=A_F+BK
}
\]

为 Hurwitz。

然后寻找

\[
P=P^T>0
\]

满足所采用广义齐次控制定理要求的 Lyapunov / dilation 兼容条件。

工程检查至少应包含

\[
\boxed{
PA_K+A_K^TP<0
}
\]

以及

\[
\boxed{
PG_d+G_d^TP>0.
}
\]

> **重要**：论文最终写定理时，应逐项对照所引用的广义齐次控制原始定理，确认全部假设和矩阵不等式完全一致；不能仅凭 “\(A_K\) Hurwitz” 就直接宣称有限时间稳定。

若已知某个 \(P>0\) 使

\[
PA_K+A_K^TP<0,
\]

则

\[
PG_d+G_d^TP
=
2P+\nu(PG_0+G_0^TP).
\]

因此当 \(\nu\) 足够接近 0 且为负时，第二个正定条件必然存在可行区间。

一个保守充分条件为

\[
\boxed{
|\nu|
<
\frac{
2\lambda_{\min}(P)
}{
\|PG_0+G_0^TP\|_2
}.
}
\]

同时满足

\[
-1<\nu<0.
\]

---

## 18. 理论齐次控制律

定义由 \(G_d\) 与 \(P\) 诱导的 canonical homogeneous norm

\[
\|e\|_d.
\]

理论控制律采用

\[
\boxed{
u_{\rm th}(e)
=
K_0e
+
\|e\|_d^{1+\nu}
(K-K_0)
\exp\!\left(
-\ln\|e\|_d\,G_d
\right)e.
}
\]

这是理论有限时间结论对应的控制律。

若所引用广义齐次定理的全部条件成立，则对冻结名义系统

\[
\dot e=A_Fe+Bu_{\rm th}(e)
\]

可建立负齐次有限时间稳定结论。

典型 Lyapunov 形式为

\[
\dot V
\le
-cV^{1+\nu},
\qquad
-1<\nu<0,
\]

从而存在有限 settling time。

---

## 19. 工程正则化控制器不能直接继承理论结论

现有工程实现若采用例如

\[
c=\operatorname{clamp}(\|e\|_d,c_{\min},1)
\]

以及

\[
u_{\rm impl}
=
c^{1+\nu}
K
\exp\!\left(
G_d(1-\ln c)
\right)e,
\]

则它与理论控制律

\[
u_{\rm th}
=
K_0e
+
\|e\|_d^{1+\nu}
(K-K_0)
e^{-\ln\|e\|_dG_d}e
\]

并不代数等价。

主要差异包括：

1. 工程式未显式包含 \(K_0e\)；
2. 工程式使用截断后的 \(c\)；
3. 指数矩阵中存在额外平移；
4. 饱和与限幅进一步破坏严格齐次缩放。

因此：

\[
\boxed{
u_{\rm impl}\ \text{不能直接继承}\ u_{\rm th}\ \text{的严格有限时间定理}.
}
\]

论文和 README 必须明确区分：

- `theoretical homogeneous controller`；
- `regularized engineering controller`。

---

## 20. 第一版理论一致 C++ 核心的推荐结构

第一阶段不要直接修改现有 `6d_artstein_disc`。

建议新增独立核心，例如：

```text
homo_controller_6d_theory_consistent.hpp
```

第一版仅支持：

```text
fixed Leader-frame d_p
d_theta = 0
no Artstein
no polygon target switching
no HOCBF
no velocity / wheel saturation inside the theoretical core
```

理论核心计算流程：

```text
map pose
  |
  +--> relative pose -> Leader frame -> e_p
  |
body velocities
  |
  +--> R(e_theta) v_f^b
  |
Leader twist + d_p
  |
  +--> v_d^L = v_l^b + omega_l J d_p
  |
  +--> e_v = R(e_theta)v_f^b - v_d^L
  |
  +--> e_theta, e_omega
  |
  +--> build F_L(omega_l)
  |
  +--> build A_F
  |
  +--> analytic G0, Y0, K0
  |
  +--> theoretical u_th
  |
  +--> inverse input transform
       to follower body-frame command
```

---

## 21. 实际输入映射

理论虚拟平移输入 \(u_v^L\) 位于 Leader frame。

实际 Follower body-frame 加速度命令应由

\[
R(e_\theta)a_f^b
=
a_l^b+\alpha_lJd_p-e_\omega Jv_d^L+u_v^L
\]

得到：

\[
\boxed{
a_f^b
=
R(-e_\theta)
\left(
a_l^b+\alpha_lJd_p-e_\omega Jv_d^L+u_v^L
\right).
}
\]

角加速度命令为

\[
\boxed{
\alpha_f=\alpha_l+u_\omega.
}
\]

对于第一版固定 Leader twist 验证，可设

\[
a_l^b=0,\qquad \alpha_l=0,
\]

从而

\[
\boxed{
a_f^b
=
R(-e_\theta)
\left(
-e_\omega Jv_d^L+u_v^L
\right).
}
\]

这一步必须与代码输出坐标系严格对应。

---

## 22. 必须增加的单元测试

### 22.1 坐标一致性测试

随机生成

\[
\theta_l,\theta_f,v_f^b
\]

检查

\[
R(e_\theta)v_f^b
\]

确实等于先旋转到 map 再旋转到 Leader frame 的结果：

\[
R(-\theta_l)R(\theta_f)v_f^b.
\]

应满足数值误差接近机器精度。

---

### 22.2 转弯零误差平衡点测试

取

\[
d_p\neq0,\qquad \omega_l\neq0.
\]

构造

\[
e_p=0,\quad e_\theta=0,\quad e_v=0,\quad e_\omega=0.
\]

这要求

\[
R(e_\theta)v_f^b
=
v_l^b+\omega_lJd_p.
\]

检查 exact error dynamics 是否满足

\[
\boxed{
\dot e=0
}
\]

在名义前馈输入下成立。

这是当前旧实现最需要补的测试。

---

### 22.3 解析矩阵恒等式测试

对多个 \(\omega_l\) 取值检查：

\[
G_0B=0,
\]

\[
A_FG_0-G_0A_F+BY_0=A_F,
\]

\[
K_0=Y_0(G_0-I)^{-1},
\]

\[
A_0=A_F+BK_0,
\]

\[
A_0^2=0.
\]

容差建议：

```text
1e-10 ~ 1e-8
```

取决于 Eigen 数值误差。

---

### 22.4 可控性测试

检查

\[
\operatorname{rank}[B,A_FB]=6.
\]

---

### 22.5 Hurwitz / Lyapunov 裕度扫描

在预计工作范围内扫描

\[
\omega_l\in[\omega_{\min},\omega_{\max}]
\]

检查：

\[
\max\Re\lambda(A_F+BK)<0,
\]

并记录稳定裕度。

同时检查

\[
\lambda_{\min}(PG_d+G_d^TP)>0.
\]

---

### 22.6 非线性残差二阶性测试

随机取一组方向 \(e_0\)，令

\[
e=\varepsilon e_0,
\]

逐步减小

\[
\varepsilon.
\]

比较 exact error dynamics 与冻结线性模型：

\[
r(\varepsilon)
=
f_{\rm exact}(\varepsilon e_0)
-
A_F(\varepsilon e_0)
-
Bu.
\]

理论上应观察到

\[
\|r(\varepsilon)\|
=
O(\varepsilon^2).
\]

在 log-log 图上斜率应接近 2。

---

## 23. 推荐工程开发顺序

### Phase 0：理论—代码对齐

只完成：

- 新误差坐标；
- \(v_d^L=v_l^b+\omega_lJd_p\)；
- exact dynamics；
- `build_A(omega_l)`；
- 解析 \(G_0,Y_0,K_0\)；
- 单元测试。

---

### Phase 1：理论控制器最小闭环

仅运行

\[
u_{\rm th}.
\]

固定 \(d_p\)，固定/缓慢变化 Leader twist。

先做 MATLAB / C++ 数值模型，再进 Gazebo。

---

### Phase 2：工程正则化

再加入：

- homogeneous norm 数值保护；
- \(c_{\min}\)；
- acceleration saturation；
- velocity limit；
- wheel-speed limit。

此阶段只讨论 numerical / practical behavior，不再直接声称严格有限时间稳定。

---

### Phase 3：6D Artstein / predictor

在 Phase 1–2 已稳定后，再加入：

- \(T_d\)；
- \(\tau\)；
- translation prediction；
- yaw prediction；
- delay-aware state reconstruction。

不要在理论核心尚未与代码一致时同时修改 Artstein。

---

### Phase 4：离散编队点与 HOCBF

最后才加入：

- polygon target switching；
- tolerance switching；
- HOCBF；
- QP；
- obstacle fitting。

这些都属于理论名义冻结模型之外的混杂/安全扩展。

---

## 24. 与旧 6D 实现的关键差异清单

Codex 修改工程前应逐项核对：

### 差异 1：速度误差

旧实现类似

\[
R(e_\theta)v_f^b-v_l^b
\]

新理论应为

\[
\boxed{
R(e_\theta)v_f^b-v_l^b-\omega_lJd_p.
}
\]

---

### 差异 2：`build_A`

旧 `A` 若包含

\[
-v_{y,l}e_\theta,\qquad v_{x,l}e_\theta
\]

则与新的 Leader-frame velocity error 不一致。

新理论在完成耦合补偿后应采用

\[
\boxed{
F_L=
\begin{bmatrix}
0&\omega_l&0\\
-\omega_l&0&0\\
0&0&0
\end{bmatrix}.
}
\]

即 \(A_F\) 不再通过 \(v_l\) 在位置方程中显式耦合 \(e_\theta\)。

---

### 差异 3：新增输入耦合补偿

新理论需要

\[
\boxed{
-e_\omega Jv_d^L
}
\]

作为 body-frame 实际输入映射中的耦合补偿项。

这是为了把一阶项 \(e_\omega Jv_d^L\) 从速度误差动力学中消除，使剩余非线性项为二阶

\[
e_\omega Je_v.
\]

---

### 差异 4：理论控制律与工程控制律

严格有限时间结论只对应理论式 \(u_{\rm th}\)。

现有截断/平移后的 `u_impl` 必须单独标记为工程正则化实现。

---

## 25. 当前理论结论的边界

可以声明：

> 对固定 Leader-frame 编队偏移、局部姿态误差、冻结 Leader yaw rate，并在采用坐标一致误差定义与名义输入耦合补偿后，6D Leader–Follower 误差系统可分解为一个具有二阶非线性残差的冻结线性可控系统。其名义线性部分可以解析构造广义齐次生成元 \(G_0\)、\(Y_0\) 与基反馈 \(K_0\)，并在满足所引用广义齐次控制定理全部矩阵条件时建立有限时间稳定性结论。

暂时不要声明：

- 原始完整 6D 非线性系统全局有限时间稳定；
- 工程正则化控制器严格有限时间稳定；
- polygon switching 下严格有限时间稳定；
- Artstein + 6D + saturation 的全局有限时间稳定；
- HOCBF 激活期间仍保留原齐次有限时间跟踪结论。

---

## 26. 给 Codex 的最短实施指令

如果只给 Codex 一段任务摘要，可以使用：

```text
Do not modify the existing 6d_artstein_disc controller directly.

Create a new theory-consistent 6D controller core.

Use:
ep = R(-theta_l) * (pf - pl) - dp
etheta = wrap(theta_f - theta_l)
vd_L = vl_body + omega_l * J * dp
ev = R(etheta) * vf_body - vd_L
eomega = omega_f - omega_l

For the first version:
- dp is fixed in Leader frame
- dtheta = 0
- no Artstein
- no polygon switching
- no HOCBF

Use the frozen nominal model:
qdot = F_L q + eta
etadot = D u
F_L = [[0, omega_l, 0],
       [-omega_l, 0, 0],
       [0, 0, 0]]

Build analytic:
G0 = [[-I, 0],
      [F_L, 0]]

Y0 = [D^{-1} F_L^2, D^{-1} F_L]

K0 = [-D^{-1} F_L^2, -D^{-1} F_L]

Verify:
G0*B = 0
A*G0 - G0*A + B*Y0 = A
(A + B*K0)^2 = 0

The physical/body-frame input mapping must compensate:
-eomega * J * vd_L

Keep theoretical u_th and regularized u_impl as two distinct code paths.
Do not claim the finite-time theorem for u_impl.
```

---

## 27. 结论

本次重新推导后的核心修正是：

\[
\boxed{
e_v
=
R(e_\theta)v_f^b
-
v_l^b
-
\omega_lJd_p
}
\]

而不是跨坐标系直接写

\[
v_f^b-v_d.
\]

在此基础上：

\[
\boxed{
\dot e_p=-\omega_lJe_p+e_v
}
\]

可精确成立。

再通过输入变换补偿

\[
e_\omega Jv_d^L,
\]

速度误差动力学化为

\[
\boxed{
\dot e_v=u_v^L+e_\omega Je_v.
}
\]

其中剩余非线性项为二阶小量。

因此冻结名义线性部分恢复为

\[
\boxed{
A_F=
\begin{bmatrix}
F_L&I\\
0&0
\end{bmatrix},
}
\]

并可继续使用解析的

\[
\boxed{
G_0,\quad Y_0,\quad K_0
}
\]

完成广义齐次升级。

这应作为后续 6D 工程重构的理论基准版本。
