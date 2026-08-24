# 第 5 章 6D Artstein 预测状态 HOCBF 安全过滤

## 本章结构

本章按“问题与模型 → HOCBF 推导 → 硬约束 QP 与半径设计 → 理论边界 → 数值验证 → 局限”的顺序组织。本文只对静态环境障碍物给出理论模型；编队恢复属于名义 6D Artstein Disc 控制器的性能问题，不属于 HOCBF 安全定理。

## 5.1 问题定义与模型

### 5.1.1 结论范围

本文为“6D Artstein Disc 名义编队控制 + 静态障碍物安全滤波”给出可核对的理论骨架。第一阶段只处理 map 坐标系二维平移、静态圆形障碍物、已知常值输入时延和准确的一阶执行器模型。

若第 4 节假设成立、HOCBF 输入集合始终非空且初始命令历史安全，第 5 节的预测状态 HOCBF 约束保证模型状态安全。该结论不覆盖现有 ROS 节点中的坐标切换、轮速缩放、命令饱和、20 Hz 保持、scan 遮挡、动态障碍物或 tau/Td 失配；第 7 节说明这些缺口。

“6D”仅指名义控制器还有 yaw 与角速度。本安全层第一版只约束位置；yaw 不进入本安全定理。

### 5.1.2 输入时延与预测器

令 \(\mathbf p=[p_x,p_y]^\mathsf T\)、\(\mathbf v=[v_x,v_y]^\mathsf T\) 为 map 系位置和速度，\(\mathbf u\) 为实际发布的 map 系线速度命令。采用一阶执行器和常值死区模型：

$$
\dot{\mathbf x}=A\mathbf x+B\mathbf u(t-T_d),\qquad
\mathbf x=\begin{bmatrix}\mathbf p\\\mathbf v\end{bmatrix},
\qquad
A=\begin{bmatrix}0&I_2\\0&-\tau^{-1}I_2\end{bmatrix},
\qquad
B=\begin{bmatrix}0\\\tau^{-1}I_2\end{bmatrix},
\quad \tau>0,\ T_d\ge0. \tag{1}
$$

给定完整准确的发布命令历史 u(s)，s in [t-Td,t]，定义

$$
\mathbf x_p(t)=e^{AT_d}\mathbf x(t)
+\int_{t-T_d}^{t}e^{A(t-s)}B\mathbf u(s)\,ds. \tag{2}
$$

变参数公式给出 xp(t)=x(t+Td)，并且

$$
\dot{\mathbf x}_p(t)=A\mathbf x_p(t)+B\mathbf u(t). \tag{3}
$$

因此，当前命令对预测状态无时延。式 (1) 是现有 Artstein 型历史积分在线性平移子系统上的精确形式；它不能无条件代表真实底盘。

## 5.2 静态障碍物与二阶 HOCBF 推导

第 j 个障碍物的中心为 oj。合并安全半径必须取

$$
R_j=r_{\mathrm{robot}}+r_{\mathrm{obs},j}
+\epsilon_{\mathrm{geom}}+\epsilon_{\mathrm{meas}}+\epsilon_{\mathrm{disc}}. \tag{4}
$$

前三项分别对应机器人包络、障碍物外接半径和几何建模误差；后两项必须由测量或保守设计给定。

设 \(\mathbf r_j=\mathbf p_p-\mathbf o_j\)，安全函数为

$$
h_j(\mathbf p_p)=\|\mathbf p_p-\mathbf o_j\|^2-R_j^2. \tag{5}
$$

选取 c1>0,c2>0，定义

$$
\psi_{0,j}=h_j,\qquad
\psi_{1,j}=2\mathbf r_j^\mathsf T\mathbf v_p+c_1h_j. \tag{6}
$$

由 (2)，二阶 HOCBF 条件是

$$
\psi_{2,j}=2\mathbf v_p^\mathsf T\mathbf v_p
+2\mathbf r_j^\mathsf T\!\left(-\frac{\mathbf v_p}{\tau}+\frac{\mathbf u}{\tau}\right)
+2c_1\mathbf r_j^\mathsf T\mathbf v_p
+c_2\left(2\mathbf r_j^\mathsf T\mathbf v_p+c_1h_j\right)\ge0. \tag{7}
$$

所以它是对当前命令的仿射硬约束：

$$
\mathbf a_j(\mathbf x_p)^\mathsf T\mathbf u\ge b_j(\mathbf x_p),
\qquad \mathbf a_j=\frac{2\mathbf r_j}{\tau}, \tag{8}
$$

$$
b_j=-2\mathbf v_p^\mathsf T\mathbf v_p
+\frac{2\mathbf r_j^\mathsf T\mathbf v_p}{\tau}
-2c_1\mathbf r_j^\mathsf T\mathbf v_p
-c_2\left(2\mathbf r_j^\mathsf T\mathbf v_p+c_1h_j\right). \tag{9}
$$

这与当前 OA 的“接近速度软惩罚”不同：式 (7) 必须作为硬约束。

真正的扩展安全集是

$$
\mathcal C=\bigcap_j
\left\{\mathbf x_p:h_j(\mathbf p_p)\ge0,\ \psi_{1,j}(\mathbf x_p)\ge0\right\}. \tag{10}
$$

h>=0 本身不足以保证可避免碰撞；高速冲向障碍物时，h 可为正但 psi1<0，受限输入下已可能不可救。

### 5.2.1 理论假设

- A1：真实平移系统严格满足状态方程；tau 与 Td 为已知常值；保存的发布命令历史无误，所以 predictor 精确。
- A2：障碍物静态，圆外接集和 (3) 的半径覆盖真实几何；机器人以 r_robot 圆包络。
- A3：执行的是连续、局部 Lipschitz 的反馈；此处不处理采样保持。
- A4：[0,Td] 内预存命令历史作用下的真实轨迹安全，且 xp(0) 属于 (8)。
- A5：对每个 \(\mathbf x_p\in\mathcal C\)，集合
  $$
  K(\mathbf x_p)=
  \left\{\mathbf u\in U:
  \mathbf a_j(\mathbf x_p)^\mathsf T\mathbf u\ge b_j(\mathbf x_p),\
  \forall j\right\} \tag{11}
  $$
  非空；\(U\) 是速度、命令变化率和轮速限制转换后的闭凸输入集。

### 5.2.2 连续时间模型级安全定理

**定理 1。** 在 A1--A5 下，若局部 Lipschitz 反馈满足 \(\mathbf u(\mathbf x_p)\in K(\mathbf x_p)\)，则预测状态保持于 \(\mathcal C\)。又因为 predictor 精确满足 \(\mathbf x_p(t)=\mathbf x(t+T_d)\)，加上 A4 的初始历史安全性，原输入时延系统满足所有 \(h_j(\mathbf p(t))\ge0\)。

**证明骨架。** 式 (7) 令每个 psi2,j>=0。比较原理先保持 psi1,j>=0，再由 h_dot+c1 h>=0 保持 hj>=0。多障碍物取半空间交，A5 保证可选择控制。式 (1) 将预测状态与真实状态平移 Td；A4 补足第一个时延窗口。

这是标准高相对阶 CBF 与 predictor feedback 的组合。它只在假设完整成立时才是模型级证明。

## 5.3 硬约束 QP、输入约束与半径设计

### 5.3.1 与 6D Artstein 名义命令的接口

将 6D Artstein Disc 的 map 系平移输出记为 \(\mathbf u_{\mathrm{nom}}\)。滤波器求解

$$
\mathbf u^*=\arg\min_{\mathbf u\in U}
\frac12(\mathbf u-\mathbf u_{\mathrm{nom}})^\mathsf T
W(\mathbf u-\mathbf u_{\mathrm{nom}})
\quad
\mathrm{s.t.}\quad
\mathbf a_j^\mathsf T\mathbf u\ge b_j,\ \forall j,
\qquad W\succ0. \tag{12}
$$

该问题在 \(K(\mathbf x_p)\) 非空时有唯一解。名义控制器只决定安全集内性能；HOCBF 不证明编队误差收敛。为了保持 A1，写入 predictor 历史的必须是最终发布的 \(\mathbf u^*\)，而不是滤波前、限幅前或车体系未旋转的命令。

### 5.3.2 输入集与可行性

实际 QP 的输入集应明确写为

$$
U_k=\left\{
u:\|u\|_\infty\le v_{\max},\quad
\|u-u_{k-1}\|_\infty\le a_{\max}T_s,\quad
\|\Omega(u,\omega)\|_\infty\le\Omega_{\max}
\right\}.
$$

其中 $T_s=0.05$ s，$\Omega$ 是全向底盘逆运动学得到的轮速向量。只有 $K(x_p)\cap U_k$ 非空时，安全 QP 才有硬约束可行解。

### 5.3.3 物理半径与过滤半径

$$
R_{\rm physical}=r_{\rm robot}+r_{\rm obstacle}+d_{\rm clearance},
\qquad
R_{\rm filter}=R_{\rm physical}+\epsilon_{\rm coupled}.
$$

$\epsilon_{\rm coupled}$ 是采样、死区、执行器滞后和预测残差的经验数值裕度；红色物理圆与 HOCBF 过滤圆必须在数值图中区分。

在定理 1 的精确连续 predictor 条件、可行输入集和扩展安全初值成立时，预测安全函数以 $R_{\rm filter}$ 定义，因此预测状态不应越过过滤圆。当前 20 Hz 6D Artstein Disc 耦合实现不满足该完整理想条件：它包含零阶保持、有限历史积分、body/map yaw 耦合、命令后处理和离散预测残差。因此实际 follower 轨迹可能轻微越过 $R_{\rm filter}$ 对应的圆。

若仍满足

$$
d_j(t)\ge R_{{\rm physical},j},
$$

则可解释为内部经验过滤裕度被采样和预测误差消耗；该现象不能被表述为过滤圆的严格前向不变性。若实际轨迹越过 $R_{{\rm physical},j}$，则当前过滤半径或安全层参数对该场景不合格，必须增加 $\epsilon_{{\rm coupled},j}$、降低控制周期或重新检查预测模型与 QP 可行性。

### 5.3.4 多静态圆柱障碍物

对第 $j$ 个圆柱，合并物理安全半径写为

$$
R_{{\rm physical},j}
=r_{\rm follower}
+r_{{\rm cylinder},j}
+d_{{\rm clearance},j},
$$

内部 HOCBF 过滤半径为

$$
R_{{\rm filter},j}
=R_{{\rm physical},j}
+\epsilon_{{\rm coupled},j}.
$$

每个圆柱各自产生一条二阶 HOCBF 半空间

$$
a_j(x_p)^\mathsf T u\ge b_j(x_p),
\qquad j=1,\ldots,M.
$$

安全 QP 同时处理所有圆柱：

$$
\begin{aligned}
u^*=\arg\min_{u\in U_k}\quad&
\frac12(u-u_{\rm Artstein})^\mathsf T W(u-u_{\rm Artstein})\\
\text{s.t.}\quad&
a_j(x_p)^\mathsf T u\ge b_j(x_p),
\qquad j=1,\ldots,M.
\end{aligned}
$$

相应可行输入集合为

$$
K_M(x_p)
=U_k\cap
\bigcap_{j=1}^{M}
\left\{u:a_j(x_p)^\mathsf T u\ge b_j(x_p)\right\}.
$$

连续时间安全结论仍要求 $K_M(x_p)\neq\varnothing$。多个圆柱将 follower 夹在狭窄区域、输入死区已经作用或速度/轮速约束过严时，该交集可能为空；QP 无解必须作为可行性边界记录，不能用松弛变量后仍宣称硬安全保证。

## 5.4 理论边界与离散实现

### 5.4.1 20 Hz 采样

定理 1 是连续时间结论。在每 50 ms 解一次 (9) 并零阶保持，不能直接引用它。必须选择：

1. 使用精确 ZOH 模型和离散 CBF；距离平方安全函数通常导致 QCQP/NLP，而不是天然 QP。
2. 保留连续 HOCBF-QP，但在紧致工作域推导 psi2 在一个采样间隔内的变化上界，并施加大于该上界的严格裕度。
3. 将 20 Hz QP 明确标为工程安全滤波器，仅用高频参考仿真验证，不声称离散前向不变性。

未完成其中之一，数值仿真只能验证实现，不能形成 20 Hz 的安全定理。

### 5.4.2 预测、感知和模型误差

若预测误差满足 \(\|\mathbf x_p-\hat{\mathbf x}_p\|\le\epsilon_x\)，且式 (7) 在紧致域上对状态 Lipschitz，常数为 \(L_{\psi,j}\)，则以估计状态计算时至少应要求

$$
\psi_{2,j}(\hat{\mathbf x}_p,\mathbf u)
\ge L_{\psi,j}\epsilon_x+\delta_{\mathrm{sd}}. \tag{13}
$$

\(\delta_{\mathrm{sd}}\) 是采样裕度。只有 \(\epsilon_x\) 和 \(L_{\psi,j}\) 都有可信上界时，式 (13) 才形成鲁棒 CBF 条件。扫描 \(\tau/T_d\) 失配没有碰撞，只能为裕度选取提供经验，不能取代上界证明。动态障碍物还要对其预测误差建立同类界，本阶段不纳入。

### 5.4.3 可行性与后备制动

若 \(K(\mathbf x_p)\) 为空，CBF 松弛变量只恢复数值可解性，会失去定理 1 的结论。后备制动是工程保护，但除非另行构造其正不变制动域，也不恢复严格保证。

一个仅用于提前降级的保守径向停止距离下界是

$$
d_{\mathrm{stop}}=v_{\max}(T_d+\tau). \tag{14}
$$

它假定时延窗口和执行器衰减阶段都以 \(v_{\max}\) 前进。真实触发阈值还要叠加采样、测量和几何裕度；这不是完整可控性核。

### 5.4.4 6D body/map 耦合残差

6D Disc 的真实平移速度满足

$$
v^m=R(\theta)v^b,\qquad
\dot v^m=R(\theta)\dot v^b+\omega Jv^m,
\qquad
J=\begin{bmatrix}0&-1\\1&0\end{bmatrix}.
$$

第二项 $\omega Jv^m$ 未包含在 map 系一阶执行器名义模型中。因而本章定理严格针对 map 系近似模型；完整 6D Disc 耦合中该项只能作为有界预测残差进入工程数值验证。

## 5.5 数值验证与理论主张

| 验证 | 支持或反驳的主张 | 不能替代 |
| --- | --- | --- |
| 式 (6)--(7) 的解析与单元测试 | HOCBF 符号和实现方向正确 | A1--A5 |
| 准确模型的连续小步长仿真 | 定理 1 的数值一致性 | 一般性数学证明 |
| 20 Hz 与 1 kHz 对照 | 离散实现的经验裕度 | sampled-data 定理 |
| 延迟和测量误差扫描 | 保守裕度的证据 | 严格误差上界 |
| QP 不可行与制动测试 | 可行域边界 | 后备的正不变证明 |

### 5.5.1 6D Artstein Disc 耦合避障的理论定位

#### 分层结构

本项目数值耦合采用下列分层：

$$
\text{6D Artstein Disc 名义编队控制}
\longrightarrow
\text{map 系 HOCBF-QP}
\longrightarrow
\text{最终 body 系 cmd\_vel}.
$$

6D Artstein Disc 提供编队性能目标；HOCBF-QP 仅修改平移命令以满足静态环境障碍约束；偏航命令第一版保持名义控制器输出。实际发布的安全命令必须回写平移 Artstein 历史，且回写时使用发布瞬间的测量 yaw 将 body 系命令转换到 map 系。使用预测 yaw 回写会破坏预测器坐标一致性。

#### HOCBF 的安全对象

环境障碍物使用

$$
h_o(p)=\|p-p_o\|^2-R_{\rm filter}^2.
$$

其中 $R_{\rm filter}$ 是 HOCBF 内部过滤半径。它应与物理安全半径区分：

$$
R_{\rm filter}=R_{\rm physical}+\epsilon_{\rm coupled},
\qquad
R_{\rm physical}=r_{\rm robot}+r_{\rm obstacle}+d_{\rm clearance}.
$$

$\epsilon_{\rm coupled}$ 是为采样、执行器死区、滞后和预测偏差保留的经验数值裕度；它不是感知、定位或任意模型不确定性的理论上界。数值图必须同时绘制 $R_{\rm physical}$ 与 $R_{\rm filter}$。

#### 编队恢复不属于 HOCBF 定理

在定理 1 的 A1--A5 条件、连续反馈、QP 始终可行且扩展安全初值成立时，HOCBF 约束保证预测状态安全集的前向不变性。它不证明 follower 能从任意绕障偏离状态回到原 Disc 编队轨迹，也不负责选择绕行侧或生成全局无碰撞路径。若原编队参考点被障碍物占据，目标本身不可行；持续 HOCBF 激活与无法恢复是正常现象，不应解释为 HOCBF 失效。

当名义 6D Artstein 命令重新落入全部 HOCBF 与输入约束的可行集时，它重新成为 QP 最优解：

$$
u_{\rm Artstein}\in K(x_p)
\Longrightarrow
u_{\rm safe}=u_{\rm Artstein}.
$$

障碍物位于原轨迹旁侧本身不足以保证上述可行性，延迟、采样、过滤半径和目标切换均可能保持约束激活。局部参考、A* 或虚拟 Leader 属于性能/规划层，不能被纳入当前 HOCBF 前向不变性结论。

#### 当前数值结论与禁止表述

当前 Python 耦合实验验证了：在精确静态圆障碍物、已知 tau/Td、固定控制周期的模型中，HOCBF 可作为 6D Artstein Disc 的命令安全过滤器。该实验中的预测状态来自离散 Artstein 近似、有限历史缓冲和 body/map 坐标变换，不能等同于定理 1 的精确连续 predictor state；实际最小物理距离、预测 $h$、QP 不可行次数、命令修正范数和末段修正仅构成工程数值证据。

不得由有限次数值实验宣称：

- 对任意初值、任意障碍物形状或任意延迟的安全；
- 含 scan、定位、TF 或未知执行器参数时的安全；
- HOCBF 单独保证编队恢复；
- 20 Hz 离散实现已具有未推导的 sampled-data 安全证明。

#### 多圆柱数值指标

多圆柱实验应分别记录

$$
d_j(t)=\|p_f(t)-p_{o,j}\|,
\qquad
h_j(t)=\|p_f(t)-p_{o,j}\|^2-R_{{\rm physical},j}^2,
$$

并报告总体最小物理距离

$$
d_{\min}=\min_{j\in\{1,\ldots,M\},\,t}d_j(t),
$$

各圆柱约束激活次数、QP 可行/不可行次数，以及圆柱间距和过滤半径扫描结果。

## 5.6 本章小结与局限

本章在已知常值输入死区、一阶执行器和静态圆障碍物的模型下，构造 predictor-HOCBF 硬约束 QP。理论结论严格限定于连续 map 系近似模型、可行输入集和扩展安全初值；20 Hz 实现、完整 6D body/map 耦合和过滤半径仅以数值证据评估。移动障碍物、Leader 相对 HOCBF、scan 感知与局部路径规划留作后续工作。

## 参考文献

1. A. D. Ames, X. Xu, J. W. Grizzle, P. Tabuada, “Control Barrier Function Based Quadratic Programs for Safety Critical Systems,” IEEE TAC, 2017. https://doi.org/10.1109/TAC.2016.2638961
2. T. G. Molnar, A. K. Kiss, A. D. Ames, G. Orosz, “Safety-Critical Control With Input Delay in Dynamic Environment,” IEEE TCST, 2023. https://doi.org/10.1109/TCST.2022.3228712
3. A. Agrawal, K. Sreenath, “Discrete Control Barrier Functions for Safety-Critical Control of Discrete Systems,” RSS, 2017. https://www.roboticsproceedings.org/rss13/p73.pdf
4. Y. Kim, J. Kim, A. Ames, C. Sloth, “Robust Safety-Critical Control for Input-Delayed System with Delay Estimation,” ECC, 2024. https://doi.org/10.23919/ECC64448.2024.10591073
