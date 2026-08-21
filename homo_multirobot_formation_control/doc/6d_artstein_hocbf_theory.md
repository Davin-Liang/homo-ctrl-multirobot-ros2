# 6D Artstein 预测状态 HOCBF：理论基础与适用边界

## 结论范围

本文为“6D Artstein Disc 名义编队控制 + 静态障碍物安全滤波”给出可核对的理论骨架。第一阶段只处理 map 坐标系二维平移、静态圆形障碍物、已知常值输入时延和准确的一阶执行器模型。

若第 4 节假设成立、HOCBF 输入集合始终非空且初始命令历史安全，第 5 节的预测状态 HOCBF 约束保证模型状态安全。该结论不覆盖现有 ROS 节点中的坐标切换、轮速缩放、命令饱和、20 Hz 保持、scan 遮挡、动态障碍物或 tau/Td 失配；第 7 节说明这些缺口。

“6D”仅指名义控制器还有 yaw 与角速度。本安全层第一版只约束位置；yaw 不进入本安全定理。

## 输入时延与预测器

令 p=[px,py]^T、v=[vx,vy]^T 为 map 系位置和速度，u 为实际发布的 map 系线速度命令。采用一阶执行器和常值死区模型：

x_dot = A x + B u(t-Td)，其中 x=[p^T,v^T]^T，

A=[[0,I2],[0,-I2/tau]]，B=[[0],[I2/tau]]，tau>0，Td>=0。

给定完整准确的发布命令历史 u(s)，s in [t-Td,t]，定义

xp(t) = exp(A Td) x(t) + integral(t-Td,t) exp(A(t-s)) B u(s) ds。 (1)

变参数公式给出 xp(t)=x(t+Td)，并且

xp_dot(t)=A xp(t)+B u(t)。 (2)

因此，当前命令对预测状态无时延。式 (1) 是现有 Artstein 型历史积分在线性平移子系统上的精确形式；它不能无条件代表真实底盘。

## 静态圆障碍物与二阶 CBF

第 j 个障碍物的中心为 oj。合并安全半径必须取

Rj = r_robot + r_obs,j + eps_geom + eps_meas + eps_disc。 (3)

前三项分别对应机器人包络、障碍物外接半径和几何建模误差；后两项必须由测量或保守设计给定。

设 rj=pp-oj，安全函数为

hj(pp)=||pp-oj||^2-Rj^2。 (4)

选取 c1>0,c2>0，定义

psi0,j=hj，

psi1,j=2 rj^T vp + c1 hj。 (5)

由 (2)，二阶 HOCBF 条件是

psi2,j = 2 vp^T vp + 2 rj^T (-vp/tau + u/tau) + 2 c1 rj^T vp + c2 (2 rj^T vp + c1 hj) >= 0。 (6)

所以它是对当前命令的仿射硬约束：

aj(xp)^T u >= bj(xp)，

aj=2 rj/tau，

bj=-2 vp^T vp + 2 rj^T vp/tau - 2 c1 rj^T vp - c2(2 rj^T vp+c1 hj)。 (7)

这与当前 OA 的“接近速度软惩罚”不同：式 (7) 必须作为硬约束。

真正的扩展安全集是

C = intersection_j { xp : hj>=0 and psi1,j>=0 }。 (8)

h>=0 本身不足以保证可避免碰撞；高速冲向障碍物时，h 可为正但 psi1<0，受限输入下已可能不可救。

## 假设

- A1：真实平移系统严格满足状态方程；tau 与 Td 为已知常值；保存的发布命令历史无误，所以 predictor 精确。
- A2：障碍物静态，圆外接集和 (3) 的半径覆盖真实几何；机器人以 r_robot 圆包络。
- A3：执行的是连续、局部 Lipschitz 的反馈；此处不处理采样保持。
- A4：[0,Td] 内预存命令历史作用下的真实轨迹安全，且 xp(0) 属于 (8)。
- A5：对每个 xp in C，集合 K(xp)={u in U: aj^T u >= bj, all j} 非空；U 是速度、命令变化率和轮速限制转换后的闭凸输入集。

## 连续时间模型级安全定理

**定理 1。** 在 A1--A5 下，若局部 Lipschitz 反馈满足 u(xp) in K(xp)，则预测状态保持于 C。又因为 predictor 精确满足 xp(t)=x(t+Td)，加上 A4 的初始历史安全性，原输入时延系统满足所有 hj(p(t))>=0。

**证明骨架。** 式 (7) 令每个 psi2,j>=0。比较原理先保持 psi1,j>=0，再由 h_dot+c1 h>=0 保持 hj>=0。多障碍物取半空间交，A5 保证可选择控制。式 (1) 将预测状态与真实状态平移 Td；A4 补足第一个时延窗口。

这是标准高相对阶 CBF 与 predictor feedback 的组合。它只在假设完整成立时才是模型级证明。

## 与 6D Artstein 名义命令的接口

将 6D Artstein Disc 的 map 系平移输出记为 u_nom。滤波器求解

min_u 0.5 (u-u_nom)^T W (u-u_nom)，

满足 u in U 和所有式 (7)，其中 W 正定。 (9)

该问题在 K(xp) 非空时有唯一解。名义控制器只决定安全集内性能；HOCBF 不证明编队误差收敛。为了保持 A1，写入 predictor 历史的必须是最终发布的 u*，而不是滤波前、限幅前或车体系未旋转的命令。

## 尚未闭合的理论缺口

### 20 Hz 采样

定理 1 是连续时间结论。在每 50 ms 解一次 (9) 并零阶保持，不能直接引用它。必须选择：

1. 使用精确 ZOH 模型和离散 CBF；距离平方安全函数通常导致 QCQP/NLP，而不是天然 QP。
2. 保留连续 HOCBF-QP，但在紧致工作域推导 psi2 在一个采样间隔内的变化上界，并施加大于该上界的严格裕度。
3. 将 20 Hz QP 明确标为工程安全滤波器，仅用高频参考仿真验证，不声称离散前向不变性。

未完成其中之一，数值仿真只能验证实现，不能形成 20 Hz 的安全定理。

### 预测、感知和模型误差

若预测误差满足 ||xp-xp_hat||<=eps_x，且式 (6) 在紧致域上对状态 Lipschitz，常数为 Lpsi,j，则以估计状态计算时至少应要求

psi2,j(xp_hat,u) >= Lpsi,j eps_x + delta_sd。 (10)

delta_sd 是采样裕度。只有 eps_x 和 Lpsi,j 都有可信上界时，(10) 才形成鲁棒 CBF 条件。扫描 tau/Td 失配没有碰撞，只能为裕度选取提供经验，不能取代上界证明。动态障碍物还要对其预测误差建立同类界，本阶段不纳入。

### 可行性与后备制动

若 K(xp) 为空，CBF 松弛变量只恢复数值可解性，会失去定理 1 的结论。后备制动是工程保护，但除非另行构造其正不变制动域，也不恢复严格保证。

一个仅用于提前降级的保守径向停止距离下界是 d_stop=v_max(Td+tau)：假定时延窗口和执行器衰减阶段都以 v_max 前进。真实触发阈值还要叠加采样、测量和几何裕度；这不是完整可控性核。

## 数值验证与理论主张的映射

| 验证 | 支持或反驳的主张 | 不能替代 |
| --- | --- | --- |
| 式 (6)--(7) 的解析与单元测试 | HOCBF 符号和实现方向正确 | A1--A5 |
| 准确模型的连续小步长仿真 | 定理 1 的数值一致性 | 一般性数学证明 |
| 20 Hz 与 1 kHz 对照 | 离散实现的经验裕度 | sampled-data 定理 |
| 延迟和测量误差扫描 | 保守裕度的证据 | 严格误差上界 |
| QP 不可行与制动测试 | 可行域边界 | 后备的正不变证明 |

## 参考文献

1. A. D. Ames, X. Xu, J. W. Grizzle, P. Tabuada, “Control Barrier Function Based Quadratic Programs for Safety Critical Systems,” IEEE TAC, 2017. https://doi.org/10.1109/TAC.2016.2638961
2. T. G. Molnar, A. K. Kiss, A. D. Ames, G. Orosz, “Safety-Critical Control With Input Delay in Dynamic Environment,” IEEE TCST, 2023. https://doi.org/10.1109/TCST.2022.3228712
3. A. Agrawal, K. Sreenath, “Discrete Control Barrier Functions for Safety-Critical Control of Discrete Systems,” RSS, 2017. https://www.roboticsproceedings.org/rss13/p73.pdf
4. Y. Kim, J. Kim, A. Ames, C. Sloth, “Robust Safety-Critical Control for Input-Delayed System with Delay Estimation,” ECC, 2024. https://doi.org/10.23919/ECC64448.2024.10591073
