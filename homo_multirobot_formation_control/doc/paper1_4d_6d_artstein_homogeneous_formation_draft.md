# 考虑输入延迟的多机器人齐次编队控制及其全向车体运动学扩展（初稿）

> **说明**：本文为“小论文 1”的中文初稿，覆盖 4D 与 6D Artstein 延迟补偿齐次编队控制。实验数据、统计显著性检验、图表和实物结果均待后续补充；文中不将尚未完成的实验设计表述为实验结论。

## 摘要

针对全向移动机器人 Leader--Follower 编队中由通信/传输死区、底盘速度响应滞后以及车体运动学耦合引起的跟踪性能退化问题，本文研究输入延迟条件下的预测补偿齐次编队控制。首先，针对以位置和 map 系速度为状态的 4D 双积分编队模型，建立“Artstein 输入死区补偿 + 一阶等效速度响应前向预测”的双层结构。Artstein 变换在已知常值死区和完整命令历史的名义条件下消除显式输入时延；随后利用一阶等效执行器模型预测未来状态，并将预测状态送入保留原双积分结构的齐次比例控制器。其次，面向全向机器人 `cmd_vel` 接口，构造包含 map 系位置、偏航角及车体系速度的 6D 混合状态扩展。鉴于姿态旋转使全局常值线性描述不再严格成立，6D 方法采用 map 系平移与偏航通道分别预测、预测状态重组以及 Leader 车体系局部冻结线性化的方式构造 6D Disc 齐次反馈。理论上，4D Artstein 变换对名义 LTI 执行器死区模型成立；6D 冻结名义系统在可控、Hurwitz 线性闭环、固定编队点和无约束条件下继承广义齐次有限时间收敛性质。对于参数失配、采样、饱和、编队点切换、Leader 非匀速运动、坐标变换误差和观测噪声，本文将其统一视为有界扰动，并采用实用稳定性讨论实际闭环。最后，设计数值仿真、ROS 2/Gazebo 双机器人仿真和实物实验的统一验证方案，以比较无补偿控制、4D Artstein-HPC 与 6D Artstein Disc-HPC 在延迟和约束条件下的编队跟踪性能。

**关键词：** 多机器人编队；输入延迟；Artstein 变换；齐次控制；全向移动机器人；预测补偿

## 1 引言

多机器人编队能够通过任务分配和空间协同提升巡检、搬运、搜救与室内服务等任务的效率。相比差速或阿克曼底盘，全向移动机器人可在不改变车体朝向的条件下产生平面任意方向速度，适用于狭窄空间内的协同运动。然而，实际系统中的命令传输、底层驱动与速度闭环会引入输入死区和执行器响应滞后；当控制器仍按无延迟模型设计时，误差反馈与实际运动之间会产生相位差，导致超调、稳态误差增大甚至编队失稳。速度、加速度和轮速约束进一步限制了能够实现的控制动作。

有限时间与齐次控制为快速收敛和鲁棒控制提供了重要工具。Bhat 和 Bernstein 给出了有限时间稳定性的基本理论 [1,2]；Polyakov 的广义齐次控制框架能够由稳定线性反馈构造齐次反馈 [3]。Yuan 等针对全向移动机器人的 Leader--Follower 跟踪与安全问题，给出了线性和齐次控制器的比较 [4]。但该类基础方法主要基于无输入延迟的名义模型，不能直接覆盖实际 `cmd_vel` 接口中的纯死区、速度响应滞后和约束问题。

Artstein reduction 是处理线性输入时延的经典工具 [5]，其核心是利用历史输入构造新状态，将显式输入时延转化为无显式时延的系统。围绕多智能体输入延迟，已有固定时间一致性、时变编队和饱和约束方面的研究 [6--10]。这些工作说明预测反馈与编队控制可以结合，但其对象、控制律和机器人执行器接口与本文不同。另一方面，全向机器人轨迹跟踪研究常采用 MPC 等约束优化路线 [11]；本文选择以齐次控制为名义编队内核，并通过预测补偿降低延迟造成的相位损失。

本文的目标不是重新提出 Artstein 变换或广义齐次理论，而是在已有齐次安全编队方法基础上，构造适用于全向机器人实际速度接口的延迟补偿实现。主要贡献如下：

1. 针对 4D 双积分编队内核，提出 Artstein 死区补偿与一阶等效速度响应预测相结合的状态映射层。该设计不把执行器动态直接并入齐次控制内核，从而保留原始 4D 双积分器的幂零结构和齐次权重。
2. 将上述思路推广至包含位置、偏航和车体系速度的 6D 全向车体级混合状态，提出平移与偏航分解预测、预测状态重组和 Leader 车体系局部冻结的 6D Artstein Disc 框架。
3. 明确区分严格名义结论与工程实现边界：4D 中 Artstein 变换严格处理名义 LTI 输入死区；一阶预测、离散实现及 6D 局部冻结模型则以预测误差和有界扰动描述，并通过统一实验链路验证性能。

## 2 问题描述与基础齐次编队控制

### 2.1 Leader--Follower 编队误差

考虑由 Leader（领航机器人）和 Follower（跟随机器人）组成的两机器人编队系统。4D 名义状态定义为

```math
x_i=[p_{x,i},p_{y,i},v_{x,i},v_{y,i}]^T=[p_i^T,v_i^T]^T,\quad i\in\{l,f\}.
```

其中，下标 $i\in\{l,f\}$ 表示机器人编号，$l$ 和 $f$ 分别对应 Leader 与 Follower。

其双积分内部模型为

```math
\dot{x}_i=A_dx_i+B_du_i,
\quad
A_d=\begin{bmatrix}0&0&1&0\\0&0&0&1\\0&0&0&0\\0&0&0&0\end{bmatrix},
\quad
B_d=\begin{bmatrix}0&0\\0&0\\1/m&0\\0&1/m\end{bmatrix}.
```

其中，下标 $d$ 表示双积分名义模型（double integrator）；$m>0$ 是将齐次控制输出映射为等效加速度的虚拟惯性/控制增益，而非真实车体质量。给定固定编队偏移 $d=[d_x,d_y,0,0]^T$，定义误差

```math
e=x_f-x_l-d.
```

实际实现使用 Leader 周围圆上的离散候选点形成 $d$，并以容差逻辑抑制频繁切换。以下连续时间分析仅在当前离散编队点固定的区间内成立；切换应视为混杂扰动事件。

### 2.2 齐次比例控制器

选取线性反馈矩阵 $K$，使闭环矩阵 $A_d+B_dK$ 为 Hurwitz 矩阵，即其全部特征值的实部均小于零。利用广义齐次构造得到生成元 $G_0$、正定矩阵 $P$ 和负齐次度 $\nu$。定义

```math
G_d=I+\nu G_0.
```

对误差 $e$ 定义齐次范数 $n_e=\operatorname{hnorm}(e,G_d,P)$，并取

```math
c=\begin{cases}
c_{\min}, & n_e<c_{\min},\\
n_e, & c_{\min}\le n_e\le 1,\\
1, & n_e>1.
\end{cases}
```

其中，$c$ 为用于调节齐次反馈尺度的有界系数；$c_{\min}\in(0,1]$ 为其下限，用于避免误差接近零时尺度系数过小，从而改善离散实现中的数值稳定性。

则名义齐次控制律写为

```math
u_h=c^{1+\nu}K\exp\left(G_d(1-\ln c)\right)e.
```

无扰动、无约束且 $\nu<0$ 时，该构造对应的名义闭环具有有限时间收敛性质 [3]。离散控制周期 $h$ 内，控制器把 $u_h$ 映射为速度命令；例如 map 系平移命令可由 $v_{cmd,k+1}=v_{base,k}+hu_{h,k}/m$ 生成。其中，map 坐标系为固定于环境的全局坐标系，$v_{cmd,k+1}$ 和 $v_{base,k}$ 分别为第 $k+1$ 与第 $k$ 个控制周期的 map 系速度命令。最终的 ROS 2 速度命令话题 `cmd_vel`（包含车体线速度与角速度命令）还需经速度、加速度与轮速约束处理，以保证命令可由底盘执行；这些约束属于工程实现层，不包含在连续时间名义有限时间稳定性分析中。

## 3 4D Artstein 预测补偿齐次编队控制

### 3.1 含死区和一阶速度响应的等效执行器模型

在主要工作速度区间内，将 Follower 的 map 系速度执行器近似为

```math
\dot p(t)=v(t),\qquad
\dot v(t)=-\frac{1}{\tau_v}v(t)+\frac{1}{\tau_v}u_c(t-T_d),
```

其中，$u_c$ 为发送给底盘的 map 系速度命令，$T_d\ge0$ 为等效纯传输死区，$\tau_v>0$ 为等效速度响应时间常数。令 $x_a=[p^T,v^T]^T$，其中下标 $a$ 表示执行器等效模型（actuator），则

```math
\dot x_a(t)=A_ax_a(t)+B_au_c(t-T_d),
```

```math
A_a=\begin{bmatrix}0&0&1&0\\0&0&0&1\\0&0&-1/\tau_v&0\\0&0&0&-1/\tau_v\end{bmatrix},
\quad
B_a=\begin{bmatrix}0&0\\0&0\\1/\tau_v&0\\0&1/\tau_v\end{bmatrix}.
```

参数 $\tau_v$ 是底盘速度闭环、电机驱动、减速机构、轮地接触与负载等综合作用的局部输入输出近似，并非单个电机的物理时间常数。若速度或加速度约束长期激活，真实系统可能更接近速率饱和而非一阶指数响应。

### 3.2 Artstein 输入死区补偿

根据 Artstein reduction [5]，在当前执行器等效状态中叠加过去 $T_d$ 时间内历史速度命令经状态转移矩阵加权后的累积影响，构造 Artstein 状态

```math
z(t)=x_a(t)+\int_{t-T_d}^{t}
e^{A_a(t-s-T_d)}B_au_c(s)\,ds.
```

该构造的目的是使求导后的延迟输入项与原执行器模型中的 $u_c(t-T_d)$ 相互抵消。

在 $T_d$ 已知且控制器保存长度不少于 $T_d$ 的历史速度命令时，Artstein 变换可利用该历史输入构造补偿状态；由 Leibniz 求导法则对变上、下限积分求导，并代入执行器模型，可得

```math
\dot z(t)=A_az(t)+e^{-A_aT_d}B_au_c(t).
```

因此，原系统的显式延迟输入在描述 Artstein 状态 $z(t)$ 时间演化的状态方程中被消除。进一步定义经 Artstein 反映射得到的执行器等效状态 $\bar{x}_a(t)$；该映射将 Artstein 辅助状态 $z(t)$ 转换为已向前补偿纯输入死区的执行器等效状态：

```math
\bar x_a(t)=e^{A_aT_d}z(t),
```

在名义连续模型下，$\bar{x}_a(t)=x_a(t+T_d)$，并满足

```math
\dot{\bar x}_a=A_a\bar x_a+B_au_c(t).
```

该反映射说明：Artstein 层补偿的是纯死区，而一阶执行器模态仍然存在，不能把完整补偿系统误写为原始双积分器。

在线离散实现以最终发布的历史命令构造缓冲区，采用数值求积近似积分项。当 $T_d/h$ 不是整数、启动时历史不足或命令受限幅改写时，$z_k$ 是连续 Artstein 状态的数值近似。

### 3.3 一阶前向预测与预测状态反馈

记 $\bar x_a=[\bar p^T,\bar v^T]^T$。在预测窗口 $T_p$ 内，将速度命令 $u_c$ 近似为常值。对反映射后的无显式死区一阶执行器模型

```math
\dot{\bar v}=-\frac{1}{\tau_v}\bar v+\frac{1}{\tau_v}u_c,
\qquad \dot{\bar p}=\bar v,
```

求解速度微分方程，并对预测窗口内的速度积分，可得

```math
\hat v(T_p)=u_c+e^{-T_p/\tau_v}(\bar v-u_c),
```

```math
\hat p(T_p)=\bar p+u_cT_p+
\tau_v(1-e^{-T_p/\tau_v})(\bar v-u_c).
```

其中，符号 $\hat{\cdot}$ 表示预测值；$\hat v(T_p)$ 和 $\hat p(T_p)$ 分别为从反映射状态 $(\bar p,\bar v)$ 出发，在预测窗口 $T_p$ 后得到的 map 系速度和位置预测值。

本文取工程预测窗口 $T_p=\tau_v$，使预测状态在名义模型下对应时刻 $t+T_d+\tau_v$。实际在线计算中，未来新命令尚未知，故以已发布且已通过约束的上一周期命令近似预测窗内常值 $u_c$。最终将

```math
x_{h,f}=[\hat p^T,\hat v^T]^T
```

送入第 2.2 节的原始 4D 齐次控制器。Leader 则采用匀速外推，以形成同一预测时标上的参考状态：

```math
\hat p_l(t)=p_l(t)+(T_d+\tau_v)v_l(t),
\qquad \hat v_l(t)=v_l(t).
```

该架构保留了 HPC 内部的双积分名义模型，但不意味着“执行器预测后的物理闭环严格等价于双积分器”。其目的在于降低 HPC 所见状态中的死区与速度相位滞后。

### 3.4 4D 方法的适用边界

4D 补偿把 body 系 `cmd_vel` 旋转为 map 系命令并按 map 系一阶模型预测。yaw 变化明显时，map 系速度实际满足额外旋转耦合项，因而该近似适合固定航向或缓慢转向工况。定义实际未来状态与预测状态之差为 $\tilde x_h$，可将其来源概括为

```math
\tilde x_h=\tilde x_{\tau}+\tilde x_{T_d}+\tilde x_{\rm ZOH}
+\tilde x_{\rm sat}+\tilde x_{\rm frame}+\tilde x_{\rm obs}.
```

各项依次代表执行器参数失配、死区失配或抖动、预测窗内命令非恒定、速度/加速度/轮速饱和、坐标耦合和状态观测误差。该分解用于归因而非声称各项唯一或正交。若上述误差有界，则预测状态上的 4D HPC 面对的是名义误差加有界扰动；本文据此评价实际系统的实用跟踪性能，而不宣称完整受约束闭环仍满足严格有限时间定理。

## 4 6D Artstein Disc 全向车体运动学扩展

### 4.1 6D 混合状态与局部误差模型

为显式描述偏航和车体系速度，定义 6D 状态

```math
x=[p_x,p_y,\theta,v_x^b,v_y^b,\omega]^T.
```

其中位置和偏航角位于 map 系，$v_x^b,v_y^b,\omega$ 为车体系速度。控制器内部将广义力/力矩 $u=[F_x,F_y,M_z]^T$ 用于二阶近似：

```math
\dot p=R(\theta)\begin{bmatrix}v_x^b\\v_y^b\end{bmatrix},\quad
\dot\theta=\omega,\quad
\dot v_x^b=F_x/m,\quad
\dot v_y^b=F_y/m,\quad
\dot\omega=M_z/I.
```

离散实现将其积分为 $v_{x,cmd}^b$、$v_{y,cmd}^b$ 和 $\omega_{cmd}$，并在输出端施加速度、加速度和三轮全向底盘轮速约束。本文不引入轮级动力学状态：实际控制边界是 `cmd_vel`，底层驱动负责轮速分配和电机闭环。

对固定离散编队点 $d_i$，在 Leader 车体系定义位置、姿态、速度和角速度误差 $e\in\mathbb R^6$。冻结当前 Leader 速度 $v_{x,l}^b,v_{y,l}^b,\omega_l$ 后，其局部名义误差系统写为

```math
\dot e=A_Le+B_6u+w,
```

```math
A_L=\begin{bmatrix}
0&\omega_l&-v_{y,l}^b&1&0&0\\
-\omega_l&0&v_{x,l}^b&0&1&0\\
0&0&0&0&0&1\\
0&0&0&0&0&0\\
0&0&0&0&0&0\\
0&0&0&0&0&0
\end{bmatrix},\quad
B_6=\begin{bmatrix}
0&0&0\\0&0&0\\0&0&0\\1/m&0&0\\0&1/m&0\\0&0&1/I
\end{bmatrix}.
```

扰动 $w$ 汇集 Leader twist 的变化、旋转 Leader 系内非零编队偏移的附加项、模型误差和测量误差。由于 $R(\theta)$ 使系统依赖姿态，6D 模型不是全局常值 LTI 系统；局部冻结是构造 6D Disc 齐次反馈的条件，而非全局精确线性化。

### 4.2 平移和偏航的分解 Artstein 预测

直接在 6D 混合坐标上构造单一 $6\times3$ Artstein 核会遇到姿态相关矩阵问题。为此，本文分别处理两个常值 LTI 近似通道。

平移通道使用 map 系状态 $x_p=[p_x,p_y,v_x^m,v_y^m]^T$，并采用与第 3.1 节相同的 $A_p,B_p,T_d,\tau_v$。依第 3.2--3.3 节构造 Artstein 状态、反映射和一阶前向预测，得到 $\hat p$ 与 $\hat v^m$。偏航通道使用

```math
x_\theta=[\theta,\omega]^T,
\quad
\dot x_\theta=A_\theta x_\theta+B_\theta u_\theta(t-T_d),
```

```math
A_\theta=\begin{bmatrix}0&1\\0&-1/\tau_\omega\end{bmatrix},
\quad B_\theta=\begin{bmatrix}0\\1/\tau_\omega\end{bmatrix},
```

其中 $u_\theta=\omega_{cmd}$，$\tau_\omega$ 为等效角速度响应时间常数。对该通道同样应用 Artstein 变换和一阶预测，得到 $\hat\theta,\hat\omega$；角度差需要归一化至 $[-\pi,\pi]$。

将预测 map 系速度依预测偏航旋转至车体系：

```math
\hat v^b=R(-\hat\theta)\hat v^m,
```

从而构成送入 6D Disc HPC 的状态

```math
\hat x=[\hat p_x,\hat p_y,\hat\theta,\hat v_x^b,\hat v_y^b,\hat\omega]^T.
```

该分解保留了每个预测通道的常值 LTI Artstein 结构；预测后的状态重组以及其与 6D 误差反馈的结合仍是近似实现。

### 4.3 6D Disc 齐次反馈与名义结论

当 $(A_L,B_6)$ 可控，且存在 $K_{lin}$ 使 $A_L+B_6K_{lin}$ Hurwitz 时，可对冻结局部系统应用广义齐次升级，得到

```math
u_h=c^{1+\nu}K_{lin}\exp\left(G_d(1-\ln c)\right)e,
\quad c=\operatorname{clamp}(\lVert e\rVert_d,c_{\min},1).
```

在固定 $A_L$、固定编队点、参数匹配、完整历史、无饱和和无扰动的名义条件下，若 $\nu<0$，可得到

```math
\dot V\le-\rho V^{1+\nu},
```

从而局部误差有限时间收敛。实际系统中，预测参数误差、采样、速度/加速度/轮速约束、Leader twist 慢变化、离散编队点切换、传感器噪声及 map/body 转换误差均破坏上述理想前提。因此，6D 方法的合理结论是：它在局部冻结的名义条件下继承齐次收敛性质；在实际系统中按 ISS/实用稳定框架解释，误差最终进入与扰动上界相关的邻域。

## 5 统一实现架构与实验方法

### 5.1 ROS 2 控制链路

本文方法部署在 ROS 2 多机器人系统中。Follower 从定位模块获取 map 系位姿和速度估计，从 Leader 接收状态信息；控制器保存最终发布的命令历史，生成预测状态，再计算齐次名义控制命令。之后依次执行速度、加速度和轮速约束，并将 map 系平移命令转换为 body 系 `cmd_vel`。为避免预测器与实际输入不一致，必须将**约束后的最终命令**写回历史缓冲。

4D 与 6D 对比应保持相同的控制频率、延迟注入频率、延迟参数、速度/加速度上限、离散编队点设置和 Leader 轨迹；同时保证线性极点尺度、HPC 参数重建逻辑与评价区间的语义一致。6D 的价值不应表述为在所有工况下优于 4D，而在于其显式揭示 yaw、车体系速度和约束耦合对延迟补偿的影响。

### 5.2 待执行的实验矩阵

实验按“数值仿真 → Gazebo 双机器人 → 实物平台”三层开展。每层至少包含下列组别：

| 组别 | 控制器/条件 | 目的 |
| --- | --- | --- |
| G1 | 4D HPC，无延迟 | 建立无延迟参考 |
| G2 | 4D HPC，延迟，无 Artstein | 量化延迟退化 |
| G3 | 4D Artstein-HPC，延迟 | 验证 4D 预测补偿 |
| G4 | 6D Disc HPC，延迟，无 Artstein | 量化 6D 中的延迟影响 |
| G5 | 6D Artstein Disc-HPC，延迟 | 验证 6D 分解预测补偿 |
| G6 | G3/G5，$T_d$、$\tau_v$、$\tau_\omega$ 失配 | 评估模型失配敏感性 |
| G7 | G3/G5，改变 Leader 速度、曲率和初始 yaw 差 | 评估适用边界 |

建议报告以下统一指标：平均位置误差、尾段平均位置误差、最大距离误差、收敛时间、速度命令范数、yaw 误差、约束激活比例，以及 Leader/定位数据新鲜度。所有对照应重复运行并报告均值、标准差和有效重复次数。实物实验另应给出时钟同步状态、通信延迟统计、底盘阶跃响应辨识过程及可达加速度范围。

### 5.3 实验结果与讨论（待补充）

本节在完成实验后补充，建议按以下顺序组织：

1. 给出延迟注入和底盘等效参数的辨识/设置，说明 $T_d,\tau_v,\tau_\omega$ 与控制周期。
2. 展示 G1--G5 的位置轨迹、位置误差、速度命令和约束激活图，并使用统一表格汇总指标。
3. 通过 G6 分析参数失配下的性能退化，避免将预测器失配组误作“无预测基线”。
4. 通过 G7 讨论转向、初始 yaw 差和约束主导工况。在可用加速度不足以追踪 Leader 时，应解释为物理可达性边界，而非简单归因于控制律失效。
5. 对数值、Gazebo 与实物差异进行归因，包括状态估计、通信新鲜度、轮地接触、饱和和未建模执行器动态。

**[待补充：图 1 统一控制架构；图 2--4 4D 对照结果；图 5--7 6D 对照结果；表 1 参数表；表 2 数值/Gazebo/实物的统一性能指标。]**

## 6 结论

本文面向输入死区和执行器响应滞后的全向多机器人 Leader--Follower 编队问题，构建了 4D 与 6D 两类 Artstein 预测补偿齐次控制框架。4D 框架以常值 LTI 执行器模型为基础，通过 Artstein 变换和反映射严格消除名义纯输入死区，并以一阶前向预测降低速度响应滞后；预测状态上的控制器保留原始双积分齐次内核。6D 框架进一步引入偏航与车体系速度，通过平移/偏航分解预测和局部冻结 Leader 车体系误差模型，将预测补偿接入 6D Disc 齐次控制器。

本文同时明确了理论与实现的边界：严格有限时间结论只适用于满足模型匹配、固定参数和无约束等条件的名义系统；真实 ROS 2/Gazebo/实物闭环包含采样、饱和、切换、噪声、坐标耦合和参数失配，应以实用稳定性和可重复实验结果评价。后续工作将完成所设计的三层实验验证，并进一步研究参数在线辨识、时变/随机网络延迟及障碍环境下的安全约束融合。

## 参考文献

[1] Bhat S P, Bernstein D S. Finite-Time Stability of Continuous Autonomous Systems. *SIAM Journal on Control and Optimization*, 2000. DOI: 10.1137/S0363012997321358.

[2] Bhat S P, Bernstein D S. Geometric Homogeneity with Applications to Finite-Time Stability. *Mathematics of Control, Signals, and Systems*, 2005. DOI: 10.1007/s00498-005-0151-x.

[3] Polyakov A. *Generalized Homogeneity in Systems and Control*. Springer, 2020. DOI: 10.1007/978-3-030-38449-4.

[4] Yuan W, Dong C, Duan X, Polyakov A, Zimenko K, Ping X. Leader-Follower Tracking with Collision Avoidance for Omni-directional Mobile Robots: Linear vs Homogeneous Controller. *Chinese Control Conference*, 2024. DOI: 10.23919/CCC63176.2024.10662358.

[5] Artstein Z. Linear Systems with Delayed Controls: A Reduction. *IEEE Transactions on Automatic Control*, 1982. DOI: 10.1109/TAC.1982.1103023.

[6] Ni J, Liu L, Liu C, Liu J. Fixed-Time Leader-Following Consensus for Second-Order Multiagent Systems With Input Delay. *IEEE Transactions on Industrial Electronics*, 2017, 64(11): 8635--8646. DOI: 10.1109/TIE.2017.2701775.

[7] Wang C, Tnunay I H P, Zuo Z, Lennox B, Ding Z. Fixed-Time Formation Control of Multi-Robot Systems: Design and Experiments. *IEEE Transactions on Industrial Electronics*, 2018. DOI: 10.1109/TIE.2018.2870409.

[8] Zhang A, Zhou D, Yang M, Yang P. Finite-Time Formation Control for Unmanned Aerial Vehicle Swarm System With Time-Delay and Input Saturation. *IEEE Access*, 2019, 7:5853--5864. DOI: 10.1109/ACCESS.2018.2889858.

[9] Jiang W, Wang C, Meng Y. Fully Distributed Time-Varying Formation Tracking Control of Linear Multi-Agent Systems With Input Delay and Disturbances. *Systems & Control Letters*, 2020, 146:104814. DOI: 10.1016/j.sysconle.2020.104814.

[10] Zhang H, Zhou D. Event-Triggered Finite-Time Consensus Scheme for Time-Delay Multi-Agent Systems with Settling Time Estimation and its Application. *Journal of Aerospace Technology and Management*, 2025, 17:e0925. DOI: 10.1590/jatm.v17.1369.

[11] Wang C, Liu X, Yang X, Hu F, Jiang A, Yang C. Trajectory Tracking of an Omni-Directional Wheeled Mobile Robot Using a Model Predictive Control Strategy. *Applied Sciences*, 2018, 8(2):231. DOI: 10.3390/app8020231.
