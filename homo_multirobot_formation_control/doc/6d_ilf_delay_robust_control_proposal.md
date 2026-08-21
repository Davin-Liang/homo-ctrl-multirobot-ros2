# 局部 6D ILF 时滞鲁棒齐次编队控制：理论审查与实施方案

> **文档状态：研究方案，尚非已证明的控制器。**
>
> 本文提出一个可能替代 `6D Artstein Disc` 的第五章研究方向：在局部 6D
> 全向车误差模型中显式纳入一阶速度执行器动态，并探索基于隐式 Lyapunov
> 函数（ILF）的、**不依赖精确输入死区值**的鲁棒控制。本文明确区分已有文献
> 可直接支撑的结论、需要在本课题中补充证明的命题，以及只应通过数值/工程实验
> 验证的假设。任何 ROS 2 控制器实现均以第 7 节的数值验证通过为前提。

---

## 1. 研究动机与目标边界

### 1.1 要替代的对象

现有 `6D Artstein Disc` 的结构为：

```text
EKF/TF 6D 状态
  -> map 系平动 4D Artstein + 一阶执行器前向预测
  -> 偏航 2D Artstein + 一阶执行器前向预测
  -> 6D Disc generalized HPC
  -> cmd_vel、轮速约束与加速度约束
```

它的优势是：若纯死区 `Td`、一阶时间常数 `tau` 及历史实际发布命令均准确，
预测状态接近无延迟名义系统，跟踪性能通常较好。其边界是：

- 预测器需要选定精确或近似精确的 `Td` 和 `tau`；
- `Td` 抖动、采样/网络变化、速度饱和和未建模执行器行为会形成预测失配；
- 6D Disc 的 `A_L` 随 Leader 速度、偏航误差和离散编队点切换而变化，现有
  结论本身是局部冻结模型下的工程性结论，而不是全局 Artstein 等价。

本方案的目标不是在所有工况下超过 Artstein，而是建立第二条、可公平比较的路径：

```text
已知且准确的恒定时延        -> 6D Artstein Disc 预计更有优势
有界但失配/时变的输入时延    -> 6D ILF 路线力求更平稳、更不易失稳
```

因此，研究假设与可宣称结论必须是“时滞鲁棒的局部实用稳定/ISS”，而不是
“精确消除任意时延”或“全局有限时间收敛”。

### 1.2 第五章建议题目

```text
面向不确定输入时延的局部 6D ILF 齐次鲁棒编队控制
```

第五章与第四章共用 Leader-Follower、6D Disc 编队目标、轮速约束、采样频率和
实验平台。区别只在延迟处理控制核心：

| 章节 | 核心方法 | 延迟信息 | 主要结论目标 |
|---|---|---|---|
| 第 4 章 | 6D Artstein Disc + HPC | `Td`、`tau` 的点估计及命令历史 | 名义预测性能 |
| 第 5 章 | 局部 6D ILF/ILKF 鲁棒控制 | 时延上界 `d_bar`，执行器参数范围 | 局部 ISS/最终有界与失配鲁棒性 |

原 QP 避障不再作为独立理论章节；如需保留，可作为第 6 章的安全扩展实验或附录，
不能与本章混为同一控制器贡献。

---

## 2. 术语澄清：ILF、ILKF 与齐次控制不是同义词

### 2.1 ILF 是方法，不是一个固定控制律类别

**隐式 Lyapunov 函数**（Implicit Lyapunov Function, ILF）以隐式方程

```math
Q(V,x)=0
```

定义正标量函数 `V(x)`，常可把控制器参数求解转化为矩阵等式或 LMI。它可以用于
齐次系统，也可以用于一般线性、非线性和时滞系统；因此不能写成
`ILF = 齐次控制`。

面对含历史状态的时滞系统，使用的是**隐式 Lyapunov--Krasovskii 泛函**
（ILKF），形式上可依赖

```math
x_t(s)=x(t+s), \qquad s\in[-d_{\max},0].
```

### 2.2 本方案中“齐次”的准确含义

原始 6D Disc/HPC 在冻结模型下，以 `lpc2hpc_nd(A_L,B_6,K)` 从一个 Hurwitz
线性反馈构造广义齐次反馈。若显式加入一阶执行器项

```math
-\Lambda v_b,
```

完整闭环一般**不再满足原始幂零积分链所用单一扩张下的严格齐次性**。因此可接受、
也必须使用的论文表述是：

```text
控制律采用局部广义齐次/ILF 设计；完整的含执行器、时滞、采样、切换与限幅闭环，
在给定局部工作域内按 ISS 或实用稳定性分析。
```

不可写成：

```text
加入 -1/tau 后的完整物理 6D 时滞系统仍保持原始严格齐次有限时间稳定性。
```

---

## 3. 应分析的局部 6D 执行器时滞模型

### 3.1 状态与时延定义

使用真实测量速度而非控制器内部 `cmd_vel` 作为状态：

```math
x_i=[p_{x,i},p_{y,i},\theta_i,v_{x,i}^{b},v_{y,i}^{b},\omega_i]^\mathsf{T},
\qquad i\in\{l,f\}.
```

其中 `p, theta` 由 TF/EKF 获取，`v_x^b,v_y^b,omega` 取自里程计 twist。对
Follower 的三个速度通道采用一阶近似：

```math
\dot v_f^b(t)=-\Lambda v_f^b(t)+\Lambda u_f(t-d(t)),
\qquad
\Lambda=\operatorname{diag}(1/\tau_x,1/\tau_y,1/\tau_\omega),
```

```math
0\le d(t)\le\bar d.
```

这里 `u_f=[v_{x,cmd}^b,v_{y,cmd}^b,\omega_{cmd}]^\mathsf{T}` 为真正发布到
`cmd_vel` 的命令；`d(t)` 合并串口/驱动死区、调度和等效零阶保持延迟。`tau_x`、
`tau_y`、`tau_omega` 可以相等，也可以由阶跃实验给出区间。

> 注意：若 `cmd_vel` 到速度的阶跃响应不能由“一阶滞后 + 有界死区”近似，则本章的
> 模型前提不成立，必须停止该方案或改用更高阶/数据驱动执行器模型。

### 3.2 冻结 Leader 的局部误差模型

沿用 6D Disc 的 Leader 车体系误差定义，在固定离散编队点 `d_j`、冻结 Leader
twist `rho=[v_{x,l}^b,v_{y,l}^b,omega_l]` 附近，记

```math
\xi=[e_x,e_y,e_\theta,e_{v_x},e_{v_y},e_\omega]^\mathsf{T}.
```

在冻结工况中，零误差对应的绝对命令不是零，而是维持 Leader twist 的平衡命令

```math
u_\star=[v_{x,l}^b,v_{y,l}^b,\omega_l]^\mathsf{T}.
```

因此定义控制器的局部输出为相对平衡命令

```math
\delta u(t)=u_f(t)-u_\star,
```

并在发布前还原为 $u_f=u_\star+\delta u$。仅在 Leader 速度/角速度冻结、
目标偏航误差为零时，上式是常值平衡命令；Leader 加速度、参考偏航变化和编队切换
引起的偏差应归入 $r_{\mathrm{leader}}$ 或 $r_{\mathrm{switch}}$。

忽略二阶余项后的候选误差模型才可写为：

```math
\dot\xi(t)=A_{\rho,\Lambda}\xi(t)+B_\Lambda\delta u(t-d(t))+r(t). \tag{1}
```

其中可从当前 `6D Disc` 的 `A_L` 得到

```math
A_{\rho,\Lambda}=
\begin{bmatrix}
0&\omega_l&-v_{y,l}&1&0&0\\
-\omega_l&0&v_{x,l}&0&1&0\\
0&0&0&0&0&1\\
0&0&0&-1/\tau_x&0&0\\
0&0&0&0&-1/\tau_y&0\\
0&0&0&0&0&-1/\tau_\omega
\end{bmatrix},
\qquad
B_\Lambda=
\begin{bmatrix}
0_{3\times3}\\ \Lambda
\end{bmatrix}. \tag{2}
```

`r(t)` 不是可忽略符号，必须明确包含：

```math
r=r_{\mathrm{nl}}+r_{\mathrm{leader}}+r_{\mathrm{switch}}
  +r_{\mathrm{sat}}+r_{\mathrm{obs}}.
```

- `r_nl`：姿态旋转、局部线性化截断和车体系变换余项；
- `r_leader`：Leader 加速度、平衡命令变化和冻结 `rho` 后的慢变项；
- `r_switch`：Disc 目标切换所造成的误差坐标跳变；
- `r_sat`：速度、加速度、轮速投影使实际 `cmd_vel` 偏离控制律输出的项；
- `r_obs`：EKF 速度误差、时间戳老化和数值差分误差。

式 (1) 的价值是：电机一阶速度滞后不是“控制器外部神秘扰动”，而是显式进入
`A_{rho,Lambda},B_Lambda`；纯死区仍保留为待鲁棒处理的输入时延。若省略
$u_\star$ 而将绝对 `cmd_vel` 直接代入式 (1)，则存在未消去的仿射项
$-\Lambda[v_{x,l}^b,v_{y,l}^b,\omega_l]^\mathsf{T}$，零误差并非平衡点，
该模型不可用于稳定性分析。

### 3.3 与现有实现的关系

现有 `formation_control_node_6d_artstein_disc` 已测量 6D 状态，但其预测层采用：

```text
map 系 4D 平动 Artstein 预测器 + 2D 偏航 Artstein 预测器
```

随后把预测状态送给理想加速度型 `LpcController6DArtsteinDisc`。本方案不复用该
历史命令预测器；新控制器应直接以测量状态形成 `xi`，并使用式 (1) 的局部模型。
在实现前必须确认误差定义、输入坐标和真实发布后的限幅命令完全一致。

---

## 4. 严格理论审查：已有支撑与未解决缺口

### 4.1 可以直接作为文献基础的部分

1. **ILF/ILKF 可以用于时滞系统。** Polyakov、Efimov、Perruquetti、Richard
   建立了 ILKF 的稳定性分析和控制设计框架，覆盖渐近、有限时间和固定时间性质。
   [IEEE TAC, 2015, DOI 10.1109/TAC.2015.2422451](https://doi.org/10.1109/TAC.2015.2422451)

2. **ILF 型控制对有界、快时变控制/测量延迟具有鲁棒结论。** Zimenko 等针对
   控制/测量通道延迟给出：小于阈值的时延可保持全局渐近稳定；任意较大延迟时可
   保持对含原点紧集的稳定；论文本身也讨论了采样数据可视作控制通道的时变延迟。
   [Automatica, 2019, DOI 10.1016/j.automatica.2018.11.051](https://doi.org/10.1016/j.automatica.2018.11.051)

3. **ILF 与齐次设计可以结合。** 隐式齐次控制 Lyapunov 函数已有构造方法；含
   扇区非线性的齐次系统也已有有限时间 ISS 的 LMI 型充分条件。
   [IFAC, 2007](https://doi.org/10.3182/20070822-3-ZA-2920.00185)，
   [Automatica, 2024, DOI 10.1016/j.automatica.2024.111872](https://doi.org/10.1016/j.automatica.2024.111872)

4. **负次数齐次闭环具有特殊的延迟鲁棒性。** 这解释了为何 ILF/齐次路线值得
   探索，但该性质不能替代对具体 6D 模型的验证。
   [Automatica, 2017, DOI 10.1016/j.automatica.2017.01.030](https://doi.org/10.1016/j.automatica.2017.01.030)

### 4.2 不能直接引用为本项目定理的部分

这是本方案最重要的审查结论。

1. **2019 ILF 时延控制论文的基本构造是单输入规范形。** 当前式 (1) 是
   `6` 状态、`3` 输入、参数调度的 MIMO 系统；不能把该文的 SISO 结论直接写成
   “本系统的 6D 定理”。

2. **`A_{rho,Lambda}` 时变且有切换。** `rho` 随 Leader 运动变化，Disc 目标
   `d_j` 会切换。逐时刻重新求一个 ILF 参数，除非存在共同 ILF/共同 LKF 或有严格的
   慢变/驻留时间论证，否则不能推出时变混杂闭环稳定性。

3. **一阶执行器项破坏原 6D 纯积分链的严格齐次结构。** 虽然它是耗散项，通常有利于
   稳定，但不能仅凭“耗散”就删除证明。特别是在输入有延迟时，不能通过即时状态反馈
   简单抵消该项。

4. **饱和不是小扰动的自动同义词。** 若轮速/加速度投影频繁激活，
   `r_sat` 可能与控制量同量级；此时局部无饱和 ILF 结论不适用。

5. **状态测量并不完美。** EKF 速度为 20 Hz 左右的估计量，不是连续精确执行器速度；
   采样、时间戳老化和滤波相位必须纳入 `r_obs` 或等效时延上界。

因此，目前唯一严谨的定位是：

```text
局部 6D MIMO ILF 时滞鲁棒控制是待完成的研究设计；
它具有可信文献基础，但没有可直接复制的完整现成定理。
```

### 4.3 最小可证明命题与禁止表述

在完成第 5 节的 LMI/构造后，建议目标命题为：对选定紧工作域 `Omega`、固定
编队目标、冻结或慢变 Leader 参数、`d(t) in [0,d_bar]` 和有界 `r`，存在控制律
`u=kappa_ILF(xi)` 与泛函 `W(xi_t)`，使

```math
\dot W\le -aW^\alpha + b\|r\|,
\qquad a>0,\quad 0<\alpha<1. \tag{3}
```

由此给出局部 ISS/实用有限时间结论，例如：

```math
\limsup_{t\to\infty}\|\xi(t)\|
\le C\|r\|_\infty^{1/\alpha}. \tag{4}
```

`C`、`a`、`alpha`、工作域和可允许延迟上界都必须由推导或数值 LMI 得出，不能
事先假定。

在此之前禁止写：

```text
对任意时变时延，全局有限时间收敛到零；
6D MIMO 系统直接满足 Zimenko 等的 SISO ILF 定理；
电机时延被 ILF 精确补偿；
轮速饱和下仍严格保持齐次性。
```

---

## 5. 控制器构造的两种候选，以及选择规则

### 5.1 候选 A：直接局部 6D MIMO ILF（主路线，理论风险较高）

对每个冻结参数 `rho`，围绕式 (1) 寻找 MIMO ILF/ILKF 构造：

```math
Q(V,\xi;P,G)=\xi^\mathsf{T}D(V^{-1})P D(V^{-1})\xi-1=0,
```

并设计三输入反馈 `u=kappa_ILF(V,xi)`。这里的 `P,G`、反馈增益和允许的
`d_bar` 必须由 MIMO 推导或 LMI 给出，不能沿用 4D/SISO 参数。

优点：与 6D Artstein 的状态、输入、任务最对称，论文章节辨识度最高。

风险：必须补上 MIMO、局部参数调度和时滞的联合证明；若没有共同泛函，只能得到
冻结模型的结论。

### 5.2 候选 B：6D 广义齐次名义反馈 + ILKF 鲁棒性分析（保底路线）

保留现有 `lpc2hpc_nd` 的局部 6D 广义齐次反馈思想，针对含延迟和执行器动态的
式 (1) 另外寻找 ILKF 或 Razumikhin 型 ISS 估计：

```math
u=\kappa_{\mathrm{HPC}}(\xi),
\qquad
\dot W\le -aW^\alpha+b_1\|u(t-d)-u(t)\|+b_2\|r\|.
```

优点：能最大程度复用当前 6D 控制器的坐标、编队目标与齐次参数化。

边界：它是“齐次名义反馈的时滞鲁棒分析”，不能称为完整的新 ILF 控制律；若对
`u(t-d)-u(t)` 只能给保守界，性能可能不佳。

### 5.3 决策规则

```text
候选 A 的 MIMO ILF/ILKF 可行性 LMI、时延裕度和数值 DDE 仿真均通过
    -> 实施候选 A，作为第五章主方法。

候选 A 的 MIMO 构造无法闭合，但候选 B 的 ISS 界与数值结果通过
    -> 将第五章降为“局部 6D 齐次反馈的 ILKF 时滞鲁棒分析与验证”。

两者均无法在执行器实测参数范围内稳定
    -> 停止该路线；不得为了章节完整性仓促写 ROS 控制器。
```

---

## 6. 必须完成的理论检查清单

以下项目不是可选优化，而是进入工程实现前的理论/数值前置条件。

### T1：执行器模型辨识与模型一致性

- 分别测量 `vx`、`vy`、`omega` 的阶跃响应，报告 `Td`、`tau_i`、置信/波动范围；
- 验证一阶加死区模型对上升段、下降段和不同幅值的拟合误差；
- 明确 `cmd_vel` 限幅后实际进入驱动器的命令，不能以限幅前控制量建模；
- 将 20 Hz 采样等效为零阶保持，并把最大采样/调度延迟计入 `d_bar`。

### T2：每个冻结工作点的线性可控性与稳定基线

对网格化的

```math
rho\in[v_{x,l}^{\min},v_{x,l}^{\max}]
\times[v_{y,l}^{\min},v_{y,l}^{\max}]
\times[\omega_l^{\min},\omega_l^{\max}],
```

计算：

```math
\operatorname{rank}\,[B_\Lambda,A_{\rho,\Lambda}B_\Lambda,\ldots,
A_{\rho,\Lambda}^{5}B_\Lambda]=6,
```

并构造有统一稳定裕度的 `K_rho`。若存在不可控点、条件数极坏点或无法获得所需
稳定裕度的点，应缩小工作域，不得在论文中声称全工作域可用。

#### T2 初步数值结果（2026-08-21）

使用脚本 `scripts/ilf_6d_feasibility.py` 对冻结模型作第一次代数筛选。网格为

```text
vx_l, vy_l, omega_l in {-0.5, 0, 0.5}
tau_x=tau_y=tau_omega in {0.25, 0.43, 0.55} s
```

| 样本数 | 可控性秩范围 | sigma_min 范围 | 条件数范围 | 结论 |
|---:|---:|---:|---:|---|
| 81 | 6--6 | 0.817913--0.970142 | 56.329--4633.233 | 通过初步可控性筛选；最大条件数须在后续 LMI 与数值仿真中重点监测 |

结果文件为 `analysis/results/6d_ilf_feasibility/controllability_scan.csv`。这一步仅表明
`(A_{rho,Lambda},B_Lambda)` 在该离散网格上可控；它**不**证明存在 MIMO ILF/ILKF
控制律，也不提供时延裕度或统一稳定裕度。

### T3：MIMO ILF/ILKF 可行性

- 名义 MIMO 控制律优先以 Polyakov、Efimov、Perruquetti 的线性 MIMO ILF
  块分解/LMI 构造为基础；该文直接覆盖多输入线性系统的有限/固定时间稳定与扰动
  鲁棒性，但**不处理输入时滞**；
- 输入时滞分析可参考第 4.1 节的 ILKF 与 2019 ILF 时延结果，但其直接控制构造
  不能被写成对本 3 输入模型的现成定理；
- 因此必须在本课题中补充“名义 MIMO ILF 闭环 + 输入时滞项”的泛函导数估计，
  或找到明确覆盖该组合假设的 MIMO 时滞定理；
- 2026 年关于切换线性 MIMO 广义齐次控制的预印本可用于讨论共同/多 Lyapunov
  函数和参数切换，但在同行评审前只作补充参考，不能替代上述证明；
- 明确使用的 MIMO 定理或给出从 ILF 条件到三输入系统的完整推导；
- 求解 `P>0`、扩张生成元 `G` 和反馈矩阵的矩阵等式/LMI；
- 检查隐式方程对 `V>0` 存在唯一正根，避免在线求根多值或病态；
- 计算可允许时延上界 `d_bar^*`，而非只报告控制器在一个仿真时延下“看起来稳定”；
- 若参数调度，寻找共同 `P,G`/共同泛函；若找不到，给出 `dot rho` 上界或目标切换
  驻留时间条件。

### T4：扰动、饱和和切换的闭环界

- 用实测速度/加速度/轮速约束估计 `r_sat` 的上界或激活占空比；
- 用 Leader 最大加速度界估计 `r_leader`；
- 对 `m_p` 目标切换定义迟滞和最小驻留时间；
- 明确初始历史 `u(s),s in [-d_bar,0]` 的有界假设；
- 将 EKF 测量噪声和观测延迟写入 `r_obs`，不要在证明中同时假设“完美状态”和
  “实物 EKF”。

---

## 7. 先数值仿真、后工程实现：硬门槛流程

### 7.1 原则

本方案**不得先写 ROS 节点再调参找理论**。时滞 DDE、局部参数调度、ILF 隐式根和
饱和叠加后，工程联调无法区分“理论不可行”“离散化错误”“ROS 时间戳问题”。

执行顺序固定为：

```text
T1--T4 理论/辨识检查
  -> N1--N5 纯数值 DDE 仿真
  -> 数值验收门
  -> I1 离线离散控制器原型
  -> I2 ROS/Gazebo
  -> I3 实物
```

任何一层未通过，应回到上一层修正模型或缩小适用域，而非直接增加滤波器、限幅或
人工补偿掩盖问题。

### 7.2 N1：冻结 6D LTI 无扰动基准

对象为式 (1)，固定 `rho`、固定 `d`，设置 `r=0`、无饱和、连续时间全状态反馈。

目的：

- 验证候选 A/B 的 ILF 求根、控制律和 DDE 历史缓存正确；
- 对 `d=0` 检查名义局部收敛；
- 扫描 `d`，估计实用延迟裕度；
- 与同一 `A_{rho,Lambda},B_Lambda` 上的 6D Artstein 基线比较。

输出至少包括：`||xi||`、ILF 值/泛函值、控制输入、输入历史、最大特征值、可控性
条件数和失稳标记。

### 7.3 N2：时延失配与快时变延迟

对两路线使用同一个真实对象，至少测试：

```text
恒定时延：d(t) = 0, Td_nom, Td_nom ± 25%
抖动时延：d(t) = Td_nom + Delta_d(t)，Delta_d 有界且分段常值/随机
采样等效：20 Hz 零阶保持 + 调度抖动
```

Artstein 控制器仍使用标称 `Td_nom`；ILF 控制器只使用 `d_bar`。评价目标不是要求
ILF 在准确常延迟下胜出，而是比较误差、指令变化和稳定性随失配的退化曲线。

### 7.4 N3：非线性 6D 连续模型与执行器参数不确定性

恢复完整：

```math
\dot p=R(\theta)v_b,\qquad \dot\theta=\omega.
```

扫描 `tau_i`、Leader 速度和角速度；先固定编队点，避免将切换问题与基本控制问题
混在一起。此阶段只要发现所需实物 `tau,d` 区间内存在明显振荡或发散，就不进入
离散化/ROS 阶段。

### 7.5 N4：混杂因素与约束

在 N3 通过后才加入：

- `m_p` 离散编队目标和迟滞切换；
- 速度、加速度、轮速投影；
- Leader 加速、圆周和八字轨迹；
- 测量噪声、状态时间戳老化和 EKF 等效低通。

该阶段检查式 (3) 中扰动解释是否合理；若轮速投影长期强激活，则应降低轨迹速度或
缩小理论工作域，而不是继续把结果解释为 ILF 性能。

### 7.6 N5：离散化一致性与 Monte Carlo

控制频率固定为实物上限 `20 Hz`，分别比较连续参考解、前向 Euler、推荐的一致
离散化/隐式实现。齐次有限时间控制直接 Euler 化可能在原点附近产生数值问题，
因此离散闭环必须独立验收。

随机初始相对位姿、`tau_i`、时延轨迹、噪声和 Leader 工况，输出成功率与 95% 分位
指标；不能只展示单条“好看”的轨迹。

与此有关的离散实现文献：
[Polyakov et al., Automatica, 2023](https://doi.org/10.1016/j.automatica.2023.111118)。

### 7.7 数值验收门

只有同时满足下列条件，才允许进入工程实现：

- T1--T3 有明确、可复现的数值结果；
- 在实测参数包络内，所有 N1--N5 基准场景无未解释发散；
- 相比 6D Artstein，ILF 在至少一个“时延失配/时变”场景呈现可重复的鲁棒优势，
  且没有以更频繁饱和或更大指令抖动换取该优势；
- 已确定连续/离散控制律的对应关系、初始历史和失败保护；
- 结论符合“局部、实用稳定”的理论边界。

若未通过，论文仍可如实报告“ILF 直接 MIMO 扩展在本平台参数范围内不可行”，但不应
把它做成 ROS 主控制器或第五章完成工作。

---

## 8. 工程实现范围（仅在数值门通过后）

通过第 7 节后，再新增独立节点，不修改现有 `6D Artstein Disc` 行为：

```text
formation_control_node_6d_ilf
  1. 读取 EKF/TF 的真实 6D 状态与时间戳；
  2. 形成 Leader 车体系误差 xi；
  3. 按已验证的调度规则更新或选择 ILF 参数；
  4. 求解 V(xi) 并计算 u_ILF；
  5. 施加与 Artstein 基线完全相同的 cmd_vel/轮速/加速度约束；
  6. 记录 xi、V、d 估计/上界、约束激活、ILF 失败回退和数据新鲜度。
```

工程保护要求：

- ILF 隐式方程求根失败、`V` 非有限或矩阵条件数超阈值时，发布零命令或已验证的
  平滑线性回退，且记录失败；
- 不允许在回调中悄然重置 `V`、隐藏异常或改写实际发布命令而不记录；
- `cmd_vel` 的历史、限幅前命令与限幅后实际命令必须分别记录；
- 在 workspace 根目录构建，遵守仓库 `AGENTS.md` 的 ROS 2 构建约定。

---

## 9. 论文中可使用的结论模板

在理论和数值门通过前，只可写：

```text
本文提出局部 6D MIMO ILF 时滞鲁棒控制的研究方案，并以数值仿真检验其在
一阶执行器滞后、输入时延失配和采样条件下的可行性。
```

在完成 T1--T4、N1--N5 且得到式 (3) 对应条件后，可写：

```text
在给定局部工作域、时延上界、Leader 慢变界、切换驻留条件及约束非持续激活条件下，
所设计的局部 6D ILF 控制器保证闭环编队误差的输入到状态稳定性（或实用稳定性）。
相较于依赖精确时延的 6D Artstein 控制器，其在时延失配和时变延迟下表现出更小的
误差退化/更大的可稳定范围。
```

无论结果如何，都不可将数值实验替代为严格证明，也不可把冻结模型结论扩展为全局、
任意 Leader 轨迹、任意饱和条件下的结论。

---

## 10. 参考文献

1. A. Polyakov, D. Efimov, W. Perruquetti, J.-P. Richard, “Implicit
   Lyapunov-Krasovskii Functionals for Stability Analysis and Control Design
   of Time-Delay Systems,” *IEEE Transactions on Automatic Control*, 60(12),
   3344–3349, 2015. DOI:
   [10.1109/TAC.2015.2422451](https://doi.org/10.1109/TAC.2015.2422451).
2. K. Zimenko, D. Efimov, A. Polyakov, A. Kremlev, “Independent of delay
   stabilization using implicit Lyapunov function method,” *Automatica*, 101,
   103–110, 2019. DOI:
   [10.1016/j.automatica.2018.11.051](https://doi.org/10.1016/j.automatica.2018.11.051).
3. K. Zimenko, D. Efimov, A. Polyakov, W. Perruquetti, “A note on delay
   robustness for homogeneous systems with negative degree,” *Automatica*,
   2017. DOI:
   [10.1016/j.automatica.2017.01.036](https://doi.org/10.1016/j.automatica.2017.01.036).
4. I. Polyakov, D. Efimov, X. Ping, et al., “Consistent discretization of
   homogeneous finite/fixed-time controllers for LTI systems,” *Automatica*,
   2023. DOI:
   [10.1016/j.automatica.2023.111118](https://doi.org/10.1016/j.automatica.2023.111118).
5. A. Polyakov et al., “Finite-time stability analysis of homogeneous systems
   with sector nonlinearities,” *Automatica*, 2024. DOI:
   [10.1016/j.automatica.2024.111872](https://doi.org/10.1016/j.automatica.2024.111872).
6. A. Polyakov, D. Efimov, W. Perruquetti, “Robust Stabilization of MIMO
   Systems in Finite/Fixed Time,” *International Journal of Robust and
   Nonlinear Control*, 26(1), 69–90, 2016. DOI:
   [10.1002/rnc.3297](https://doi.org/10.1002/rnc.3297).
7. M. Labbadi, A. Polyakov, D. Efimov, “Accelerated Stabilization of Switched
   Linear MIMO Systems using Generalized Homogeneity,” arXiv preprint,
   2026. [arXiv:2602.08903](https://arxiv.org/abs/2602.08903).
