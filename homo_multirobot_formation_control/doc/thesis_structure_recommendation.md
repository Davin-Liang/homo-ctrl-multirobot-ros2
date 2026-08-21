# 硕士毕业课题章节结构建议

本文档整理当前 `homo_multirobot_formation_control` 功能包中已经形成的算法、仿真和实验工作，
用于规划硕士毕业论文主线和可投稿小论文方向。

## 1. 推荐论文题目方向

可选题目：

```text
考虑输入延迟与安全约束的多机器人齐次编队控制研究
```

或：

```text
面向全向移动机器人的延迟补偿齐次编队控制与避障方法研究
```

第一种更偏控制理论，第二种更突出全向移动机器人平台和工程实验。若后续小论文以 4D/6D
Artstein 和 QP 避障为主，第一种题目更稳。

## 2. 推荐章节结构

### 第 1 章 绪论

主要内容：

- 多机器人编队控制的研究背景和应用价值；
- 全向移动机器人在室内协同任务中的优势；
- 输入延迟、执行器滞后、速度/加速度约束对编队控制的影响；
- 现有齐次控制、延迟补偿和避障方法的不足；
- 本文主要研究内容和贡献。

建议贡献表述：

```text
本文围绕多机器人 leader-follower 编队中的延迟补偿、车体运动学扩展和安全避障问题，
构建了从理论算法、数值仿真、Gazebo 仿真到实物实验的完整验证链路。
```

### 第 2 章 多机器人系统建模与基础齐次编队控制

主要内容：

- ROS 2 多机器人系统结构；
- Gazebo 仿真平台与全向移动机器人模型；
- leader-follower 编队框架；
- 4D 双积分质点模型；
- 原始齐次编队控制方法；
- 离散编队点和 `tol` 切换机制；
- 速度、轮速、加速度约束建模。

本章定位为基础章节，不建议把创新点写得过重。它主要负责给第 3、4、5 章提供统一模型、
符号和实验平台。

### 第 3 章 面向输入延迟的 4D Artstein 齐次编队控制

主要内容：

- 4D 双积分 leader-follower 编队误差模型；
- 输入延迟和低阶等效速度执行器响应问题描述；
- Artstein 变换及其名义无显式输入时延模型；
- `tau` 预测状态、预测误差及其适用假设；
- 预测状态下的齐次比例控制；
- Leader 加速度、坐标系耦合、采样、限幅与编队点切换的有界扰动分析；
- 原始延迟控制、LPC、HPC、Artstein-HPC 对比；
- 数值仿真、Gazebo 仿真和实物实验。

本章核心贡献：

```text
在原始 4D 齐次编队控制框架中构造“Artstein 纯死区补偿 + 一阶执行器前向预测”的双层结构：
Artstein 变换及其反映射在名义模型和已知常值死区下精确消除显式输入时延，前向预测降低一阶速度响应的相位滞后，
并保留原始 4D HPC 作为预测状态上的名义控制律。针对采样、参数失配、姿态耦合、饱和和 Leader 非匀速运动，
以预测误差和有界扰动形式说明其适用边界，并通过多层实验验证延迟条件下的编队跟踪性能。
```

建议重点展示：

- 无延迟基准；
- 原始控制 + 延迟；
- Artstein 预测补偿 + 延迟；
- HPC 与 LPC 消融对比；
- `tau`、`Td`、`hpc_c_min`、`initial_min_lambda`、`switch_min_lambda` 对性能的影响；
- `tau`、`Td` 失配、Leader 加速度和 yaw 变化对预测误差的影响；
- 实物中执行器加速度上限对效果的限制。

### 第 4 章 面向全向车体运动学的 6D Artstein Disc 齐次编队控制

主要内容：

- 6D 混合状态模型：

```text
x = [p_x, p_y, theta, v_x^b, v_y^b, omega]^T
```

- map 系位置、车体系速度和 yaw 的关系；
- leader 车体系下的编队误差；
- 固定航向 leader 下 follower 轨迹为 leader 圆轨迹平移版本；
- 6D Disc 离散编队点；
- 局部冻结线性系统和 HPC 升级条件；
- 平移 Artstein 预测与 yaw 预测；
- 4D Artstein 与 6D Artstein 的公平对比。

本章核心贡献：

```text
将延迟补偿齐次编队控制从 4D 质点双积分模型推广到包含航向角和车体系速度的
6D 全向车体运动学表达，形成更贴近全向移动机器人 cmd_vel 接口的预测补偿控制框架。
```

论文表述要点：

- 不要写成“简单把 4D 扩成 6D”；
- 应强调 6D 使用的是 map 系位置 + yaw + body 系速度的混合状态；
- 需要说明 `R(theta)` 使全局常值线性系统不严格成立，因此采用局部冻结线性化和实用稳定表述；
- 6D 不一定全面优于 4D，它的价值在于模型更贴近全向底盘命令接口，并揭示 yaw/车体系速度耦合和饱和约束的影响；
- 和 4D 对比时必须保证 `min_lambda` 语义、HPC/K 同步、延迟节点频率等实现条件一致。

建议重点展示：

- 6D 原始 Disc 无延迟；
- 6D 原始 Disc + 延迟；
- 6D Artstein Disc + 延迟；
- 6D HPC 与 6D LPC 消融；
- follower 初始 yaw 不同条件下的对比；
- 低速和中速下效果差异；
- 与 4D Artstein 的同参数公平对比。

### 第 5 章 基于 QP 安全修正的避障编队控制

主要内容：

- 障碍物环境下的编队控制问题；
- nominal 编队控制输入；
- 激光雷达距离约束；
- 安全距离约束；
- QP 最小修正控制律；
- 避障约束和编队目标之间的权衡；
- Gazebo 仿真验证。

本章核心贡献：

```text
在齐次编队控制输出的基础上，引入基于二次规划的安全修正层，
在尽量保持原始编队控制指令的同时满足障碍物安全距离约束。
```

建议重点展示：

- 无避障控制在障碍物环境下的风险；
- QP 避障后最小距离保持；
- 避障过程中的编队误差变化；
- 避障结束后的编队恢复；
- 不同安全距离和权重参数下的对比。

### 第 6 章 仿真与实物实验综合分析

如果学校更偏工程验证，建议单独设置本章；如果篇幅有限，也可以把实验分别放在第 3、4、5 章，
第 6 章只做总结分析。

可包含：

- 数值仿真环境；
- Gazebo 双机器人仿真；
- 实物全向机器人实验平台；
- 定位链路：rf2o、EKF、TF；
- 延迟测量与执行器响应标定；
- 4D Artstein、6D Artstein、QP 避障的统一对比；
- 误差、速度和控制输入等指标统计。

### 第 7 章 总结与展望

主要内容：

- 总结 4D Artstein 延迟补偿；
- 总结 6D Artstein Disc 车体运动学扩展；
- 总结 QP 避障安全修正；
- 分析当前方法局限；
- 展望多 follower、通信拓扑、实物复杂环境和更严格约束控制。

## 3. 建议小论文拆分

### 小论文 1：4D + 6D Artstein 延迟补偿齐次编队

建议题目：

```text
考虑输入延迟的多机器人齐次编队控制及其全向车体运动学扩展
```

对应毕业论文第 3、4 章。

适合强调：

- Artstein 预测补偿；
- 4D 双积分齐次编队；
- 6D 全向车体运动学扩展；
- 延迟条件下的仿真和实物验证；
- 4D/6D 公平对比。

不建议把 6D 单独作为第一篇小论文，因为 6D 的主要价值是建模扩展和应用边界分析，
和 4D Artstein 合并后更完整。

### 小论文 2：QP 避障融合齐次编队控制

建议题目：

```text
基于二次规划安全修正的多机器人齐次编队避障控制
```

对应毕业论文第 5 章。

适合强调：

- nominal 编队控制与安全修正分层；
- 激光雷达障碍约束；
- QP 最小修改控制输入；
- 安全距离、编队误差和控制平滑性的综合验证。

### 可选小论文 3：ROS 2 多机器人实验平台与延迟诊断

建议只作为备用。该方向偏工程系统实现，理论创新较弱，但可以支撑毕业论文实验平台章节。

可写内容：

- ROS 2 多机器人命名空间和 TF 管理；
- Gazebo/实物一致实验流程；
- 执行器延迟和网络延迟诊断；
- 全向机器人定位链路和控制链路。

## 4. 推荐主线

推荐使用如下主线组织全文：

```text
基础齐次编队控制
  -> 输入延迟和执行器滞后补偿
  -> 面向全向车体运动学的 6D 扩展
  -> 面向障碍环境的 QP 安全修正
```

这样第 3、4、5 章之间是递进关系，而不是几个松散算法的堆叠。

## 5. 与原始齐次控制论文的创新性关系

本课题主要参考的原始论文 `homogeneous_control.pdf` 题为：

```text
Leader-Follower Tracking with Collision Avoidance for Omni-directional Mobile Robots:
Linear vs Homogeneous Controller
```

该论文的主要创新不在全向轮底盘的轮级建模，而在以下控制理论链条：

```text
leader-follower 安全编队点
  -> 非超调线性跟踪控制器
  -> 齐次有限时间控制器升级
  -> 正不变锥/齐次 barrier 保证非超调避碰
  -> 有界扰动下 ISS 鲁棒性证明
```

其中最核心的是：先构造能保证 `e(t) >= 0` 的线性控制器，使 follower 不越过 leader
安全边界；再基于该线性控制器构造齐次反馈，使无扰动系统有限时间收敛，并在有界扰动下保持
输入到状态稳定。它的理论证明比较集中、干净，属于“一个理论点打得较深”的工作。

本课题的创新定位不宜写成“重新提出齐次控制理论”，而应写成在原始齐次安全编队控制基础上的
实际问题扩展：

```text
原论文: 无延迟 4D 质点误差系统 + 非超调齐次安全编队
本文第 3 章: 加入输入延迟和低阶等效速度执行器响应，构造 4D Artstein-HPC 预测补偿
本文第 4 章: 推广到 map 位置 + yaw + body 速度的 6D 全向车体运动学状态
本文第 5 章: 引入 QP 安全修正，处理真实障碍物约束
实验部分: 通过数值仿真、Gazebo 和实物平台验证延迟、约束和定位链路影响
```

从创新性强弱看，可以这样把握：

| 维度 | 原始齐次控制论文 | 本课题 |
|------|------------------|--------|
| 核心理论原创性 | 更强，集中在非超调齐次有限时间控制和 ISS 证明 | 中等偏强，主要是面向延迟、运动学和安全约束的扩展 |
| 数学证明干净程度 | 更强，基于 4D 常值线性双积分系统 | 4D 的 Artstein 消纯时延推导严格；包含一阶执行器预测的完整闭环采用实用稳定与误差边界表述，6D 进一步需要局部冻结 |
| 工程真实性 | 较弱，主要是仿真验证 | 更强，包含 ROS 2、Gazebo、实物、延迟和执行器约束 |
| 输入延迟/执行器滞后 | 未处理 | 第 3 章核心贡献 |
| 全向车体 yaw 与 body-frame 速度 | 未充分纳入主状态 | 第 4 章核心贡献 |
| 障碍物避障 | 只考虑 leader 安全区 | 第 5 章扩展到真实障碍安全约束 |
| 毕业论文支撑度 | 单点理论强 | 体系完整，适合硕士课题 |

因此，论文中建议采用如下总体表述：

```text
在已有齐次安全编队控制方法基础上，本文进一步考虑输入延迟、执行器滞后和全向移动机器人
车体运动学约束，提出面向实际多机器人系统的预测补偿齐次编队控制方法，并结合 QP 安全修正
实现障碍环境下的编队避障。
```

这类表述避免和原论文正面比较“谁的理论更强”，而是突出本文解决了原论文未覆盖的实际问题。
更准确的评价是：

```text
原论文的理论纯度更高；本文的系统完整度、工程落地性和实际约束覆盖更强。
```

## 6. 可参考的真实文献基础

本节列出可支撑本课题选题和章节设计的真实论文/专著。条目按用途分类，后续写开题报告、
毕业论文和小论文时可据此扩展文献综述。以下文献均已通过 DOI、出版社页面、IEEE/Springer/SIAM
页面、arXiv/HAL 页面或作者/学校公开页面核验。

### 6.1 齐次控制、有限时间稳定与非超调安全控制

| 文献 | 可支撑内容 | 与本课题关系 |
|------|------------|--------------|
| S. P. Bhat and D. S. Bernstein, "Finite-Time Stability of Continuous Autonomous Systems," SIAM Journal on Control and Optimization, 2000. DOI: [10.1137/S0363012997321358](https://doi.org/10.1137/S0363012997321358) | 有限时间稳定的经典 Lyapunov 理论基础 | 第 2、3、4 章中有限时间收敛表述的基础 |
| S. P. Bhat and D. S. Bernstein, "Geometric Homogeneity with Applications to Finite-Time Stability," Mathematics of Control, Signals, and Systems, 2005. DOI: [10.1007/s00498-005-0151-x](https://doi.org/10.1007/s00498-005-0151-x) | 齐次系统与有限时间稳定的经典关系 | 支撑“负齐次度 + 渐近稳定推出有限时间稳定”的理论背景 |
| A. Polyakov, *Generalized Homogeneity in Systems and Control*, Springer, 2020. DOI: [10.1007/978-3-030-38449-4](https://doi.org/10.1007/978-3-030-38449-4) | 广义齐次控制、齐次范数、线性控制器升级到齐次反馈 | 原始 `homogeneous_control.pdf` 及本课题 HPC 实现的主要理论工具 |
| A. Polyakov and M. Krstic, "Finite- and Fixed-Time Nonovershooting Stabilizers and Safety Filters by Homogeneous Feedback," IEEE Transactions on Automatic Control, 2023. DOI: [10.1109/TAC.2023.3237907](https://doi.org/10.1109/TAC.2023.3237907) | 齐次反馈、非超调稳定、安全滤波 | 支撑“有限/固定时间 + 非超调安全”的研究脉络 |
| A. Polyakov, D. Efimov and X. Ping, "Consistent discretization of homogeneous finite/fixed-time controllers for LTI systems," Automatica, 2023. DOI: [10.1016/j.automatica.2023.111118](https://doi.org/10.1016/j.automatica.2023.111118) | 齐次有限/固定时间控制器的采样实现 | 支撑 ROS 2 离散控制周期实现和数值仿真离散化讨论 |
| K. Zimenko, D. Efimov, A. Polyakov and A. Kremlev, "Finite-time stability analysis of homogeneous systems with sector nonlinearities," Automatica, 2024. DOI: [10.1016/j.automatica.2024.111872](https://doi.org/10.1016/j.automatica.2024.111872) | 近年来齐次系统有限时间稳定分析进展 | 可用于绪论中说明齐次控制仍是活跃研究方向 |
| W. Yuan, C. Dong, X. Duan, A. Polyakov, K. Zimenko and X. Ping, "Leader-Follower Tracking with Collision Avoidance for Omni-directional Mobile Robots: Linear vs Homogeneous Controller," Chinese Control Conference, 2024. DOI: [10.23919/CCC63176.2024.10662358](https://doi.org/10.23919/CCC63176.2024.10662358) | 原始 leader-follower 齐次安全编队论文 | 本课题第 2 章基础算法，后续第 3、4、5 章均在其基础上扩展 |

写作建议：上述文献中，Bhat & Bernstein、Polyakov 专著适合放在“有限时间/齐次控制基础”综述；
Yuan 等 2024 是本课题最直接的基础工作；Polyakov & Krstic 2023 可用于说明“非超调安全”
和“齐次安全滤波”不是凭空提出，而是有清晰理论脉络。

### 6.2 Artstein 变换、预测反馈与输入延迟补偿

| 文献 | 可支撑内容 | 与本课题关系 |
|------|------------|--------------|
| Z. Artstein, "Linear Systems with Delayed Controls: A Reduction," IEEE Transactions on Automatic Control, 1982. DOI: [10.1109/TAC.1982.1103023](https://doi.org/10.1109/TAC.1982.1103023) | 将线性输入延迟系统约化为无延迟系统的经典 Artstein reduction | 第 3 章 4D Artstein 预测变换的核心理论来源 |
| M. Krstic, "Input Delay Compensation for Forward Complete and Strict-Feedforward Nonlinear Systems," IEEE Transactions on Automatic Control, 2010. DOI: [10.1109/TAC.2009.2034923](https://doi.org/10.1109/TAC.2009.2034923) | 非线性系统输入延迟补偿和预测反馈 | 支撑“预测补偿不局限于线性系统”的研究背景 |
| M. Krstic, *Delay Compensation for Nonlinear, Adaptive, and PDE Systems*, Birkhauser, 2009. Springer page: [Delay Compensation for Nonlinear, Adaptive, and PDE Systems](https://link.springer.com/book/10.1007/978-0-8176-4877-0) | 预测反馈和延迟补偿的系统性专著 | 可作为第 3 章延迟补偿综述的经典参考 |
| I. Karafyllis and M. Krstic, *Predictor Feedback for Delay Systems: Implementations and Approximations*, Birkhauser, 2017. Springer preview: [Predictor Feedback for Delay Systems](https://flyingv.ucsd.edu/krstic/B11-preface%2Bcontents.pdf) | 预测反馈的实现、近似、采样、噪声和建模误差问题 | 支撑本文 ROS/Gazebo/实物中预测补偿实现和误差来源分析 |
| J. Ni, L. Liu, C. Liu and J. Liu, "Fixed-Time Leader-Following Consensus for Second-Order Multiagent Systems With Input Delay," IEEE Transactions on Industrial Electronics, 2017, 64(11): 8635-8646. DOI: [10.1109/TIE.2017.2701775](https://doi.org/10.1109/TIE.2017.2701775) | 对二阶 Leader-Follower 多智能体输入延迟误差使用扩展 Artstein reduction，结合固定时间观测器和非奇异终端滑模控制 | 与本文同属“二阶多智能体 + Artstein + 固定时间”理论路线；可作为 Zhang 与 Zhou（2025）之前的重要直接基线，但未考虑全向底盘的低阶等效速度执行器响应、速度接口和安全约束 |
| A. Zhang, D. Zhou, M. Yang and P. Yang, "Finite-Time Formation Control for Unmanned Aerial Vehicle Swarm System With Time-Delay and Input Saturation," IEEE Access, 2019, 7: 5853-5864. DOI: [10.1109/ACCESS.2018.2889858](https://doi.org/10.1109/ACCESS.2018.2889858) | 采用 Artstein 变换处理 UAV 群的输入延迟，并同时研究有限时间编队和输入饱和 | 可支撑“Artstein + 编队 + 执行器约束”的相关工作；本文进一步面向全向地面机器人，采用低阶等效速度执行器响应模型并在 ROS 2 平台验证 |
| C. Wang, I. H. P. Tnunay, Z. Zuo, B. Lennox and Z. Ding, "Fixed-Time Formation Control of Multi-Robot Systems: Design and Experiments," IEEE Transactions on Industrial Electronics, 2018. DOI: [10.1109/TIE.2018.2870409](https://doi.org/10.1109/TIE.2018.2870409) | 多机器人固定时间编队、输入延迟、预测状态变换和实验验证 | 说明“延迟 + 有限/固定时间编队”已有相关研究，本文可强调与齐次安全编队和全向机器人平台结合 |
| W. Jiang, C. Wang and Y. Meng, "Fully Distributed Time-Varying Formation Tracking Control of Linear Multi-Agent Systems With Input Delay and Disturbances," Systems & Control Letters, 2020, 146: 104814. DOI: [10.1016/j.sysconle.2020.104814](https://doi.org/10.1016/j.sysconle.2020.104814) | 基于 Artstein model reduction 构造状态预测器，研究输入延迟、扰动和时变编队下的全分布式跟踪 | 支撑本文对 ROS 测量噪声、Leader 信息新鲜度和动态编队的误差来源讨论；其对象为一般线性系统，未涉及全向底盘执行器模型 |
| X. Ai and L. Wang, "Distributed Fixed-Time Event-Triggered Consensus of Linear Multi-Agent Systems With Input Delay," International Journal of Robust and Nonlinear Control, 2021, 31(7): 2526-2545. DOI: [10.1002/rnc.5404](https://doi.org/10.1002/rnc.5404) | 使用 Artstein-Kwon-Pearson reduction，将输入延迟线性多智能体系统转化为无延迟系统，并设计固定时间事件触发一致性协议 | 支撑“Artstein + 固定时间 + 降低控制/通信更新频率”的扩展方向；可作为未来 ROS 2 事件触发实现的理论参考 |
| X. Ai, Y.-Y. Chen and H. Yu, "Adaptive Fault-Tolerant Formation Tracking Control of Networked Mobile Robots With Input Delays," Journal of the Franklin Institute, 2024, 361: 248-264. DOI: [10.1016/j.jfranklin.2023.11.020](https://doi.org/10.1016/j.jfranklin.2023.11.020) | 网络移动机器人在输入延迟、参数不确定和执行器故障下的自适应容错编队跟踪 | 与本文移动机器人和网络化输入延迟场景高度相关；适合说明本文对电机动态、饱和与实物延迟的工程问题关注，但其控制框架不同于本文的 Artstein-HPC |
| H. Zhang and D. Zhou, "Event-Triggered Finite-Time Consensus Scheme for Time-Delay Multi-Agent Systems with Settling Time Estimation and its Application," Journal of Aerospace Technology and Management, 2025, 17: e0925. DOI: [10.1590/jatm.v17.1369](https://doi.org/10.1590/jatm.v17.1369) | 对带常值输入延迟的二阶多智能体 Leader-Follower 误差构造 Artstein 变换，得到无显式延迟的双积分误差系统，并设计事件触发有限时间一致性/编队控制 | 与本文使用同一 Artstein reduction 理论。该文属于“相对误差侧、纯双积分器”形式；本文属于“执行器状态侧、含低阶等效速度响应”形式，且额外采用前向预测并接入齐次安全编队控制 |

写作建议：第 3 章不要只说“本文提出 Artstein 预测”，应明确说明经典 Artstein reduction
已存在，本文贡献是将其嵌入原始齐次安全编队控制，并进一步考虑低阶等效速度执行器响应、ROS 2
离散实现和实物约束。Zhang 与 Zhou（2025）可作为近期直接相关工作：两者共享 Artstein
变换的理论核心，但不要表述为“本文复现其算法”。应说明该文在相对误差上处理纯输入延迟，
而本文在全向底盘执行器状态上处理输入死区，并结合低阶等效速度响应、前向预测、速度/加速度
饱和和齐次安全编队控制。

### 6.3 全向移动机器人建模与轨迹跟踪

| 文献 | 可支撑内容 | 与本课题关系 |
|------|------------|--------------|
| C. Ren and S. Ma, "Dynamic Modeling and Analysis of an Omnidirectional Mobile Robot," IEEE/RSJ IROS, 2013. DOI: [10.1109/IROS.2013.6697041](https://doi.org/10.1109/IROS.2013.6697041) | 全向移动机器人动力学建模和分析 | 支撑第 2、4 章中全向底盘模型和运动学/动力学背景 |
| C. Wang, X. Liu, X. Yang, F. Hu, A. Jiang and C. Yang, "Trajectory Tracking of an Omni-Directional Wheeled Mobile Robot Using a Model Predictive Control Strategy," Applied Sciences, 2018. DOI: [10.3390/app8020231](https://doi.org/10.3390/app8020231) | 全向轮移动机器人轨迹跟踪与约束控制 | 可用于说明全向机器人跟踪中约束处理和 MPC 是常见路线，本文选择 Artstein-HPC 作为不同路线 |
| Z. Zeng, H. Lu and Z. Zheng, "High-speed trajectory tracking based on model predictive control for omni-directional mobile robots," CCDC, 2013. DOI: [10.1109/CCDC.2013.6561493](https://doi.org/10.1109/CCDC.2013.6561493) | 高速全向机器人轨迹跟踪和 MPC 控制 | 可支撑第 4 章中“中高速受约束影响更明显”的背景 |
| M. Galicki, "Finite-time control of omnidirectional mobile robots," in *Nonlinear Dynamics and Control*, Springer, 2020. | 全向移动机器人的有限时间控制 | 可作为 6D Artstein Disc 的相关有限时间全向机器人控制参考 |

写作建议：第 4 章应说明本文并非提出全向机器人建模本身，而是把延迟补偿齐次编队控制放入
更贴近全向底盘 `cmd_vel` 接口的 6D 混合状态框架中。

### 6.4 QP、CBF 和多机器人避障/约束优化

| 文献 | 可支撑内容 | 与本课题关系 |
|------|------------|--------------|
| A. D. Ames, X. Xu, J. W. Grizzle and P. Tabuada, "Control Barrier Function Based Quadratic Programs for Safety Critical Systems," IEEE Transactions on Automatic Control, 2017. DOI: [10.1109/TAC.2016.2638961](https://doi.org/10.1109/TAC.2016.2638961) | CBF-QP 安全关键控制的经典论文 | 第 5 章 QP 安全修正层的核心理论参考 |
| A. D. Ames, S. Coogan, M. Egerstedt, G. Notomista, K. Sreenath and P. Tabuada, "Control Barrier Functions: Theory and Applications," ECC, 2019. DOI: [10.23919/ECC.2019.8796030](https://doi.org/10.23919/ECC.2019.8796030) | CBF 理论和应用综述 | 可用于第 5 章综述 CBF/QP 安全滤波思想 |
| P. Glotfelter, J. Cortes and M. Egerstedt, "Nonsmooth Barrier Functions with Applications to Multi-Robot Systems," IEEE Control Systems Letters, 2017. DOI: [10.1109/LCSYS.2017.2710949](https://doi.org/10.1109/LCSYS.2017.2710949) | 非光滑 barrier、多机器人组合安全约束 | 支撑多机器人避障和组合安全约束表述 |
| J. Alonso-Mora, S. Baker and D. Rus, "Multi-robot formation control and object transport in dynamic environments via constrained optimization," The International Journal of Robotics Research, 2017. DOI: [10.1177/0278364917719333](https://doi.org/10.1177/0278364917719333) | 动态环境中多机器人编队、避障和约束优化 | 支撑第 5 章“编队控制 + 障碍约束优化”的工程相关性 |
| A. Singletary, W. Guffey, T. G. Molnar, R. Sinnet and A. D. Ames, "Comparative Analysis of Control Barrier Functions and Artificial Potential Fields for Obstacle Avoidance," IROS, 2021. DOI: [10.1109/IROS51168.2021.9636670](https://doi.org/10.1109/IROS51168.2021.9636670) | CBF 与人工势场避障对比 | 可用于说明 QP/CBF 避障相对于势场法的优势和边界 |

写作建议：第 5 章不要把 QP 避障写成替代齐次控制，而应写成 safety filter：

```text
齐次编队控制给出 nominal command，QP/CBF 层在最小修改 nominal command 的前提下满足障碍物安全约束。
```

### 6.5 文献综述与本文创新的连接方式

综述中建议按以下逻辑组织：

```text
1. 有限时间/齐次控制提供快速收敛和鲁棒性理论基础；
2. 原始齐次安全编队论文已解决无延迟 4D leader-follower 非超调安全跟踪；
3. Artstein reduction 和 predictor feedback 是处理输入延迟的经典工具；
4. 多机器人延迟编队已有固定时间/预测状态变换研究，但未直接处理本文的齐次安全编队 + 全向平台实现链路；
5. 全向移动机器人轨迹跟踪研究多采用 MPC/滑模/ADRC 等路线，本文选择延迟补偿齐次控制路线；
6. CBF/QP 和约束优化文献说明安全修正层是处理障碍约束的主流方法；
7. 因此，本文贡献是把 4D/6D Artstein-HPC 和 QP 安全修正结合到 ROS 2/Gazebo/实物全向多机器人平台中。
```

可在绪论末尾这样收束：

```text
现有研究分别在齐次有限时间控制、输入延迟预测补偿、全向机器人轨迹跟踪和 CBF/QP 安全控制方面
取得了大量成果，但面向全向多机器人 leader-follower 编队，将输入延迟、执行器滞后、车体运动学状态、
速度/加速度约束和障碍物安全修正放在同一实验链路下验证的工作仍相对不足。
```

## 7. 风险与表述建议

### 6D Artstein 的表述风险

6D Artstein 不应承诺在所有场景下优于 4D Artstein。更稳的表述是：

```text
6D Artstein Disc 在更贴近全向移动机器人 cmd_vel 接口的车体运动学状态下，
保持了延迟补偿齐次编队控制的有效性；在低速和实物加速度约束范围内可达到与
4D Artstein 接近的跟踪效果，在中高速下则体现出 yaw、车体系速度和饱和约束耦合带来的性能边界。
```

### 轮级动力学建模的取舍

当前课题不建议加入完整轮级动力学建模。原因是本文控制器的实际接口是 ROS/Gazebo/实物底盘通用的
`cmd_vel`：

```text
控制器输出: v_x^b, v_y^b, omega
底层驱动器/planar_move/STM32: 完成轮速分配和电机闭环控制
```

因此，第 4 章采用车体级全向运动学建模更符合当前系统边界：

```text
x = [p_x, p_y, theta, v_x^b, v_y^b, omega]^T
dot p = R(theta) v_body
dot theta = omega
```

完整轮级动力学会额外引入轮子转动惯量、电机力矩常数、减速比、轮地摩擦、滑移、驱动器电流环/速度环等参数。
若没有系统辨识和轮级实验支撑，模型复杂度会上升，但实验可信度不一定提高，反而会稀释本文“齐次编队控制 +
Artstein 延迟补偿 + 6D 车体运动学扩展 + QP 避障”的主线。

本文更合理的做法是：不把轮速作为状态，而是在控制输出端加入三轮全向底盘轮速约束和加速度约束：

```text
w1 = ( v_y^b + L omega ) / r
w2 = ( -cos(30deg) v_x^b - sin(30deg) v_y^b + L omega ) / r
w3 = (  cos(30deg) v_x^b - sin(30deg) v_y^b + L omega ) / r
```

论文中建议这样表述：

```text
本文不建立完整轮级动力学模型，而采用车体级全向运动学建模。这是因为实际控制器通过 cmd_vel
接口向底盘发送期望车体速度，底层驱动器完成轮速分配和电机闭环控制。因此，本文在控制器输出端
引入三轮全向底盘轮速约束和加速度约束，以保证命令的可实现性。
```

只有在以下情况下才建议进一步引入完整轮级动力学：

- 直接控制每个轮子的转速、电流或电压；
- 研究轮胎打滑、摩擦和地面接触；
- 证明电机动力学和轮级动力学下的闭环稳定性；
- 实验误差主要来自单个轮子动态响应不一致；
- 导师明确要求动力学建模或参数辨识作为独立创新点。

### 实物实验的表述风险

当实物最大加速度约为 `0.25-0.30 m/s^2` 时，`0.5 m/s` leader 速度下的误差很可能主要由
执行器能力和饱和相位滞后决定。论文中应避免写成“控制器失效”，建议写成：

```text
受执行器速度/加速度约束影响，系统进入约束主导区间，Artstein 预测补偿可以降低延迟引起的额外相位损失，
但不能突破物理可达控制能力。
```

### QP 避障章节的表述风险

QP 避障层的贡献应写成安全修正或安全约束融合，不要写成替代原齐次控制器：

```text
QP 层以齐次编队控制输出为 nominal command，通过最小修改满足障碍物安全约束。
```

## 8. 硕士毕业支撑度评估

从硕士论文评审视角看，当前课题内容足够支撑毕业，但它的类型应定位为：

```text
基于齐次控制理论的工程增强型多机器人编队控制研究
```

而不是：

```text
全新齐次控制理论研究
```

当前工作的主线是：

```text
已有齐次安全编队控制
  -> 加入输入延迟和执行器滞后补偿
  -> 推广到全向车体级 6D 状态
  -> 加入 QP 避障安全修正
  -> ROS 2 / Gazebo / 实物验证
```

因此，课题的理论创新不是单点特别强，而是由多个实际问题扩展形成完整体系。

### 最有理论支撑的部分

第 3 章 `4D Artstein-HPC 延迟补偿齐次编队控制` 是当前课题中理论链条最清晰的部分：

```text
输入延迟系统
  -> Artstein 无显式输入时延系统（名义模型）
  -> 一阶执行器前向预测
  -> 预测误差/有界扰动分析
  -> 预测状态上的齐次控制
```

这部分应作为全文最主要的方法创新。论文中应把 Artstein 等价变换、预测状态定义、
HPC 使用条件、预测误差来源和延迟补偿后的实用稳定性表述写扎实。不能把一阶执行器预测后的
完整闭环直接表述为原始双积分 HPC 的严格有限时间稳定系统。

### 工程和系统支撑较强的部分

当前项目的优势不只在算法，还在完整实验链路：

- 数值仿真；
- Gazebo 双机器人仿真；
- ROS 2 控制节点；
- 延迟注入节点；
- EKF / rf2o 定位链路；
- 实物加速度和延迟参数讨论；
- 控制器日志诊断；
- Bug 记录和工程复现。

这些内容对工程型硕士论文很有支撑力。论文中应把它们组织成统一实验体系，而不是零散截图和调参记录。

### 主要风险

当前课题的风险不是工作量不足，而是论文结构容易显得分散：

```text
4D Artstein
6D Artstein
QP 避障
ROS 2 平台
```

如果直接堆叠这些内容，评审可能会觉得理论主线不够集中。因此全文必须围绕一个统一主题：

```text
面向实际全向多机器人系统的延迟补偿与安全编队控制
```

在这个主题下，第 3、4、5 章分别对应延迟补偿、车体运动学扩展和安全约束扩展，形成递进关系。

### 建议补强项

为提高毕业论文说服力，建议补强以下三点：

1. **4D Artstein 理论证明写完整**

   包括适用假设、Artstein 无显式输入时延系统、预测时标、预测误差、HPC 名义控制律以及
   Leader 加速度、姿态耦合、采样和饱和下的实用稳定性表述。

2. **6D Artstein 不硬写成强理论创新**

   更稳的定位是局部冻结线性化、实用稳定、模型扩展和适用边界分析。

3. **实验指标统一**

   每个主要实验都尽量使用统一指标：

```text
mean position error
tail mean error
max distance error
velocity command norm
yaw error
settling time
```

这样论文会表现为系统研究，而不是单纯工程调参。

### 结论判断

```text
硕士毕业: 够
普通小论文/工程应用类论文: 有机会
高水平纯理论论文: 当前深度不够
```

因此，本文应强调“在已有齐次控制理论基础上面向实际全向多机器人系统补齐延迟、运动学、避障和实验验证”，
而不是强调“提出全新的齐次控制理论”。

## 9. 齐次控制的控制谱系与对比算法设计

齐次控制应被归入以下控制方法谱系：

```text
非线性控制
  -> 有限时间/固定时间控制
    -> 齐次系统方法
```

它不是按工程结构命名的 PID、MPC 或滑模控制，而是一类利用系统在特定 dilation 下的齐次性来构造反馈律、
证明稳定性和收敛速度的非线性控制方法。论文中可采用如下表述：

```text
齐次控制是一类基于齐次系统理论的非线性控制方法。其核心思想是利用系统在特定尺度变换下的齐次性
构造反馈律。与传统渐近稳定控制相比，负齐次度系统可实现有限时间收敛，并具有较好的鲁棒性。
```

### 题目不要只锁定“齐次控制”

如果论文题目写成：

```text
基于齐次控制的多机器人编队控制研究
```

容易被追问：

```text
为什么只研究齐次控制？是否和其他控制方法比较？
```

更稳的题目应让“实际问题”成为主角：

```text
考虑输入延迟与安全约束的全向多机器人编队控制研究
```

或：

```text
面向全向移动机器人的延迟补偿与安全编队控制方法研究
```

然后在摘要和贡献中说明：

```text
本文以齐次控制为主要理论工具，结合 Artstein 预测补偿和 QP 安全修正，研究输入延迟、
执行器滞后和障碍约束下的多机器人编队控制问题。
```

这样即使导师要求加入其他控制算法对比，论文主线也不会被“齐次控制”四个字锁死。

### 推荐对比算法

建议至少准备以下对比组：

1. **LPC / PD 类线性反馈**

   这是最自然的 baseline。原始参考论文本身就是 `Linear vs Homogeneous Controller`，
   因此 `LPC vs HPC` 对比最容易解释。

2. **无延迟补偿控制**

   用于直接证明 Artstein 预测补偿的价值：

```text
原始控制 + delay
Artstein 预测补偿 + delay
```

3. **MPC**

   MPC 是约束处理能力强的代表性方法，但计算量和模型依赖更高。本文已有 `MPC 6D`
   控制器，可作为对照组，而不建议把 MPC 扩展为主章节。

4. **人工势场或简单斥力避障**

   可作为第 5 章 QP 避障的工程 baseline，用于说明简单避障方法容易引入震荡、局部拉扯或编队误差增大。

5. **CBF/QP 安全滤波**

   这不是齐次控制的竞争者，而是安全层：

```text
nominal controller: 齐次控制 / Artstein-HPC
safety layer: QP / CBF
```

### 推荐实验矩阵

如果时间充足，可使用较完整的对比矩阵：

```text
第 3 章 4D 延迟补偿:
  - LPC 无延迟
  - HPC 无延迟
  - LPC + delay
  - HPC + delay
  - Artstein-LPC + delay
  - Artstein-HPC + delay

第 4 章 6D 车体运动学:
  - 6D LPC
  - 6D HPC
  - 6D Artstein-LPC
  - 6D Artstein-HPC
  - 4D Artstein-HPC 对照

第 5 章 避障:
  - nominal 编队控制，无避障
  - 人工势场/简单斥力避障
  - QP 安全修正避障
```

如果工作量压力较大，最小可接受对比为：

```text
LPC vs HPC
无 Artstein vs Artstein
无 QP vs QP
```

这三组已经能回答“是否只局限于齐次控制、是否和其他算法比较”的问题。

### 对导师质疑的建议回答

如果导师认为题目不能只围绕齐次控制，可以这样回答：

```text
本文不是只研究齐次控制本身，而是围绕实际全向多机器人编队问题，比较线性控制、齐次控制、
Artstein 预测补偿、MPC 对照和 QP 安全修正等方法。齐次控制是主要理论工具，
Artstein 和 QP 分别解决延迟补偿与安全约束问题。
```

这能把课题从“单一控制器研究”提升为“问题驱动的多方法编队控制研究”。

## 10. 当前最推荐的论文目录

```text
第 1 章 绪论
第 2 章 多机器人系统建模与基础齐次编队控制
第 3 章 面向输入延迟的 4D Artstein 齐次编队控制
第 4 章 面向全向车体运动学的 6D Artstein Disc 齐次编队控制
第 5 章 基于 QP 安全修正的避障编队控制
第 6 章 仿真与实物实验综合分析
第 7 章 总结与展望
```

若学校或导师希望论文更紧凑，也可以合并为 6 章：

```text
第 1 章 绪论
第 2 章 系统建模与基础控制
第 3 章 4D/6D Artstein 延迟补偿齐次编队控制
第 4 章 QP 安全避障编队控制
第 5 章 仿真与实物实验
第 6 章 总结与展望
```

但从工作量展示和章节独立性看，更推荐 7 章版本。
