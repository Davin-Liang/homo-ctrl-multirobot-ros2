# 执行器滞后增广齐次编队控制：6D 电机感知模型与实物大延迟补偿

## 摘要

针对 mini_omni 全向移动机器人在实物场景下执行器响应延迟大（T_90% ≈ 1.24s，等效时间常数 τ ≈ 0.43s）导致 Leader-Follower 编队震荡的问题，本文将执行器一阶滞后动力学显式增广为系统状态，提出 6D 电机感知模型 $[p_x, p_y, v_x^{\mathrm{cmd}}, v_y^{\mathrm{cmd}}, v_x^{\mathrm{real}}, v_y^{\mathrm{real}}]^{\mathsf{T}}$。核心创新在于使齐次比例控制器（HPC）**天然感知**"指令速度 $\neq$ 实际速度"的物理现实——不是通过外加 Smith 预估器等反馈层补偿，而是在 HPC 的增益调度（gain scheduling）和误差翘曲（error warping）层面直接利用延迟信息，从根本上消除控制器因误判"执行器不响应"而累积的过度补偿。文中给出三阶自适应极点配置的解析解（保证 $(\mathrm{s}+\lambda)^3$ 三重极点，$\lambda \geq \omega_d$）、系统矩阵可控性与齐次性近似分析、8D 后续扩展的数学框架，以及经仿真实物联合扫参标定的默认参数集。

**关键词**：齐次控制，Leader-Follower 编队，执行器动力学，电机延迟，全向移动机器人，有限时间控制

## 1. 引言

### 1.1 问题背景

多机器人编队控制已发展出一系列成熟方法。Yuan et al. [1] 针对全向移动机器人将各机器人建模为二维双积分器 $\mathbf{x} = [p_x, p_y, v_x, v_y]^{\mathsf{T}} \in \mathbb{R}^4$，并通过 Polyakov 的广义齐次化框架 [2–4] 将线性比例控制器（LPC）升级为齐次比例控制器（HPC），实现有限时间收敛与增强鲁棒性。

然而，该 4D 质点模型假设控制输入 $\mathbf{u}$ 直接对应加速度 $\dot{\mathbf{v}}$，即**指令速度与执行器实际速度之间不存在任何延迟**：

$$\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}, \quad
A = \begin{bmatrix} 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 \end{bmatrix}, \quad
B = \begin{bmatrix} 0 & 0 \\ 0 & 0 \\ \frac{1}{m} & 0 \\ 0 & \frac{1}{m} \end{bmatrix}\tag{1}$$

在实物系统上，这条假设严重不成立。实测 mini_omni 三轮全向底盘的电机响应链（表 1）表明：从控制器发出 `cmd_vel` 到轮子速度接近目标值，总延迟约 **1.3 秒**，其中电机响应爬升段（~1.0s）占主导。

**表 1. 电机响应实测数据（阶跃响应法，cmd_vel=0.3 m/s）**

| 指标 | 值 | 说明 |
|------|-----|------|
| 起步延迟（→0.01 m/s） | P50 ~250ms | 指令死区（串口+STM32 启动） |
| 到达 90% 目标速度（→0.27 m/s） | P50 ~1240ms | 电机爬升段 |
| 等效加速度 | ~0.22–0.27 m/s² | 上位机视角 |
| 等效一阶时间常数 τ | ~0.43s | T_90% = ln(10)·τ |

**核心矛盾**：控制器每 50ms 查一次 EKF 估计的速度，每次都看到"慢了"，在约 1 秒钟内（约 20 个控制周期）连续误判为"控制不足"而不断堆叠输出，导致 overshoot → 拉回 → 震荡。这是模型失配导致的系统性问题，而非单纯的调参问题。

### 1.2 现有补偿方案的局限

**Smith 预估器** [5] 是处理时延系统的经典方法：将被控对象模型 $\hat{G}(s)$ 在无延迟条件下的输出前馈到反馈通道，使控制器"看到"无延迟的预测输出。本项目在 4D 控制器中已实现并接入 Smith 预估器（`motor_predictor.hpp`），在仿真验证中取得约 20% 的震荡改善 [内部文档]。

然而 Smith 预估器的补偿仅限于反馈层面——控制器本身仍然认为 $\dot{\mathbf{v}} = \mathbf{u}/m$。HPC 的非线性误差翘曲（$\exp(-G_d \ln\|\mathbf{e}\|_d)\mathbf{e}$）和基于齐次范数的增益缩放（$\|\mathbf{e}\|_d^{1+\mu}$）对延迟产生的误差分量与对普通跟踪误差的分量**不做区分**，两者在翘曲空间中被同等处理。换言之，Smith 在"入口"前修正了信号，但 HPC 内部不知道这个修正的存在。

### 1.3 本文贡献

1. **6D 电机感知模型**：将执行器一阶滞后 $\dot{v}^{\mathrm{real}} = (v^{\mathrm{cmd}} - v^{\mathrm{real}})/\tau$ 显式增广为系统状态，得到 $[p_x, p_y, v_x^{\mathrm{cmd}}, v_y^{\mathrm{cmd}}, v_x^{\mathrm{real}}, v_y^{\mathrm{real}}]^{\mathsf{T}} \in \mathbb{R}^6$。状态方程中 $\dot{\mathbf{p}} = \mathbf{v}^{\mathrm{real}}$ 而非 $\mathbf{v}^{\mathrm{cmd}}$，位置演化由物理真实速度驱动。

2. **三阶自适应极点配置**：每轴从 4D 的二阶链 $[\mathrm{p}, \mathrm{v}]$ 变为三阶链 $[\mathrm{p}, \mathrm{v}^{\mathrm{cmd}}, \mathrm{v}^{\mathrm{real}}]$。推导出 $(\mathrm{s}+\lambda)^3$ 三重极点配置的解析解参数化公式（定理 1），$\lambda = a/m$ 的自适应逻辑与 4D 原版完全兼容。

3. **HPC 行为分析**：从理论上分析 6D 电机模型在齐次框架下的近似性质——系统矩阵含特征值 $-\tau^{-1}$（非幂零），闭环齐次性以近似形式成立，与已有 6D 运动学控制器 [6] 的近似程度同性质，且 $-\tau^{-1}$ 为耗散项（偏安全侧）。指出 6D 三阶链 $([2,1,0])$ 在 $c_{\min}=0.5$ 时的噪声翘曲放大（~30× vs 4D 的 ~5×）是导致弛豫振荡的数学根因，并给出 $c_{\min}$ 的经验标定值。

4. **经仿真实物联合扫参标定的默认参数集**：$\mathrm{mass}=2.0, \tau=0.43, \omega_d=0.7, c_{\min}=0.9, \mathrm{accel}_{\max}=0.25$，并分析各参数与执行器物理上限的约束关系。

5. **8D 扩展（6D 运动学 + 电机滞后）的数学框架**：预留给出一体化模型 $[p_x, p_y, \theta, v_x^{\mathrm{cmd}}, v_y^{\mathrm{cmd}}, v_x^{\mathrm{real}}, v_y^{\mathrm{real}}, \omega]^{\mathsf{T}}$ 的 A/B 矩阵结构，可控链 $[3,3,2]$ 分析，以及代码层面的复用策略。

### 1.4 记号

$\mathbb{R}$ 为实数集。$\|\cdot\|$ 表示 Euclidean 范数。$R(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}$ 为二维旋转矩阵。$\mathrm{clamp}(x, a, b) = \min\{\max\{a, x\}, b\}$。$\mathrm{hnorm}(\cdot)$ 为典范 $d$-齐次范数（二分法数值求解）。

## 2. 执行器滞后增广模型

### 2.1 状态空间定义

$$\mathbf{x} = [p_x, p_y, v_x^{\mathrm{cmd}}, v_y^{\mathrm{cmd}}, v_x^{\mathrm{real}}, v_y^{\mathrm{real}}]^{\mathsf{T}} \in \mathbb{R}^6\tag{2}$$

| 状态分量 | 来源 | 物理含义 |
|----------|------|----------|
| $p_x, p_y$ | TF（map→odom）+ EKF 里程计 | map 系全局位置 |
| $v_x^{\mathrm{cmd}}, v_y^{\mathrm{cmd}}$ | **控制器内部积分状态**，每周期发布后用最终 cmd_vel 回写 | map 系指令速度（控制器"认为自己在发"的速度） |
| $v_x^{\mathrm{real}}, v_y^{\mathrm{real}}$ | EKF `/odometry/filtered` 速度旋转到 map 系 | map 系实际速度（轮子真正转出来的速度） |

$v^{\mathrm{cmd}}$ 为控制器内部积分状态：(1) 初始化时对齐 EKF 速度；(2) 每周期由控制器自行积分 $v^{\mathrm{cmd}} \gets v^{\mathrm{cmd}} + h \cdot \mathbf{u} / m$；(3) 发布后把经 clamp + 轮速约束后的最终 body 系 cmd_vel 旋转回 map 系回写（抗饱和/anti-windup）。Leader 的指令速度不可直接获得，取 $v^{\mathrm{cmd}} = v^{\mathrm{real}}$（稳态假设）。

### 2.2 系统方程

$$\begin{cases}
\dot{\mathbf{p}} = \mathbf{v}^{\mathrm{real}} \\[4pt]
\dot{\mathbf{v}}^{\mathrm{cmd}} = \dfrac{1}{m}\,\mathbf{u} \\[4pt]
\dot{\mathbf{v}}^{\mathrm{real}} = \dfrac{1}{\tau}\left(\mathbf{v}^{\mathrm{cmd}} - \mathbf{v}^{\mathrm{real}}\right)
\end{cases}\tag{3}$$

线性系统矩阵：

$$A = \begin{bmatrix}
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & \tau^{-1} & 0 & -\tau^{-1} & 0 \\
0 & 0 & 0 & \tau^{-1} & 0 & -\tau^{-1}
\end{bmatrix}, \quad
B = \begin{bmatrix}
0 & 0 \\ 0 & 0 \\
m^{-1} & 0 \\ 0 & m^{-1} \\
0 & 0 \\ 0 & 0
\end{bmatrix}\tag{4}$$

**与 4D 的关系**：$\tau \to 0^+$ 时 $v^{\mathrm{real}}$ 瞬时跟上 $v^{\mathrm{cmd}}$，$(A,B)$ 通过奇异摄动退化为 4D 双积分器。但 $\tau^{-1} \to \infty$ 在数值上不可行，故不设计 $\tau$ 退化开关——需要 4D 行为时直接使用原 4D 节点。

### 2.3 物理含义

| 设计选择 | 物理原因 |
|---------|---------|
| $\dot{\mathbf{p}} = \mathbf{v}^{\mathrm{real}}$（非 $\mathbf{v}^{\mathrm{cmd}}$） | 位置是物理现实，由轮子实际转动决定，不由指令控制 |
| $\dot{\mathbf{v}}^{\mathrm{cmd}} = \mathbf{u}/m$ | 控制力作用于"期望"通道，保留原 HPC 语义 |
| $\dot{\mathbf{v}}^{\mathrm{real}} = (\mathbf{v}^{\mathrm{cmd}} - \mathbf{v}^{\mathrm{real}})/\tau$ | 一阶低通模拟电机速度环响应；$\tau$ 越小响应越快 |
| cmd_vel 取自 $\mathbf{v}^{\mathrm{cmd}}$ | 发给 STM32 的应是指令值 |
| 输入保持 2 维 | 偏航控制独立（P+前馈），与 4D 一致，便于对比 |

**已知局限（v1）**：一阶滞后仅建模 ~1s 的爬升过程，不建模 ~250ms 的指令死区（纯时延 $T_d$）。死区补偿可后续通过 Pade 近似将 $(A,B)$ 扩展至 8D–10D，或叠加 `motor_predictor.hpp`（Smith 预估器，$\tau + T_d$ 双模型并行）。

### 2.4 可控性与齐次性分析

**可控性**：对每轴三阶链 $[p, v^{\mathrm{cmd}}, v^{\mathrm{real}}]$，可控性矩阵 $\mathcal{C} = \begin{bmatrix} B & AB & A^2B \end{bmatrix}$ 的秩为 3（x 轴）和 3（y 轴），块尺寸 $\mathbf{nt} = [2, 2, 2]$，`trans_con_nd` 的 SVD 秩检验通过。

**齐次性近似**：$A$ 含特征值 $-\tau^{-1}$（$v^{\mathrm{real}}$ 的自阻尼项），非幂零。$B$ 不作用于 $v^{\mathrm{real}}$ 行，`lpc2hpc_nd` 的线性补偿增益 $K_0 = -B_0^\dagger \cdot A_0 \cdot T$ 无法抵消该行。因此 $A + BK_0$ 不是严格积分链，**闭环齐次性以近似形式成立**。此近似与已有 6D 运动学控制器（时变 $A$ 含 $\omega$ 耦合项）性质相同 [6]，且 $-\tau^{-1}$ 为耗散项（能量衰减），闭环行为偏安全侧。此外，本模型 $A$ 为**常值**——HPC 参数 ($G_d, P, \nu$) 仅需在编队点切换时重算，无需 6D 运动学控制器的 $\omega$ 阈值触发重算。

## 3. 自适应三阶极点配置

### 3.1 每轴三阶链的极点配置问题

对 x 轴子系统（y 轴同构）：

$$\frac{d}{dt} \begin{bmatrix} p \\ v^{\mathrm{cmd}} \\ v^{\mathrm{real}} \end{bmatrix} =
\begin{bmatrix} 0 & 0 & 1 \\ 0 & 0 & 0 \\ 0 & \tau^{-1} & -\tau^{-1} \end{bmatrix}
\begin{bmatrix} p \\ v^{\mathrm{cmd}} \\ v^{\mathrm{real}} \end{bmatrix} +
\begin{bmatrix} 0 \\ m^{-1} \\ 0 \end{bmatrix} u\tag{5}$$

反馈 $u = k_1 p + k_2 v^{\mathrm{cmd}} + k_3 v^{\mathrm{real}}$ 对应的闭环特征多项式为：

$$\det\left(\begin{bmatrix} s & 0 & -1 \\ -\frac{k_1}{m} & s - \frac{k_2}{m} & -\frac{k_3}{m} \\ 0 & -\tau^{-1} & s + \tau^{-1} \end{bmatrix}\right) = s^3 + \left(\frac{1}{\tau} - \frac{k_2}{m}\right)s^2 - \frac{k_2 + k_3}{m\tau}s - \frac{k_1}{m\tau}\tag{6}$$

### 3.2 定理 1（三阶解析解）

**定理 1**：对 $(s + \lambda)^3$ 三重极点配置，增益的解析解为：

$$\boxed{
\begin{aligned}
\lambda &= \frac{a}{m} \quad (a\text{ 沿用 4D 自适应逻辑: } a = \max\{\mathrm{clamp}(-m\cdot e_v / e_p, \pm\omega_d m),\; \omega_d m\}) \\[4pt]
k_1 &= -\lambda^3 m \tau \\[4pt]
k_2 &= m\left(\frac{1}{\tau} - 3\lambda\right) \\[4pt]
k_3 &= -3\lambda^2 m \tau - k_2
\end{aligned}}\tag{7}$$

**证明**：将 (7) 代入 (6) 的特征多项式：

$$\begin{aligned}
s^3 &+ \left(\frac{1}{\tau} - \frac{m(\tau^{-1} - 3\lambda)}{m}\right)s^2
- \frac{m(\tau^{-1} - 3\lambda) + (-3\lambda^2 m\tau - m(\tau^{-1} - 3\lambda))}{m\tau}s
- \frac{-\lambda^3 m\tau}{m\tau} \\[4pt]
=&\; s^3 + 3\lambda s^2 + 3\lambda^2 s + \lambda^3 = (s + \lambda)^3 \quad \blacksquare
\end{aligned}$$

**注 1**：4D 的 `calculate_klin` 中 $a$ 不是极点本身——4D 闭环为 $s^2 + (2a/m)s + (a/m)^2$，极点为 $a/m$。本文先换算 $\lambda = a/m$（$\lambda \geq \omega_d$），再对 $(s+\lambda)^3$ 配置，保证与 4D 的自适应逻辑（$e_v$ 取 $v^{\mathrm{real}}$ 误差分量、clamp 到 $\pm\omega_d m$）完全兼容。

**注 2**：$\tau$ 较小时 $k_2 = m(\tau^{-1} - 3\lambda)$ 为大正值（正反馈），由快电机模态 $s \approx -\tau^{-1}$ 补偿，闭环整体仍为 $(s+\lambda)^3$ 稳定。但数值上 $\tau^{-1}$ 过大时增益对噪声敏感——**实现中 $\tau < 0.1$ 时拒绝构造**。

### 3.3 6D 全矩阵 K 结构

$K \in \mathbb{R}^{2 \times 6}$ 为 x/y 两轴解耦：

$$K = \begin{bmatrix}
k_{1,x} & 0 & k_{2,x} & 0 & k_{3,x} & 0 \\
0 & k_{1,y} & 0 & k_{2,y} & 0 & k_{3,y}
\end{bmatrix}\tag{8}$$

其中 $(k_{1,x}, k_{2,x}, k_{3,x})$ 和 $(k_{1,y}, k_{2,y}, k_{3,y})$ 分别由定理 1 独立计算（位置/速度误差取各自通道分量）。三阶单通道函数 `compute_channel_3rd(e_p, e_v, m, τ, ω_d)` 为独立静态函数，便于后续 8D 扩展的 x/y 通道复用。

## 4. 齐次控制器（HPC）升级

### 4.1 6D Motor 模型的 LPC→HPC 升级

对可控对 $(A, B)$ 和 Hurwitz 闭环 $A + BK$（由定理 1 保证），HPC 升级过程与标准框架 [2–4] 相同，利用已实现的 `lpc2hpc_nd` / `hnorm_nd` 库（支持任意维度）：

1. **块可控分解**：`trans_con_nd` + `block_con_nd` 得到变换矩阵 $T$ 和块尺寸 $\mathbf{nt} = [2, 2, 2]$。
2. **齐次度权重**：$G_0$ 对角块权重为 $[2, 2, 1, 1, 0, 0]$（x/y 各占一列），对应三阶链 $(k-1, k-2, \ldots, 0)$，$k=3$。
3. **控制律**（标准 HPC 形式）：

$$\boxed{\mathbf{u} = \|\mathbf{e}\|_d^{1+\mu} K \, \exp(-G_d \ln\|\mathbf{e}\|_d) \, \mathbf{e}}\tag{9}$$

其中 $\mathbf{e} = \mathbf{x}_2 - \mathbf{x}_1 - \mathbf{d}$，$\|\mathbf{e}\|_d = \mathrm{hnorm}(\mathbf{e}, G_d, P)$，输出经饱和函数 clamp$(0.5, 1.0)$ → $c_{\min}$（标定值 0.9）。

4. **前向欧拉输出**（车体系）：

$$\mathbf{v}^{\mathrm{cmd}}_{\mathrm{goal}} = \mathbf{v}^{\mathrm{cmd}}_{\mathrm{current}} + h \cdot \mathbf{u} / m\tag{10}$$

其中 $h = 1/\mathrm{control\_rate}$（**必须等于真实控制周期**；4D 的 $h=0.1$ 是输出整形系数，非积分步长——照抄会导致等效 $B$ 矩阵缩放 2 倍、三阶极点失配为欠阻尼震荡。此 bug 已修复 [内部 BUG_RECORD #30]）。

### 4.2 齐次链深度与翘曲放大的 $\omega_d$ 依赖性

6D Motor 模型的齐次链权重 $[2, 2, 1, 1, 0, 0]$（三阶链，$k=3$）比 4D 模型 $[1, 1, 0, 0]$（二阶链，$k=2$）深一级。HPC 的翘曲放大倍数由 $\exp(G_d \cdot (1 - \ln c))$ 决定，链越深，$G_d$ 的特征值越大。

**表 2. 不同链深度下 $c = c_{\min}$ 时的翘曲放大**

| 模型 | 链深度 | 最大权重 | $c_{\min}=0.5$ 时放大倍数 | $c_{\min}=0.9$ 时放大倍数 |
|------|--------|---------|------|------|
| 4D | $k=2$ | 1 | ~5× | ~1.04× |
| 6D Motor | $k=3$ | 2 | ~30× | ~1.17× |

翘曲放大本身是数学事实（直接由 $G_d$ 和 $c_{\min}$ 计算，与参数无关）。但**它是否导致震荡取决于 $\omega_d$**：

- **$\omega_d$ 偏高（如 1.2–1.5）时**：线性增益已经接近或超过物理上限，$c_{\min}=0.5$ 时 30× 翘曲将噪声和控制力进一步放大，触发达 ~1Hz 的弛豫振荡。此时需提高 $c_{\min}$ 到 0.9 压制翘曲，或降低 $\omega_d$ 到物理可达范围。
- **$\omega_d$ 标定在物理极限内（0.7）时**：线性增益本身远低于饱和阈值，即使 $c_{\min}=0.5$（30× 翘曲），总控制力仍不超物理上限，**翘曲不再构成震荡源**。此时 $c_{\min}$ 对跟踪精度的影响很小，可保留 4D 默认值 0.5，或按工程偏好微调。

早期实验中观察到的"$c_{\min} < 0.85$ 显著加剧震荡"是在 $\omega_d=1.2$–$1.5$ + h bug（等效 B 矩阵 ×2）的叠加条件下得出的。在 $\omega_d=0.7$ + h 修复 + rf2o vy 补丁的最终标定配置下，**震荡的主因是 $\omega_d$ 超物理极限和定位链路缺陷，$c_{\min}$ 为协同因素而非主因**。

## 5. 完整控制管线

### 5.1 系统总览

```
 EKF /odometry/filtered ──→ 缓冲区（回调存储最新消息）
 TF  map→robot*_odom     ──→ 定时器（20Hz）读取当前 TF
                              │
                              ├─→ map 系位置: p = T_map_odom · p_ekf
                              ├─→ map 系速度: v_real = R(total_yaw) · v_body_ekf
                              │
                    ┌─────────┴─────────┐
                    │  Leader 状态 x1   │   Follower 状态 x2
                    │  v_cmd = v_real   │   v_cmd = 内部积分状态
                    │  (稳态假设)        │   v_real = EKF 测量
                    └─────────┬─────────┘
                              │
              ┌───────────────┴───────────────┐
              │  LpcController6DMotor         │
              │  · compute_error → e (6D)     │
              │  · calculate_klin → K (2×6)   │
              │  · hnorm_nd → c → u           │
              │  · goal_vcmd = vcmd + h·u/m   │
              └───────────────┬───────────────┘
                              │ map 系 → 车体系旋转
                              │ clamp ±max_linear_vel
                              │ kinematic_constraint (轮速+加速度)
                              │ → /cmd_vel 发布
                              │
                              └─→ 回写: body 系 cmd 旋转回 map 系
                                  更新 v_cmd 内部状态（抗饱和）
```

### 5.2 数据管线

**Follower 状态构造**（每周期）：
- $p_x, p_y$：TF map→odom 作用于 EKF 在 odom 系中的位姿 → map 系
- $v_x^{\mathrm{real}}, v_y^{\mathrm{real}}$：EKF body 系速度 $R(\mathrm{total\_yaw})$ → map 系
- $v_x^{\mathrm{cmd}}, v_y^{\mathrm{cmd}}$：内部成员变量，初始化 = EKF 速度，之后积分+回写

**Leader 状态构造**（每周期）：
- $p_x, p_y$：同上
- $v_x^{\mathrm{cmd}}, v_y^{\mathrm{cmd}} = v_x^{\mathrm{real}}, v_y^{\mathrm{real}}$：同上（稳态假设）

### 5.3 参数汇总

**表 3. 控制器参数（经仿真实物联合扫参标定的默认值）**

| 参数 | 默认值 | 物理含义 | 约束 |
|------|--------|----------|------|
| $\mathrm{mass}$ | 2.0 | 控制力→加速度增益（调参，非物理质量） | 4D 用 8.0，6D Motor 降为 2.0 以匹配 0.25 加速度上限 |
| $\tau$ | 0.43 | 电机一阶时间常数 (s) | 实物标定值；$\tau < 0.1$ 时 $k_2$ 发散，构造器拒绝 |
| $\omega_d$ | 0.7 | 期望闭环带宽 (rad/s) | $\omega_d$ 需求加速度 ≈ $\omega_d \cdot v$ 须 ≤ 0.25 m/s²，否则物理不可达 |
| $c_{\min}$ | 0.9 | HPC warp clamp 下界 | 6D 三阶链 $c=0.5$ 翘曲 ~30×，须提高至 0.9 (~1.17×) |
| $\mathrm{accel}_{\max}$ | 0.25 | 运动学加速度约束 (m/s²) | 对齐实物电机等效加速度 |
| $\mathrm{h}$ | 1/20 = 0.05s | v_cmd 积分步长 | 必须等于真实控制周期（不等于 4D 的 0.1） |

## 6. 仿真实验

### 6.1 实验设置

| 条件 | 配置 |
|------|------|
| Leader | `leader_circle.py`，绕圈半径 0.5–1.0 m，速度 0.2–0.4 m/s |
| Follower | `sim_motor_delay.py` 注入：$\tau_m=0.43$, accel=0.25 m/s², 传输延迟=0 |
| 编队半径 | 2.0 m，离散多边形 $m_p=4$ |
| 仿真时长 | 45–90 s |
| 评价指标 | 编队距离均值/标准差，raw 速度翻转次数，饱和率 |

### 6.2 关键参数扫参结果

| $\omega_d$ | $c_{\min}$ | 编队距离 σ (稳态) | raw 翻转/90s | 结论 |
|------|------|------|------|------|
| 1.5 | 0.5 | 0.079 | 19 | 震荡（翘曲过大 + 物理超限） |
| 1.5 | 0.9 | 0.073 | 20 | 翘曲减弱但 $\omega_d$ 仍超物理极限 |
| 1.2 | 0.5 | 0.078 | 26 | 翘曲主导震荡 |
| 1.2 | 0.9 | 0.061 | 10 | 翘曲改善，残余由 $\omega_d$ 偏大引起 |
| **0.7** | **0.9** | **0.026** | **7** | **最优（标定值，编队误差 12cm @ 0.2m/s）** |

### 6.3 振动源隔离分析

经对照实验确认的三类振动源及其解决：

| 振动源 | 诊断方法 | 修法 |
|--------|---------|------|
| rf2o vy=0 → EKF 融合假测量（BUG #29） | 纯横向 cmd_vel 指令下 vy 恒零 | 补丁 rf2o 发布 `lin_speed_y` |
| v_cmd 积分步长 h=0.1 ≠ 控制周期（BUG #30） | LPC 下仍有 3–4s 周期慢震荡 | $h = 1/\mathrm{control\_rate}$ |
| HPC warp $c_{\min}=0.5$ 对 6D 链过大（BUG #31） | LPC-only 稳定、开 HPC 震 | $c_{\min}$ 默认 0.9 |

## 7. 后续工作：8D 电机-运动学一体化模型

### 7.1 状态扩展

将 6D 电机感知模型（map 系速度）与 6D 运动学模型（车体系速度 + 偏航）融合为 8D：

$$\mathbf{x} = [p_x, p_y, \theta, v_x^{\mathrm{cmd}}, v_y^{\mathrm{cmd}}, v_x^{\mathrm{real}}, v_y^{\mathrm{real}}, \omega]^{\mathsf{T}} \in \mathbb{R}^8\tag{11}$$

其中线速度在**车体系**（与 `cmd_vel` 语义一致，避免 map↔body 旋转），偏航角速度 $\omega$ 不拆分 cmd/real（偏航响应通常快于线速度，待实测 $\omega$ 阶跃响应后决定是否扩展至 9D）。

### 7.2 A/B 矩阵结构

$$A = \begin{bmatrix}
0 & \omega_l & -v_{y,l}^b & 0 & 0 & 1 & 0 & 0 \\
-\omega_l & 0 & v_{x,l}^b & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & \tau^{-1} & 0 & -\tau^{-1} & 0 & 0 \\
0 & 0 & 0 & 0 & \tau^{-1} & 0 & -\tau^{-1} & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}, \quad
B = \begin{bmatrix}
0 & 0 & 0 \\
0 & 0 & 0 \\
0 & 0 & 0 \\
m^{-1} & 0 & 0 \\
0 & m^{-1} & 0 \\
0 & 0 & 0 \\
0 & 0 & 0 \\
0 & 0 & I^{-1}
\end{bmatrix}\tag{12}$$

可控链 $[3, 3, 2]$（x 三阶 + y 三阶 + θ 二阶），`trans_con_nd` 直接支持不等长块。

### 7.3 代码复用策略

实现 6D Motor 时已做以下 8D 预留：
- `compute_channel_3rd` 为独立静态函数，x/y 原样复用；θ 通道复用现有二阶公式
- 全量用动态 `MatrixXd` + `_nd` 库，换维度改动仅限于 A/B 尺寸
- v_cmd 内部积分状态 + $h$ 步长参数化，接口无需变更
- 编队点策略沿用离散多边形 + tol（与 6d_disc 同款）

## 8. 工程实现说明

### 8.1 文件清单

| 文件 | 内容 |
|------|------|
| `homo_controller_6d_motor.hpp` | `LpcController6DMotor` 类：A/B 构造，编队点逻辑（离散多边形+tol），三阶 klin |
| `formation_control_node_6d_motor.cpp` | 节点实现：TF+EKF 管线，v_cmd 内部状态+回写，偏航独立 P+前馈 |
| `formation_control_node_6d_motor.hpp` | 节点头文件 |
| `main_6d_motor.cpp` | 入口 |
| `formation_single_follower_6d_motor.launch.py` | Launch 文件 |

### 8.2 实现中发现并修复的缺陷

详见 `BUG_RECORD.md` 第 29–33 条。本文档第 6.3 节已概述。

### 8.3 与 4D 的 v_cmd 积分语义差异

这是实现中最关键的细节，必须与 4D 明确区分：

| | 4D `LpcController` | 6D Motor `LpcController6DMotor` |
|------|------|------|
| 输出公式 | `goal_v = v(EKF) + h·u/m` | `goal_v = v_cmd(内部) + h·u/m` |
| `v` 来源 | 每周期从 EKF 重新测量 | `v_cmd` 是跨周期积分状态 |
| `h` 含义 | 输出整形系数（0.1），可任意调 | 积分步长，必须 = 1/control_rate |
| 含义 | 单周期线性修正 | 持续速度指令的演化 |

在 4D 中，$h=0.1$ 只是将控制力按某个比例映射为速度修正，v 每周期重新对齐 EKF 测量值，因此不累积。在 6D Motor 中，$v^{\mathrm{cmd}}$ 跨周期积分，$h$ 的有效缩放直接改变 $B$ 矩阵的等效增益——若 $h$ 不等于控制周期 $T_s$，闭环极点将偏离设计位置。

## 参考文献

1. W. Yuan, C. Dong, X. Duan, A. Polyakov, K. Zimenko, X. Ping, "Leader-Follower Tracking with Collision Avoidance for Omni-directional Mobile Robots: Linear vs Homogeneous Controller."
2. A. Polyakov, *Generalized Homogeneity in Systems and Control*, Springer, 2020.
3. A. Polyakov and M. Krstic, "Finite-and Fixed-Time Nonovershooting Stabilizers and Safety Filters by Homogeneous Feedback," *IEEE Trans. Autom. Control*, 68(11): 6434–6449, 2023.
4. A. Polyakov, "Sliding Mode Control Design Using Canonical Homogeneous Norm," *Int. J. Robust Nonlinear Control*, 29(25): 682–701, 2019.
5. O. J. M. Smith, "A Controller to Overcome Dead Time," *ISA Journal*, 6(2): 28–33, 1959.
6. 本仓库内部文档, `kinematic_homogeneous_control_full.md` — 基于运动学模型的齐次编队控制：6D 混合坐标框架与方位角约束编队.
