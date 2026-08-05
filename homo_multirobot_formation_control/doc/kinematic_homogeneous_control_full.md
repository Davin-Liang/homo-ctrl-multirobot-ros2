# 基于车体级混合模型的齐次编队控制：6D 混合坐标框架与方位角约束编队

## 摘要

针对全向移动机器人 Leader-Follower 编队控制问题，本文在原始 4D 质点双积分器模型基础上，引入偏航角、角速度以及车体系线速度，构造 6D 车体级混合坐标二阶模型。该模型的位姿部分满足平面刚体运动学，速度部分仍采用加速度输入的二阶积分形式；因此它不是轮级运动学模型，也不是纯一阶运动学模型。新模型采用混合坐标系——位置与偏航角定义于全局 map 系、速度定义于车体系——消除了质点模型中全局速度指令与 `cmd_vel` 接口之间的坐标系失配。编队策略采用方位角约束方法：将目标编队点固定于 Leader 车体系下安全圆上指定方位角 $\phi_d$ 处，Follower 的 Cartesian 位置误差同时编码了径向距离误差和切向方位角误差，使 Follower 沿平滑弧线收敛至目标点，无需离散编队点的切换逻辑。误差动力学在 Leader 车体系下导出，其系统矩阵随 Leader 速度时变，含旋转与平移耦合项。控制器采用线性比例控制器（LPC）到齐次比例控制器（HPC）的升级框架（Polyakov, 2023），在固定编队点、冻结 Leader twist、忽略饱和的 nominal 局部模型下继承有限时间收敛结论；实际系统中的时变项和约束项需作为扰动处理并通过仿真/实验验证。文中给出数学构造、局部稳定性分析以及 Gazebo 仿真环境下的工程实现。

**关键词**：齐次控制，Leader-Follower 编队，全向移动机器人，有限时间控制，车体级混合模型，方位角约束

## 1. 引言

### 1.1 问题背景

多机器人编队控制是协同机器人系统的核心问题之一。全向移动机器人因具备完整约束（holonomic）特性——可在平面内任意方向平移并独立控制偏航——在工业物流、仓储搬运、医疗服务等场景中得到广泛应用 [1–4]。

Yuan et al. [参考文献1] 针对两台全向移动机器人的 Leader-Follower 编队跟踪控制问题，将各机器人建模为二维双积分器：

$$\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}, \quad \mathbf{x} = [p_x, p_y, v_x, v_y]^{\mathsf{T}} \in \mathbb{R}^4\tag{1}$$

$$A = \begin{bmatrix} 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 \end{bmatrix}, \quad B = \begin{bmatrix} 0 & 0 \\ 0 & 0 \\ \frac{1}{m} & 0 \\ 0 & \frac{1}{m} \end{bmatrix}\tag{2}$$

该 4D 质点模型的隐含假设是速度定义在全局 map 坐标系下，机器人无朝向概念。然而在实际机器人系统中，ROS `cmd_vel` 接口（`geometry_msgs/Twist`）的语义定义在**车体坐标系**：`linear.x` 沿前进方向，`linear.y` 沿横向，`angular.z` 绕垂直轴。质点模型输出的全局系速度需外加坐标旋转才能写入 `cmd_vel`，且偏航控制作为独立回路的 P+前馈与编队解耦，无法实现平移与旋转的协同优化。

此外，原方法 [参考文献1] 的离散编队点策略（$m_p$ 个安全点均匀分布在圆周上 + 最近点选择 + tol 滞后切换）存在两个固有问题：（1）编队偏移在切换瞬间发生阶跃，造成控制指令不连续，需触发 HPC 参数重算；（2）Follower 绕 Leader 运动时，轨迹在切换点出现折线拐角，影响编队平滑性。

### 1.2 本文贡献

1. **6D 车体级混合坐标二阶模型**：将状态扩展为 $[p_x, p_y, \theta, v_x^b, v_y^b, \omega]^{\mathsf{T}}$，位置偏航在 map 系、速度在车体系。模型只在位姿更新处使用车体级刚体运动学，速度演化仍为加速度输入的二阶积分结构，使控制输出直接对应 `cmd_vel` 语义，偏航控制集成于编队主回路。

2. **方位角约束编队策略**：将目标编队点固定于 Leader 车体系下安全圆上指定方位角 $\phi_d$ 处。Cartesian 位置误差自然包含径向（距离）和切向（方位角）两个分量——径向分量将 Follower 推向/拉离安全圆，切向分量驱动 Follower 沿圆弧滑向目标方位——无需任何切换逻辑。

3. **基于 Leader 车体系的误差动力学**：通过双重坐标变换将误差统一到 Leader 车体系，导出含 Leader 速度耦合的时变系统矩阵 $A_l(\omega_l, v_{x,l}^b, v_{y,l}^b)$，并分析其时变性的物理来源。

4. **将齐次比例控制器（HPC）推广到该时变 6D 系统**，通过 Leader 机动触发的时变 HPC 重算策略适应动态环境。

5. 在 ROS 2 + Gazebo 仿真环境中完整实现并验证。

### 1.3 记号

$\mathbb{R}$ 为实数集，$\mathbb{R}_+$ 为非负实数集。$\|\cdot\|$ 表示 $\mathbb{R}^n$ 中的 Euclidean 范数。$R(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}$ 为二维旋转矩阵。$P > 0$ 表示 $P$ 对称正定。$\mathrm{atan2}(y, x)$ 为四象限反正切。

## 2. 预备知识：齐次控制框架

> 本节概述 Polyakov 广义齐次化控制框架 [2-4] 的核心概念与构造过程，仅保留理解后续内容所必需的要素。详细理论与证明请参见原文。

### 2.1 齐次性定义

**定义 1（线性膨胀）**：映射 $d(s) = \exp(s G_d)$ 称为 $\mathbb{R}^n$ 上的线性膨胀，其中 $G_d \in \mathbb{R}^{n \times n}$ 为 anti-Hurwitz 矩阵（膨胀生成元）。膨胀满足 $\lim_{s \to +\infty} \|d(s)x\| = +\infty$ 和 $\lim_{s \to -\infty} \|d(s)x\| = 0$（$\forall x \neq 0$）。

**定义 2（$d$-齐次向量场）**：向量场 $f: \mathbb{R}^n \to \mathbb{R}^n$ 具有齐次度 $\mu \in \mathbb{R}$，若 $f(d(s)x) = \exp(\mu s) d(s) f(x)$，$\forall x, s$。

**定义 3（典范齐次范数）**：由加权范数 $\|x\| = \sqrt{x^{\mathsf{T}} P x}$（$P > 0$，$P G_d + G_d^{\mathsf{T}} P > 0$）诱导的典范 $d$-齐次范数定义为 $\|0\|_d = 0$，$\|x\|_d = \exp(s_x)$，其中 $s_x \in \mathbb{R}$ 满足 $\|d(-s_x)x\| = 1$。数值上通过二分法求解：寻找 $c$ 使得 $\exp(-G_d c)x$ 落在椭球 $\{y : y^{\mathsf{T}} P y = 1\}$ 上，返回 $q = \exp(c)$。

### 2.2 LPC 到 HPC 的升级

对于可控对 $(A, B)$ 和 Hurwitz 闭环系统 $A + BK$，HPC 升级步骤如下：

**步骤 1（块可控分解）**：通过 `block_con` 将 $(A, B)$ 变换为块可控标准型，得变换矩阵 $T$ 和块尺寸 $\mathbf{nt} = [n_1, \ldots, n_k]$。

**步骤 2（齐次度权重）**：

$$G_0 = -T^{-1} \cdot \mathrm{diag}(\underbrace{k-1,\ldots,k-1}_{n_1}, \underbrace{k-2,\ldots,k-2}_{n_2}, \ldots, \underbrace{0,\ldots,0}_{n_k}) \cdot T\tag{3}$$

$$G_d = I + \mu G_0\tag{4}$$

**步骤 3（线性补偿增益）**：$K_0 = -B_0^\dagger \cdot A_{\mathrm{bottom}} \cdot T$，零化 $A + B K_0$ 的底部块行。对于积分链模型（底部块行全为零），$K_0 \equiv 0$。

**步骤 4（Lyapunov 矩阵）**：解 $(A + BK)^{\mathsf{T}} P + P(A + BK) = -2I$。

**步骤 5（齐次度范围）**：由 $M = \sqrt{P} G_0 \sqrt{P}^{-1} + \sqrt{P}^{-1} G_0^{\mathsf{T}} \sqrt{P}$ 的特征值确定 $\mu \in [\mu_{\min}, \mu_{\max}]$。

### 2.3 齐次比例控制器（HPC）

$$\boxed{\mathbf{u} = K_0 \mathbf{e} + \|\mathbf{e}\|_d^{1+\mu} (K - K_0) \, \exp(-G_d \ln\|\mathbf{e}\|_d) \, \mathbf{e}}\tag{5}$$

- $K_0 \mathbf{e}$：线性补偿项，消除不满足齐次结构的漂移分量
- $(K - K_0)$：齐次反馈增益
- $\exp(-G_d \ln\|\mathbf{e}\|_d) \mathbf{e}$：非线性误差翘曲——将误差按其齐次度权重做各向异性缩放
- $\|\mathbf{e}\|_d^{1+\mu}$：基于距离度量的增益缩放——误差大时放大控制，误差小时趋于线性

**关键性质**（证明见 [3]）：

- **有限时间稳定**（$\mu < 0$）：存在 $\rho > 0$ 使 $\dot{V} \leq -\rho V^{1+\mu}$，收敛时间 $T \leq V^{-\mu}(\mathbf{e}(0)) / (-\mu \rho)$。
- **非超调**：存在齐次锥 $\Omega$ 为严格正不变集（通过齐次障碍函数 $\phi(\mathbf{e})$ 和 Metzler 矩阵条件保证）。
- **ISS**：对有界扰动 $\gamma$，$\dot{V} \leq -0.5\rho V^{1+\mu}$ 当 $\|\gamma/C\|_{d_\gamma} \leq V(\mathbf{e})$，误差收敛到与扰动幅度成正比的邻域。
- **饱和**：$\mathbf{u}'_{\mathrm{hom}} = \mathrm{sat}_{a,b}^{1+\mu}(\|\mathbf{e}\|_d) (K - K_0) d(-\ln \mathrm{sat}_{a,b}(\|\mathbf{e}\|_d)) \mathbf{e}$，其中 $\mathrm{sat}_{a,b}(\delta) = \min\{\max\{a, \delta\}, b\}$。

## 3. 六维车体级混合模型

### 3.1 状态空间定义

针对三全向轮底盘的完整约束特性，本文将状态从 4D 质点模型改写为 6D **混合坐标系**表示：

$$\mathbf{x} = [p_x, p_y, \theta, v_x^b, v_y^b, \omega]^{\mathsf{T}} \in \mathbb{R}^6\tag{6}$$

| 状态分量 | 所在坐标系 | 物理含义 |
|----------|-----------|----------|
| $p_x, p_y$ | map 系 | 全局位置 |
| $\theta$ | map 系 | 偏航角 |
| $v_x^b, v_y^b$ | 车体系 | 前进/横向线速度 |
| $\omega$ | 车体系 | 角速度 |

控制输入为车体系加速度：

$$\mathbf{u} = [a_x, a_y, \alpha]^{\mathsf{T}} \in \mathbb{R}^3\tag{7}$$

**关键设计动机**：后三维 $(v_x^b, v_y^b, \omega)$ 直接对应 ROS `cmd_vel` 语义（`linear.x`, `linear.y`, `angular.z`），不再需要输出后的坐标旋转步骤。前三维 $(p_x, p_y, \theta)$ 由 EKF + TF 在 map 系下提供。

### 3.2 位姿运动学与速度二阶模型

全向底盘在平面内的刚体运动遵循：

$$\begin{bmatrix} \dot{p}_x \\ \dot{p}_y \\ \dot{\theta} \end{bmatrix} =
\begin{bmatrix} \cos\theta & -\sin\theta & 0 \\ \sin\theta & \cos\theta & 0 \\ 0 & 0 & 1 \end{bmatrix}
\begin{bmatrix} v_x^b \\ v_y^b \\ \omega \end{bmatrix}\tag{8}$$

完整非线性状态方程为：

$$\dot{\mathbf{x}} = \mathbf{f}(\mathbf{x}, \mathbf{u}) =
\begin{bmatrix}
v_x^b \cos\theta - v_y^b \sin\theta \\
v_x^b \sin\theta + v_y^b \cos\theta \\
\omega \\
a_x / m \\
a_y / m \\
\alpha / I
\end{bmatrix}\tag{9}$$

其中 $m$ 和 $I$ 分别为平移和旋转通道的调谐参数（不要求与真实物理质量/惯量一致）。式 (8) 是车体级平面刚体运动学；式 (9) 的后三行是加速度输入驱动的速度积分。因此本文所谓 6D 模型应理解为“车体级混合坐标二阶模型”，而不是把三全向轮逆运动学矩阵并入状态方程的轮级运动学模型。轮速映射只作为第 5.5 节中的输出约束使用。

## 4. 方位角约束编队策略与误差动力学

### 4.1 双重坐标变换

编队控制的核心操作是将 Leader 和 Follower 状态统一到同一坐标系下求差。本文选择 **Leader 车体系**作为误差定义空间。

设 Leader 状态 $\mathbf{x}_l$，Follower 状态 $\mathbf{x}_f$。定义旋转矩阵 $R(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}$。

**位置误差**从 map 系旋转到 Leader 车体系：

$$\begin{bmatrix} \Delta e_x^L \\ \Delta e_y^L \end{bmatrix} =
R(-\theta_l) \begin{bmatrix} p_{x,f} - p_{x,l} \\ p_{y,f} - p_{y,l} \end{bmatrix}\tag{10}$$

**Follower 速度**也需旋转到 Leader 车体系（消除两车朝向差的影响）：

$$\begin{bmatrix} v_{x,f}^L \\ v_{y,f}^L \end{bmatrix} =
R(-\Delta\theta) \begin{bmatrix} v_{x,f}^b \\ v_{y,f}^b \end{bmatrix},\quad \Delta\theta = \theta_f - \theta_l\tag{11}$$

### 4.2 方位角约束编队策略

#### 4.2.1 动机

离散编队点方法在安全圆上均匀分布 $m_p$ 个候选目标，Follower 选择最近者跟踪 [参考文献1]。该方法的核心缺陷是**编队偏移在切换瞬间发生阶跃**——当 Follower 越过两编队点的中垂线时，目标从 $\mathbf{d}_i$ 跳变至 $\mathbf{d}_{i+1}$，导致：

1. 误差 $\mathbf{e}$ 不连续，控制指令突变
2. HPC 参数（$G_0$, $P$, $G_d$, $\mu$）需全部重算
3. Follower 轨迹在安全圆附近出现折线拐角

连续边界投影虽然避免了切换，但编队偏移退化到径向方向——Follower 只是"被推到圆上任意位置"，失去了对编队几何形状（如"右后方 45°"）的控制能力。

#### 4.2.2 方位角约束方法

本文提出将编队目标**固定于 Leader 车体系下安全圆上的一个唯一位置**，由方位角 $\phi_d$ 参数化。

设 $\phi_d \in (-\pi, \pi]$ 为期望编队方位角（定义在 Leader 车体系下，$\phi_d = 0$ 表示 Leader 正后方，$\phi_d = \pi/2$ 表示 Leader 左侧）：

$$\mathbf{d} = r_s \begin{bmatrix} \cos\phi_d \\ \sin\phi_d \end{bmatrix}_{\text{Leader 车体系}}\tag{12}$$

完整的 6D 编队偏移向量为 $\mathbf{d} = [\mathbf{d}^{\mathsf{T}}, 0, 0, 0, 0]^{\mathsf{T}}$。

**关键洞察**：Cartesian 位置误差本身已同时编码了距离和方位角信息：

$$\mathbf{e}_{\mathrm{pos}} = \Delta\mathbf{e}^L - \mathbf{d} = \begin{bmatrix} \Delta e_x^L - r_s \cos\phi_d \\ \Delta e_y^L - r_s \sin\phi_d \end{bmatrix}\tag{13}$$

设 Follower 当前方位角为 $\phi = \mathrm{atan2}(\Delta e_y^L, \Delta e_x^L)$，当前距离为 $\rho = \|\Delta\mathbf{e}^L\|$。在目标点附近做局部线性化：

$$\mathbf{e}_{\mathrm{pos}} \approx \begin{bmatrix} \rho - r_s \\ r_s(\phi - \phi_d) \end{bmatrix}_{\text{径向-切向坐标系}}\tag{14}$$

即：

- **径向分量** $\rho - r_s$：将 Follower 推向/拉离安全圆，控制编队距离
- **切向分量** $r_s(\phi - \phi_d)$：驱动 Follower 沿圆弧滑向目标方位角，控制编队方位

两者通过同一个 Cartesian 误差向量 (13) 自然表达——Cartesian 误差在径向-切向坐标系下的投影恰好给出距离误差和方位角误差。**不需要**显式的势函数叠加或额外控制回路。

#### 4.2.3 与现有策略的对比

| 特性 | 离散编队点 [参考文献1] | 连续边界投影 | 方位角约束 [本文] |
|------|----------------------|-------------|-------------------|
| 编队偏移 | $\mathbf{d}_i$（$m_p$ 个离散点） | 径向投影（随 Follower 位置变化） | $\mathbf{d}(\phi_d)$（固定方位角） |
| 唯一性 | 非唯一，取决于最近点 | 唯一但方位角不固定 | 唯一且方位角固定 |
| 切换逻辑 | tol 滞后切换 | 无 | 无 |
| HPC 重算 | 切换时额外触发 | 仅 Leader 机动 | 仅 Leader 机动 |
| Follower 轨迹 | 折线拐角 | 任意方向逼近 | 平滑弧线收敛 |
| 参数数量 | $m_p$, tol, $r_s$ | $r_s$ | $\phi_d$, $r_s$ |

### 4.3 误差向量定义

完整的 6D 误差（Leader 车体系下）：

$$\mathbf{e} = \begin{bmatrix}
(p_{x,f} - p_{x,l})\cos\theta_l + (p_{y,f} - p_{y,l})\sin\theta_l - r_s \cos\phi_d \\
-(p_{x,f} - p_{x,l})\sin\theta_l + (p_{y,f} - p_{y,l})\cos\theta_l - r_s \sin\phi_d \\
\theta_f - \theta_l \\
v_{x,f}^L - v_{x,l}^b \\
v_{y,f}^L - v_{y,l}^b \\
\omega_f - \omega_l
\end{bmatrix}\tag{15}$$

### 4.4 线性时变误差动力学

在 Leader 当前状态附近线性化（假设 $\Delta\theta$ 较小，$\cos\Delta\theta \approx 1$，$\sin\Delta\theta \approx \Delta\theta$），并在一次 HPC 参数计算内冻结 Leader twist 与编队偏移，得到 nominal **时变冻结**误差动力学：

$$\dot{\mathbf{e}} = A_l(\omega_l, v_{x,l}^b, v_{y,l}^b) \, \mathbf{e} + B \, \mathbf{u}\tag{16}$$

$$A_l = \begin{bmatrix}
0 & \omega_l & -v_{y,l}^b & 1 & 0 & 0 \\
-\omega_l & 0 & v_{x,l}^b & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}, \quad
B = \begin{bmatrix}
0 & 0 & 0 \\
0 & 0 & 0 \\
0 & 0 & 0 \\
1/m & 0 & 0 \\
0 & 1/m & 0 \\
0 & 0 & 1/I
\end{bmatrix}\tag{17}$$

**$A_l$ 时变性的物理解释**：

| 元素 | 来源 | 物理机制 |
|------|------|----------|
| $A_l(0,1) = \omega_l$ | Leader 车体系旋转 | Leader 旋转时其车体系也在旋转，导致 X/Y 误差交叉耦合 |
| $A_l(1,0) = -\omega_l$ | 同上 | 同上 |
| $A_l(0,2) = -v_{y,l}^b$ | 偏航-平移耦合 | Leader 横向速度通过 $\Delta\theta$ 耦合到 X 位置误差，来自 $R(-\theta_l)$ 求导的链式法则 |
| $A_l(1,2) = v_{x,l}^b$ | 同上 | Leader 前进速度通过 $\Delta\theta$ 耦合到 Y 位置误差 |

这里的 (16) 是用于齐次升级的 nominal 局部模型。完整非线性误差方程还包含 Leader twist 变化、非零编队偏移在旋转坐标系中的附加项、离散目标点切换、采样积分和速度/轮速饱和等项，实际分析应写作 $\dot{\mathbf{e}} = A_l\mathbf{e}+B\mathbf{u}+\mathbf{w}$，并把 $\mathbf{w}$ 作为有界扰动处理。

**与 4D 质点模型的关键区别**：4D 模型误差定义在静止的 map 系，$A = \begin{bmatrix} 0 & I \\ 0 & 0 \end{bmatrix}$ 恒定。6D 模型误差定义在运动的 Leader 车体系，坐标系自身的运动以速度参数形式进入了 $A_l$。此即混合坐标系建模的核心代价——用 $A_l$ 的时变性换取了输出速度语义与 `cmd_vel` 的自然对齐。

**可控性**：$(A_l, B)$ 在任意冻结时刻均为可控（Kalman 秩条件满足），底部 $3 \times 3$ 块 $B_0 = \mathrm{diag}(1/m, 1/m, 1/I)$ 满秩，保证了 HPC 构造的可行性。

## 5. 控制器设计

### 5.1 自适应线性比例控制器（LPC）

对线性化系统 (16)，设计分块解耦的线性状态反馈 $\mathbf{u}_{\mathrm{lin}} = K \mathbf{e}$。三个通道（$X, Y, \theta$）独立配置为临界阻尼双极点：

$$K = \begin{bmatrix}
k_{1,x} & 0 & 0 & k_{2,x} & 0 & 0 \\
0 & k_{1,y} & 0 & 0 & k_{2,y} & 0 \\
0 & 0 & k_{1,\theta} & 0 & 0 & k_{2,\theta}
\end{bmatrix}\tag{18}$$

以 X 通道为例：

$$k_{2,x} = -2a, \quad k_{1,x} = \frac{a(k_{2,x} + a)}{m}\tag{19}$$

$$a = \max(\bar{a}, \omega_d m), \quad \bar{a} = \mathrm{clamp}\!\left(-m \cdot \frac{e_{v_x}}{e_{p_x}}, -\omega_d m, \omega_d m\right)\tag{20}$$

$\bar{a}$ 为防超调自适应项：当位置误差很大而速度误差较小时，$-m \cdot e_v / e_p$ 较小，$a$ 取下界 $\omega_d m$，避免过弱控制导致超调。$\theta$ 通道以 $I$ 替代 $m$ 同理。

### 5.2 HPC 升级与时变重算

将 LPC 增益 $K$ 通过第 2 节所述流程升级为 HPC 参数 $(K_0, G_0, P, \mu)$。该升级严格适用于每个冻结时刻的 LTI nominal 模型 $(A_l,B)$；当 Leader twist 缓慢变化且扰动有界时，实际闭环可理解为该 nominal 有限时间稳定系统的扰动版本。方位角约束策略的编队偏移 $\mathbf{d}$ 固定不变，因此 HPC 重算仅由 Leader 机动触发：

$$\|[\omega_l, v_{x,l}^b, v_{y,l}^b] - [\omega_l^{\mathrm{prev}}, v_{x,l}^{b,\mathrm{prev}}, v_{y,l}^{b,\mathrm{prev}}]\| > \varepsilon_v\tag{21}$$

或

$$|\Delta\theta - \Delta\theta^{\mathrm{prev}}| > \varepsilon_\theta\tag{22}$$

与离散编队点策略相比，省去了编队点切换带来的额外重算触发——HPC 参数仅在 $A_l$ 实际变化时才更新。$\mu$ 取 $\mu_{\min}$ 以实现最快有限时间收敛。

### 5.3 控制力坐标变换

HPC 输出力 $(F_x^L, F_y^L, \tau)$ 定义在 Leader 车体系，需旋转到 Follower 车体系：

$$\begin{bmatrix} F_x^f \\ F_y^f \end{bmatrix} = R(\Delta\theta) \begin{bmatrix} F_x^L \\ F_y^L \end{bmatrix}\tag{23}$$

Follower 车体系速度通过前向欧拉积分更新：

$$v_{x,f}^b[k+1] = v_{x,f}^b[k] + h \cdot F_x^f / m, \quad v_{y,f}^b[k+1] = v_{y,f}^b[k] + h \cdot F_y^f / m, \quad \omega_f[k+1] = \omega_f[k] + h \cdot \tau / I\tag{24}$$

其中 $h = 0.1\mathrm{s}$ 为控制步长。输出速度直接写入 `cmd_vel`（车体系），无需额外旋转。

### 5.4 方位角约束的收敛行为分析

Follower 从任意初始位置收敛至目标点 $\mathbf{d}$ 的轨迹可分为两个阶段：

**阶段 1——径向主导**：当 Follower 远离安全圆（$|\rho - r_s| \gg r_s|\phi - \phi_d|$）时，位置误差的径向分量占主导。控制器以高增益将 Follower 向安全圆推进，轨迹接近径向直线。

**阶段 2——切向滑行**：当 Follower 接近安全圆（$\rho \approx r_s$）时，径向误差趋于零，切向分量 $r_s(\phi - \phi_d)$ 驱动 Follower 沿圆弧滑向目标方位角 $\phi_d$。由于全向底盘可在任意方向平移，此切向运动无需偏航旋转——Follower 在保持当前朝向的同时沿圆弧侧滑。

最终收敛轨迹呈现平滑弧线，到安全圆前直线逼近，到安全圆后沿圆滑行至 $\phi_d$。

### 5.5 全向轮运动学约束

三轮全向底盘的速度-轮速映射为：

$$\begin{bmatrix} \omega_1 \\ \omega_2 \\ \omega_3 \end{bmatrix} =
\frac{1}{r} \begin{bmatrix}
0 & 1 & L \\
-\frac{\sqrt{3}}{2} & -\frac{1}{2} & L \\
\frac{\sqrt{3}}{2} & -\frac{1}{2} & L
\end{bmatrix}
\begin{bmatrix} v_x^b \\ v_y^b \\ \omega \end{bmatrix}\tag{25}$$

其中 $r = 0.03$ m（轮半径），$L = 0.11$ m（底盘半径）。若任一 $|\omega_i| > \omega_{\max}$，对速度向量等比缩放。同时施加加速度 slew rate 限幅。

## 6. 完整控制管线

```
Leader EKF + TF ──→ x_l = [p_x,p_y,θ,v_x^b,v_y^b,ω]_l
Follower EKF+TF ──→ x_f = [p_x,p_y,θ,v_x^b,v_y^b,ω]_f
         │
         ▼
┌──────────────────────────────────────────┐
│ 1. 位置误差旋转到 Leader 车体系          │
│ 2. Follower 速度旋转到 Leader 车体系      │
│ 3. 固定编队偏移 d = r_s[cos φ_d, sin φ_d]│
│ 4. 计算误差 e = x_f - x_l - d            │
│ 5. 自适应 LPC 增益: K = f(e, m, I, ω_d) │
│ 6. A_l 更新 → HPC 时变重算 (仅Leader机动)│
│ 7. 齐次范数: q = ‖e‖_{G_d,P} (二分法)   │
│ 8. 齐次控制律: u^L = K_0 e + q^{1+μ}·(K-K_0)·expm(-G_d ln q)·e │
│ 9. 力旋转: u^f = R(Δθ)·u^L              │
│ 10. 前向欧拉: v_cmd = v + h·u^f/M       │
│ 11. 全向轮逆运动学 + 轮速钳位           │
│ 12. 发布 cmd_vel (车体系)               │
└──────────────────────────────────────────┘
```

## 7. 与 4D 质点模型及 6D 离散编队点的对比

| 特性 | 质点模型 4D [参考文献1] | 6D 离散编队点 | 6D 方位角约束 [本文] |
|------|------------------------|--------------|---------------------|
| 状态空间 | $[p_x, p_y, v_x, v_y]$ 全 map 系 | $[p_x, p_y, \theta, v_x^b, v_y^b, \omega]$ 混合系 | 同左 |
| 朝向建模 | 无 | $\theta$ 参与运动学 | 同左 |
| 速度输出 | map 系，需旋转 → `cmd_vel` | 车体系，直接对应 | 同左 |
| 偏航控制 | 独立 P+前馈 | 集成于 6D 主回路 | 同左 |
| 编队策略 | 离散点 + tol（map 系） | 离散点 + tol（Leader 车体系） | 固定方位角 $\phi_d$（Leader 车体系） |
| 切换逻辑 | 有 | 有 | **无** |
| 编队偏移连续性 | 不连续（切换时阶跃） | 不连续（切换时阶跃） | **连续恒定** |
| HPC 重算触发 | 初始化 + 编队点切换 | Leader 机动 + 切换 + $\Delta\theta$ | **仅 Leader 机动 + $\Delta\theta$** |
| 系统矩阵 | $A$ 恒定 | $A_l$ 时变 | 同左 |
| 收敛轨迹 | 折线拐角 | 折线拐角 | **平滑弧线** |
| 参数数量 | $r_s$, $m_p$, tol | $r_s$, $m_p$, tol | **$r_s$, $\phi_d$** |

## 8. 仿真结果

> 本章为预留结构，待仿真数据采集后填写。每组实验建议在同一条件下对比三种策略：LPC（线性控制）、HPC 离散编队点、HPC 方位角约束。

### 8.1 实验设置

| 项目 | 设定 |
|------|------|
| 仿真环境 | Gazebo Classic 11 + ROS 2 Humble |
| 机器人模型 | mini_omni 三全向轮底盘 |
| Leader 轨迹 | 圆形 ($r=2.0$ m, $v=0.5$ m/s) |
| 控制频率 | 20 Hz |
| HPC 参数 | $m=1.5$, $I=0.3$, $\omega_d=0.8$, $\omega_d^\theta=0.8$ |
| 编队参数 | $r_s=2.0$ m, $\phi_d=\pi$ (正后方) |
| 初始位置 | Leader $(0, 0)$, Follower $(2, 0)$ |
| 仿真时长 | 60 s |

### 8.2 对比实验设计

| 编号 | 控制器 | 编队策略 | 目的 |
|------|--------|----------|------|
| E1 | 4D LPC | — | 基准：原始质点模型在线性控制下的性能 |
| E2 | 4D HPC | 离散编队点 | 论文 [1] 复现：验证有限时间收敛 |
| E3 | 6D LPC | 方位角约束 $\phi_d=\pi$ | 消融：关闭齐次升级，单独评估 6D 模型的改进 |
| E4 | 6D HPC | 离散编队点 | 对比：切换逻辑 vs 无切换 |
| E5 | 6D HPC | 方位角约束 $\phi_d=\pi$ | **本文方法** |

### 8.3 评价指标

| 指标 | 符号 | 含义 |
|------|------|------|
| 位置误差 RMS | $\bar{e}_{\mathrm{pos}}$ | $\frac{1}{T}\int_0^T \|\mathbf{e}_{\mathrm{pos}}(t)\| dt$ |
| 稳态误差 | $e_{\mathrm{pos}}^{\mathrm{ss}}$ | $t > 20$s 后的平均 $\|\mathbf{e}_{\mathrm{pos}}\|$ |
| 收敛时间 | $t_c$ | $\|\mathbf{e}_{\mathrm{pos}}\|$ 首次进入并保持 $< 0.1$ m 的时间 |
| 控制能量 | $E_u$ | $\frac{1}{T}\int_0^T \|\mathbf{u}(t)\|^2 dt$ |
| 编队点切换次数 | $N_{\mathrm{sw}}$ | 仅离散编队点策略，记录 tol 切换频次 |
| 轨迹平滑度 | $\kappa$ | 轨迹曲率的平均绝对值 |

### 8.4 预期结果

**E5 vs E4（方位角约束 vs 离散编队点）**：
- $N_{\mathrm{sw}} = 0$（E5 无切换）
- 轨迹平滑度 $\kappa$ 更低
- HPC 重算次数更少（无编队点切换触发）

**E5 vs E3（HPC vs LPC，同在 6D + 方位角约束下）**：
- 收敛时间 $t_c$ 更短
- 稳态误差 $e_{\mathrm{pos}}^{\mathrm{ss}}$ 更小
- 对 Leader 机动的跟踪滞后更小

**E5 vs E2（6D 方位角 vs 4D 离散，本文 vs 论文 [1]）**：
- 偏航通道无需额外 P+前馈
- Follower 朝向自然跟随 Leader（集成于 6D 主回路）
- 轨迹不出现折线拐角（无切换 + 固定方位角）

### 8.5 图表计划

| 图号 | 内容 | 对比维度 |
|------|------|----------|
| Fig.1 | XY 平面轨迹 | E2 vs E4 vs E5（三种编队策略的轨迹形状对比） |
| Fig.2 | 位置误差 $\|\mathbf{e}_{\mathrm{pos}}\|$ 时间历程 | E3 vs E5（LPC vs HPC 在 6D 模型下的收敛速度） |
| Fig.3 | 跟踪误差分量 ($e_x, e_y$) | E5 各通道误差的解耦表现 |
| Fig.4 | 控制输入 $\mathbf{u}(t)$ | E4 vs E5（切换是否造成控制量跳变） |
| Fig.5 | 方位角误差 $\phi - \phi_d$ 时间历程 | E5 方位角约束的收敛过程（验证两阶段行为） |
| Fig.6 | HPC 重算次数时间线 | E4 vs E5（验证重算触发频率差异） |
| Fig.7 | 轮速 + 约束触发 | E5 运动学约束的激活频率 |
| Tab.1 | 定量指标汇总 | 全部实验的 $t_c$, $e_{\mathrm{pos}}^{\mathrm{ss}}$, $\bar{e}_{\mathrm{pos}}$, $E_u$, $N_{\mathrm{sw}}$ |

## 9. 参数汇总（仿真默认值）

| 参数 | 符号 | 默认值 | 含义 |
|------|------|--------|------|
| 安全半径 | $r_s$ | 2.0 m | 编队保持圆半径 |
| 期望方位角 | $\phi_d$ | $\pi$ (正后方) | Leader 车体系下目标方位角 |
| 质量调谐参数 | $m$ | 1.5 | 平移通道响应惯性 |
| 转动惯量调谐参数 | $I$ | 0.3 | 偏航通道响应惯性 |
| 期望带宽 | $\omega_d$ | 0.8 | 平移通道临界阻尼带宽 |
| 偏航带宽 | $\omega_d^\theta$ | 0.8 | 偏航通道临界阻尼带宽 |
| HPC 重算阈值 | $\varepsilon_v$ | 0.3 | Leader 速度变化触发门限 |
| 偏航重算阈值 | $\varepsilon_\theta$ | 0.3 rad | $\Delta\theta$ 变化触发门限 |
| 控制步长 | $h$ | 0.1 s | 前向欧拉积分步长 |
| 轮半径 | $r$ | 0.03 m | 逆运动学参数 |
| 底盘半径 | $L$ | 0.11 m | 逆运动学参数 |
| 最大轮速 | $\omega_{\max}$ | 20 rad/s | 电机硬件约束 |

## 10. 工程实现说明

> C++ 实现位于 `homo_multirobot_formation_control` 包。方位角约束版本对应 `homo_controller_6d_bearing.hpp`。本节说明实现与前述理论的对应关系及差异。

### 9.1 已实现组件

| 组件 | 实现位置 |
|------|----------|
| 块可控分解 + $G_0$ 构造 | `lpc2hpc_nd.hpp: trans_con_nd / block_con_nd` |
| Lyapunov 方程求解 $P$ | `lpc2hpc_nd.hpp: solve_lyapunov_nd` |
| $\mu$ 可容许范围计算 | `lpc2hpc_nd.hpp: lpc2hpc_nd` |
| 齐次范数二分法 | `hnorm_nd.hpp` |
| 6D LPC 自适应增益 | `homo_controller_6d_bearing.hpp: calculate_klin` |
| 固定方位角编队偏移 | `homo_controller_6d_bearing.hpp: compute_error` |
| 时变 HPC 重算 | `homo_controller_6d_bearing.hpp: lpc_calculate` |
| 全向轮运动学约束 | `kinematic_constraint.hpp` |

### 9.2 简化项及原因

| 项目 | 理论 | 实现 | 原因 |
|------|------|------|------|
| $K_0$ 项 | 参与控制律 (5) | 计算但不使用 | 积分链模型 $K_0 \equiv 0$，无需代码嵌入 |
| 齐次障碍函数 $\phi(x)$ | 第 2.3 节 | 未实现 | 非超调由自适应 LPC 增益 $a = \max(\bar{a}, \omega_d m)$ 工程保证 |
| ISS 扰动估计 | 第 2.3 节 | 未实现 | $\gamma = Q_1 - u_1 - Q_2$ 需外部建模，由 HPC 固有鲁棒性补偿 |
| Metzler 参数条件 | 第 2.3 节 | 未实现 | 对 6D 时变 $A_l$ 不直接适用 |
| 完整饱和 $\mathrm{sat}_{a,b}^{1+\mu}$ | [参考文献1] 公式 (31) | 简化为 $q$ 的 clamp(0.5, 1.0) | 工程实践中足够约束控制量 |

### 9.3 控制律公式差异

实现使用简化形式 $\mathbf{u} = q^{1+\nu} K \exp(G_d(1 - \ln q)) \mathbf{e}$。与理论公式 (5) 的差异在于省略 $K_0$ 项（对积分链模型 $K_0 = 0$）和指数规范化约定。两者在 $K_0 = 0$ 和适当参数选取下闭环性质等价。

## 参考文献

1. W. Yuan, C. Dong, X. Duan, A. Polyakov, K. Zimenko, X. Ping, "Leader-Follower Tracking with Collision Avoidance for Omni-directional Mobile Robots: Linear vs Homogeneous Controller."
2. A. Polyakov, *Generalized Homogeneity in Systems and Control*, Springer, 2020.
3. A. Polyakov and M. Krstic, "Finite-and Fixed-Time Nonovershooting Stabilizers and Safety Filters by Homogeneous Feedback," *IEEE Trans. Autom. Control*, 68(11): 6434–6449, 2023.
4. A. Polyakov, "Sliding Mode Control Design Using Canonical Homogeneous Norm," *Int. J. Robust Nonlinear Control*, 29(25): 682–701, 2019.
