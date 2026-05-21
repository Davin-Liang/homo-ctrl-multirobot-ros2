# 6D 齐次编队控制中的 QP 避障融合：数学原理

## 1 问题表述

### 1.1 编队控制背景

考虑 Leader-Follower 编队场景。领航者（Leader）状态由 6D 运动学模型描述：

$$\mathbf{x}_l = [p_{x,l},\; p_{y,l},\; \theta_l,\; v_{x,l}^b,\; v_{y,l}^b,\; \omega_l]^{\mathsf{T}} \in \mathbb{R}^6$$

其中 $p_{x,l}, p_{y,l}, \theta_l$ 为 map 系下的位置与偏航角，$v_{x,l}^b, v_{y,l}^b, \omega_l$ 为车体系下的线速度与角速度。跟随者（Follower）状态 $\mathbf{x}_f$ 具有相同结构。

齐次控制器（HPC）在每一控制周期输出一个车体系下的期望速度 $\mathbf{v}_{\text{hpc}} = [v_x^{\text{hpc}}, v_y^{\text{hpc}}, \omega^{\text{hpc}}]^{\mathsf{T}} \in \mathbb{R}^3$，使 follower 收敛到以 leader 为中心的指定编队构型。

### 1.2 避障问题

当 follower 与 leader 之间存在障碍物时，直接执行 $\mathbf{v}_{\text{hpc}}$ 可能导致碰撞。本文的目标是：在每一控制周期求解一个最优速度指令 $\mathbf{v}^* = [v_x, v_y, \omega]^{\mathsf{T}}$，使得：

1. **编队跟踪**：$\mathbf{v}^*$ 尽可能接近 $\mathbf{v}_{\text{hpc}}$；
2. **避障**：$\mathbf{v}^*$ 遵守基于激光雷达数据的障碍物安全约束；
3. **运动学可行**：满足速度上限、加速度上限和全向轮约束。

该问题被建模为一个带软约束的凸二次优化（QP）问题。

### 1.3 坐标系约定

所有障碍物检测与速度优化均在 **follower 车体系**（body frame）下进行：
- 原点位于 follower 底盘中心
- $+x$ 轴指向机器人前进方向
- $+y$ 轴指向机器人左侧

此约定与 `cmd_vel` 消息的坐标系一致，优化输出可直接发布为控制指令。

---

## 2 障碍物检测与表示

### 2.1 激光数据处理

单线激光雷达每帧产生 $N = 720$ 个采样点 $(\rho_k, \alpha_k)$，其中 $\rho_k$ 为距离，$\alpha_k$ 为扫描角度（从 $+\!x$ 轴逆时针为正）。处理流程：

**步骤 1 — 无效点滤除**

剔除 $\rho_k \notin [\rho_{\min}, \rho_{\max}]$ 或 $\rho_k$ 为无穷/NaN 的采样点。

**步骤 2 — 笛卡尔转换**

将有效极坐标点转换到车体系：

$$\mathbf{p}_k = \begin{bmatrix} \rho_k \cos\alpha_k \\ \rho_k \sin\alpha_k \end{bmatrix}$$

**步骤 3 — 欧几里得聚类**

利用扫描点按角度排序的性质，采用连续点距离阈值聚类：

$$\mathcal{C}_j = \{\mathbf{p}_k, \mathbf{p}_{k+1}, \ldots, \mathbf{p}_{k+m}\} \quad \text{s.t.} \quad \|\mathbf{p}_{i+1} - \mathbf{p}_i\| \leq d_{\text{cluster}}$$

其中 $d_{\text{cluster}}$ 为聚类距离阈值（默认 0.1 m）。聚类最小点数设为 5，以滤除传感器噪声。

对于环绕 $0^\circ$/$360^\circ$ 边界的聚类，额外检查首尾聚类的端点距离，判断是否需要合并。

### 2.2 障碍物表示

对每个聚类 $\mathcal{C}_j$，取其中**离机器人最近的点**作为障碍物参考位置：

$$\mathbf{o}_j = \underset{\mathbf{p} \in \mathcal{C}_j}{\arg\min}\; \|\mathbf{p}\|$$

障碍物半径取聚类内点到 $\mathbf{o}_j$ 的最大距离，并加以裁剪：

$$r_j = \min\left(0.5,\; \max\left(0.05,\; \max_{\mathbf{p} \in \mathcal{C}_j} \|\mathbf{p} - \mathbf{o}_j\|\right)\right) \quad \text{[m]}$$

此表示法的优势：排斥力方向 $\mathbf{n}_j$ 始终指向障碍物表面最近点，即垂直于局部曲面。

### 2.3 多帧跟踪与速度估计

采用最近邻关联跨帧匹配障碍物。对匹配成功的障碍物，位置和速度均做指数滑动平均（低通滤波）：

$$\mathbf{o}_j^{(t)} = (1 - \alpha_p)\,\mathbf{o}_j^{(t-1)} + \alpha_p\,\mathbf{o}_j^{\text{raw}}$$

$$\dot{\mathbf{o}}_j^{(t)} = (1 - \alpha_v)\,\dot{\mathbf{o}}_j^{(t-1)} + \alpha_v\,\frac{\mathbf{o}_j^{(t)} - \mathbf{o}_j^{(t-1)}}{\Delta t}$$

其中 $\alpha_p = 0.4$、$\alpha_v = 0.3$ 为平滑系数，$\Delta t$ 为扫描帧间隔。未匹配超过 3 帧的障碍物被丢弃。

每个障碍物被分配一个持久化的整数 ID，用于跨帧标识。

---

## 3 QP 优化问题

### 3.1 决策变量与目标函数

决策变量为 follower 车体系下的速度指令 $\mathbf{v} \in \mathbb{R}^3$。目标函数包含编队跟踪项与避障项：

$$\min_{\mathbf{v}} \quad J(\mathbf{v}) = \underbrace{\|\mathbf{v} - \mathbf{v}_{\text{hpc}}\|^2}_{\text{编队跟踪}} \;+\; \sum_{j=1}^{M} w_j \cdot \underbrace{\phi^2\!\left(\mathbf{v}_{xy} \cdot \mathbf{n}_j - v_{\text{safe},j}\right)}_{\text{避障代价}}$$

其中：
- $\mathbf{v}_{xy} = [v_x, v_y]^{\mathsf{T}}$ 为速度的平动分量
- $\mathbf{n}_j$ 为从机器人指向障碍物 $j$ 的单位向量（车体系）：$\mathbf{n}_j = \mathbf{o}_j / \|\mathbf{o}_j\|$
- $v_{\text{safe},j}$ 为障碍物 $j$ 的安全接近速度（见 §4）
- $M$ 为当前障碍物数量（最多 $M_{\max} = 10$）
- $w_j$ 为障碍物 $j$ 的有效权重（见 §5）

### 3.2 光滑惩罚函数

为避免 $\max(0, x)$ 在 $x = 0$ 处的不可导性，采用光滑近似：

$$\phi(x) = \frac{1}{2}\left(x + \sqrt{x^2 + \varepsilon^2}\right), \quad \varepsilon = 10^{-4}$$

$\phi(x)$ 满足：当 $x \gg 0$ 时 $\phi(x) \approx x$，当 $x \ll 0$ 时 $\phi(x) \approx 0$，且处处二阶连续可微。其导数为：

$$\phi'(x) = \frac{1}{2}\left(1 + \frac{x}{\sqrt{x^2 + \varepsilon^2}}\right) \in [0, 1]$$

### 3.3 约束条件

QP 问题受限于两类硬约束：

**速度边界约束**：

$$-v_{\max} \leq v_x \leq v_{\max}, \quad -v_{\max} \leq v_y \leq v_{\max}, \quad -\omega_{\max} \leq \omega \leq \omega_{\max}$$

其中 $v_{\max} = 1.0$ m/s，$\omega_{\max} = 0.5$ rad/s。

**加速度约束**（斜坡约束，确保速度变化连续）：

$$v_{x}^{\text{prev}} - a_{\max} \Delta t \;\leq\; v_x \;\leq\; v_{x}^{\text{prev}} + a_{\max} \Delta t$$

$$v_{y}^{\text{prev}} - a_{\max} \Delta t \;\leq\; v_y \;\leq\; v_{y}^{\text{prev}} + a_{\max} \Delta t$$

$$\omega^{\text{prev}} - \alpha_{\max} \Delta t \;\leq\; \omega \;\leq\; \omega^{\text{prev}} + \alpha_{\max} \Delta t$$

其中 $a_{\max} = 2.0$ m/s²，$\alpha_{\max} = 4.0$ rad/s²，$\Delta t = 0.05$ s（20 Hz 控制周期）。

合记为盒状约束：$\mathbf{v} \in \mathcal{B} = [\mathbf{l}, \mathbf{u}]$。

### 3.4 优化问题汇总

$$\begin{aligned}
\min_{\mathbf{v}} \quad & \|\mathbf{v} - \mathbf{v}_{\text{hpc}}\|^2 + \sum_{j=1}^{M} w_j \cdot \phi^2\!\left(\mathbf{v}_{xy} \cdot \mathbf{n}_j - v_{\text{safe},j}\right) \\
\text{s.t.} \quad & \mathbf{v} \in \mathcal{B}
\end{aligned}$$

这是一个在盒约束上的**无约束结构凸优化问题**：目标函数 $J(\mathbf{v})$ 为凸函数，约束集为盒。我们采用投影梯度下降法直接求解，无需引入通用 QP 求解器依赖。

---

## 4 安全速度计算

对每个障碍物 $j$，定义其相对于机器人的关键几何量：

- 障碍物中心距离：$d_j = \|\mathbf{o}_j\|$
- 障碍物表面距离：$d_j^{\text{surf}} = d_j - r_j$
- 安全间隙：$\delta_j = d_j^{\text{surf}} - d_{\text{safe}}$

其中 $d_{\text{safe}}$ 为安全距离参数。根据 $\delta_j$ 的正负，安全速度采用不同策略。

### 4.1 安全距离外（$\delta_j \geq 0$）

机器人尚未进入安全区域，只需限制靠近速度。几何安全速度为：

$$v_j^{\text{geo}} = \frac{\delta_j}{T}$$

其中 $T$ 为预测时域。若机器人以速度 $v_j^{\text{geo}}$ 靠近障碍物，$T$ 秒后恰好抵达安全边界。综合障碍物自身运动（若为动态障碍物）：

$$v_{\text{safe},j} = \max\!\left(0,\; v_j^{\text{geo}} + \dot{\mathbf{o}}_j \cdot \mathbf{n}_j\right)$$

其中 $\dot{\mathbf{o}}_j \cdot \mathbf{n}_j$ 为障碍物在机器人连线方向的速度分量。当障碍物远离时（$\dot{\mathbf{o}}_j \cdot \mathbf{n}_j > 0$），允许更快的靠近速度。

障碍物速度仅在幅值大于 0.2 m/s 时被信任（用于区分真正的动态障碍物与最近点滑动引起的虚假速度）。

### 4.2 安全距离内（$\delta_j < 0$）

机器人已侵入安全区域，需要**主动后退**。后退速度与侵入深度成正比：

$$v_j^{\text{retreat}} = \frac{\delta_j}{T} \quad (< 0)$$

后退速度受机器人物理能力约束，并要求至少以 0.15 m/s 远离：

$$v_j^{\text{retreat}} \leftarrow \max\!\left(v_j^{\text{retreat}},\; -v_{\max}\right)$$
$$v_j^{\text{retreat}} \leftarrow \min\!\left(v_j^{\text{retreat}},\; -0.15\right)$$

最终安全速度为：

$$v_{\text{safe},j} = v_j^{\text{retreat}} + \dot{\mathbf{o}}_j \cdot \mathbf{n}_j$$

此时 $v_{\text{safe},j} < 0$，惩罚项 $\phi(\mathbf{v}_{xy} \cdot \mathbf{n}_j - v_{\text{safe},j})$ 仅在 $\mathbf{v}_{xy} \cdot \mathbf{n}_j > v_{\text{safe},j}$（即未充分后退）时被激活。

---

## 5 障碍物权重设计

为平衡编队跟踪与避障，障碍物权重 $w_j$ 随距离自适应调节。定义基础有效权重：

$$w_j = w_0 \cdot \eta(d_j^{\text{surf}})$$

其中 $w_0$ 为全局避障权重参数（默认 1.0），$\eta(\cdot)$ 为距离相关的严重度函数：

$$\eta(d) = \begin{cases}
1.0, & d \geq d_{\text{safe}} \\[4pt]
\text{clamp}\!\left(\dfrac{d_{\text{safe}}}{\max(d, 0.01)},\; 1.5,\; 8.0\right), & d < d_{\text{safe}}
\end{cases}$$

$\eta(d)$ 的性质：
- 安全距离外：权重 $= w_0$，编队跟踪为主导
- 刚进入安全区（$d \approx d_{\text{safe}}$）：$\eta \approx 1.5$，避障力开始增强
- 接近表面（$d \to 0$）：$\eta \to 8.0$（上限），避障力达到最强但不会爆炸

双曲型增长（$\propto 1/d$）确保了在障碍物表面附近有足够的排斥力，而上限 8.0 避免了数值刚度问题。

---

## 6 投影梯度下降求解

由于 QP 仅有盒约束，采用投影梯度下降法（Projected Gradient Descent）求解。
每步先沿梯度方向更新，再将结果投影到可行盒 $\mathcal{B}$ 内。

### 6.1 梯度计算

目标函数的梯度为三项之和：

$$\nabla J(\mathbf{v}) = \underbrace{2(\mathbf{v} - \mathbf{v}_{\text{hpc}})}_{\text{编队项}} + \underbrace{\sum_{j=1}^{M} \mathbf{g}_j^{\text{obs}}}_{\text{避障项}}$$

其中避障项梯度仅在约束激活时（$\mathbf{v}_{xy} \cdot \mathbf{n}_j > v_{\text{safe},j}$）非零：

$$\mathbf{g}_j^{\text{obs}} = 2w_j \cdot \phi(d_j) \cdot \phi'(d_j) \cdot \begin{bmatrix} n_{j,x} \\ n_{j,y} \\ 0 \end{bmatrix}$$

$d_j = \mathbf{v}_{xy} \cdot \mathbf{n}_j - v_{\text{safe},j}$ 为约束违反量。注意 $\mathbf{g}_j^{\text{obs}}$ 的第三分量（角速度通道）为 0，因为平动速度方向与障碍物方向的夹角不受角速度直接影响（在短预测时域内）。

### 6.2 投影算子

盒约束 $\mathcal{B}$ 上的投影为逐分量截断：

$$\Pi_{\mathcal{B}}(\mathbf{v}) = \begin{bmatrix} \text{clamp}(v_x, l_x, u_x) \\ \text{clamp}(v_y, l_y, u_y) \\ \text{clamp}(v_\omega, l_\omega, u_\omega) \end{bmatrix}$$

### 6.3 步长选择 — Armijo 回溯线搜索

初始化步长 $\alpha = 1.0$，参数 $\beta = 0.5$，$c = 10^{-4}$。每步迭代：

$$\mathbf{v}_{\text{new}} = \Pi_{\mathcal{B}}\!\left(\mathbf{v} - \alpha \nabla J(\mathbf{v})\right)$$

若 Armijo 条件不满足：

$$J(\mathbf{v}_{\text{new}}) > J(\mathbf{v}) + c \cdot \nabla J(\mathbf{v})^{\mathsf{T}} (\mathbf{v}_{\text{new}} - \mathbf{v})$$

则 $\alpha \leftarrow \beta \cdot \alpha$ 并重新计算，最多回溯 12 次。

### 6.4 收敛准则

递代在以下条件之一满足时终止：
- 步长 $\|\mathbf{v}_{\text{new}} - \mathbf{v}\| < 10^{-4}$（收敛）
- 所有障碍物约束均非激活（$\mathbf{v}_{xy} \cdot \mathbf{n}_j \le v_{\text{safe},j}, \forall j$）
- 达到最大迭代次数 $K_{\max} = 20$

### 6.5 初始化

初始点设为 $\mathbf{v}^{(0)} = \Pi_{\mathcal{B}}(\mathbf{v}_{\text{hpc}})$。这一初始化保证了初始点始终在可行域内，且在没有障碍物或障碍物较远时，$\mathbf{v}^{(0)}$ 即为最优解，一次迭代即可收敛。

---

## 7 系统集成

### 7.1 整体数据流

```
TF + EKF          HPC 齐次控制器
  │                      │
  ├─ x_l (leader 6D) ───┤
  ├─ x_f (follower 6D) ─┤
  │                      ↓
  │              v_hpc ∈ R³ (车体系)
  │                      │
  │              ┌───────┘
  │              ↓
  │         QP 避障融合 ◄── /scan 障碍物检测
  │              │
  │              ↓
  │         v* ∈ R³ (车体系)
  │              │
  │              ↓
  │         运动学约束
  │      (KinematicConstraint:
  │       全向轮速/加速度限幅)
  │              │
  │              ↓
  └─────────── cmd_vel 发布
```

### 7.2 与 HPC 的接口

QP 避障模块 `ObstacleAvoider` 以 `v_hpc`（HPC 输出的期望速度）和 `v_current`（当前速度）为输入，输出最优速度 `v_opt`。模块对 HPC 核心算法完全透明——HPC 不感知障碍物的存在，只负责编队跟踪。

### 7.3 后处理

QP 求解之后，进一步施加全向轮运动学约束（`KinematicConstraint`）：
- 将车体系速度通过逆运动学映射为三轮角速度
- 若任一车轮速度超过轮速上限，等比缩放所有速度分量
- 执行加速度斜坡速率限制

---

## 8 参数汇总

### 8.1 QP 避障参数

| 符号 | 参数名 | 默认值 | 单位 | 说明 |
|------|--------|--------|------|------|
| $d_{\text{safe}}$ | `safety_distance` | 0.5 | m | 安全距离阈值 |
| $w_0$ | `obstacle_weight` | 1.0 | — | 全局避障权重 |
| $T$ | `time_horizon` | 0.5 | s | 碰撞预测时域 |
| $M_{\max}$ | `max_obstacles` | 10 | — | 最大考虑障碍物数 |
| $d_{\text{cluster}}$ | `cluster_tolerance` | 0.1 | m | 聚类距离阈值 |
| $N_{\min}$ | `min_cluster_size` | 5 | — | 聚类最少点数 |

### 8.2 运动学约束参数（与 6D 编队控制器共享）

| 符号 | 参数名 | 默认值 | 单位 | 说明 |
|------|--------|--------|------|------|
| $v_{\max}$ | `max_linear_vel` | 1.0 | m/s | 最大线速度 |
| $\omega_{\max}$ | `max_angular_vel` | 0.5 | rad/s | 最大角速度 |
| $a_{\max}$ | `max_linear_accel` | 2.0 | m/s² | 最大线加速度 |
| $\alpha_{\max}$ | `max_angular_accel` | 4.0 | rad/s² | 最大角加速度 |

### 8.3 内部硬编码参数

| 符号 | 值 | 说明 |
|------|-----|------|
| $\varepsilon$ | $10^{-4}$ | 光滑 max 近似参数 |
| $K_{\max}$ | 20 | 最大梯度下降迭代 |
| $\alpha_p$ | 0.4 | 障碍物位置平滑系数 |
| $\alpha_v$ | 0.3 | 障碍物速度平滑系数 |
| $r_{\max}$ | 0.5 m | 障碍物半径上限 |
| $r_{\min}$ | 0.05 m | 障碍物半径下限 |
| $\eta_{\max}$ | 8.0 | 严重度上限 |
| $\eta_{\min}$ | 1.5 | 安全区内严重度下限 |
| $v_{\text{retreat}}^{\min}$ | 0.15 m/s | 最小后退速度 |

---

## 9 已知局限

1. **障碍物形状假设**：最近点表示法假设障碍物表面是局部光滑的。圆柱体、球体等曲面障碍物工作良好；正方体、长方体等多面体在棱边处最近点会发生跳变，导致排斥方向突变和速度振荡。

2. **局部最优**：QP 优化的是瞬时速度，无法规划完整的绕行路径。当 leader 恰好位于障碍物正后方时，存在"对称死锁"风险——编队跟踪力指向障碍物，避障力后退，两者相互抵消。

3. **静态环境偏向**：动态障碍物速度估计仅依赖两帧差分，低通滤波引入延迟，对快速移动障碍物的跟踪有滞后。

4. **无全局路径规划**：避障是纯反应式的，不维护地图或全局代价图，无法处理 U 形障碍物等需要前瞻规划的场景。

---

## 参考文献

1. Fiorini, P., & Shiller, Z. (1998). Motion planning in dynamic environments using velocity obstacles. *The International Journal of Robotics Research*, 17(7), 760–772.

2. Khatib, O. (1986). Real-time obstacle avoidance for manipulators and mobile robots. *The International Journal of Robotics Research*, 5(1), 90–98.

3. Nocedal, J., & Wright, S. J. (2006). *Numerical Optimization* (2nd ed.). Springer. Chapter 3: Line Search Methods.

4. Boyd, S., & Vandenberghe, L. (2004). *Convex Optimization*. Cambridge University Press. Chapter 9: Unconstrained Minimization.
