# 基于顺序线性化的 MPC 6D 编队控制

## 1. 问题背景

### 1.1 为什么需要 MPC 作为对照组

齐次编队控制（HPC）的核心是非线性增益调度——通过齐次范数 $q = \|\mathbf{e}\|_G$ 和矩阵指数
$\exp(G_d(1-\ln q))$ 对误差做非线性 warping，实质上是一种**隐式的最优控制**。
HPC 的闭环性能缺乏显式的最优性证明，且调参（$m, I, \omega_d$）对闭环行为的影响不够直观。

模型预测控制（MPC）提供了另一种路径：**在每个控制周期显式求解一个有限时域最优控制问题**。
MPC 具有明确的代价函数、约束建模能力、以及预测能力，适合作为 HPC 的对照组——
两者输出同样的车体系速度指令，通过相同的 `KinematicConstraint` 后处理，在完全一致的仿真条件下对比。

### 1.2 方法定位

本文的 MPC 不是直接求解完整非线性规划（NLP），而是在每个控制周期对非线性模型做**单点局部线性化**、
将问题转化为凸 QP 求解。准确名称应为 **Single-point Linearized MPC**。

## 2. 六维运动学模型

### 2.1 状态与输入

与 HPC 6D 使用相同的混合坐标系状态：

$$\mathbf{x} = [p_x, p_y, \theta, v_x^b, v_y^b, \omega]^{\mathsf{T}} \in \mathbb{R}^6$$

$$\mathbf{u} = [a_x^b, a_y^b, \alpha]^{\mathsf{T}} \in \mathbb{R}^3$$

- $p_x, p_y, \theta$：map 系位置和偏航角
- $v_x^b, v_y^b, \omega$：车体系速度（直接对应 `cmd_vel`）
- $a_x^b, a_y^b, \alpha$：车体系加速度/角加速度

注意：MPC 内部不使用 $m, I$（质量/惯量参数），输入直接解释为加速度。
这避免了输入定义模糊（力 vs 加速度）导致的约束单位混乱。

### 2.2 非线性连续时间模型

$$\dot{\mathbf{x}} = \mathbf{f}(\mathbf{x}, \mathbf{u}) =
\begin{bmatrix}
v_x^b \cos\theta - v_y^b \sin\theta \\
v_x^b \sin\theta + v_y^b \cos\theta \\
\omega \\
a_x^b \\
a_y^b \\
\alpha
\end{bmatrix}$$

### 2.3 单点局部线性化

在每个控制周期，于当前跟随者状态 $\bar{\mathbf{x}} = \mathbf{x}_{\text{follower}}$ 处做一阶 Taylor 展开：

$$\dot{\mathbf{x}} \approx \mathbf{f}(\bar{\mathbf{x}}, \mathbf{0}) +
\frac{\partial \mathbf{f}}{\partial \mathbf{x}}\bigg|_{\bar{\mathbf{x}},\mathbf{0}} (\mathbf{x} - \bar{\mathbf{x}}) +
\frac{\partial \mathbf{f}}{\partial \mathbf{u}}\bigg|_{\bar{\mathbf{x}},\mathbf{0}} \mathbf{u}$$

记 $A_c = \partial \mathbf{f}/\partial \mathbf{x}|_{\bar{\mathbf{x}},\mathbf{0}}$，$B = \partial \mathbf{f}/\partial \mathbf{u}$，得仿射线性模型：

$$\dot{\mathbf{x}} = A_c \mathbf{x} + B \mathbf{u} + \mathbf{c}$$

其中仿射常数项 $\mathbf{c} = \mathbf{f}(\bar{\mathbf{x}}, \mathbf{0}) - A_c \bar{\mathbf{x}}$。

**$A_c$ 矩阵（6×6）**：

$$A_c = \begin{bmatrix}
0 & 0 & -v_x^b s_\theta - v_y^b c_\theta & c_\theta & -s_\theta & 0 \\
0 & 0 & v_x^b c_\theta - v_y^b s_\theta & s_\theta & c_\theta & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}$$

其中 $c_\theta = \cos\bar{\theta}, s_\theta = \sin\bar{\theta}$。
仅右上角 3×3 旋转耦合块依赖于当前状态，其余为零。$A_c$ 的稀疏性源于速度通道的线性动力学。

**$B$ 矩阵（6×3，常数）**：

$$B = \begin{bmatrix}
0 & 0 & 0 \\
0 & 0 & 0 \\
0 & 0 & 0 \\
1 & 0 & 0 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}$$

### 2.4 前向 Euler 离散化

离散时间步长 $dt = 1/\text{control\_rate}$（通常 0.05s @ 20Hz）：

$$A_d = I_6 + dt \cdot A_c, \quad B_d = dt \cdot B, \quad \mathbf{c}_d = dt \cdot \mathbf{c}$$

离散仿射模型为：

$$\mathbf{x}_{k+1} = A_d \mathbf{x}_k + B_d \mathbf{u}_k + \mathbf{c}_d$$

整个预测时域使用**同一组** $A_d, B_d, \mathbf{c}_d$（单点线性化假设）。

**局限性**：若 Leader 旋转或高速运动下，恒定线性化与真实时变动力学差异增大，预测精度下降。
升级路径：沿预测轨迹的时变线性化 $A_k, B_k, \mathbf{c}_k$（需多轮 SQP/RTI 迭代）。

## 3. MPC 最优化问题

### 3.1 预测时域

$$N = 40, \quad dt = 0.05\text{s}, \quad T = N \cdot dt = 2.0\text{s}$$

该时域足够覆盖大初始误差（>4m）的收敛路径，同时 QP 变量数（366）在 OSQP 实时求解能力内（~1-5ms）。

### 3.2 决策变量（非紧凑形式）

$$\mathbf{z} = [\mathbf{x}_0, \mathbf{u}_0, \mathbf{x}_1, \mathbf{u}_1, \dots, \mathbf{x}_{N-1}, \mathbf{u}_{N-1}, \mathbf{x}_N]^{\mathsf{T}} \in \mathbb{R}^{9N+6 = 366}$$

非紧凑形式保持了 Hessian 矩阵的块对角结构，无需 Riccati 递归，简化了 QP 构建。

变量索引 helper：
$$\mathbf{x}_k \text{ at } \text{x\_idx}(k) = k(n+m) = 9k$$
$$\mathbf{u}_k \text{ at } \text{u\_idx}(k) = k(n+m) + n = 9k + 6$$

### 3.3 代价函数

$$\min_{\mathbf{z}} \sum_{k=0}^{N-1} \left[ (\mathbf{x}_k - \mathbf{x}_{\text{ref},k})^{\mathsf{T}} Q (\mathbf{x}_k - \mathbf{x}_{\text{ref},k}) + \mathbf{u}_k^{\mathsf{T}} R \mathbf{u}_k \right] + (\mathbf{x}_N - \mathbf{x}_{\text{ref},N})^{\mathsf{T}} Q_f (\mathbf{x}_N - \mathbf{x}_{\text{ref},N})$$

以标准 QP 形式 $\min \frac{1}{2}\mathbf{z}^{\mathsf{T}}P\mathbf{z} + \mathbf{q}^{\mathsf{T}}\mathbf{z}$ 表示：

$$P = \text{blkdiag}(2Q, 2R, 2Q, 2R, \dots, 2R, 2Q_f) \in \mathbb{R}^{366 \times 366}$$

$$\mathbf{q}_{\mathbf{x}_k} = -2Q \cdot \mathbf{x}_{\text{ref},k}, \quad \mathbf{q}_{\mathbf{u}_k} = \mathbf{0}, \quad \mathbf{q}_{\mathbf{x}_N} = -2Q_f \cdot \mathbf{x}_{\text{ref},N}$$

由于 $Q, R, Q_f$ 均为对角矩阵，$P$ 为对角矩阵（diagonal），数值条件良好。

### 3.4 权重矩阵

$$Q = \operatorname{diag}(q_{px}, q_{py}, q_{\theta}, q_{vx}, q_{vy}, q_{\omega})$$
$$R = \operatorname{diag}(r_{ax}, r_{ay}, r_{\alpha})$$
$$Q_f = \eta \cdot Q \quad (\eta = \text{terminal\_factor})$$

默认值：
- $q_{px} = q_{py} = 5.0$（位置跟踪主导）
- $q_{\theta} = 20.0$（优先旋转对准——大初始航向偏差的刚性要求）
- $q_{vx} = q_{vy} = q_{\omega} = 0.5$（速度阻尼，抑制超调）
- $r_{ax} = r_{ay} = r_{\alpha} = 0.01$（输入惩罚——R 权重下限为 0.01，避免 Q/R 比超过 2000:1 导致 QP 条件数极端）
- $\eta = 10.0$（终端代价，鼓励收敛而非"靠近就行"）

### 3.5 约束

**动力学等式约束（$6N$ 行）**：

$$\mathbf{x}_{k+1} - A_d \mathbf{x}_k - B_d \mathbf{u}_k = \mathbf{c}_d, \quad k = 0, \dots, N-1$$

**初始状态等式约束（6 行）**：

$$\mathbf{x}_0 = \mathbf{x}_{\text{follower}}$$

**输入不等式约束（$3N$ 行）**：

$$|\mathbf{u}_k| \le \mathbf{u}_{\max} = [2.0, 2.0, 6.0]^{\mathsf{T}} \;\text{(m/s}^2, \text{rad/s}^2\text{)}$$

**速度不等式约束（从 $k=3$ 到 $N$）**：

$$|v_{x,k}^b| \le 1.0, \quad |v_{y,k}^b| \le 1.0, \quad |\omega_k| \le 2.0$$

速度约束从 $x_3$（而非 $x_1$）开始施加，留出 $3 \cdot dt \cdot a_{\max}$ 的缓冲步数。
避免因当前速度超限 $v_0 > v_{\max}$ 时，单步最大减速 $a_{\max} \cdot dt$ 无法使 $x_1$ 回到限幅内
导致的 primal infeasible。若 $v_0$ 严重超限（$>v_{\max} + 3 \cdot dt \cdot a_{\max}$），仍需 fallback 零速度。

**轮速约束**：轮速 $w_i = f(v_x^b, v_y^b, \omega)$ 通过 `KinematicConstraint::apply()` 后处理施加
（等比缩放 + slew rate 限幅），不进入 QP 约束矩阵。全向轮的轮速约束对车体速度是线性的
（$J_{\text{wheel}}[v_x, v_y, \omega]^{\mathsf{T}} \le w_{\max}$），后续可升级为直接加入 QP 约束。

### 3.6 求解器

使用 OSQP（Operator Splitting Quadratic Program），通过 `ros-humble-osqp-vendor` 安装：

- 每步完整 rebuild + solve（优先验证正确性，后续优化为 workspace 复用 + warm-start）
- 最大迭代 2000 次
- 收敛精度 $\varepsilon_{\text{abs}} = \varepsilon_{\text{rel}} = 10^{-3}$（放松以提升数值稳定性）
- 求解失败时 fallback：单次失败发布零速度（安全停车），连续 5 次失败进入安全停车状态

## 4. 参考轨迹构造

### 4.1 Leader 预测（恒定车体速度）

假设 Leader 在车体系下保持恒定速度 $(v_x^L, v_y^L, \omega_L)$，对预测步 $k$ 积分：

$$\theta_L(t_k) = \theta_L(0) + \omega_L \cdot t_k, \quad t_k = k \cdot dt$$

**位置积分**：

若 $|\omega_L| < \varepsilon$（近似直线运动）：
$$\begin{bmatrix} p_x^L(t_k) \\ p_y^L(t_k) \end{bmatrix} =
\begin{bmatrix} p_x^L(0) \\ p_y^L(0) \end{bmatrix} +
t_k \begin{bmatrix} v_x^L \cos\theta_L(0) - v_y^L \sin\theta_L(0) \\ v_x^L \sin\theta_L(0) + v_y^L \cos\theta_L(0) \end{bmatrix}$$

若 $|\omega_L| \ge \varepsilon$（旋转运动，精确积分）：
$$\begin{bmatrix} p_x^L(t_k) \\ p_y^L(t_k) \end{bmatrix} =
\begin{bmatrix} p_x^L(0) \\ p_y^L(0) \end{bmatrix} +
\frac{1}{\omega_L}
\begin{bmatrix} v_x^L \Delta s + v_y^L \Delta c \\ -v_x^L \Delta c + v_y^L \Delta s \end{bmatrix}$$

其中 $\Delta s = \sin\theta_L(t_k) - \sin\theta_L(0)$，$\Delta c = \cos\theta_L(t_k) - \cos\theta_L(0)$。

实现时需验证 $|\omega_L| \to 0$ 时旋转积分与直线积分的连续性，避免数值跳变。

### 4.2 编队参考位置

当前默认使用 Leader 车体系**固定偏移**：
$$\begin{bmatrix} p_x^{\text{ref}} \\ p_y^{\text{ref}} \end{bmatrix} =
\begin{bmatrix} p_x^L \\ p_y^L \end{bmatrix} +
R(\theta_L) \begin{bmatrix} d_x \\ d_y \end{bmatrix}$$

其中 $d_x, d_y$ 为可配置偏移量（如 `formation_offset_x=-1.0` 表示 Leader 后方 1m）。

**边界投影模式**（与 HPC 6D 一致）：
$$\begin{bmatrix} p_x^{\text{ref}} \\ p_y^{\text{ref}} \end{bmatrix} =
\begin{bmatrix} p_x^L \\ p_y^L \end{bmatrix} -
r \cdot \frac{\mathbf{p}^L - \mathbf{p}^f}{\|\mathbf{p}^L - \mathbf{p}^f\|}$$

边界投影使参考点随 Leader-Follower 相对方向动态移动，初始误差更小（约 $r/d$ 比例缩放）。
但位置参考与速度参考的一致性需要多轮 SQP 迭代解决——当前初版以固定偏移为默认策略，
边界投影待后续完善。

### 4.3 参考航向

参考航向为 Leader 航向，经 unwrap 连续化处理：

$$\theta_{\text{ref}} = \theta_f + \operatorname{wrap}(\theta_L - \theta_f)$$

其中 $\operatorname{wrap}(\alpha) = \alpha - 2\pi \lfloor (\alpha + \pi) / 2\pi \rfloor$。
此处理使 $\theta_{\text{ref}}$ 始终落在当前 $\theta_f$ 的同一角度分支，避免 QP 代价中出现
$\pm 2\pi$ 的虚假大误差。

### 4.4 参考速度（车体系）

参考速度需同时考虑 Leader 平移速度和编队偏移点的旋转速度：

$$\mathbf{v}_{\text{ref}}^{\text{map}} = \mathbf{v}_L^{\text{map}} + R(\theta_L) \cdot (\omega_L \times \mathbf{d}^{\text{body}})$$

其中 $\mathbf{v}_L^{\text{map}} = R(\theta_L)[v_x^L, v_y^L]^{\mathsf{T}}$ 为 Leader 车体速度转 map 系，
$\omega_L \times \mathbf{d}^{\text{body}} = [-\omega_L d_y^{\text{body}}, \omega_L d_x^{\text{body}}]^{\mathsf{T}}$
为旋转线速度（偏移量 $d^{\text{body}}$ 为 Leader 到参考点的车体系偏移）。

**关键处理**：使用跟随者的实际朝向将 map 系速度转回车体系：

$$\mathbf{v}_{\text{ref}}^{\text{body}} = R(\theta_f)^{\mathsf{T}} \cdot \mathbf{v}_{\text{ref}}^{\text{map}}$$

$$R(\theta_f)^{\mathsf{T}} = \begin{bmatrix} \cos\theta_f & \sin\theta_f \\ -\sin\theta_f & \cos\theta_f \end{bmatrix}$$

使用 $\theta_f$（而非 $\theta_{\text{ref}}$）进行旋转，保证了速度参考在跟随者车体系下的正确性——
跟随者用自己的车体轴理解 map 系运动。之前使用 $\theta_{\text{ref}}$（Leader 朝向）时，
两车朝向不同导致前后/侧向跟踪不对称——侧向响应显著弱于前后向。

### 4.5 角度误差处理

QP 内不 normalize 角度误差。而是在构造 $\theta_{\text{ref}}$ 时通过 unwrap 连续化（见 §4.3），
使得 QP 可直接使用普通二次代价 $(\theta - \theta_{\text{ref}})^2$。

## 5. 工程细节

### 5.1 OSQP 稀疏矩阵构建

Eigen `SparseMatrix<double>`（ColMajor 存储，天然 CSC 格式）通过 `eigen_to_csc()` 转换为
OSQP 原生 `csc` 结构。约束矩阵 $A$ 和 Hessian $P$ 的稀疏结构在整个运行期间不变
（$P$ 始终对角，$A$ 的 sparsity pattern 仅依赖于 $N$ 和模型维度），
为后续 workspace 复用和 warm-start 优化提供了基础。

### 5.2 控制输出与后处理

MPC 解出的最优加速度 $\mathbf{u}_0^*$ 经前向 Euler 积分得到期望速度：

$$\mathbf{v}_{\text{cmd}} = \mathbf{v}_{\text{current}} + dt \cdot \mathbf{u}_0^*$$

然后依次通过：
1. 车体速度 clamp（至 $v_{\max}$ / $\omega_{\max}$）
2. `KinematicConstraint::apply()`：全向轮逆运动学 + slew rate 限幅
3. 发布 `cmd_vel`（body-frame Twist）

### 5.3 求解失败的渐退策略

| 情况 | 行为 |
|------|------|
| 单次求解失败 | 发布零速度，日志警告 |
| 连续失败 < 5 次 | 同上，维持安全输出 |
| 连续失败 ≥ 5 次 | 进入安全停车状态，不再输出 MPC 指令，等待外部恢复 |

## 6. 参数参考

| 符号 | 参数名 | 默认值 | 说明 |
|------|--------|--------|------|
| $N$ | `mpc_horizon` | 40 | 预测时域步数 |
| $q_{px}, q_{py}$ | `mpc_q_px/py` | 5.0 | 位置跟踪权重 |
| $q_{\theta}$ | `mpc_q_theta` | 20.0 | 航向跟踪权重 |
| $q_{vx}, q_{vy}, q_{\omega}$ | `mpc_q_vx/vy/omega` | 0.5 | 速度阻尼权重 |
| $r_{ax}, r_{ay}$ | `mpc_r_ax/ay` | 0.01 | 线加速度惩罚 |
| $r_{\alpha}$ | `mpc_r_alpha` | 0.01 | 角加速度惩罚 |
| $\eta$ | `mpc_terminal_factor` | 10.0 | 终端代价倍数 |
| $v_{\max}$ | `max_linear_vel` | 1.0 | 线速度上限 (m/s) |
| $\omega_{\max}$ | `max_angular_vel` | 2.0 | 角速度上限 (rad/s) |
| $a_{\max}$ | `max_linear_accel` | 2.0 | 线加速度上限 (m/s²) |
| $\alpha_{\max}$ | `max_angular_accel` | 6.0 | 角加速度上限 (rad/s²) |
| $d_x, d_y$ | `formation_offset_x/y` | -2.0, 0.0 | Leader 车体系固定偏移 (m) |
| $r$ | `formation_radius` | 2.0 | 编队圆半径 (m, 边界投影模式) |

## 7. 与 HPC 6D 对比

| 项目 | HPC 6D | MPC 6D |
|------|--------|--------|
| 控制策略 | 齐次非线性增益调度 | 显式最优控制 (QP) |
| 编队策略 | 边界投影 | 固定偏移（默认）/ 边界投影 |
| 预测能力 | 无（瞬时反馈） | 2.0s 有限时域预测 |
| 约束处理 | 后处理（KinematicConstraint） | QP 内 (输入/速度) + 后处理 (轮速) |
| 调参方式 | mass, I, ω_d（物理直觉弱） | Q, R 权重（物理直觉强） |
| 求解耗时 | ~0.01ms（解析式） | ~1-5ms（数值 QP） |
| 增益适应 | 自动（hnorm warping） | 恒定（单点线性化） |

## 8. 已知局限与升级路径

1. **单点线性化**：大角度 / 高速下预测不准 → 升级为沿预测轨迹时变线性化 $A_k, B_k, \mathbf{c}_k$
2. **固定偏移 vs 边界投影**：位置/速度参考一致性问题 → 多轮 SQP/RTI 迭代
3. **无 warm-start**：每步完整 rebuild → workspace 复用 + `osqp_update_data_vec/mat()`
4. **初值收敛慢**：大初始误差下 MPC 增益恒定 → 引入非线性增益（参考 HPC warping 思路）
5. **轮速约束未入 QP**：后处理可能裁剪 MPC 预测 → 将 $J_{\text{wheel}}$ 线性约束直接加入 QP
