# 基于运动学模型的齐次编队控制

## 1. 问题背景

### 1.1 质点模型的局限性

原始齐次编队控制算法将每个机器人建模为二维双积分器：

$$\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}, \quad 
\mathbf{x} = [p_x, p_y, v_x, v_y]^\top \in \mathbb{R}^4$$

其中 $A = \begin{bmatrix} 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 \end{bmatrix}$, $B = \begin{bmatrix} 0 & 0 \\ 0 & 0 \\ \frac{1}{m} & 0 \\ 0 & \frac{1}{m} \end{bmatrix}$。

该模型的隐含假设是：速度分量 $(v_x, v_y)$ 定义在全局（map）坐标系下，机器人被视为无朝向的质点。然而在实际系统中，ROS 的 `cmd_vel` 接口（`geometry_msgs/Twist`）语义为**车体坐标系**：
- `linear.x`：沿机器人前进方向（body +X）
- `linear.y`：沿机器人横向（body +Y）
- `angular.z`：绕垂直轴旋转

质点模型输出的全局系速度直接写入 `cmd_vel` 将导致坐标系不匹配——机器人朝向后，速度指令的实际方向与期望方向之间存在由偏航角（yaw）决定的旋转偏差。此外，偏航控制作为独立回路的 P+前馈与编队控制解耦，无法协同。

### 1.2 运动学模型的必要性

三全向轮底盘是完整约束（holonomic）系统，可在任意方向平移、可原地旋转。其运动学可以用刚体平面运动描述：

$$\begin{bmatrix} \dot{p}_x \\ \dot{p}_y \\ \dot{\theta} \end{bmatrix} = 
\begin{bmatrix} \cos\theta & -\sin\theta & 0 \\ \sin\theta & \cos\theta & 0 \\ 0 & 0 & 1 \end{bmatrix}
\begin{bmatrix} v_x^b \\ v_y^b \\ \omega \end{bmatrix}$$

其中 $\theta$ 为机器人偏航角，$(v_x^b, v_y^b, \omega)$ 为车体系速度。

## 2. 六维运动学模型

### 2.1 状态定义

我们将状态从 4 维扩展到 6 维，采用**混合坐标系**表示：

$$\mathbf{x} = [p_x, p_y, \theta, v_x^b, v_y^b, \omega]^\top \in \mathbb{R}^6$$

- $p_x, p_y, \theta$：map 系下的位置和偏航角（全局可观测）
- $v_x^b, v_y^b, \omega$：车体系下的线速度和角速度（直接对应 `cmd_vel` 输出）

控制输入为车体系下的加速度：

$$\mathbf{u} = [a_x, a_y, \alpha]^\top \in \mathbb{R}^3$$

### 2.2 非线性连续时间模型

完整的非线性运动学为：

$$\dot{\mathbf{x}} = \mathbf{f}(\mathbf{x}, \mathbf{u}) = 
\begin{bmatrix}
v_x^b \cos\theta - v_y^b \sin\theta \\
v_x^b \sin\theta + v_y^b \cos\theta \\
\omega \\
a_x / m \\
a_y / m \\
\alpha / I
\end{bmatrix}$$

其中 $m$ 为平移方向的质量调谐参数，$I$ 为转动惯量调谐参数。

## 3. 误差动力学与线性化

### 3.1 Leader-Follower 误差定义

设 leader 状态为 $\mathbf{x}_l$，follower 状态为 $\mathbf{x}_f$。编队偏移向量 $\mathbf{d} \in \mathbb{R}^6$ 定义在 **leader 车体坐标系**下（即编队点随 leader 旋转——"跟随在 leader 右后方 2m"在 leader 转弯时保持不变）。

首先将位置误差从 map 系旋转到 leader 车体系：

$$\begin{bmatrix} \Delta e_x^L \\ \Delta e_y^L \end{bmatrix} = 
R(-\theta_l) \begin{bmatrix} p_{x,f} - p_{x,l} \\ p_{y,f} - p_{y,l} \end{bmatrix}$$

其中 $R(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}$ 为旋转矩阵。

follower 车体系速度也需旋转到 leader 车体系才能求差：

$$\begin{bmatrix} v_{x,f}^L \\ v_{y,f}^L \end{bmatrix} = 
R(-\Delta\theta) \begin{bmatrix} v_{x,f}^b \\ v_{y,f}^b \end{bmatrix}$$

其中 $\Delta\theta = \theta_f - \theta_l$。

完整的误差向量（leader 车体系下）：

$$\mathbf{e} = \begin{bmatrix}
(p_{x,f} - p_{x,l})\cos\theta_l + (p_{y,f} - p_{y,l})\sin\theta_l - d_x \\
-(p_{x,f} - p_{x,l})\sin\theta_l + (p_{y,f} - p_{y,l})\cos\theta_l - d_y \\
\theta_f - \theta_l \\
v_{x,f}^L - v_{x,l}^b \\
v_{y,f}^L - v_{y,l}^b \\
\omega_f - \omega_l
\end{bmatrix}$$

### 3.2 边界投影编队策略

区别于传统的离散编队点方法（在 leader 周围均匀分布 $m_p$ 个点，follower 选择最近者），本文采用**连续边界投影**策略：

$$\mathbf{d}_{\text{pos}} = r_s \cdot \frac{\mathbf{p}_f - \mathbf{p}_l}{\|\mathbf{p}_f - \mathbf{p}_l\|}$$

其中 $r_s$ 为安全圆半径。编队偏移 $\mathbf{d} = [\mathbf{d}_{\text{pos}}^\top, 0, 0, 0, 0]^\top$ 在 leader 车体系下。

此策略的优势：
1. 无需离散切换逻辑和滞后参数（tol）
2. follower 始终沿最短路径逼近安全边界
3. 编队偏移连续变化，不破坏齐次控制的平滑性

### 3.3 误差动力学线性化

在 leader 当前状态附近线性化。假设 $\Delta\theta$ 较小（$\cos\Delta\theta \approx 1$, $\sin\Delta\theta \approx \Delta\theta$），得到线性时变误差动力学：

$$\dot{\mathbf{e}} = A_l \mathbf{e} + B \mathbf{u}$$

其中 $A_l$ 包含 leader 速度带来的耦合项：

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
\end{bmatrix}$$

**耦合项解释**：当 leader 旋转时（$\omega_l \neq 0$），leader 车体系也在旋转，导致 X/Y 位置误差通道出现交叉耦合。类似地，leader 平移速度 $(v_{x,l}^b, v_{y,l}^b)$ 通过偏航误差 $\Delta\theta$ 耦合到位置通道。

## 4. 齐次控制律设计

### 4.1 线性比例控制器（LPC）

对于线性误差动力学 $\dot{\mathbf{e}} = A_l\mathbf{e} + B\mathbf{u}$，设计线性状态反馈：

$$\mathbf{u}_{\text{lin}} = K \mathbf{e}$$

增益矩阵 $K \in \mathbb{R}^{3 \times 6}$ 采用**分块解耦 + 临界阻尼**设计。X、Y、θ 三个通道独立配置：

$$K = \begin{bmatrix}
k_{1,x} & 0 & 0 & k_{2,x} & 0 & 0 \\
0 & k_{1,y} & 0 & 0 & k_{2,y} & 0 \\
0 & 0 & k_{1,\theta} & 0 & 0 & k_{2,\theta}
\end{bmatrix}$$

每个通道配置为临界阻尼双极点。以 X 通道为例：

$$k_{2,x} = -2a, \quad k_{1,x} = \frac{a(k_{2,x} + a)}{m}$$

其中 $a = \max(\bar{a}, \omega_d m)$，$\bar{a} = \text{clamp}(-m \cdot e_{v_x}/e_{p_x}, -\omega_d m, \omega_d m)$ 为防超调自适应项。这使得闭环极点位于 $s = -a/m$（重根），无超调收敛。

**θ 通道使用 $I$ 替代 $m$**：

$$k_{2,\theta} = -2c, \quad k_{1,\theta} = \frac{c(k_{2,\theta} + c)}{I}$$

其中 $c = \max(\bar{c}, \omega_d^\theta I)$。

### 4.2 齐次比例控制器（HPC）

齐次控制的核心思想是引入**非线性增益缩放**：误差大时增益指数放大，误差小时增益抑制。这是通过**齐次范数**和**膨胀生成元**实现的。

#### 4.2.1 齐次度与膨胀

定义对角权重矩阵（编码各状态通道的齐次度）：

$$G_0 = -T^{-1} \cdot \text{diag}(\underbrace{1,\ldots,1}_{n_p}, \underbrace{0,\ldots,0}_{n_v}) \cdot T$$

其中 $T$ 为块可控分解变换矩阵。对于 6D 系统，块尺寸 $\mathbf{nt} = [3, 3]$（3 个位置/角度状态权重 1，3 个速度状态权重 0）。

膨胀生成元 $G_d = I + \nu G_0$，其中齐次度 $\nu \in [\nu_{\min}, \nu_{\max}]$ 由 Lyapunov 分析确定。

#### 4.2.2 齐次范数

定义齐次范数 $\|\mathbf{e}\|_{G_d, P}$ 为满足以下条件的数 $c$：

$$\| \exp(-G_d \ln q) \cdot \mathbf{e} \|_P = 1$$

即寻找缩放因子 $q = e^c$ 使得膨胀后的误差向量落在 $P$-单位椭球上。通过二分法求解。

#### 4.2.3 控制律

齐次控制律表达为**LPC 的非线性升级**：

$$\mathbf{u} = q^{1+\nu} \cdot K \cdot \exp(G_d(1 - \ln q)) \cdot \mathbf{e}$$

其结构可解读为：
- $q = \|\mathbf{e}\|_{G_d,P}$：齐次范数（距离度量）
- $q^{1+\nu}$：基于距离的增益缩放因子
- $\exp(G_d(1 - \ln q))$：非线性误差翘曲——误差各分量按其齐次度权重非线性膨胀/收缩

**物理直觉**：当误差很大时（$q \ll 1$），$\ln q$ 很负，$(1-\ln q)$ 很大，$\exp(G_d(1-\ln q))$ 显著放大误差——产生强控制动作。当误差很小时（$q \to 1$），翘曲接近恒等变换——控制趋于线性。

### 4.3 控制力坐标变换

齐次控制律在 **leader 车体系**下输出力/力矩 $(F_x^L, F_y^L, \tau)$。需将其旋转到 follower 车体系以驱动实际底盘：

$$\begin{bmatrix} F_x^f \\ F_y^f \end{bmatrix} = R(\Delta\theta) \begin{bmatrix} F_x^L \\ F_y^L \end{bmatrix}$$

follower 车体系速度通过前向欧拉积分更新：

$$\begin{aligned}
v_{x,f}^b[k+1] &= v_{x,f}^b[k] + h \cdot F_x^f / m \\
v_{y,f}^b[k+1] &= v_{y,f}^b[k] + h \cdot F_y^f / m \\
\omega_f[k+1] &= \omega_f[k] + h \cdot \tau / I
\end{aligned}$$

其中 $h = 0.1\text{s}$ 为控制步长。

### 4.4 HPC 参数重算策略

$A_l$ 包含 leader 速度 $(\omega_l, v_{x,l}^b, v_{y,l}^b)$，是时变的。HPC 参数 $(G_0, P, \nu, G_d)$ 在以下条件下重新计算：

$$\|[\omega_l, v_{x,l}^b, v_{y,l}^b] - [\omega_l^{\text{prev}}, v_{x,l}^{b,\text{prev}}, v_{y,l}^{b,\text{prev}}]\| > \varepsilon_v$$

或偏航误差变化 $|\Delta\theta - \Delta\theta^{\text{prev}}| > \varepsilon_\theta$。

## 5. 全向轮运动学约束

### 5.1 逆运动学

三轮全向底盘的车体系速度到轮角速度的映射（URDF 几何参数）：

$$\begin{bmatrix} \omega_1 \\ \omega_2 \\ \omega_3 \end{bmatrix} = 
\frac{1}{r} \begin{bmatrix}
0 & 1 & L \\
-\frac{\sqrt{3}}{2} & -\frac{1}{2} & L \\
\frac{\sqrt{3}}{2} & -\frac{1}{2} & L
\end{bmatrix}
\begin{bmatrix} v_x^b \\ v_y^b \\ \omega \end{bmatrix}$$

其中 $r$ 为轮半径，$L$ 为底盘半径。

### 5.2 轮速约束

每个轮子的角速度需满足 $|\omega_i| \leq \omega_{\max}$。若任意轮子超限，对 $(v_x^b, v_y^b, \omega)$ 做等比缩放：

$$\text{scale} = \frac{\omega_{\max}}{\max_i |\omega_i|}$$

同时施加加速度 slew rate 限幅，防止相邻周期指令跳变。

## 6. 完整控制管线

```
┌──────────────────┐
│  Leader EKF + TF │──→ x_l = [p_x,p_y,θ,v_x^b,v_y^b,ω]_l  (map系+车体系)
│  Follower EKF+TF │──→ x_f = [p_x,p_y,θ,v_x^b,v_y^b,ω]_f
└──────────────────┘
         │
         ▼
┌──────────────────────────────────────────┐
│ 1. 位置误差旋转到 leader 车体系          │
│ 2. follower 速度旋转到 leader 车体系      │
│ 3. 边界投影计算编队偏移 d               │
│ 4. 计算误差 e = x_f - x_l - d            │
└──────────────────────────────────────────┘
         │
         ▼
┌──────────────────────────────────────────┐
│ 5. 自适应 LPC 增益: K = f(e, m, I, ω_d) │
│ 6. A_l 更新 + HPC 时变重算(按需)        │
│ 7. 齐次范数: q = ‖e‖_{G_d,P} (二分法)   │
│ 8. 齐次控制律: u^L = q^{1+ν}·K·expm(·)e │
│ 9. 力旋转变换: u^f = R(Δθ)·u^L          │
│ 10. 前向欧拉: v_{cmd} = v + h·u^f/M     │
└──────────────────────────────────────────┘
         │
         ▼
┌──────────────────────────────────────────┐
│ 11. 全向轮逆运动学约束(等比缩放)        │
│ 12. 加速度 slew rate 限幅               │
│ 13. 发布 cmd_vel (车体系)               │
└──────────────────────────────────────────┘
```

## 7. 参数汇总

| 参数 | 符号 | 默认值 | 物理含义 |
|------|------|--------|----------|
| 安全半径 | $r_s$ | 2.0 m | 编队保持的最小距离 |
| 质量参数 | $m$ | 8.0 | 平移通道调谐（越大响应越迟缓） |
| 转动惯量 | $I$ | 1.0 | 偏航通道调谐 |
| 期望带宽 | $\omega_d$ | 1.5 | 位置通道临界阻尼带宽 |
| 偏航带宽 | $\omega_d^\theta$ | 1.5 | 偏航通道临界阻尼带宽 |
| HPC 重算阈值 | $\varepsilon_v$ | 0.3 | leader 速度变化触发阈值 |
| 轮半径 | $r$ | 0.03 m | 运动学约束 |
| 底盘半径 | $L$ | 0.11 m | 运动学约束 |
| 最大轮速 | $\omega_{\max}$ | 20 rad/s | 运动学约束 |

## 8. 与原始质点模型的对比

| 特性 | 质点模型 (4D) | 运动学模型 (6D) |
|------|--------------|-----------------|
| 状态 | $[p_x, p_y, v_x, v_y]$ (全 map 系) | $[p_x, p_y, \theta, v_x^b, v_y^b, \omega]$ (混合系) |
| 朝向 | 无（质点无朝向概念） | $\theta$ 参与运动学和误差计算 |
| 速度语义 | map 系，输出需外加坐标旋转 | 车体系，输出直接对应 `cmd_vel` |
| yaw 控制 | 独立 P+前馈回路 | 集成于 6D 主回路，统一优化 |
| 编队点 | 离散多边形 + 滞后切换 | 连续边界投影 |
| 系统矩阵 | $A$ 恒定（纯积分链） | $A_l$ 时变（含 leader 速度耦合） |
| HPC 重算 | 仅初始化 + 编队点切换 | 额外条件：leader 速度变化、Δθ 变化 |
| 坐标系 | 单一 map 系 | leader 车体系（误差）+ follower 车体系（输出） |

## 参考文献

原始齐次控制算法论文：`homogeneous_control.pdf`
三全向轮运动学：Santos et al. (2018), omnidirectional_controllers package
