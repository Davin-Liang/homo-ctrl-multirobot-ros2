# 6D Motor 模型：执行器动力学增广的齐次编队控制

## 组会汇报 (2 页 PPT)

---

## 第 1 页：问题与模型

### 为什么需要 6D

原始 4D 控制器假设 $v^{\mathrm{cmd}}$ 即刻生效（$\dot{p} = u/m$，双重积分器）。实物电机存在一阶滞后：

$$\dot{v}^{\mathrm{real}} = \frac{v^{\mathrm{cmd}} - v^{\mathrm{real}}}{\tau}, \quad \tau \approx 250\sim550\text{ ms}$$

不建模的后果：控制器以为发出指令后速度立刻到位，但实际速度需要 $\tau$ 时间爬升。大指令时控制器"过度自信"，超调加重。

### 6D Motor 模型

将电机滞后显式增广为状态：

$$\mathbf{x} = [p_x, p_y, v_x^{\mathrm{cmd}}, v_y^{\mathrm{cmd}}, v_x^{\mathrm{real}}, v_y^{\mathrm{real}}]^\top \in \mathbb{R}^6$$

$$\boxed{\begin{aligned}
\dot{p}_x &= v_x^{\mathrm{real}} \\
\dot{p}_y &= v_y^{\mathrm{real}} \\
\dot{v}_x^{\mathrm{cmd}} &= \frac{F_x}{m} \\
\dot{v}_y^{\mathrm{cmd}} &= \frac{F_y}{m} \\
\dot{v}_x^{\mathrm{real}} &= \frac{v_x^{\mathrm{cmd}} - v_x^{\mathrm{real}}}{\tau} \\
\dot{v}_y^{\mathrm{real}} &= \frac{v_y^{\mathrm{cmd}} - v_y^{\mathrm{real}}}{\tau}
\end{aligned}}$$

控制输入 $\mathbf{u} = [F_x, F_y]^\top$（map 系控制力），输出 $v^{\mathrm{cmd}}$ 经前向欧拉积分后作为 cmd_vel 发布。

写成矩阵形式 $\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}$：

$$\mathbf{x} = \begin{bmatrix} p_x \\ p_y \\ v_x^{\mathrm{cmd}} \\ v_y^{\mathrm{cmd}} \\ v_x^{\mathrm{real}} \\ v_y^{\mathrm{real}} \end{bmatrix}, \quad
A = \begin{bmatrix} 0 & 0 & 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 0 & 0 & 1 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & \tau^{-1} & 0 & -\tau^{-1} & 0 \\ 0 & 0 & 0 & \tau^{-1} & 0 & -\tau^{-1} \end{bmatrix}, \quad
B = \begin{bmatrix} 0 & 0 \\ 0 & 0 \\ m^{-1} & 0 \\ 0 & m^{-1} \\ 0 & 0 \\ 0 & 0 \end{bmatrix}$$

**三层齐次链**（块可控分解）：$p$（权重 2）→ $v^{\mathrm{real}}$（权重 1）→ $v^{\mathrm{cmd}}$（权重 0）

**极点配置**：三阶闭环 $(s+\lambda)^3$，$\lambda = \max(\omega_d, -m \cdot e_v / e_p / m)$ 自适应防超调。

**自适应 τ**：实物实测 $\tau_{\mathrm{eff}}$ 随 $|v^{\mathrm{cmd}}|$ 从 244 ms (@0.03 m/s) 变到 580 ms (@0.40 m/s)。模型中 $\tau$ 在 $[\tau_{\min}, \tau_{\max}]$ 间随 $|v^{\mathrm{cmd}}|$ 线性过渡。

### 齐次控制适用性验证（Bhat & Bernstein, 2005）

**1. 可控性（带阻尼）**：计算 $\mathcal{C} = [B, AB, A^2B, A^3B, A^4B, A^5B]$：

$$B = \begin{bmatrix} 0 & 0 \\ 0 & 0 \\ 1/m & 0 \\ 0 & 1/m \\ 0 & 0 \\ 0 & 0 \end{bmatrix}, \;
AB = \begin{bmatrix} 0 & 0 \\ 0 & 0 \\ 0 & 0 \\ 0 & 0 \\ 1/(m\tau) & 0 \\ 0 & 1/(m\tau) \end{bmatrix}, \;
A^2B = \begin{bmatrix} 1/(m\tau) & 0 \\ 0 & 1/(m\tau) \\ 0 & 0 \\ 0 & 0 \\ -1/(m\tau^2) & 0 \\ 0 & -1/(m\tau^2) \end{bmatrix}$$

$$\mathrm{rank}(\mathcal{C}) = 6 \quad \text{——可控。}$$

但 $A^3 \neq 0$：阻尼使 $A^kB$ 永不归零（$A^kB$ 的元素按 $(-1/\tau)^{k-2}$ 衰减）。对比：去掉阻尼后 $A_{\mathrm{nil}}^3 = 0$，纯积分器链。

**2. 齐次伸缩**：取权重 $\mathbf{r} = [2,2,1,1,0,0]$（位置 2、$v^{\mathrm{cmd}}$ 1、$v^{\mathrm{real}}$ 0）：

$$\Delta_\varepsilon(\mathbf{x}) = [\varepsilon^2 p,\; \varepsilon^2 p,\; \varepsilon^1 v^{\mathrm{cmd}},\; \varepsilon^1 v^{\mathrm{cmd}},\; v^{\mathrm{real}},\; v^{\mathrm{real}}]^\top$$

**3. 阻尼项破坏齐次性**：真实 $A$ 含 $-1/\tau$ 自阻尼：

$$f_5(\mathbf{x}) = \dot{v}_x^{\mathrm{real}} = \frac{v_x^{\mathrm{cmd}} - v_x^{\mathrm{real}}}{\tau}$$

$$f_5(\Delta_\varepsilon \mathbf{x}) = \frac{\varepsilon^1 v_x^{\mathrm{cmd}} - \varepsilon^0 v_x^{\mathrm{real}}}{\tau} \neq \varepsilon^{0+\tau} \cdot \frac{v_x^{\mathrm{cmd}} - v_x^{\mathrm{real}}}{\tau}$$

**$v^{\mathrm{cmd}}$（权 1）和 $v^{\mathrm{real}}$（权 0）的伸缩指数不一致**——齐次性不成立。

**4. 近似齐次控制**：系统可控（rank 6），但严格齐次性被阻尼破坏。HPC 在理想幂零链 $A_{\mathrm{nil}}$ 上设计（Bhat & Bernstein, Theorem 5.1），阻尼视为稳定扰动。实验表明该近似有效，但需对 $G_0$ 做正则化（第 2 页）。

**参照**：Bhat & Bernstein (2005) 齐次系统几何理论 [1]；动态扩展（dynamic extension）在非线性控制中用于处理执行器动力学 [2]。

---

## 第 2 页：HPC 兼容性与关键发现

### HPC 齐次翘曲在三层链上的退化

HPC 通过翘曲矩阵 $G_d = I + \nu G_0$ 对不同尺度的误差做几何缩放。块分解给出的 $G_0$ 特征值 $[-2,-2,-1,-1,0,0]$（对应权重 $[2,1,0]$），Lyapunov 方程求解的齐次度 $\nu = -1$。

$\nu = -1$ 时 $c^{1+\nu} = 1$（增益缩放失效），$G_d = I - G_0$ 特征值 $[3,3,2,2,1,1]$——**位置通道翘曲 3:1，方向严重失真，导致编队点附近震荡。**

### 对比：原始 4D 为什么能工作

原始 4D（双重积分器）：$G_0$ 特征值 $[-1,-1,0,0]$（权重 $[1,0]$），$\nu$ 同样是 $-1$，但 $G_d$ 特征值 $[2,2,1,1]$——翘曲比 2:1，温和无震荡。

| | 4D | 6D |
|------|------|------|
| 层数 | 2 | 3 |
| G₀ 权重 | [1, 0] | [2, 1, 0] |
| Gd 翘曲比 | 2:1 | 3:2:1 |
| 近目标行为 | 稳定收敛 | 震荡 |

**结论：$\nu=-1$ 不是问题——4D 也是 $-1$。问题在 $G_0$ 的权重梯度太陡。三块链的标准块分解不适合直接用于 HPC warp。**

### 修正：齐次权重的正则化

将 $G_0$ 缩放 $0.5\times$：特征值 $[-1,-1,-0.5,-0.5,0,0]$，$G_d$ 翘曲比 $2:1.5:1$，逼近 4D 的温和行为。可解释为深链的齐次权重正则化。

### 后续工作

- **死区补偿**：实物 ~220ms 纯延迟 $T_d$ 尚未建模。计划引入 Artstein 模型约简 [3] 将输入时延 $\dot{x} = Ax + Bu(t-T_d)$ 等价变换为无时延系统，保持现有 HPC 结构不变。

### 参考文献

[1] S. P. Bhat and D. S. Bernstein, "Geometric homogeneity with applications to finite-time stability," *Math. Control Signals Syst.*, 17: 101–127, 2005.

[2] H. K. Khalil, *Nonlinear Systems*, 3rd ed., Prentice Hall, 2002. (Chapter on dynamic extension & backstepping)

[3] Z. Artstein, "Linear systems with delayed controls: A reduction," *IEEE Trans. Autom. Control*, 27(4): 869–879, 1982.

[4] J. Jiang et al., "Fully distributed time-varying formation tracking control of linear multi-agent systems with input delay," *Systems & Control Letters*, 146, 2020.
