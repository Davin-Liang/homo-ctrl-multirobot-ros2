# Pade 死区增广齐次编队控制：8D 执行器全链路模型

## 1. 动机

6D Motor 模型将执行器爬升段（~1s）通过一阶滞后 $\dot{v}^{\mathrm{real}} = (v^{\mathrm{cmd}} - v^{\mathrm{real}})/\tau$ 显式建模。但实物实测存在 ~220ms 的**纯死区**（指令发出到轮子响应之间的空窗期），一阶滞后在 $t \to 0^+$ 时其脉冲响应非零，无法表达纯时延 $T_d$。

纯时延 $e^{-T_d s}$ 是无限维系统，标准工程处理是 Pade 有理近似——将死区展开为有限阶传递函数，再转化为状态空间方程。本文采用 Pade(1,1) 近似，将 6D 电机模型扩展为 **8D 电机-死区全链路模型**，使 HPC 统一感知"指令→死区→爬升→到位"的完整物理过程。

## 2. Pade(1,1) 近似

纯时延传递函数 $G_d(s) = e^{-T_d s}$ 的 Pade(1,1) 有理近似：

$$e^{-T_d s} \approx \frac{1 - T_d s / 2}{1 + T_d s / 2}\tag{1}$$

将该有理函数转化为状态空间实现。引入中间状态 $\omega$：

$$\begin{cases}
\dot{\omega} = -\dfrac{2}{T_d}\,\omega + v^{\mathrm{cmd}} \\[6pt]
v^{\mathrm{delayed}} = \dfrac{4}{T_d}\,\omega - v^{\mathrm{cmd}}
\end{cases}\tag{2}$$

**验证**：从 $v^{\mathrm{cmd}}$ 到 $v^{\mathrm{delayed}}$ 的传递函数：

$$\frac{v^{\mathrm{delayed}}}{v^{\mathrm{cmd}}} = \frac{4/T_d}{s + 2/T_d} - 1 = \frac{4/T_d - s - 2/T_d}{s + 2/T_d} = \frac{1 - T_d s/2}{1 + T_d s/2} = e^{-T_d s}\;\text{(Pade(1,1))}\quad \blacksquare$$

**物理含义**：$\omega$ 是死区的"记忆状态"——它暂存了 $v^{\mathrm{cmd}}$ 在 $T_d$ 时间窗内的历史信息。$\omega$ 越大意味着"待释放的延迟指令"越多，$v^{\mathrm{delayed}}$ 随之增大。

**Pade(1,1) 的瞬态行为**：Pade(1,1) 含右半平面零点 $s = 2/T_d$，阶跃响应有初始 undershoot——$v^{\mathrm{delayed}}(0^+) = -v^{\mathrm{cmd}}(0^+)$，约 $0.35T_d$（~77ms @ $T_d=0.22$）后过零。好在电机滞后（$\tau=0.43$）会通过 $\dot{v}^{\mathrm{real}} = (v^{\mathrm{delayed}} - v^{\mathrm{real}})/\tau$ 平滑此瞬态，$\dot{v}^{\mathrm{real}}(0^+) = -v^{\mathrm{cmd}}/\tau$。持续约 100ms 累积的速度误差 ~0.25m/s，位置误差 ~2.5cm，在编队控制容差内。更高带宽场景应考虑 Pade(2,2)。

## 3. 8D 状态空间模型

### 3.1 状态定义

$$\mathbf{x} = [p_x, p_y, v_x^{\mathrm{cmd}}, v_y^{\mathrm{cmd}}, \omega_x, \omega_y, v_x^{\mathrm{real}}, v_y^{\mathrm{real}}]^{\mathsf{T}} \in \mathbb{R}^8\tag{3}$$

每轴 4 个状态：$p$（位置）、$v^{\mathrm{cmd}}$（指令）、$\omega$（死区记忆）、$v^{\mathrm{real}}$（实际）。两轴共 8 维，控制输入保持 2 维。

### 3.2 系统方程

每轴（$i \in \{x, y\}$）：

$$\begin{cases}
\dot{p}_i = v_i^{\mathrm{real}} \\[4pt]
\dot{v}_i^{\mathrm{cmd}} = u_i / m \\[4pt]
\dot{\omega}_i = -\dfrac{2}{T_d}\,\omega_i + v_i^{\mathrm{cmd}} \\[4pt]
\dot{v}_i^{\mathrm{real}} = \dfrac{1}{\tau}\left(\dfrac{4}{T_d}\,\omega_i - v_i^{\mathrm{cmd}} - v_i^{\mathrm{real}}\right)
\end{cases}\tag{4}$$

8D 系统矩阵（状态按 $(p_x, p_y, v_x^{\mathrm{cmd}}, v_y^{\mathrm{cmd}}, \omega_x, \omega_y, v_x^{\mathrm{real}}, v_y^{\mathrm{real}})$ 排列）：

$$A = \begin{bmatrix}
0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & -\frac{2}{T_d} & 0 & 0 & 0 \\
0 & 0 & 0 & 1 & 0 & -\frac{2}{T_d} & 0 & 0 \\
0 & 0 & -\frac{1}{\tau} & 0 & \frac{4}{\tau T_d} & 0 & -\frac{1}{\tau} & 0 \\
0 & 0 & 0 & -\frac{1}{\tau} & 0 & \frac{4}{\tau T_d} & 0 & -\frac{1}{\tau}
\end{bmatrix}\tag{5}$$

$$B = \begin{bmatrix}
0 & 0 \\ 0 & 0 \\ m^{-1} & 0 \\ 0 & m^{-1} \\ 0 & 0 \\ 0 & 0 \\ 0 & 0 \\ 0 & 0
\end{bmatrix}\tag{6}$$

### 3.3 退化分析

- **$T_d \to 0$**：此时 $2/T_d \to \infty$，$\omega$ 模态无限快（奇异摄动），$v^{\mathrm{delayed}} \to v^{\mathrm{cmd}}$，退化为 6D Motor 模型。
- **$\tau \to 0^+$**：爬升段消失，$v^{\mathrm{real}}$ 瞬时跟上 $v^{\mathrm{delayed}}$，退化为纯死区 Pade 模型 + 双积分器。
- **$T_d \to 0$ 且 $\tau \to 0^+$**：全链路退化为 4D 双积分器。

### 3.4 与 6D Motor 的核心差异

| | 6D Motor | 8D Pade |
|------|------|------|
| 状态维度 | 6 | 8 |
| 死区建模 | 无（靠 Smith 外挂） | Pade(1,1) 内嵌 |
| 每轴阶数 | 3 阶 | 4 阶 |
| A 矩阵性质 | 常值（τ 自适应时为缓变） | 常值 |
| 可控链 | [3,3] = x/y 三阶 | [4,4] = x/y 四阶 |
| 齐次权重 | [2,2,1,1,0,0] | [3,3,2,2,1,1,0,0] |
| warp 放大 (@c=0.5) | ~30× | **~54×**（链更深，需 $c_{\min}$ 更高） |

## 4. 可控性分析

对 x 轴四阶链 $[p, v^{\mathrm{cmd}}, \omega, v^{\mathrm{real}}]$，可控性矩阵 $\mathcal{C}_x = [B_x\; AB_x\; A^2B_x\; A^3B_x]$：

$$B_x = \begin{bmatrix} 0 \\ m^{-1} \\ 0 \\ 0 \end{bmatrix},\quad
AB_x = \begin{bmatrix} 0 \\ 0 \\ m^{-1} \\ -(\tau m)^{-1} \end{bmatrix},\quad
A^2B_x = \begin{bmatrix} -1/(\tau m) \\ 0 \\ -2/(T_d m) \\ \frac{4}{\tau T_d m} + \frac{1}{\tau^2 m} \end{bmatrix},\quad \ldots$$

秩为 4，可控。`trans_con_nd` 的 SVD 秩检验通过，块尺寸 $\mathbf{nt} = [4, 4]$。

**齐次性近似**：$A$ 含特征值 $-2/T_d$ 和 $-\tau^{-1}$（两个非幂零项），需 $K_0$ 零化底部行。与 6D Motor 同级近似，两个耗散项均为负实部（能量衰减），闭环行为偏安全侧。

## 5. 四阶自适应极点配置

x 轴子系统（y 轴同构）：

$$\frac{d}{dt}\begin{bmatrix}p \\ v^{\mathrm{cmd}} \\ \omega \\ v^{\mathrm{real}}\end{bmatrix} =
\begin{bmatrix}0 & 0 & 0 & 1 \\ 0 & 0 & 0 & 0 \\ 0 & 1 & -2/T_d & 0 \\ 0 & -\tau^{-1} & 4/(\tau T_d) & -\tau^{-1}\end{bmatrix}
\begin{bmatrix}p \\ v^{\mathrm{cmd}} \\ \omega \\ v^{\mathrm{real}}\end{bmatrix} +
\begin{bmatrix}0 \\ m^{-1} \\ 0 \\ 0\end{bmatrix} u\tag{7}$$

反馈增益 $u = [k_1, k_2, k_3, k_4] \cdot [p, v^{\mathrm{cmd}}, \omega, v^{\mathrm{real}}]^{\mathsf{T}}$。

### 5.1 解析解推导路线

闭环特征多项式为 4 阶。对 $(s + \lambda)^4$ 配置：

$$(s + \lambda)^4 = s^4 + 4\lambda s^3 + 6\lambda^2 s^2 + 4\lambda^3 s + \lambda^4$$

将反馈 $K_x$ 代入从 $(A_x, B_x)$ 导出的特征多项式，逐项匹配，解出 $(k_1, k_2, k_3, k_4)$ 关于 $(m, \tau, T_d, \lambda)$ 的闭式表达。4 个方程、4 个未知数，解析可解（推荐用 sympy 符号推导，导出 C++ 代码）。

$\lambda = a/m$ 的自适应逻辑与 6D Motor 完全相同，$e_v$ 取 $v^{\mathrm{real}}$ 误差分量。

### 5.2 全矩阵 K 结构

x/y 解耦，$K \in \mathbb{R}^{2 \times 8}$：

$$K = \begin{bmatrix}
k_{1,x} & 0 & k_{2,x} & 0 & k_{3,x} & 0 & k_{4,x} & 0 \\
0 & k_{1,y} & 0 & k_{2,y} & 0 & k_{3,y} & 0 & k_{4,y}
\end{bmatrix}\tag{8}$$

## 6. 齐次控制升级

8D 的 LPC→HPC 升级与 6D Motor 完全相同——仅将矩阵尺寸从 $6 \times 6$ 换为 $8 \times 8$：

$$\boxed{\mathbf{u} = \|\mathbf{e}\|_d^{1+\mu} K \, \exp(-G_d \ln\|\mathbf{e}\|_d) \, \mathbf{e}}\tag{9}$$

其中 `lpc2hpc_nd(A_8x8, B_8x2, K_2x8)` → `hnorm_nd(e_8x1, Gd_8x8, P_8x8)`。

**关键差异**：8D 齐次权重 $[3,3,2,2,1,1,0,0]$ 比 6D Motor $[2,2,1,1,0,0]$ 更深一级。$c_{\min}=0.5$ 时翘曲放大 ~54×（6D 为 ~30×，4D 为 ~5×）。因此 $c_{\min}$ 的默认值可能需要从 0.9 进一步提高到 0.95，或在论文中明确讨论"链深度-翘曲-噪声"三者的 scaling law 作为理论贡献。

## 7. 参数汇总

| 参数 | 默认值 | 含义 |
|------|--------|------|
| $m$ | 2.0 | 控制力→加速度增益 |
| $\tau$ | 0.43 | 电机爬段时间常数 |
| $T_d$ | 0.22 | 死区时延 (实测值) |
| $\omega_d$ | 0.7 | 期望闭环带宽 |
| $c_{\min}$ | 0.9 (建议 0.95) | HPC warp clamp 下界 |
| $h$ | 0.05 | 控制周期 |

**关于 τ 的实物变化**：实测等效 τ 随 $|v^{\mathrm{cmd}}|$ 从 ~244ms（小指令）变到 ~580ms（大指令），这是加速度限幅的宏观表现——8D Pade 只建模了死区，未建模加速度限幅。因此建议将 6D Motor 的自适应 τ 直接叠加到 8D 上：修改 A 中 row 6/7（0-indexed，即第 7/8 行）含 $1/\tau$ 的 6 个元素（与 6D Motor 的 `update_A_tau()` 同模式）。自适应 τ 和 Pade Td 互不干扰——Td 是 Pade ω 状态的事，τ 是 v_real 行的事，在 A 矩阵中分属不同行块。

两者在 A 中的位置（0-indexed 行号，row 6 = $\dot{v}_x^{\mathrm{real}}$ 行，row 7 = $\dot{v}_y^{\mathrm{real}}$ 行；对应 1-indexed 的第 7/8 行）：

$$\dot{v}_x^{\mathrm{real}} = \underbrace{-\frac{1}{\tau} v_x^{\mathrm{cmd}}}_{\tau\text{ 项}} + \underbrace{\frac{4}{\tau T_d} \omega_x}_{\tau,\,T_d\text{ 耦合项}} \underbrace{-\frac{1}{\tau} v_x^{\mathrm{real}}}_{\tau\text{ 项}}$$

$\tau$ 变化时只需更新第 6/7 行（0-indexed，即 A 矩阵第 7/8 行）中含 $1/\tau$ 的 6 个元素：
$A(6,2) = -1/\tau$（$v_x^{\mathrm{cmd}}$ 系数）、$A(6,4) = 4/(\tau T_d)$（$\omega_x$ 系数）、$A(6,6) = -1/\tau$（$v_x^{\mathrm{real}}$ 系数）、
$A(7,3) = -1/\tau$（$v_y^{\mathrm{cmd}}$ 系数）、$A(7,5) = 4/(\tau T_d)$（$\omega_y$ 系数）、$A(7,7) = -1/\tau$（$v_y^{\mathrm{real}}$ 系数）。
$T_d$ 相关项（含 $2/T_d$ 的 $\omega$ 行）保持不变。

## 8. 与 6D Motor 的关系及 Smith 的去留

8D Pade 模型的目标是**将死区从外挂补偿（Smith）升级为内部模型状态**。一旦 8D 跑通并达到预期效果，Smith 预估器应被移除：

| 组件 | 6D Motor（当前） | 8D Pade（目标） |
|------|------|------|
| 死区 | Smith 外挂（comp_vx 加到 v_real） | Pade 状态 $\omega$ 内嵌在 A 中 |
| 自适应 τ | 需要 | 需要（叠加到 8D A 矩阵，Td 与 τ 为独立物理效应） |
| v_cmd 管线 | 内部积分 + 回写 | 同 |
| min_cmd_vel | 需要（STM32 死区） | 需要（物理死区无法建模） |

过渡期可将 Smith 作为对照实验的消融组（ablation study）：8D 无 Smith vs 6D 有 Smith，看死区是否真的被 Pade 内化了。

## 9. 实现计划

### 9.1 文件清单

| 文件 | 内容 |
|------|------|
| `homo_controller_8d_motor.hpp`（新建） | `LpcController8DMotor`：A(8×8), B(8×2)，编队点逻辑（离散多边形+tol），四阶 `compute_channel_4th` |
| `formation_control_node_8d_motor.cpp/.hpp`（新建） | 节点：TD+EKF 管线，v_cmd 积分+回写，偏航独立 |
| `main_8d_motor.cpp`（新建） | 入口 |
| `formation_single_follower_8d_motor.launch.py`（新建） | Launch |
| `CMakeLists.txt` | 新增 target |
| `doc/pade_deadtime_full.md` | （本文档）完整数学推导 |

### 9.2 可复用组件

- `types_nd.hpp` / `lpc2hpc_nd.hpp` / `hnorm_nd.hpp` — 全量复用，换 8×8 尺寸即可
- `kinematic_constraint.hpp` — 无需改动
- `formation_control_node_6d_motor.cpp` 的 TF+EKF 管线、v_cmd 回写、偏航控制 — 照搬
- `motor_predictor.hpp` — 不接入（8D 不吃外挂）

### 9.3 核心工作量

1. **四阶极点配置解析推导**（~2 小时）：sympy 符号解 $(k_1,k_2,k_3,k_4)$，导出 C++ 代码 `compute_channel_4th(e_p, e_v, m, τ, Td, ωd)`。
2. **控制器类 `LpcController8DMotor`**（~1 小时）：照搬 6D Motor 结构，换维度 + 新 klin + 新 A/B。**不需要 Smith 相关代码**。
3. **节点 + launch**（~30 分钟）：照搬 6D Motor 节点，去掉 Smith。
4. **仿真验证**（~1 天）：$T_d$ 扫 0.1–0.3，$\omega_d$ 扫 0.5–1.0，与 6D Motor + Smith 对比。
5. **实物验证**（~1 天）：与 6D Motor + Smith 对比编队距离 σ 和 overshoot 次数。

### 9.4 风险与缓解

| 风险 | 缓解 |
|------|------|
| 四阶极点配置无闭式解（符号表达式过于庞大） | 用数值求解替代 sympy：每周期 $A_x, B_x$ 构建后直接 `place()` 或手动配平特征多项式 |
| 8D $c_{\min}=0.95$ 时翘曲过小、HPC 效果不明显 | 与 LPC-only 消融对比，量化 HPC 剩余增益 |
| $T_d$ 的实物标定误差较大 | $T_d$ 作为 launch 参数，scan $T_d = 0.15–0.30$ 找最优 |
| 实车 ARM 编译 8D 可能 OOM | 8D 矩阵复杂度与 6D 同级（8×8 expm vs 6×6 expm），差异不大；若仍不够，降 Eigen 优化或只编译 8D target |

## 10. 创新点论述（论文用）

1. **首次将执行器纯死区通过 Pade(1,1) 近似显式增广为齐次状态维度**，使 HPC 的误差翘曲和增益调度统一感知"指令→死区→爬升→到位"的完整物理链路。
2. **揭示了齐次链深度推广中的 scaling law**：8D 链权重 $[3,2,1,0]$ 在 $c_{\min}=0.5$ 时翘曲 ~54×，翘曲随链深度单调递增（4D ~5× → 6D ~30× → 8D ~54×），并提出 $c_{\min}$ 调节作为通用稳定化策略。
3. **将 6D Motor 模型（纯爬升）和 8D Pade 模型（爬升+死区）统一在同一个数学框架下**，$T_d \to 0$ 时 8D 连续退化为 6D，为全向底盘执行器延迟的齐次建模建立了完整的层级理论。
