# Artstein 模型约简：输入时延系统的等价无时延齐次控制

## 1. 动机

6D Motor 模型通过一阶滞后 $\dot{v}^{\mathrm{real}} = (v^{\mathrm{cmd}} - v^{\mathrm{real}})/\tau$ 显式建模了电机爬升段。实物存在额外的 ~220ms 纯死区（指令发出到轮子响应的空窗期）——这是输入时延 $T_d$，不能由一阶滞后表达。目前通过 Smith 预估器（`motor_predictor.hpp`，$\tau + T_d$ 双模型）在反馈通道上外挂补偿。

本文采用 **Artstein 模型约简**——将输入时延系统 $\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}(t-T_d)$ 通过状态预测变换**精确等价**为无时延系统 $\dot{\mathbf{z}} = A\mathbf{z} + B_{\mathrm{eff}}\,\mathbf{u}(t)$，其中 $B_{\mathrm{eff}} = e^{-A T_d}B$。$A$ 矩阵完全不变，齐次控制器内在结构零改动，死区补偿从外挂升级为模型内嵌。

与此前评估过的 Pade 有理近似方案对比：

| | Pade | Artstein |
|------|------|------|
| 死区表达 | 近似（有理逼近，Pade(1,1) 含 undershoot） | **精确**（等价变换，无近似误差） |
| 维度 | 8D（每轴 +1） | **6D（不增维）** |
| 齐次链 | 加深 → 翘曲 54× → 需调高 $c_{\min}$ | 不变 → 翘曲 30× → $c_{\min}$ 复用 |
| 极点配置 | 重新推导四阶 klin | **复用三阶 klin（定理 1）** |
| 自适应 τ | 需叠加 | **直接兼容** |
| 结论 | 被考虑但放弃 | **采用** |

## 2. Artstein-Kwon-Pearson 变换

### 2.1 标准形式

**定理（Artstein, 1982；Kwon & Pearson, 1980）**：对线性时不变系统 $\dot{\mathbf{x}}(t) = A\mathbf{x}(t) + B\mathbf{u}(t - T_d)$，定义 Artstein 状态变换：

$$\boxed{\mathbf{z}(t) = \mathbf{x}(t) + \int_{t-T_d}^{t} e^{A(t-s-T_d)}\,B\,\mathbf{u}(s)\,\mathrm{d}s}\tag{1}$$

则 $\mathbf{z}(t)$ 满足**无时延**的 ODE：

$$\boxed{\dot{\mathbf{z}}(t) = A\,\mathbf{z}(t) + \underbrace{e^{-A T_d}B}_{B_{\mathrm{eff}}}\,\mathbf{u}(t)}\tag{2}$$

**物理含义**：$\mathbf{z}(t)$ 是 "$T_d$ 秒前发出的指令若瞬间到位，当前状态应该是什么"的精确预测。积分项 $\int_{t-T_d}^{t} e^{A(t-s-T_d)}\,B\,\mathbf{u}(s)\,\mathrm{d}s$ 是过去 $T_d$ 秒内所有未到期指令对当前状态的累积贡献。

**证明**（概要）：对 (1) 求导，利用 Leibniz 积分规则和 $\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}(t-T_d)$，消去含 $\mathbf{x}$ 和积分的项即得 (2)。完整的 $A, B$ 替换为 $(A, e^{-A T_d}B)$ 后系统**严格无时延**。

### 2.2 应用于 6D Motor 模型

6D Motor 的 $(A, B)$ 为：

$$A = \begin{bmatrix}
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & \tau^{-1} & 0 & -\tau^{-1} & 0 \\
0 & 0 & 0 & \tau^{-1} & 0 & -\tau^{-1}
\end{bmatrix}, \quad
B = \begin{bmatrix}
0 & 0 \\ 0 & 0 \\ m^{-1} & 0 \\ 0 & m^{-1} \\ 0 & 0 \\ 0 & 0
\end{bmatrix}\tag{3}$$

有效输入矩阵 $B_{\mathrm{eff}} = e^{-A T_d}B$。$A$ 是分块对角（x/y 解耦），`expm(-A*Td)` 可由 Eigen 的矩阵指数计算（一次性，构造时完成）。

**关键性质**：$e^{-A T_d}B$ 的前两行（位置通道的行）非零——这意味着 Artstein 变换自动将"$T_d$ 前发出的指令对当前位置的未到期贡献"编码进了 $B_{\mathrm{eff}}$。Smith 预估器只补偿 $v^{\mathrm{real}}$ 的速度分量，Artstein 同时补偿**位置与速度全状态**，是对死区的完整状态空间表达。

### 2.3 积分项的离散实现

$T_d = 0.22$ s，控制周期 $dt = 0.05$ s → 缓冲 $N = \lceil T_d/dt \rceil = 5$ 个周期的 $\mathbf{u}$ 历史。每周期：

1. 存入当前 $\mathbf{u}$（发布后的 body 系 cmd_vel 旋转到 map 系）
2. 计算积分项：

$$\mathbf{I}(t) = \int_{t-T_d}^{t} e^{A(t-s-T_d)}\,B\,\mathbf{u}(s)\,\mathrm{d}s \approx \sum_{k=0}^{N-1} e^{A(k\,dt - T_d)}\,B\,\mathbf{u}(t - k\,dt)\,dt\tag{4}$$

3. 构造预测状态：$\mathbf{z}(t) = \mathbf{x}(t) + \mathbf{I}(t)$
4. 将 $\mathbf{z}_1, \mathbf{z}_2$（leader 和 follower 的 Artstein 状态）送入 `lpc_calculate`

矩阵 $e^{A(k\,dt - T_d)}B$（$k = 0, \ldots, N-1$）在构造时一次性预计算，每周期只需 $N$ 次矩阵-向量乘加。

## 3. 控制管线

### 3.1 改动范围

| 改动点 | 内容 |
|------|------|
| 控制器构造 | 加 `Td` 参数，预计算 $B_{\mathrm{eff}} = e^{-A T_d}B$ 和 $N$ 个积分核矩阵 |
| `lpc_calculate` | 不改——仍接收 $(\mathbf{z}_1, \mathbf{z}_2)$，内部 $A, K, \mathrm{HPC}$ 全不变 |
| 节点 `timer_cb` | (1) 维护 $\mathbf{u}$ 历史环形缓冲；(2) 算积分项 $\mathbf{I}(t)$；(3) $\mathbf{z} = \mathbf{x} + \mathbf{I}$ 送入控制器。**Smith 相关代码删除** |
| 构造参数 | `Td`（默认 0.22），无 `use_smith_predictor`/`smith_tau`/`smith_Td` |

### 3.2 数据流

```
 EKF + TF → x (6D 测量状态)
              │
   u 历史缓冲 (环形, N=5) → 积分项 I(t)
              │               │
              └───────┬───────┘
                      ↓
              z = x + I  (Artstein 预测状态)
                      ↓
              LpcController6DMotor::lpc_calculate(z1, z2)
              (内部: A 不变, B_eff = exp(-A*Td)*B)
                      ↓
              goal_v_cmd → 旋转 → clamp → 约束 → cmd_vel 发布
                      ↓
              u = 最终 body 系 cmd 旋转回 map 系 → 存入历史缓冲
              v_cmd 回写（同 6D Motor）
```

### 3.3 与 Smith 方案的数学对应

Smith 的 `comp_vx, comp_vy` 本质是 Artstein 积分项中 $v^{\mathrm{real}}$ 通道的**一阶近似**：

- Smith: 用一阶低通 $\tau$ + 纯延迟 $T_d$ 两模型的速度差补偿
- Artstein: 用 $e^{A(\cdot)}B$ 的全状态卷积积分——自动包含位置、指令、实际速度三个通道的预测修正

Smith 只看到 $v^{\mathrm{real}}$ 的延迟，Artstein 看到**整个 6D 状态的延迟**——包括 $v^{\mathrm{cmd}}$ 通道的内部指令记忆和位置通道的位移累积。

## 4. $B_{\mathrm{eff}}$ 的计算

Eigen 可直接算。`B_eff = (-A * Td).exp() * B`（不需要 C++ 代码改动，仅一行 `Eigen::MatrixXd` 操作）。逻辑放在 `controller_initial()` 和 `check_and_switch_target()` 中（切换编队点后重算 HPC 时同步更新 $B_{\mathrm{eff}}$）。

## 5. 与自适应 τ 的兼容性

Artstein 变换是对 $(A, B)$ 系统的等价变换，$\tau$ 的变化影响 $A$ 矩阵。当自适应 τ 更新 $A$ 时：

1. `update_A_tau()` 修改 $A$ 中含 $1/\tau$ 的项（同 6D Motor）
2. 重算 $B_{\mathrm{eff}} = e^{-A T_d}B$
3. 更新积分核矩阵 $e^{A(k\,dt - T_d)}B$

τ 的变化频率由 $|v^{\mathrm{cmd}}|$ 的演化速度决定（秒级），远低于控制频率（50ms），重算开销可忽略。

## 6. 参数汇总

| 参数 | 默认值 | 含义 |
|------|--------|------|
| $m$ | 2.0 | 控制力→加速度增益 |
| $\tau$ | 0.43 | 电机爬段时间常数（可为自适应 τ 的基准值） |
| $T_d$ | 0.22 | 死区时延（实物实测值） |
| $\tau_{\min}$ | 0.25 | 自适应 τ 下限 |
| $\tau_{\max}$ | 0.55 | 自适应 τ 上限 |
| $v_{\tau,\mathrm{trans}}$ | 0.10 | 自适应 τ 过渡速度 |
| $\omega_d$ | 0.7 | 期望闭环带宽 |
| $c_{\min}$ | 0.9 | HPC warp clamp 下界 |
| $h$ | 0.05 | 控制周期 |

## 7. 创新点论述（论文用）

1. **首次将 Artstein 模型约简应用于全向移动机器人编队控制的输入时延问题**——将死区从外挂补偿（Smith 预估器）升级为状态空间的等价无时延变换，$A$ 矩阵和齐次控制器内部结构**零改动**。
2. **揭示了 Artstein 积分项与 Smith 补偿量的数学关系**：Smith 的 `comp_vx/comp_vy` 是 Artstein 全状态卷积积分中 $v^{\mathrm{real}}$ 通道的一阶近似，Artstein 额外补偿了位置通道的未到期指令贡献。
3. **提出 $B_{\mathrm{eff}} = e^{-A T_d}B$ 作为输入时延齐次控制的统一等效输入矩阵**——推导简洁（一行 expm），不需要 Pade 近似引入的额外状态维度和齐次链加深。
4. **与 6D Motor 模型（爬升段建模）和自适应 τ（加速度限幅自适应）构成三层执行器动力学建模的完整体系**：爬升（$\tau$）+ 死区（$T_d$）+ 加速度限幅（自适应 τ）。

## 8. 实现计划

### 8.1 文件

| 文件 | 改动 |
|------|------|
| `homo_controller_6d_motor.hpp` | +`Td_`, +`B_eff_`, +积分核矩阵预计算，+`compute_artstein_integral()`，**删除 Smith 相关** |
| `formation_control_node_6d_motor.cpp` | u 历史缓冲 → Artstein 积分 → $\mathbf{z}$ 送入控制器；**删除 Smith 参数和代码** |
| `formation_control_node_6d_motor.hpp` | 删除 Smith 成员，+`Td_` 参数 |
| `launch/formation_single_follower_6d_motor.launch.py` | 删除 `use_smith_predictor/smith_tau/smith_Td`，+`Td`（默认 0.22） |
| `doc/artstein_reduction.md` | （本文档）完整推导 |

### 8.2 工作量

| 步骤 | 时间 |
|------|------|
| 控制器加 Artstein 积分 + $B_{\mathrm{eff}}$ | ~1h |
| 节点改 u 历史缓冲 + 删除 Smith | ~30min |
| launch + 编译 | ~15min |
| 仿真验证（$T_d$ 扫 0.15–0.30） | ~半天 |
| 实物验证 | ~半天 |

## 参考文献

- Z. Artstein, "Linear systems with delayed controls: A reduction," *IEEE Trans. Autom. Control*, 27(4): 869–879, 1982.
- W. H. Kwon and A. E. Pearson, "Feedback stabilization of linear systems with delayed control," *IEEE Trans. Autom. Control*, 25(2): 266–269, 1980.
- M. Krstic, *Delay Compensation for Nonlinear, Adaptive, and PDE Systems*, Birkhäuser, 2009. (Chapter 2: Artstein reduction for LTI systems)
