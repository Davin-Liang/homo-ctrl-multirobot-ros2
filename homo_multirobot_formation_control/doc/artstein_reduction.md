# Artstein 模型约简：输入时延系统的等价无时延齐次控制

## 1. 问题与动机

### 1.1 物理事实

6D Motor 模型通过一阶滞后 $\dot{v}^{\mathrm{real}} = (v^{\mathrm{cmd}} - v^{\mathrm{real}})/\tau$ 建模电机爬升段。该模型假定等式右边的 $v^{\mathrm{cmd}}$ 和 $v^{\mathrm{real}}$ 是**同一时刻**的量——前者是控制指令，后者是该指令驱动下的实际速度，二者通过 $\tau$ 滞后建立因果。

实物存在 ~220ms 纯死区 $T_d$：控制器以 20 Hz 持续发出 $v^{\mathrm{cmd}}$，但每个指令要等 $T_d$ 后才开始被电机执行。因此在任意时刻 $t$，**正在驱动 $v^{\mathrm{real}}$ 的是 $T_d$ 前发出的那个指令**——管道里同时有 4~5 个更晚发出的指令在排队，但都还没出死区窗口：

$$\dot{v}^{\mathrm{real}}(t) = \frac{v^{\mathrm{cmd}}(t-T_d) - v^{\mathrm{real}}(t)}{\tau}$$

等式右边是 $v^{\mathrm{cmd}}(t-T_d)$（220ms 前的指令），不是 $v^{\mathrm{cmd}}(t)$（控制器刚发出的新指令）。死区不是"每隔 $T_d$ 才收一次指令"，而是**每个指令都要排队 $T_d$**。

物理时间线上：

```
t−Td:  发出 v_cmd(t−Td)
         ↓  220ms 死区
t:     v_cmd(t−Td) 开始驱动 v_real
       控制器发出 v_cmd(t)（马上进入死区，Td 后才生效）
       模型 ez 右边混入了两个时刻的量
```

**6D 电机模型假定 $v^{\mathrm{cmd}}$ 和 $v^{\mathrm{real}}$ 同步——前者驱动后者。但死区使驱动关系变成 $v^{\mathrm{cmd}}(t-T_d) \to v^{\mathrm{real}}(t)$，中间差了 $T_d$。把不同步的量放入同一个状态向量，模型的基本等式不成立。**

### 1.2 不处理的后果

若将测到的 $v^{\mathrm{real}}(t)$（由 $v^{\mathrm{cmd}}(t-T_d)$ 驱动）和当前的 $v^{\mathrm{cmd}}(t)$ 一起送入模型：

$$\dot{v}^{\mathrm{real}} \approx \frac{v^{\mathrm{cmd}}(t) - v^{\mathrm{real}}(t)}{\tau}$$

等式左边的 $\dot{v}^{\mathrm{real}}$ 实际上由 $v^{\mathrm{cmd}}(t-T_d)$ 决定，右边代入的却是 $v^{\mathrm{cmd}}(t)$——驱动源被替换成了未来值。控制器看到"误差很大"→ 加大 $v^{\mathrm{cmd}}(t)$ → $T_d$ 后旧指令才追上 → 但此时 $v^{\mathrm{cmd}}$ 已变 → 新一轮"误差很大"。**不是参数问题，是模型中等式两边的变量在时间上不匹配。**

### 1.3 解决思路

Artstein 模型约简（Artstein, 1982；Kwon & Pearson, 1980）：对输入时延系统 $\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}(t-T_d)$，构造预测状态变换：

$$\mathbf{z}(t) = \mathbf{x}(t) + \int_{t-T_d}^{t} e^{A(t-s-T_d)} B \mathbf{u}(s) ds$$

$\mathbf{z}(t)$ 满足无时延 ODE：$\dot{\mathbf{z}} = A\mathbf{z} + B_{\mathrm{eff}}\mathbf{u}(t)$，其中 $B_{\mathrm{eff}} = e^{-A T_d}B$。

**物理直觉**：积分项是"过去 $T_d$ 秒内所有已发出但尚未变现的指令对状态的累积贡献"。加上它之后，$\mathbf{z}(t)$ 等价于"如果 220ms 前的指令瞬间到位，当前状态应该是什么"。此时 $\mathbf{z}$ 和 $\mathbf{u}(t)$ 在预测空间中处于同一时刻——模型等式恢复成立。

Artstein 没有消除物理延迟（$T_d$ 仍在执行器链路中）。它做的是构造一个**预测空间**，让控制器在这个空间中决策——物理状态 $\mathbf{x}$ 经 $T_d$ 后追上预测状态 $\mathbf{z}$。

### 1.4 与输出端延迟补偿方法的区别

典型的工程补偿方法（如 Smith 预估器）在输出端对延迟效应做修正——利用模型预测无延迟输出与有延迟输出的差值，叠加到实测信号上。这类方法将延迟视为"输出需要修正的量"，补偿效果依赖于所选输出通道。

Artstein 约简采取不同的路径：不是修正输出，而是**变换状态空间**。它将整个含时延的状态向量 $\mathbf{x}$ 映射为等价无时延的预测状态 $\mathbf{z}$，使延迟被吸收进变换本身。因此 $\mathbf{z}$ 自然保留了所有状态分量（位置、速度等）在延迟期间的累积信息，而不限于预先选定的输出通道。这一性质对于需要完整状态反馈的齐次控制器设计至关重要——HPC 的齐次投影和 Lyapunov 稳定性分析都在 $\mathbf{z}$-空间中进行，不因延迟的存在而需修改。

## 2. 架构：4D Artstein 预测 + 6D Motor HPC 级联

### 2.1 为什么不是纯 4D Artstein-HPC

最初尝试将 Artstein 约简和 HPC 统一在一个 4D 模型（状态 $[p, v^{\mathrm{real}}]$，输入 $v^{\mathrm{cmd}}$）中。这要求 HPC 的齐次投影在 4D 系统 $(A_4, B_{\mathrm{eff}})$ 上进行。

$A_4$ 含电机滞后的阻尼项 $-1/\tau$：

$$A_4 = \begin{bmatrix} 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 0 & -1/\tau & 0 \\ 0 & 0 & 0 & -1/\tau \end{bmatrix}$$

$-1/\tau$ 使 $v^{\mathrm{real}}$ 成为"带阻尼的积分器"而非纯积分器。HPC 的齐次加权结构（`lpc2hpc_nd` 中的块可控分解）天然适配纯积分器链——每个积分器贡献一级齐次权重，形成 $[2,1,0]$ 的三层梯度。$-1/\tau$ 阻尼项破坏了严格积分链结构，导致块分解将 $v^{\mathrm{real}}$ 的齐次权重退化为 0，齐次度 $\nu = -1$，warp 缩放因子 $c^{1+\nu} = c^0 = 1$ 恒等——齐次翘曲完全失效。实验证实纯 4D HPC 在编队点附近持续大幅震荡。

### 2.2 级联架构

将 Artstein 约简和 HPC 分离到各自的优势空间中：

```
┌──────────────────────────────────────────────────┐
│ 4D Artstein 层 — 死区时间对齐                       │
│                                                    │
│ 模型: [p, v_real] (4D), 输入 v_cmd (2D)             │
│ A_4, B_4 含 τ 阻尼 → 准确建模执行器物理               │
│                                                    │
│ v_cmd 历史缓冲 → 积分 I(t) → z_4D = x_4D + I        │
│                                                    │
│ 输出: z_p (预测位置), z_vreal (预测实际速度)           │
│ → z 和 v_cmd(t) 在预测空间中处于同一时刻               │
└────────────────────┬─────────────────────────────┘
                     │ 嵌入 6D 状态
                     ↓
┌──────────────────────────────────────────────────┐
│ 6D Motor HPC 层 — 齐次控制                           │
│                                                    │
│ 状态: [z_p, v_cmd, z_vreal] (6D)                    │
│ A_6 含 v_cmd 积分态 → 幂零 → 齐次权重 [2,1,0]         │
│                                                    │
│ 三阶极点配置 + HPC warp → 齐次控制律                   │
│ A_6, B_6, K, G0, P, ν, Gd — 完全不变                │
└──────────────────────────────────────────────────┘
```

**关键**：4D 层处理时间对齐（Artstein 的职责），6D 层处理齐次控制（HPC 的职责）。两层各用各的 $A$ 矩阵——Artstein 用物理准确的 $A_4$（含 $\tau$），HPC 用幂零的 $A_6$（含 $v^{\mathrm{cmd}}$ 积分态）。不互相破坏。

### 2.3 $B_{\mathrm{eff}}$ 的级数展开与多通道补偿

$B_{\mathrm{eff}}^{(4)} = e^{-A_4 T_d} B_4$ 的级数展开揭示了 Artstein 变换如何将延迟期间输入对各状态分量的累积影响编码进有效输入矩阵：

$$B_{\mathrm{eff}} = B_4 - T_d A_4 B_4 + \frac{T_d^2}{2} A_4^2 B_4 - \cdots$$

$$A_4 B_4 = \begin{bmatrix} 0 & 0 \\ 0 & 0 \\ -1/\tau^2 & 0 \\ 0 & -1/\tau^2 \end{bmatrix}, \quad
A_4^2 B_4 = \begin{bmatrix} -1/\tau^2 & 0 \\ 0 & -1/\tau^2 \\ 1/\tau^3 & 0 \\ 0 & 1/\tau^3 \end{bmatrix}$$

- **一阶项** $-T_d A_4 B_4$：修正 $v^{\mathrm{real}}$ 通道增益——延迟使有效驱动力减弱
- **二阶项** $\frac{T_d^2}{2} A_4^2 B_4$：使**位置通道获得非零增益**——$T_d$ 内发出的 $v^{\mathrm{cmd}}$ 对当前位置的未到期位移累积贡献

与仅针对输出响应的延迟补偿方法相比，Artstein 变换通过 $B_{\mathrm{eff}}$ 的状态空间表达自然保留了延迟对位置和速度通道的耦合影响。对于需要完整状态反馈的 HPC 设计，这意味着预测状态 $\mathbf{z}$ 的所有分量——而非仅是选定的输出通道——都经过了延迟补偿。

## 3. 4D Artstein 预测层

### 3.1 系统矩阵

执行器模型（map 系，x/y 解耦）：状态 $\mathbf{x} = [p_x, p_y, v_x^{\mathrm{real}}, v_y^{\mathrm{real}}]^\top$，输入 $\mathbf{u} = [v_x^{\mathrm{cmd}}, v_y^{\mathrm{cmd}}]^\top$。

$$\boxed{A_4 = \begin{bmatrix}
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
0 & 0 & -\tau^{-1} & 0 \\
0 & 0 & 0 & -\tau^{-1}
\end{bmatrix}, \quad
B_4 = \begin{bmatrix}
0 & 0 \\ 0 & 0 \\ \tau^{-1} & 0 \\ 0 & \tau^{-1}
\end{bmatrix}}$$

动力学：$\dot{p} = v^{\mathrm{real}}$, $\dot{v}^{\mathrm{real}} = (v^{\mathrm{cmd}}(t-T_d) - v^{\mathrm{real}})/\tau$。

### 3.2 积分离散化

控制频率 20 Hz → $h = 0.05$ s。实物 STM32 固件同样以 20 Hz 更新 cmd_vel，因此 $v^{\mathrm{cmd}}(t)$ 在连续两个控制周期间确实保持恒定——Riemann 和的零阶保持假设与物理实际一致，不是纯数值近似。

$T_d = 0.22$ s，$T_d/h = 4.4$ 非整数，最旧样本使用截断权重：

$$N = \lceil T_d/h \rceil = 5, \quad w_k = \begin{cases} h & k = 0, \ldots, N-2 \\ T_d - (N-1)h & k = N-1 \end{cases}$$

即 $w_0 = w_1 = w_2 = w_3 = 0.05$, $w_4 = 0.02$。

$$\boxed{\mathbf{I}(t) = \int_{t-T_d}^{t} e^{A_4(t-s-T_d)} B_4 \mathbf{v}^{\mathrm{cmd}}(s) ds
\approx \sum_{k=0}^{N-1} e^{A_4(kh - T_d)} B_4 \mathbf{v}^{\mathrm{cmd}}(t - kh) w_k}$$

矩阵 $e^{A_4(kh - T_d)} B_4$（$k = 0, \dots, N-1$，每个 $4 \times 2$）在构造时预计算。

### 3.3 数据流

每周期（~20 Hz）：

```
1. EKF/TF → leader, follower 的 4D 测量 [p_meas, v_real_meas]

2. Leader Artstein:
     vcmd_buffer_leader (存 leader 测速, 稳态 v_cmd ≈ v_real)
     → I1 → z1_4D = x1_4D + I1

3. Follower Artstein:
     vcmd_buffer_follower (存 v_cmd_map_, 实际发布值)
     → I2 → z2_4D = x2_4D + I2

4. 嵌入 6D 状态:
     leader:   x1 = [z1_p,   v_meas,   z1_vreal]
     follower: x2 = [z2_p,   v_cmd_map_, z2_vreal]

5. 6D HPC: lpc_calculate(x1, x2) → v_cmd_new

6. v_cmd_new → 限幅 → 发布 cmd_vel → v_cmd_map_ 回写 → 存入 follower 缓冲
```

## 4. 6D Motor HPC 层

### 4.1 为什么 6D HPC 能工作

6D Motor 模型的 $A_6$ 矩阵中，$v^{\mathrm{cmd}}$ 对应的第 3、4 行全零——**它是纯积分器，幂零**。

$$A_6 = \begin{bmatrix}
0 & 0 & \mathbf{0} & 0 & 1 & 0 \\
0 & 0 & 0 & \mathbf{0} & 0 & 1 \\
0 & 0 & \mathbf{0} & 0 & 0 & 0 \\
0 & 0 & 0 & \mathbf{0} & 0 & 0 \\
0 & 0 & 1/\tau & 0 & -1/\tau & 0 \\
0 & 0 & 0 & 1/\tau & 0 & -1/\tau
\end{bmatrix}$$

块可控分解得到每轴三层的齐次链：$p$（权重 2）→ $v^{\mathrm{real}}$（权重 1）→ $v^{\mathrm{cmd}}$（权重 0）。三层梯度使得 HPC warp 能有效运作——这是 4D Artstein 模型（仅有权重 $[1,0]$）失败的根本原因。

### 4.2 控制器不变性

级联架构中，6D HPC 控制器**零改动**：

- 系统矩阵 $(A_6, B_6)$ 不变
- 三阶极点配置 `compute_channel_3rd` 不变
- `lpc2hpc_nd(A_6, B_6, K)` 不变
- 齐次参数 $G_0, P, \nu, G_d$ 不变
- 前向欧拉积分 $\dot{v}^{\mathrm{cmd}} = u_{\mathrm{ctl}}/m$ 不变

唯一变化是**反馈信号的来源**：$[p, v^{\mathrm{real}}]$ 从 EKF 原始测量替换为 4D Artstein 预测值。从 6D 控制器的视角看，它收到的始终是"无死区的等价状态"——死区的存在被 Artstein 层完全吸收。

### 4.3 自适应 τ

电机时间常数 $\tau$ 随 $|v^{\mathrm{cmd}}|$ 变化（低速 ~0.25s，高速 ~0.55s）。每周期更新 $A_6$ 中含 $1/\tau$ 的四个项（与 6D Motor 原版相同）。Artstein 层的 $A_4, B_4$ 和积分核矩阵同步更新。

## 5. 实现

### 5.1 文件

| 文件 | 改动 |
|------|------|
| `formation_control_node_6d_motor.hpp` | +`ArtsteinPredictor4D` 结构体（~70 行，含 $A_4, B_4$、积分核预计算、截断权重），+`Td_`, +`vcmd_hist_` 环形缓冲，−Smith 相关成员 |
| `formation_control_node_6d_motor.cpp` | timer_cb 加 Artstein 积分管线：leader/follower 各维护 v_cmd 缓冲 → 积分 → z_4D → 嵌入 6D 状态 → `lpc_calculate`；−Smith 代码 |
| `homo_controller_6d_motor.hpp` | **不改** |
| `launch/formation_single_follower_6d_motor.launch.py` | −`use_smith_predictor`/`smith_tau`/`smith_Td`，+`Td`（默认 0.22） |
| `launch/formation_single_follower_4d_artstein.launch.py` | 指向 6D motor 可执行文件，带 `Td` 默认值 |

### 5.2 运行

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_motor.launch.py \
  leader_ns:=/virtual_leader follower_ns:=/robot2 \
  Td:=0.22 use_motor_delay:=true motor_tau:=0.43 transport_delay:=0.22 delay_max_accel:=0.25
```

`Td:=0.0` 关闭 Artstein → 退化为原始 6D Motor（无死区补偿），可做对照实验。

## 6. 贡献与创新点

1. **针对全向移动机器人速度指令链路中的固定输入死区，提出 Artstein 预测层与 6D Motor HPC 控制层解耦的时延补偿架构**。预测层负责将含时延的执行器状态变换到无时延等价空间，控制层在等价空间中进行齐次控制器设计。该架构将延迟补偿从输出端修正提升为状态空间变换，使延迟被吸收进预测状态而非作为外加修正量，6D 控制器无需改动。

2. **分析并解决了纯 4D Artstein-HPC 统一方案中电机阻尼项破坏齐次加权结构的问题**。带 $\tau$ 阻尼的执行器模型破坏了纯积分器链的齐次性条件，导致 HPC warp 失效。通过将 $\tau$ 阻尼保留在 4D 预测层、将 $v^{\mathrm{cmd}}$ 纯积分器保留在 6D 控制层的两层分离设计，在各自空间中满足各自的理论前提。

3. **从 $B_{\mathrm{eff}}$ 的级数展开分析了 Artstein 变换中延迟对多状态通道的耦合影响**。一阶项修正速度通道增益，二阶项使位置通道获得非零增益——延迟不仅影响执行器响应速度，还通过位置通道对编队几何产生直接影响。这一分析与仅针对输出通道的补偿方法不同，反映了状态空间变换自然保留全部状态分量延迟信息的性质。

4. **执行器时间尺度分解建模**：从控制器输出到执行器响应，依次建模指令死区（$T_d$，Artstein 约简）、电机爬升（$\tau$，一阶滞后）、加速度限幅（执行器速率约束），构成三层执行器动力学链路。

## 附录：Leibniz 求导推导

对 $\mathbf{z}(t) = \mathbf{x}(t) + \int_{t-T_d}^{t} e^{A(t-s-T_d)} B \mathbf{u}(s) ds$ 求导。

令 $f(t, s) = e^{A(t-s-T_d)} B \mathbf{u}(s)$，$a(t)=t-T_d$, $b(t)=t$。Leibniz 积分规则：

$$\frac{d}{dt}\int_{a(t)}^{b(t)} f(t,s) ds = f(t, b) \cdot b' - f(t, a) \cdot a' + \int_a^b \frac{\partial f}{\partial t} ds$$

- **上界** ($s=t$)：$f(t,t) = e^{-A T_d} B \mathbf{u}(t)$
- **下界** ($s=t-T_d$)：$f(t, t-T_d) = e^{A \cdot 0} B \mathbf{u}(t-T_d) = B \mathbf{u}(t-T_d)$
- **偏导积分**：$\frac{\partial f}{\partial t} = A e^{A(t-s-T_d)} B \mathbf{u}(s)$

代入 $\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}(t-T_d)$：

$$\dot{\mathbf{z}} = [A\mathbf{x} + B\mathbf{u}(t-T_d)] + e^{-A T_d} B \mathbf{u}(t) - B\mathbf{u}(t-T_d) + A\int_{t-T_d}^{t} e^{A(t-s-T_d)} B \mathbf{u}(s) ds$$

$B\mathbf{u}(t-T_d)$ 项抵消：

$$\dot{\mathbf{z}} = A[\mathbf{x} + \int_{t-T_d}^{t} e^{A(t-s-T_d)} B \mathbf{u}(s) ds] + e^{-A T_d} B \mathbf{u}(t) = A\mathbf{z} + B_{\mathrm{eff}}\mathbf{u}(t)$$

得证 $\dot{\mathbf{z}} = A\mathbf{z} + B_{\mathrm{eff}}\mathbf{u}(t)$，$B_{\mathrm{eff}} = e^{-A T_d} B$。

## 参考文献

- Z. Artstein, "Linear systems with delayed controls: A reduction," *IEEE Trans. Autom. Control*, 27(4): 869–879, 1982.
- W. H. Kwon and A. E. Pearson, "Feedback stabilization of linear systems with delayed control," *IEEE Trans. Autom. Control*, 25(2): 266–269, 1980.
- M. Krstic, *Delay Compensation for Nonlinear, Adaptive, and PDE Systems*, Birkhäuser, 2009.
- J. Jiang et al., "Fully distributed time-varying formation tracking control of linear multi-agent systems with input delay and disturbances," *Systems & Control Letters*, 146, 2020. — Artstein 约简在多智能体编队中的直接应用。
- Y. Li et al., "Fixed-time formation control for multi-USV systems with input delay," *Journal of Unmanned Undersea Systems*, 2025. — Artstein 约简在无人系统编队控制中的近期应用。
- S. Y. Meng et al., "Safety-critical control with input delay in dynamic environment," *arXiv:2112.08445*, 2021. — 预测反馈在机器人输入时延场景中的应用。
