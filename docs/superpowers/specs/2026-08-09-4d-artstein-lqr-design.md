# 4D Artstein-LQR 数值仿真设计

## 1. 目标

新增一个 **4D DARE-LQR** 数值仿真对照组，用于和现有 4D HPC 方法比较。该对照组不是新的主控制方法，而是标准线性二次调节基线：

```text
original_4d_delay:
  无预测补偿 + 原 4D HPC

artstein_hpc_delay:
  当前 Artstein 输入时延补偿 + 一阶电机前向预测 + 4D HPC

artstein_lqr_delay:
  当前 Artstein 输入时延补偿 + 一阶电机前向预测 + 4D DARE-LQR
```

第一版不把 `artstein_mpc_delay` 纳入 LQR 主对照，避免实验表格混入 MPC 变量。MPC 仍保留在现有脚本和文档中，但本设计只聚焦 HPC 与 LQR。

## 2. 理论定位

LQR 使用离散时间代数 Riccati 方程（DARE），作用对象是当前项目预测补偿层输出的等效 4D 双积分状态：

```math
x = [p_x, p_y, v_x, v_y]^T .
```

连续 4D 双积分模型为：

```math
\dot p = v,\qquad \dot v = u/m .
```

采样周期 `h=dt` 下使用精确 ZOH 离散化：

```math
A_d =
\begin{bmatrix}
1 & 0 & h & 0 \\
0 & 1 & 0 & h \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix},
\qquad
B_d =
\begin{bmatrix}
h^2/(2m) & 0 \\
0 & h^2/(2m) \\
h/m & 0 \\
0 & h/m
\end{bmatrix}.
```

对误差状态

```math
e_k = x_{2,h,k} - x_{1,h,k} - d
```

构造离散二次型代价：

```math
J=\sum_{k=0}^{\infty} e_k^T Q e_k + u_k^T R u_k .
```

DARE 为：

```math
P=A_d^TPA_d-A_d^TPB_d(R+B_d^TPB_d)^{-1}B_d^TPA_d+Q .
```

反馈矩阵为：

```math
K=(R+B_d^TPB_d)^{-1}B_d^TPA_d ,
\qquad
u_k=-K e_k .
```

这里 `u` 是与 4D HPC/MPC 一致的 force-like 等效输入，实际加速度为 `u/m`。为了和现有 HPC 速度命令生成保持同口径，LQR 输出 map 系原始速度命令：

```math
v_{cmd,raw}=v_{2,h}+h u_k/m .
```

随后仍经过现有数值仿真的速度范数限幅和加速度 slew-rate 限幅。

## 3. 延迟处理边界

本设计采用项目当前 4D Artstein 架构处理延迟：

```text
Follower 测量状态 [p, v_real]
  + 历史 cmd_vel 缓冲
  -> Artstein 输入时延补偿 Td
  -> 一阶电机响应前向预测 tau
  -> 等效 4D 双积分状态 x_h
  -> DARE-LQR
```

Leader 仍使用当前项目的一致近似：

```math
x_{1,h} =
\begin{bmatrix}
p_1 + (T_d+\tau)v_1 \\
v_1
\end{bmatrix}.
```

理论表述必须保持克制：`artstein_lqr_delay` 是“共享同一预测补偿层的离散 LQR 基线”，不是对原始输入时延与执行器滞后系统的严格最优 delayed LQR。若后续要做严格 delayed LQR，应另行构造输入队列或执行器状态增广模型，并重新定义 Riccati 系统与约束边界。

## 4. 文献依据

- R. E. Kalman, “Contributions to the Theory of Optimal Control,” 1960。用于支撑 LQ 最优调节和 Riccati 方法的基础。
- B. D. O. Anderson and J. B. Moore, *Optimal Control: Linear Quadratic Methods*, 1990。用于支撑连续/离散 LQR 与 Riccati 方程。
- Z. Artstein, “Linear Systems with Delayed Controls: A Reduction,” IEEE TAC, 1982。用于支撑先通过 reduction/predictor 处理输入延迟，再在无显式延迟系统上设计反馈。
- A. Z. Manitius and A. W. Olbrot, “Finite Spectrum Assignment Problem for Systems with Delays,” IEEE TAC, 1979。用于支撑 predictor feedback 处理输入延迟的经典路线。
- I. Karafyllis and M. Krstic, *Predictor Feedback for Delay Systems: Implementations and Approximations*, 2017。用于支撑历史输入缓冲与近似 predictor 的工程实现口径。

## 5. 数值仿真范围

复用并扩展现有共享仿真脚本：

```text
homo_multirobot_formation_control/scripts/sim_4d_artstein_mpc_compare.py
```

新增 `Lqr4D` 类，并在 `make_controller()`、`raw_velocity_command()` 中接入 `controller_kind == "lqr"`。为避免改变现有 MPC 对照脚本的默认行为，新增一个薄入口脚本：

```text
homo_multirobot_formation_control/scripts/sim_4d_artstein_lqr_compare.py
```

该入口只负责组织 LQR 实验组、写图、写 CSV。

输出目录建议：

```text
homo_multirobot_formation_control/analysis/results/4d_artstein_lqr/
```

主实验组：

```text
original_4d_delay
artstein_hpc_delay
artstein_lqr_delay
```

no-delay sanity check：

```text
artstein_hpc_no_delay
artstein_lqr_no_delay
```

默认 LQR 权重与现有 MPC 第一轮权重同口径，便于解释：

```text
lqr_q_px = 40.0
lqr_q_py = 40.0
lqr_q_vx = 1.0
lqr_q_vy = 1.0
lqr_r_ux = 0.02
lqr_r_uy = 0.02
```

## 6. 成功标准

- DARE 离散反馈矩阵能被稳定求解，闭环矩阵 `A_d - B_d K` 的特征值均在单位圆内。
- `artstein_lqr_no_delay` 在短时圆轨迹仿真中能降低编队距离误差。
- `artstein_lqr_delay` 在默认 `tau=0.43, Td=0.22` 下不发散，并输出 summary metrics 和 timeseries CSV。
- 主图只展示 `original_4d_delay`、`artstein_hpc_delay`、`artstein_lqr_delay`。
- 现有 MPC 测试不被破坏。

## 7. 明确不做

- 不实现 ROS C++ LQR 节点。
- 不实现输入队列增广或执行器状态增广 delayed LQR。
- 不把 `artstein_mpc_delay` 放入 LQR 主对照图。
- 不引入新的 QP/优化求解器。
