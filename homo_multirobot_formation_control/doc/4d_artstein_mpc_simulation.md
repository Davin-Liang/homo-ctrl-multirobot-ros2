# 4D Artstein-MPC 数值仿真说明

## 1. 目的

`scripts/sim_4d_artstein_mpc_compare.py` 用于验证 4D Artstein-MPC 对照组的理论闭环是否合理。
它不是最终控制节点，而是离线数值实验；ROS C++ 实现见
`formation_control_node_4d_artstein_mpc`：

```text
4D Artstein-HPC:
  Artstein/执行器预测补偿层 + 4D HPC 上层控制律

4D Artstein-MPC:
  Artstein/执行器预测补偿层 + 4D linear MPC 上层控制律
```

仿真重点是保证二者共享同一预测补偿层、leader 匀速预测、follower 状态构造和离散编队目标切换逻辑，
只替换上层平移控制律。

## 2. 模型与输出语义

MPC 使用 4D 双积分模型：

```math
x=[p_x,p_y,v_x,v_y]^T,\quad u=[u_x,u_y]^T
```

```math
\dot p=v,\qquad \dot v=u/m
```

离散化使用精确 ZOH：

```math
A_d =
\begin{bmatrix}
1 & 0 & h & 0 \\
0 & 1 & 0 & h \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix},
\quad
B_d =
\begin{bmatrix}
h^2/(2m) & 0 \\
0 & h^2/(2m) \\
h/m & 0 \\
0 & h/m
\end{bmatrix}.
```

MPC 每步求得整段预测输入 `u_0 ... u_{N-1}` 和预测状态 `x_0 ... x_N` 后，
输出的是第一步预测速度：

```text
cmd_map = x_{1|0}.tail(2)
```

不是 `u_0`，也不在 MPC 外层再积分一次。`u_0` 只作为 force-like 等效输入用于预测和日志分析。

Python 脚本使用 condensed QP + 小型 ADMM 投影循环做离线求解，服务数值预研。
ROS C++ 节点使用仓库已有 `osqp_interface.hpp` 和 `ros-humble-osqp-vendor` 构建非紧凑 QP。

## 3. 运行命令

从源码仓库根目录运行：

```bash
python3 homo_multirobot_formation_control/scripts/sim_4d_artstein_mpc_compare.py
```

默认参数：

```text
dt=0.05
circle_tmax=45.0
tau=0.43
Td=0.22
mass=2.0
radius=2.0
m_p=4
max_linear_vel=0.5
max_linear_accel=0.4
mpc_horizon=30
Q=diag(40,40,1,1)
R=diag(0.02,0.02)
terminal_factor=10
mpc_max_iter=320
```

输出目录：

```text
homo_multirobot_formation_control/analysis/results/4d_artstein_mpc/
```

主要输出：

```text
circle_no_delay_hpc_vs_mpc.png
circle_delay_hpc_vs_mpc.png
circle_all_compare.png
summary_metrics.csv
timeseries_circle_compare.csv
```

## 4. 2026-08-08 数值结果

默认参数下，仿真包含五组：

```text
artstein_hpc_no_delay
artstein_mpc_no_delay
original_4d_delay
artstein_hpc_delay
artstein_mpc_delay
```

关键 summary metrics：

```text
case                   tail_mean_distance  final_distance  mean_cmd_delta  mean_solve_ms  solver_failures
artstein_hpc_no_delay  0.0141              0.0129          0.0197          0.000          0
artstein_mpc_no_delay  0.0100              0.0100          0.0020          0.998          0
original_4d_delay      0.0711              0.0788          0.0119          0.000          0
artstein_hpc_delay     0.0115              0.0119          0.0193          0.000          0
artstein_mpc_delay     0.0159              0.0159          0.0020          3.052          0
```

调参记录：

- 初始 `Q=diag(10,10,1,1), R=diag(0.05,0.05)` 时，delay 场景
  `artstein_mpc_delay` 的 `tail_mean_distance` 约为 `0.0259`。
- 将位置权重提高到 `q_px=q_py=40`，并将输入惩罚降到 `r_ux=r_uy=0.02` 后，
  delay 稳态误差降至约 `0.0159`。
- 继续降低 `R` 或提高 `Q` 可略微降低误差，但在当前 Python ADMM 原型中更容易出现
  `solver_failures` 或求解时间上升；因此默认采用 `Q=diag(40,40,1,1), R=diag(0.02,0.02),
  mpc_max_iter=320` 作为更稳的数值仿真参数。

结论：

- no-delay 下，4D Artstein-MPC 能稳定收敛到离散编队目标，尾段误差与 HPC 同量级。
- delay 下，共享 Artstein/执行器预测层后，4D Artstein-MPC 未发散，尾段误差明显小于 original 4D + delay。
- 调参后 MPC 命令仍明显更平滑，`mean_cmd_delta` 显著低于 HPC；delay 场景稳态误差仍略大于 Artstein-HPC，
  但已经从初始参数的约 `0.0259` 降到约 `0.0159`。
- 当前结果足以支持“同一 Artstein 预测层 + 4D linear MPC”可作为 4D Artstein-HPC 的公平数值对照组。
- 这一步不是最优调参结论；后续 C++/Gazebo 实现前可继续扫描 `Q/R/N/max_linear_accel`。

## 5. 验证命令

脚本配套了最小 pytest 检查：

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_sim_4d_artstein_mpc_compare.py -q
```

覆盖内容：

- ZOH 离散化矩阵符合 4D 双积分模型。
- MPC `command()` 输出等于 `x_{1|0}.tail(2)`，不是 force-like `u_0`。
- 短时 no-delay MPC 闭环能降低编队误差。
