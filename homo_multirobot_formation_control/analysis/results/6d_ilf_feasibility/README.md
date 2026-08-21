# 局部 6D ILF 可行性结果

本目录记录第五章“局部 6D ILF 时滞鲁棒控制”在进入控制律合成前的离线筛选结果。

## T2：冻结模型可控性扫描

运行日期：2026-08-21

对象为 `doc/6d_ilf_delay_robust_control_proposal.md` 式 (2) 的偏差输入局部模型。
绝对命令在控制实现中应写为 `u_cmd = u_star + delta_u`；本次扫描只分析
`delta_u -> xi` 的线性对。

运行命令：

```bash
python3 homo_multirobot_formation_control/scripts/ilf_6d_feasibility.py \
  --csv homo_multirobot_formation_control/analysis/results/6d_ilf_feasibility/controllability_scan.csv
```

参数网格：

```text
vx_l, vy_l, omega_l in {-0.5, 0, 0.5}
tau_x=tau_y=tau_omega in {0.25, 0.43, 0.55} s
```

| 样本数 | 可控性秩范围 | sigma_min 范围 | 条件数范围 |
|---:|---:|---:|---:|
| 81 | 6--6 | 0.817913109945--0.970142115979 | 56.3290069571--4633.23311414 |

结论：所有采样点均满秩可控，因此允许进入 MIMO ILF/ILKF 构造条件的文献筛选与
推导阶段。最大条件数约为 `4.63e3`，不应被误写为“统一良态”；后续 LMI 可行性和
闭环仿真必须报告它们对参数的敏感性。

该结果不是稳定性、时延裕度或控制器性能证明。

## T3 的第一步：冻结零时滞 MIMO-ILF 名义基线

运行日期：2026-08-21

本次计算只复现 `doc/6d_ilf_delay_robust_control_proposal.md` 式 (5)--(7) 的连续时间、
零时滞基线：`rho=0`、`d=0`、`r=0`、固定编队目标、无饱和。物理执行器时间常数取
`tau_x=tau_y=tau_omega=0.43 s` 时，式 (6) 仍将命令还原为
`delta_u=e_v+0.43 nu`；由于名义变换精确，规范形的 ILF 综合本身不依赖这个正的
`tau` 数值。

使用 Polyakov、Efimov、Perruquetti (2016) 的 MIMO ILF 有限时间构造（Theorem 10），
对三组二阶积分链取 `mu=0.5`，并在 `trace(X)=1` 的数值归一化下求解矩阵等式。运行
命令：

```bash
python3 homo_multirobot_formation_control/scripts/ilf_6d_feasibility.py \
  --run-nominal-ilf \
  --nominal-csv homo_multirobot_formation_control/analysis/results/6d_ilf_feasibility/nominal_ilf_run.csv
```

| 指标 | 结果 |
|---|---:|
| `lambda_min(X)` | 0.0177779476967 |
| `lambda_min(XH+HX)` | 0.0424297084865 |
| Theorem 10 矩阵等式的无穷范数残差 | `5.52e-15` |
| `max Re eig(A_tilde+B_tilde K)` | -1.25 |
| 初值 `xi(0)` | `[1, -0.7, 0.3, 0, 0, 0]^T` |
| `V(0)` | 4.14025770191 |
| `V(4 s)` | 0.00120841894328 |
| `||xi(4 s)||_2` | 0.000567927972065 |

结果文件 `nominal_ilf_run.csv` 保存了 4002 个连续求解器输出点的状态、隐式 Lyapunov
根和规范形输入 `nu`。该运行验证的是：MIMO 矩阵综合、正根计算、理论
`dot V=-V^(1-mu)` 的数值实现，以及冻结零时滞模型的收敛。它**不**验证输入时滞、
采样、Leader 变化、Disc 切换、饱和、测量误差或 ROS 控制器；尤其不能把式 (7) 的
结论用于含 `delta_u(t-d(t))` 的式 (8)。下一道不可跳过的门是针对该延迟差分项建立
ILKF/Razumikhin 界，并在 DDE 中扫描延迟上界和失配。

## T3 的第二步：匹配历史扰动的 DDE 审计

运行日期：2026-08-21

这一步使用方案文档式 (10)--(13) 的精确分解：DDE 中真正被审计的不是
`nu(t-d)-nu(t)`，而是包含一阶执行器速度差的 `w_d`。先以 MIMO ILF 文献的 Theorem 15
鲁棒矩阵不等式、`mu=0.5`、`trace(X)=1` 和 `R=10^-3 I_6` 生成一个条件证书；其数值检查
为 `lambda_min(X)=0.0175195893944`、`lambda_min(XH+HX)=0.0417341248671`，LMI 左端最大
特征值为 `-2.40e-11`。

运行命令：

```bash
python3 homo_multirobot_formation_control/scripts/ilf_6d_feasibility.py \
  --run-delayed-ilf-audit \
  --delayed-audit-csv homo_multirobot_formation_control/analysis/results/6d_ilf_feasibility/delayed_ilf_audit.csv
```

固定设置为 `tau=[0.43,0.43,0.43] s`、`dt=0.001 s`、持续 `8 s`、常值初始历史
`xi(s)=[1,-0.7,0.3,0,0,0]^T`；这只是方法步进 Euler 的 N1 数值审计。

| 延迟 (s) | 末端 `||xi||_2` | 末端 `V` | 最大 `||xi||_2` | 采样 `max R_d` | 全部采样 `<1` |
|---:|---:|---:|---:|---:|:---:|
| 0.00 | 6.031e-7 | 1.628e-6 | 1.256981 | 0 | 是 |
| 0.05 | 2.376e-3 | 1.560e-2 | 1.256981 | 861.026 | 否 |
| 0.10 | 4.880e-3 | 4.743e-2 | 1.256981 | 772.016 | 否 |
| 0.15 | 3.285e-3 | 7.754e-2 | 1.256981 | 699.726 | 否 |
| 0.22 | 3.260e-2 | 1.919e-1 | 1.256981 | 623.801 | 否 |
| 0.30 | 3.858e-2 | 2.907e-1 | 1.256981 | 547.776 | 否 |

结论必须严格限于此表：零时滞与名义构造一致；所有测试的非零时滞在采样点上都**未能
满足**该 `R=10^-3 I_6` 下的 Theorem-15 充分条件。它们在这 8 秒、无饱和的单条 Euler
轨迹上没有超过初始状态范数，但这既不是稳定性证明，也不是时延裕度 `d_bar^*`，更不能
据此宣称 ILF 对 `0.22 s` 已鲁棒。下一步应寻找/推导覆盖历史项的 ILKF 或 Razumikhin
泛函；若不能闭合，应把候选 A 降为“名义 ILF + 数值鲁棒性评估”，而非写成时滞定理。
