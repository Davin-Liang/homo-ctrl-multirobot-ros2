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
