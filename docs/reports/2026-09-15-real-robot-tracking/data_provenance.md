# 实物跟踪实验数据溯源说明

本文档用于内部整理与复查，不作为 Word 正文或 PPT 展示内容。它记录报告图表、指标与原始实验记录之间的对应关系，确保后续修改数据或重新生成图表时可以准确定位来源。

## 图表与实验标签

| 来源代号 | 实验标签 | 用途 |
| --- | --- | --- |
| A | Artstein-HPC（Leader 命令前馈：开） | 前馈开/关对比 |
| B | Artstein-HPC（Leader 命令前馈：关） | 前馈开/关对比 |
| C | Artstein-LPC（λ初值=2.0，Leader 速度=0.20 m/s） | 调参记录，不纳入报告图表与指标表 |
| D | Artstein-LPC（λ初值=2.5，Leader 速度=0.25 m/s） | HPC/LPC 阶段性图表及最新可运行结果 |
| E | Artstein-HPC（实验二参考组） | HPC/LPC 阶段性图表的 HPC 参考组 |

## 原始记录定位

| 来源代号 | 原始实验目录 | trial-ID | 参数核对重点 |
| --- | --- | --- | --- |
| A | `robot_traj/real/4d_artstein_forward_8traj_20260914_204226` | `trial_02` | `use_hpc=true`、`enable_leader_cmd_feedforward=true` |
| B | `robot_traj/real/4d_artstein_no_forward_8traj_20260914_204546` | `trial_01` | `use_hpc=true`、`enable_leader_cmd_feedforward=false` |
| C | `robot_traj/real/4d_artstein_lpc_20260915_104805` | `trial_01` | `use_hpc=false`、`initial_min_lambda=2.0`、Leader 速度 0.20 m/s |
| D | `robot_traj/real/4d_artstein_lpc_20260915_105440` | `trial_02` | `use_hpc=false`、`initial_min_lambda=2.5`、Leader 速度 0.25 m/s |
| E | `robot_traj/real/4d_artstein_20260909_173424` | `trial_03` | 用户确认 λ初值=2.5 的 Artstein-HPC 实验二参考组；动捕实物记录 |

实验配置以各目录的 `metadata.yaml` 为依据，轨迹与误差计算以 `raw.csv` 为依据。报告中的距离评价基准统一为控制器 `radius=1.0 m`。

## 图表对应关系

| 报告图表 | 对应来源 |
| --- | --- |
| 图 3--图 6：Leader 命令速度前馈开/关对比 | A、B |
| 图 7--图 10：Artstein-HPC 与 Artstein-LPC 阶段性结果 | E、D |
