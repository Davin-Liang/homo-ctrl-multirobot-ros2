# 6D map-frame HPC 与 Artstein 数值仿真

该脚本验证 `6D_齐次编队控制理论_Codex工程压缩版.md` 指定的 map-frame 固定偏移模型。它是独立于旧 `6d_disc_artstein` 的数值实验，不修改 ROS 2 控制器。

## 模型与对照

- 误差：`[ex, ey, e_theta, evx_map, evy_map, e_omega]`；位置偏移固定为 `[-1.0, 0.0] m`。
- 控制器：沿用项目现有的正则化工程 HPC `u_impl`，而非包含 `K0` 的理论控制律。
- plant：map-frame 平移和 yaw-rate 的一阶速度响应，默认 `Td=0.22 s`、`tau=0.43 s`、控制周期 `0.05 s`、积分周期 `0.01 s`。
- `ideal`：无延迟、无一阶滞后。
- `delayed`：含相同 plant，仅使用测量状态反馈。
- `artstein`：含相同 plant，使用 map-frame 状态预测后再进入同一 HPC。

三个组共享 Leader 圆轨迹、Follower 初值、HPC 参数、命令限幅和随机种子。

## 运行

```bash
python3 homo_multirobot_formation_control/scripts/sim_6d_map_hpc_artstein_compare.py \
  --out-dir /tmp/6d_map_hpc_artstein
```

输出：

- `comparison.png`：轨迹、位置误差、yaw 误差、map x 速度命令；
- `summary_metrics.csv`：峰值、尾段均值和最终误差；
- `timeseries.csv`：完整时序；
- `diagnostics.txt`：可控性和齐次代数恒等式残差。

## 结论边界

该脚本只评估正则化工程控制律在指定延迟 plant 下的数值表现。它不构成理论 `u_th` 的有限时间稳定性证明，也不覆盖 ROS 2 的实际 body-input 映射、离散选点、HOCBF、饱和后严格稳定性或实车鲁棒性。
