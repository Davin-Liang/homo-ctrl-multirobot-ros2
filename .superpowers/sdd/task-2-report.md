# Task 2：统一指标计算与分析校验报告

## 实现内容

- 新增 `homo_multirobot_formation_control/scripts/analyze_real_tracking_report.py`。
  该脚本读取实验清单、每个试验的 `metadata.yaml` 与 `raw.csv`，统一生成
  `docs/reports/2026-09-15-real-robot-tracking/metrics.csv`。
- 指标以固定的理想编队半径 `1.0 m` 计算距离误差，输出样本数、距离均值/标准差、
  绝对距离误差均值、RMS 距离误差、最终距离误差、时长、Leader/Follower 轨迹长度及
  速度均值/标准差。
- CSV 输入严格要求且仅允许规定的 12 列。空数据、任意非数值或非有限输入、缺列、
  非法理想半径和无效 YAML 均带具体路径及原因失败；所有试验都成功分析后才原子替换
  `metrics.csv`，不会写出部分新报告。

## TDD 记录

先新增 `test_distance_error_metrics_use_ideal_radius`，然后执行：

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_analyze_real_tracking_report.py -q
```

红灯结果：退出码 2，测试收集阶段因
`homo_multirobot_formation_control/scripts/analyze_real_tracking_report.py` 尚不存在而报
`FileNotFoundError`；即目标 `compute_distance_metrics` 尚不可用。

实现后同一命令绿灯：`1 passed in 0.00s`。

## 真实数据分析与校验

执行：

```bash
python3 homo_multirobot_formation_control/scripts/analyze_real_tracking_report.py \
  --manifest homo_multirobot_formation_control/analysis/real_tracking_report/experiment_manifest.yaml \
  --output docs/reports/2026-09-15-real-robot-tracking
```

结果：`wrote 4 experiment rows to docs/reports/2026-09-15-real-robot-tracking/metrics.csv`。

随后以 Python 校验输出：4 行、每行 `sample_count > 0`、所有数值字段有限，且每行
`ideal_radius_m == 1.0`；校验结果为：

```text
metrics validation OK: 4 rows; positive samples; finite numeric values; ideal radius 1.0 m
```

## 适用边界与关注事项

- 9 月 14 日 HPC 前馈开/关记录均采用同一 `1.0 m` 理想半径误差口径。
- HPC 与 LPC 的结果只能表述为“不同实物工况下的阶段性结果”，不能称为公平单变量
  对比：两组 LPC 的 `initial_min_lambda` 和 Leader 速度均不同，且 `lambda=1.5`、
  `0.25 m/s` 的未录制条件为碰撞。
- 源 `metadata.yaml` 中的 `recording.ideal_radius_m` 记录为 `2.0`，而本报告任务明确
  固定分析口径为 `1.0 m`；输出将此分析口径明确写入 `ideal_radius_m`，不修改原始数据。
- 未修改 `homo_multirobot_formation_control/robot_traj/real`。
