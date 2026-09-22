# 无前馈敏感性实验统计设计

## 范围

对 `homo_multirobot_formation_control/doc/experimental_statistics.md` 中的两组实物、无 Leader 命令速度前馈实验进行统计：

- 齐次度对比：`nu = -1.0, -0.9, -0.8, -0.7, -0.6, -0.5`；每档三次。
- 控制频率对比：10、20、25、40 Hz；每档三次。

每次实验的输入为对应记录目录中的 `raw.csv` 和 `metadata.yaml`。

## 指标与计算口径

令实际两车间距为 `d(t)`，从该次 `metadata.yaml` 读取的期望编队半径为 `r`，距离误差为：

`e_d(t) = d(t) - r`。

相对速度误差为 Leader 和 Follower 的 map 系二维速度差的模长：

`e_v(t) = sqrt((leader_vx_map_ms - follower_vx_map_ms)^2 + (leader_vy_map_ms - follower_vy_map_ms)^2)`。

每次实验计算以下六项：

| 指标 | 定义 |
| --- | --- |
| 平均距离误差 (m) | 全程 `mean(abs(e_d))` |
| RMS 距离误差 (m) | 全程 `sqrt(mean(e_d^2))` |
| 末 10 s 距离误差 (m) | 最后 10 s 内 `mean(abs(e_d))` |
| 末 10 s 距离误差标准差 (m) | 最后 10 s 内带符号 `e_d` 的样本标准差 |
| 平均相对速度误差 (m/s) | 全程 `mean(e_v)` |
| 末 10 s 相对速度误差 (m/s) | 最后 10 s 内 `mean(e_v)` |

末 10 s 窗口由 `time_s >= max(time_s) - 10.0` 确定。距离误差的均值使用绝对值，避免正负偏差抵消；标准差保留带符号误差的波动含义。

## 汇总与校验

先逐次计算六项指标，再按同一参数档的三次结果计算算术均值与样本标准差（`ddof=1`）。不混合各试验的原始采样点。汇总表单元格表示为“均值 ± 标准差”。

计算前校验 CSV 必备列、持续时长不少于 10 s、元数据中的 `enable_leader_cmd_feedforward: false`，以及齐次度或控制频率与分组标签一致。发现重复目录、缺失文件或元数据不一致时，显式报告，不静默修正。

## 交付

将逐次明细与两张组级汇总表写入 `experimental_statistics.md`：一张按齐次度分组，一张按控制频率分组。距离与速度值均保留四位小数。
