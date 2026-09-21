# 4D Artstein 控制频率数值扫描设计

## 目标

使用既有 `sim_4d_hpc_artstein_compare.py`，比较 4D Artstein + prediction 在 10、25、40 Hz 控制频率下的数值跟踪效果，不修改控制器或 ROS 节点。

## 固定条件

三组均使用 `tau=0.43 s`、`predict_tau=0.43 s`、`Td=0.22 s`、`leader_speed=0.5 m/s`、`circle_tmax=60 s`、`hpc_c_min=0.1`、`max_cmd=1.5 m/s`。真实 plant 积分步长固定为 `plant_dt=0.005 s`（200 Hz）；它同时整除 0.1、0.04、0.025 s，且 `Td` 恰为 44 个 plant 步长，从而精确表示延迟并隔离控制更新频率的影响。

## 扫描与输出

控制器 `dt` 分别为 0.100、0.040、0.025 s，对应 10、25、40 Hz。每次运行写入独立的 `analysis/results/4d_artstein_rate_{frequency}hz_20260921/` 目录，避免覆盖。既有脚本的完整对照图和 CSV 全部保留；分析只横向读取 `circle_artstein_prediction_clean` 和 `circle_artstein_prediction_noise`。

## 报告指标

每个场景报告最大编队误差、后 30% 时间段平均误差和标准差、最终误差，并说明这些指标来自同一实际 plant、不同控制更新率的实验。结果只代表数值模型，不外推为 Gazebo 或实车结论。
