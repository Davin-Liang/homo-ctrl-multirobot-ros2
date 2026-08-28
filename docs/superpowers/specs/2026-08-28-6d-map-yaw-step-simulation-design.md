# 6D map-frame Leader yaw 阶跃数值实验设计

## 目标

扩展现有 map-frame 6D HPC 对照仿真，观察 Leader 航向在运动中突变时，Follower 的位置和 yaw 瞬态响应，以及 Artstein 预测的影响。

## 场景

Leader 继续以半径 `2 m`、速度 `0.45 m/s` 的 map-frame 圆轨迹运动。`t < 30 s` 时其 yaw 为切线航向；`t >= 30 s` 时 yaw 取切线航向加 `+pi/2`。位置和 map-frame 平移速度不变，因此该输入是纯航向测量/参考阶跃，而非物理连续转动模型。

Follower 期望 map-frame 位置偏移仍为 `[-1.0, 0.0] m`，目标 yaw 始终追踪当前 Leader yaw。保持既有 `ideal`、`delayed`、`artstein` 三组相同 plant、初值、HPC 参数和限幅。

## 输出与判据

新增 yaw-step 对比图：位置误差、yaw 误差和 yaw 命令，且在 `30 s` 标出阶跃时刻。summary CSV 新增阶跃后峰值 yaw 误差、最终 yaw 误差和阶跃后位置误差峰值。测试验证 Leader yaw 仅在阶跃后改变 `pi/2`，但位置和 map velocity 连续不变。

该实验只评价工程正则化控制器的数值瞬态，不能解释为对不可实现无限角加速度的实车安全性结论。
