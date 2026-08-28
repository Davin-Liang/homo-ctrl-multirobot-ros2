# 6D map-frame 连续 Leader yaw 动态仿真设计

## 目标

在既有 6D map-frame HPC 对照仿真中增加两种连续 Leader yaw 动态，比较理想、延迟未补偿和 Artstein 预测补偿的姿态与位置响应。

## 共同条件

Leader 的 map 位置与 map 平移速度维持半径 `2 m`、速度 `0.45 m/s` 的圆轨迹。Follower 的 map-frame 期望偏移保持 `[-1.0, 0.0] m`。仅改变 Leader yaw、body-frame 速度表示和 yaw rate；三组共享同一 plant、初值、HPC、限幅和 60 s 仿真时长。

## 场景 A：恒定角加速度

相对圆轨迹的 yaw-rate 偏置满足

\[
\dot\omega_{off}=0.05\ \mathrm{rad/s^2},
\]

直到总 yaw rate 达到 `0.8 rad/s` 后保持该上限。yaw 连续积分；每时刻依据保持不变的 map 速度重新计算 Leader body velocity。

## 场景 B：周期时变角加速度

相对圆轨迹的 yaw-rate 偏置采用

\[
\omega_{off}(t)=\frac{0.08}{0.4}\sin(0.4t),
\qquad
\alpha_{off}(t)=0.08\cos(0.4t).
\]

因此 yaw rate 周期性增减，yaw 连续且无突变。

## 输出与边界

每个场景生成独立的三组比较图、summary CSV 与完整时序 CSV，报告位置/yaw 的峰值、尾段均值和最终误差。Artstein 使用可从当前 Leader 状态推导的连续 yaw-rate 模型；场景 A 的限幅点和场景 B 的解析函数均对预测器可知。

这些是理想连续运动模型，不涉及电机角加速度限幅、IMU 噪声或真实 Leader 未来轨迹不可知性。
