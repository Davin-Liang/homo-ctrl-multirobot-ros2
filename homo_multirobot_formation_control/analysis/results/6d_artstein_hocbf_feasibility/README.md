# 6D Artstein HOCBF 数值可行性扫描

本目录保存静态圆障碍物 Oracle 数值扫描结果。它不使用 ROS、Gazebo 或激光聚类：障碍物中心和保守安全半径在 map 系中已知，用于验证预测器、HOCBF 硬约束和延迟一阶执行器模型之间的一致性。

运行命令：

    python3 homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py --output homo_multirobot_formation_control/analysis/results/6d_artstein_hocbf_feasibility/scan.csv

模型以 10 ms 积分，并每 50 ms 更新一次安全滤波器；因此可以精确表示默认的 Td=0.22 s。每个扫描场景均为正面接近静态圆障碍物。

CSV 字段：

- tau：真实一阶执行器时间常数。
- delay_model：预测器使用的输入时延。
- delay_actual：植物实际输入时延。
- initial_clearance：初始中心距离减安全半径。
- min_h、min_distance：从 10 ms 内部轨迹计算的最小安全函数和最小中心距离。
- min_psi2：20 Hz 控制时刻的最小 HOCBF 余量。
- max_command_norm：最终发布 map 系速度命令的最大二范数。
- infeasible_steps、braking_steps：硬约束 QP 无解和后备制动的控制周期数。

只有 delay_model 等于 delay_actual、infeasible_steps 等于 0、braking_steps 等于 0 且 min_h 大于等于 0 的行，才与准确模型、可行 HOCBF 情形的模型级安全结论一致。失配行和制动行用于刻画适用边界，不能作为鲁棒安全证明。

同一次命令还会生成 sampling_rate_compare.csv。它比较默认正面接近场景在 20 Hz 控制与 1 kHz 控制参考下的最小安全函数、最小距离和两者差值。该对照量化采样实现误差，不单独构成 sampled-data 安全定理。
