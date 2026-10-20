# Delay-Aware Constrained MPC 数值仿真

本目录是独立于 ROS 2、Gazebo、HPC、Artstein 和 HOCBF 的 Python 数值实验。它验证 map 坐标系下的 Leader--Follower 线性 MPC：Follower 优化相对速度指令，真实对象同时包含一阶速度滞后与固定输入纯时延。

## 名义模型与时序

误差为 `e=[ex, ey, e_theta, evx, evy, e_omega]`，固定编队偏移在 map 系中。名义对象为

```text
q_dot = v
v_dot = -T^-1 v + T^-1 delta_u(t-Td)
```

默认控制周期为 `0.05 s`，名义纯时延为 `0.30 s`，即 **6 个采样步**。队列按最旧到最新排列；第 `k` 步的对象使用 `delta_u[k-6]`，而新求出的 `delta_u[k]` 在第 `k+6` 个控制区间才生效。离散化采用精确 ZOH：`Ad=[[I,H],[0,E]]`、`Bd=[[hI-H],[I-E]]`。

MPC 是 CVXPY/OSQP 的凸二次规划。输入幅值和变化率约束作用于实际发送给 Follower 的 `u_f=u_l+delta_u`，而不是只约束相对指令。

## 依赖与运行

```bash
python3 -c 'import numpy, scipy, cvxpy, osqp, matplotlib; print("dependencies ready")'
python3 homo_multirobot_formation_control/mpc_sim/run_experiments.py
python3 -m pytest homo_multirobot_formation_control/mpc_sim/tests -v
```

可指定输出目录或缩短时长做冒烟验证：

```bash
python3 homo_multirobot_formation_control/mpc_sim/run_experiments.py \
  --output-dir /tmp/delay_aware_mpc --duration-scale 0.1
```

默认实验包括静止 Leader、直线、平移/角速度变化的曲线、严格输入约束、普通 MPC（内部假定零时延）与 6 步延迟感知 MPC 的公平比较，以及真实对象为 4 步或 8 步时延的失配测试。

## 输出

每个案例目录包含：

- `timeseries.csv`：误差、Leader/Follower 指令、实际作用输入、OSQP 状态与求解时间；
- `metrics.csv`：位置 RMSE、最大值、尾段均值、约束违反次数和求解耗时；
- `trajectory.png`、`errors.png`、`inputs_and_solver.png`；
- 根目录 `experiment_summary.md`：各案例的统一指标表。

`results/` 被 Git 忽略。非最优 QP、数值异常或约束违反不会被隐藏，会写入 CSV、指标和汇总表。

## 边界

这是忽略 `omega Jv` 耦合的名义 map-frame 模型；不包含轮速逆运动学、障碍物、通信延迟、稳定性证明、ROS 2 或实车接入。实验结果只能用于判断该 MPC 架构的数值可行性，不能作为严格闭环稳定性证明。
