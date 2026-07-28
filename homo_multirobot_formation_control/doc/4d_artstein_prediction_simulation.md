# 4D Artstein + Prediction 数值仿真说明

## 1. 仿真目的

`scripts/sim_4d_hpc_artstein_compare.py` 是 4D Artstein + motor forward prediction
进入 ROS/Gazebo/实车联调前的 Python 数值仿真脚本。它的目的不是替代 ROS 仿真，
而是在更小、更可控的模型中先验证控制架构是否合理：

- 原始 4D HPC 能否复现 MATLAB 论文结果。
- 有输入时间延迟 `Td` 和电机一阶响应 `tau` 后，原始 4D HPC 的性能退化。
- 4D Artstein + motor forward prediction 能否在不改变 HPC 幂零结构的前提下改善延迟场景。
- 后续 ROS C++ 新架构应尽量与 Python 数值仿真结构对应，避免 Python、C++、实物参数语义漂移。

## 2. 仿真依据

仿真依据包括：

- 论文：`homogeneous_control.pdf`。
- MATLAB 源码：
  `/home/l1anggmgo/ros-projects/homo_multirobot_ws/src/homo-ctrl-multirobot-ros2/homo_multirobot_formation_control/matlab/source`
- 排除 `demo_4d` 前缀文件；这些文件不是论文主图对应的 4D baseline。
- 论文图主要来自 `lpc_hpc_distance_square.m`。
- Python 脚本是对 MATLAB 4D HPC 数值仿真的工程化复现和扩展，额外加入了输入延迟、电机一阶响应、Artstein 输入延迟补偿、forward prediction、圆轨迹和噪声测试。

相关理论说明见：

```text
doc/4d_artstein_prediction_theory.md
doc/artstein_reduction.md
```

## 3. 仿真模型

### A. 原始 4D HPC，无延迟

状态：

```math
x=[p_x,p_y,v_x,v_y]^T
```

系统：

```math
\dot{p}=v,\qquad \dot{v}=u/m
```

HPC 反馈直接使用真实状态。该模型用于复现 MATLAB 原始 4D 双积分器控制效果，
确认 Python 版本的 `lpc2hpc`、`hnorm`、编队点选择和 4D double-integrator
主回路与论文/MATLAB baseline 一致。

### B. 原始 4D HPC + 延迟执行器

控制器仍以为 Follower 是理想双积分器，但实际执行器链路为：

```text
cmd_vel -> pure input delay (Td) -> first-order motor response (tau) -> v_real
```

也就是控制器输出速度命令后，命令先经过纯输入时间延迟 `Td`，再经过一阶电机响应
`tau`，最终实际速度 `v_real` 滞后于指令速度。该模型用于测试不补偿时的性能退化，
典型表现包括跟踪滞后、误差周期振荡、速度指令增大或变宽。

### C. 4D Artstein + Prediction 新架构

结构：

```text
measured follower [p, v_real]
    -> Artstein input-delay compensation (Td)
    -> first-order motor forward prediction (tau)
    -> predicted HPC state x_h=[p_pred, v_pred]
    -> original 4D double-integrator HPC
    -> acceleration/equivalent force command
    -> velocity command integration
    -> delayed motor plant
```

重点：

- HPC 核心仍是原始 4D 双积分器，不把 `-1/tau` 极点放进 HPC 矩阵。
- Artstein 处理纯输入/传输延迟 `Td`。
- Forward prediction 处理一阶电机响应滞后 `tau`。
- 预测层给 HPC 构造更接近未来执行状态的反馈状态 `x_h=[p_pred,v_pred]`。
- HPC 输出仍按原始 4D double-integrator 解释为加速度/等效力，再积分成速度命令。

## 4. 输出图怎么看

脚本当前输出目录默认是：

```bash
analysis/results/4d_artstein
```

常见输出包括：

```text
paper_lpc_hpc_distance_square_reproduction.png
delay_original_vs_artstein_prediction.png
circle_original_vs_artstein_clean.png
circle_original_vs_artstein_noise.png
summary_metrics.csv
```

图和指标的读取方式：

- `Trajectory`：Leader/Follower 轨迹。用于判断是否能形成圆轨迹、8 字轨迹或 MATLAB 原始轨迹下的编队形状。
- `Formation distance`：Leader-Follower 距离或最近编队点误差。用于判断是否稳定在期望编队半径附近。
- `Component error`：x/y 方向误差。用于判断是否存在周期性振荡，以及补偿后误差是否更接近 0。
- `Velocity command`：控制器速度指令。用于判断是否频繁撞限幅、速度指令是否变宽、是否出现高频抖动。
- `Speed magnitude`：全程总速度绝对值 `|v|`。用于判断圆轨迹中速度是否保持在目标值附近，以及速度是否被延迟和限幅拉宽。
- `Noise test`：加入位置/速度测量噪声后，观察控制器鲁棒性和速度抖动放大情况。

只看轨迹是否漂亮不够。延迟补偿方案必须同时看 component error、speed magnitude、
`cmd_vel_raw/cmd_vel/odom` 速度链路图，才能判断是控制器改善了延迟，还是限幅/滤波把问题遮住了。

## 5. 主要实验结论

以下为前期数值仿真和 ROS/Gazebo 参数对齐过程中的定性结论，不写成精确指标：

- 无延迟下，原始 4D HPC 能正常跟踪并形成圆轨迹。
- 加 `Td + tau` 后，原始 4D HPC 出现明显滞后和振荡。
- 新架构在同样延迟下 component error 更接近 0，周期性大振荡明显减小。
- 20Hz 控制频率更贴近实车，不应继续使用过高控制频率掩盖问题。
- 执行器内部积分步长应与控制周期一致；不能用 `0.01s` 小步长让执行器比真实硬件更理想。
- Leader 速度 `0.2m/s` 和圆/8 字轨迹下表现较稳。
- Leader 速度升高到 `0.5m/s` 后，受加速度限幅和延迟影响，跟踪滞后明显增大。
- 加速度限幅不能只求大。过大可能导致速度变化幅度大、指令更激进；实物应从保守值开始。
- 噪声增大后，新架构仍可跟踪，但速度指令会更容易出现抖动，需要结合滤波和限幅处理。
- `leader_vel_lpf_tau` 会平滑 Leader 速度估计，但也会引入相位滞后；之前测试中推荐默认 `0.0`。
- `cmd_integrator_base:=pred` 比 `cmd` 更符合预测补偿架构，仿真表现更好。

注意：Python 脚本默认 `--dt 0.01` 便于复现连续时间数值结果；面向 ROS/Gazebo/实物延迟链路时，
应使用 `--dt 0.05` 对齐 20Hz 控制周期，或至少单独做 20Hz 消融对照。

## 6. 推荐仿真参数

### Gazebo/ROS 延迟仿真基线

```text
tau:=0.43
Td:=0.22
control_rate:=20.0
mass:=2.0
hpc_c_min:=0.1
initial_min_lambda:=1.5
switch_min_lambda:=4.0
min_cmd_vel:=0.0
max_linear_accel:=0.5
use_motor_delay:=true
motor_tau:=0.43
transport_delay:=0.22
delay_max_accel:=0.5
cmd_integrator_base:=pred
leader_vel_lpf_tau:=0.0
```

对应 ROS 启动示例：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_4d_artstein.launch.py \
  leader_ns:=/robot1 follower_ns:=/robot2 \
  tau:=0.43 Td:=0.22 control_rate:=20.0 \
  mass:=2.0 hpc_c_min:=0.1 \
  initial_min_lambda:=1.5 switch_min_lambda:=4.0 \
  min_cmd_vel:=0.0 max_linear_accel:=0.5 \
  use_motor_delay:=true motor_tau:=0.43 transport_delay:=0.22 delay_max_accel:=0.5 \
  cmd_integrator_base:=pred leader_vel_lpf_tau:=0.0
```

### 实物保守起步

```text
tau:=0.43
Td:=0.22
control_rate:=20.0
mass:=2.0
hpc_c_min:=0.1
initial_min_lambda:=1.5
switch_min_lambda:=4.0
min_cmd_vel:=0.03
max_linear_accel:=0.25
use_motor_delay:=false
cmd_integrator_base:=pred
leader_vel_lpf_tau:=0.0
```

对应 ROS 启动示例：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_4d_artstein.launch.py \
  leader_ns:=/virtual_leader follower_ns:=/robot2 \
  tau:=0.43 Td:=0.22 control_rate:=20.0 \
  mass:=2.0 radius:=1.0 hpc_c_min:=0.1 \
  initial_min_lambda:=1.5 switch_min_lambda:=4.0 \
  min_cmd_vel:=0.03 max_linear_accel:=0.25 \
  use_motor_delay:=false \
  cmd_integrator_base:=pred leader_vel_lpf_tau:=0.0
```

## 7. Python 数值仿真与 ROS C++ 的对应关系

- Python 先验证算法逻辑，重点是延迟补偿结构是否能改善原始 4D HPC 的延迟退化。
- C++ 节点 `formation_control_node_4d_artstein.cpp` 尽量对应 Python 的 Artstein + prediction 流程。
- Python 中的执行器延迟仿真对应 ROS 中 `use_motor_delay`、`motor_tau`、`transport_delay`、`delay_max_accel`。
- Python 中的速度积分对应 C++ 中 HPC 输出后转换为 `cmd_vel` 的积分环节。
- Python 中的预测反馈对应 C++ 中 controller wrapper 构造 predicted state 后调用原始 4D HPC。
- Python 中的 `tau/Td` 是控制器侧预测参数；ROS 延迟节点中的 `motor_tau/transport_delay` 是仿真注入的物理延迟参数。

## 8. 注意事项

- `tau` 不是直接设定固定加速度，而是一阶响应时间常数。
- 真实全向轮底盘速度变化未必是恒定加速度，`tau_eff` 会随指令幅值变化。
- 如果设置 `tau:=0.0 Td:=0.0`，只是让控制器侧预测器失效；若仿真环境仍有 `motor_tau/transport_delay`，系统仍然有实际延迟。
- 对比实验必须区分控制器参数和仿真注入的物理延迟参数。
- 严格消融“不使用延迟预测器”时，应同步设置 `transport_delay:=0.0`，必要时 `motor_tau` 也要对应设置。
- 只比较漂亮轨迹不够，还要看 component error、speed magnitude、`cmd_vel_raw/cmd_vel/odom` 的速度链路图。
- 20Hz 是贴近实车 STM32 `/odom` 频率的关键约束；高控制频率可能让仿真比实车更理想。
