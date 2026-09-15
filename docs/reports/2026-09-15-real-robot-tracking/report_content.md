# 双移动机器人 Leader–Follower 实物跟踪实验总结

## 1. 实验任务说明

本实验在双移动机器人 Leader–Follower 场景中，记录并整理 Artstein-HPC 与 Artstein-LPC 的阶段性实物跟踪结果。评价半径固定为 1.0 m，所有统计均以该理想编队半径为唯一评价基准。

## 2. 实物实验平台、数据流与控制方法

两台 mini_omni 全向移动机器人采用动捕状态源。Leader 的轨迹由速度命令驱动，Follower 根据相对编队误差输出速度命令。控制频率为 20 Hz，单次有效记录时长约 45 s。

Follower 的前向预测使用当前动捕状态、最近输出命令和电机时间常数 `tau` 预测短时状态，以补偿执行器响应。其原理是 Artstein 积分项将历史控制输入纳入状态，先补偿输入纯滞后，再将状态前向预测到控制时刻，使 HPC/LPC 针对预测状态而非滞后测量状态计算控制量。

Leader 命令速度前馈是可选的 `Leader cmd_vel` 速度增量支路，直接叠加到控制输出。它与 Follower 前向预测是两条不同支路；前馈开/关不是 Follower 前向预测开/关。

<!-- 图 1：assets/control_pipeline.png，标题：实物跟踪控制与数据流 -->

## 3. 实验过程与统一条件

四组有效记录均来自实物平台和动捕状态源。统计只使用条件表与指标表中理想半径为 1.0 m 的数据，不根据原始目录名或元数据中的控制器字符串重新推断前馈状态。

共同配置：Leader `/robot1`；Follower `/robot2`；控制频率 20 Hz；电机时间常数 `tau=0.43 s`；纯滞后 `Td=0.22 s`。

实验流程：准备机器人与动捕，核对状态话题和控制参数；启动 Follower 控制器与 Leader 轨迹；录制 45 s 并保存 `raw.csv`、`metadata.yaml`；停止轨迹和控制器，确认两车停止。

<!-- 表 1：实验条件表，数据：experiment-conditions.csv -->

### 数据来源对照

图表中的来源代号与原始数据对应如下：

| 来源 | 实验标签 | 原始目录 / trial-ID |
| --- | --- | --- |
| A | Artstein-HPC（Leader 命令前馈：开） | `robot_traj/real/4d_artstein_forward_8traj_20260914_204226 / trial_02` |
| B | Artstein-HPC（Leader 命令前馈：关） | `robot_traj/real/4d_artstein_no_forward_8traj_20260914_204546 / trial_01` |
| C | Artstein-LPC（λ初值=2.0，Leader 速度=0.20 m/s） | `robot_traj/real/4d_artstein_lpc_20260915_104805 / trial_01` |
| D | Artstein-LPC（λ初值=2.5，Leader 速度=0.25 m/s） | `robot_traj/real/4d_artstein_lpc_20260915_105440 / trial_02` |

## 4. 实验一：Artstein-HPC 的 Leader 命令速度前馈开/关对比

在两组约 45 s、Leader 平均速度约 0.246 m/s 的 Artstein-HPC 实测记录中，开启 Leader 命令速度前馈的平均绝对距离误差为 0.1202 m、RMS 距离误差为 0.2791 m；关闭时分别为 0.1282 m 和 0.2895 m。开启组在这两个汇总指标上较低。

末帧绝对距离误差则为 0.0355 m，高于关闭组的 0.0165 m。因此该组对比仅描述已记录的实测差异，不将其概括为所有指标的一致改善。

<!-- 图 2：assets/feedforward_comparison.png，来源 A、B -->

<!-- 图 3：assets/feedforward_distance_error.png，来源 A、B -->

<!-- 表 2：前馈开/关指标表，数据：metrics.csv，筛选：leader_command_feedforward -->

## 5. 实验二：Artstein-HPC 与 Artstein-LPC 的阶段性结果

LPC 两条有效记录的平均绝对距离误差分别为 0.4001 m 与 0.4144 m，RMS 距离误差分别为 0.4547 m 与 0.4771 m。对应的 `initial_min_lambda` 与 Leader 速度同时变化，结果只作为阶段性观察。

LPC 在 `initial_min_lambda=1.5`、Leader 速度 0.25 m/s 的实物工况下发生碰撞，未形成可用于轨迹统计的有效记录。LPC 的 λ初值=2.0 记录同时将 Leader 速度降至 0.20 m/s；λ初值=2.5 记录使用 0.25 m/s。因此现有 HPC/LPC 结果用于说明阶段性可运行性和现象，不用于宣称严格单变量条件下的性能优劣。

<!-- 图 4：assets/hpc_lpc_trajectory.png，来源 A、C、D -->

<!-- 图 5：assets/hpc_lpc_distance_error.png，来源 A、C、D -->

<!-- 表 3：HPC/LPC 阶段性指标表，数据：metrics.csv，筛选：hpc_lpc_stage_result -->

## 6. 阶段性成果、当前问题与下一步工作

已形成统一的实物实验条件表、1.0 m 基准指标表和可复核的图表总结。下一步应在固定 Leader 速度、初始 λ 和其他运行条件后，补充可重复试验；同时继续核对前馈支路标注与原始实验配置的一致性。
