# 4D 仅一阶前向预测数值消融实验设计

## 目标

在现有 4D HPC 延迟数值实验中增加一组公平的消融实验：原始 4D HPC 仅配合一阶执行器前向预测。该实验用于展示：在不使用 Artstein 补偿纯输入死区的前提下，仅预测电机滞后能带来多少改善。

## 范围

修改 `homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py` 及其针对性的 Python 回归测试；更新对应的 4D 数值仿真说明，加入第三组及输出标签。不修改 ROS 节点、launch 文件、C++ 控制器、plant 参数或既有仿真结果文件。

## 实验组

所有含延迟场景使用相同的初始条件、4D HPC 参数、命令限幅、延迟队列、一阶 plant、`Td`、`tau`、采样周期及（适用时）噪声随机种子。

1. `original`：反馈直接使用测得的 Leader 和 Follower 状态，不进行补偿。
2. `forward_prediction_only`：反馈使用测得的 Leader 状态，以及仅跨越一阶电机滞后 `tau` 预测得到的 Follower 状态；不计算 Artstein 积分，也不将 Leader 或 Follower 预推进纯延迟 `Td`。
3. `compensated`：反馈保持现有流程，先通过 Artstein 变换补偿输入延迟，再进行一阶电机前向预测；Leader 预测仍保持现有的 `Td + tau` 预测时域。

三个组的物理 plant 均为 `cmd_vel -> Td -> tau -> v_real`。因此，新组刻意不补偿真实存在的纯输入死区。

## 设计

增加一个小型辅助函数：根据 Follower 的测量状态和上一周期已发布的速度命令，预测经过 `tau` 后的 Follower 位置和速度。闭式预测必须与现有一阶执行器模型一致：

```math
v_{pred}=v_{cmd}+e^{-1}(v_{meas}-v_{cmd}),
```

```math
p_{pred}=p_{meas}+\tau v_{cmd}+\tau(1-e^{-1})(v_{meas}-v_{cmd}).
```

延迟场景和圆轨迹场景均按实验组选择反馈状态。既有 `original` 与 `compensated` 行为保持不变；`forward_prediction_only` 调用新辅助函数，Leader 直接使用测量状态。

延迟图和圆轨迹图改为三组对比：原始组为红色，仅预测组为橙色，完整 Artstein + prediction 组为蓝色。汇总 CSV 为每种新实验增加一行稳定命名（包含 `forward_prediction_only`），既有行名不变。

## 验证

增加针对性的 Python 测试，通过模块方式加载脚本，验证：

- 仅预测辅助函数对给定状态和命令符合一阶响应闭式解；
- 延迟仿真与圆轨迹仿真均接受 `forward_prediction_only`，产生有效样本，且仍保留物理延迟 plant；
- 汇总 CSV 包含仅预测组的行名，绘图函数能接收三组结果序列。

先运行针对性 Python 测试；随后以较短时间运行脚本至临时目录，确认产生预期图像和 CSV。默认运行命令仍可用于重新生成完整的论文级结果。

## 验收标准

- 一条命令可为 MATLAB Leader、无噪声圆轨迹和含噪声圆轨迹延迟场景生成公平的三组图和指标。
- 新组只使用 `tau` 预测；`Td` 始终保留为 plant 延迟，且绝不通过 Artstein 补偿。
- 既有原始组与完整补偿组的数值行为和标签保持不变。
- 针对性回归测试通过。
