# 6D Map HPC Artstein 延迟补偿开关

## 目标

为 `formation_control_node_6d_map_hpc_artstein` 增加
`enable_artstein_compensation` 布尔参数，以独立控制 Artstein 输入延迟补偿；它与
已有 `enable_forward_prediction` 组合使用。

## 行为

- 两个参数默认均为 `true`，维持原有的 `Td` 延迟补偿加 `tau/tau_yaw` 前向预测。
- 关闭 Artstein 时，Follower 不再叠加命令历史积分或执行 `Td` 外推；Leader 也不做
  `Td` 外推。
- 前向预测仍独立工作：若开启，Follower 与 Leader 从当前测量状态前进
  `tau/tau_yaw`；若关闭，直接使用当前测量状态。
- `tau` 与 `tau_yaw` 仍要求为正数，保持预测器模型校验一致。

## 接口与验证

- 参数写入 Artstein YAML 与 launch，可由
  `enable_artstein_compensation:=false` 覆盖。
- 预测器单测覆盖关闭 Artstein 后跳过历史积分和 `Td` 外推、但仍可前向预测的路径。
- launch/YAML 单测确认参数存在并传入节点；构建包并执行相关 CTest。
