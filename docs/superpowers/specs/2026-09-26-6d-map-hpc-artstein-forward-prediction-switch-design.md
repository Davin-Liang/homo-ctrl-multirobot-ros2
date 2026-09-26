# 6D Map HPC Artstein 前向预测开关

## 目标

为 `formation_control_node_6d_map_hpc_artstein` 提供
`enable_forward_prediction` 布尔参数，用于在保留 Artstein 纯输入延迟补偿
(`Td`) 的同时，可选地关闭由 `tau` 与 `tau_yaw` 驱动的前向状态预测。

## 行为

- 默认值为 `true`，保持既有行为和预测时域：`Td + max(tau, tau_yaw)`。
- 参数为 `false` 时，Follower 状态仍先加上 Artstein 历史积分并外推 `Td`，但
  不执行额外的 `tau/tau_yaw` 一阶电机响应预测；Leader 也仅外推 `Td`。
- `tau` 与 `tau_yaw` 仍要求为正数，以维持 Artstein 模型和参数校验。

## 接口与验证

- 在默认 YAML 与 launch 文件中声明并透传该参数，支持 launch 命令行覆盖。
- 为预测器增加单元测试：关闭前向预测时，结果等于 Artstein 延迟补偿状态；开启时
  保持现有前向预测结果。
- 构建 `homo_multirobot_formation_control`，并执行对应 CTest。
