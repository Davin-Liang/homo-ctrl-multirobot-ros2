# 6D Map HPC Artstein 前馈与切换带宽设计

## 目标

使 `formation_control_node_6d_map_hpc_artstein` 具备与 4D Artstein 一致的 Leader 命令增量前馈，并在离散编队目标切换时采用独立的闭环极点尺度下界。

## Leader 命令增量前馈

节点新增 `enable_leader_cmd_feedforward`（默认 `false`）、`leader_cmd_timeout`（默认 `0.15` 秒）与 `leader_cmd_delta_lpf_tau`（默认 `0.10` 秒）参数。启用后订阅 `<leader_ns>/cmd_vel`，复用 `LeaderCommandDeltaFeedforward` 计算相邻 Leader 线速度命令的单次、低通并限幅后的增量。

在控制周期内，使用当前 Leader yaw 将该 body 系线速度命令转换至 map 系；消费到的增量叠加到 6D HPC 产生的 `map_out.head<2>()`，然后继续现有的 body 系转换和速度、加速度、轮速约束。不会叠加 Leader 的角速度命令，6D HPC 的 yaw Artstein 通道仍是唯一的偏航控制来源。前馈关闭、首条 Leader 命令、重复消费或命令超时均产生零增量。

## 切换后的独立带宽

控制器新增 `switch_min_lambda` 参数（默认 `4.0`）。首次 `initialize()` 使用 `initial_min_lambda`；`select_target()` 触发离散目标切换时，使用 `switch_min_lambda` 重建 `K/P/Gd/nu`。初始化过程必须让本轮线性增益与新计算的 HPC 矩阵来自同一带宽，以保持当前实现的同步性。

## 测试与配置

在现有 `test_6d_map_hpc_artstein_controller.cpp` 中验证初始与切换初始化可选择不同的最小极点尺度；为便于直接断言，控制器暴露只读的当前 `min_lambda`。前馈辅助类已有独立单元测试，节点集成遵循 4D 已验证的调用顺序。YAML 和 launch 显式提供四个新增/对齐参数，README 补充 6D Map HPC 的参数与行为说明。

## 非目标

不改变 6D 的状态定义、平移/yaw Artstein 预测、HPC 数学结构、约束器或仿真延迟模型；不新增避障或径向刹停层。
