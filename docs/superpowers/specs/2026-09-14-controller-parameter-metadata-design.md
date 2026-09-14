# 控制器参数元数据快照设计

## 目标

每次 `record_trajectory.py` 完成录制时，`metadata.yaml` 的
`controller_parameters` 应保存指定控制器节点全部当前生效的 ROS 参数，而不是
维护一个会遗漏新参数的固定白名单。

## 方案

记录器通过 `<follower_ns>/<controller_node_name>/list_parameters` 枚举控制器参数
名称，再通过同节点的 `get_parameters` 读取其值。参数值转换为 YAML 原生标量或
数组后写入 `controller_parameters`。

控制器未启动或参数服务不可用时，保留现有的等待与重试行为；服务最终不可用时，
快照为空，不阻断已有的记录流程。

## 范围

- 支持 ROS 参数的布尔、整数、浮点、字符串及数组类型。
- 移除图表顶部的控制器参数摘要；参数只保存在 `metadata.yaml`，避免遮挡图表。
- 不修改既有实验目录中的 metadata，也不改变延迟节点参数的单独记录方式。

## 验证

为参数名枚举和 `ParameterValue` 转换增加单元测试，覆盖控制器当前使用的数值、
布尔与字符串类型及数组类型；运行现有记录器 YAML 测试。
