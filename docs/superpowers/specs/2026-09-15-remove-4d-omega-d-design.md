# 移除 4D omega_d 参数设计

## 目标

移除原始 4D 与 4D Artstein 控制器中不再具有独立调参意义的 `omega_d` 接口。
两种 4D 控制器均直接使用 `initial_min_lambda` 与
`switch_min_lambda` 定义反馈增益下界。

## 设计

- 共用的 `LpcController` 删除 `omega_d` 构造参数、成员变量及其作为 lambda
  默认值的兜底逻辑；两个 lambda 的默认值固定为 4D 当前接口值 `1.5`、`4.0`。
- 原始 4D 与 4D Artstein 节点、wrapper、launch、YAML、README、实验规划和轨迹
  标签说明中删除 `omega_d`。
- 6D Map 已不提供该参数，保持不变；普通 6D、6D Disc 与 6D OA 的
  `omega_d` 仍参与控制，保持不变。

## 兼容性与验证

原有依赖 4D `omega_d` 的启动命令将不再被接受，用户应改用两个 lambda 调参。
更新 C++ 构造调用和接口测试，构建 4D 控制器相关目标并运行对应测试；使用
仓库搜索确认 4D 文档和配置不再暴露该参数。
