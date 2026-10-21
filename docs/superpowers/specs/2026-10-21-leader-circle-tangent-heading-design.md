# 闭环 Leader 圆轨迹切线航向设计

## 目标

为 `leader_circle_closed_loop.py` 增加可选切线航向，使 Leader 在闭环跟踪圆周位置时车头沿实际参考运动方向转动。现有固定航向行为必须保持为默认值。

## 接口与兼容性

- 新增字符串参数 `heading_mode`，默认 `fixed`。
- `heading_mode:=fixed`：维持现有语义，`heading`（度）为 map 系固定目标 yaw。
- `heading_mode:=tangent`：目标 yaw 从当前前瞻圆参考速度 `v_d` 计算：`atan2(v_d.y, v_d.x)`；`heading` 被忽略。
- 因参考速度已由 `direction` 计算，`ccw` 和 `cw` 自动得到正确的前进切线方向。
- 对未知的 `heading_mode` 记录警告并回退到 `fixed`，避免静默改变已有实验的航向行为。

## 控制流程

节点继续用 `elapsed + Td + tau_v` 和相位回退生成位置、速度前瞻参考。线速度控制器、预测器、map/body 变换以及线速度限幅均不改变。

在 yaw 环中选择目标角：固定模式读取配置的 `heading`；切线模式读取同一轮控制中已生成的 `reference_velocity`。之后仍使用原来的角度 wrap、比例反馈、最大角速度和最大角加速度限制。因此启动、模式切换和跟踪误差均不会绕过既有转动安全约束。

## 测试与文档

- 为纯函数增加测试：CCW 与 CW 圆参考速度对应的切线 yaw 正确。
- 覆盖固定模式仍返回配置的 `heading`，以及切线模式返回参考速度方向。
- 更新包 README 的节点说明、参数表和一条 `heading_mode:=tangent` 示例。

不新增独立节点、launch 文件或额外方向参数；切线模式始终让车头朝前进方向。
