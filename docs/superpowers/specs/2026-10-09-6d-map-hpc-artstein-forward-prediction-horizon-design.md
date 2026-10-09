# 6D Map HPC Artstein 独立前向预测时域设计

## 目标

将 6D map HPC Artstein 控制器的前向预测时域从电机响应时间常数中解耦，以便实验时独立调节预测距离，同时保持所有既有配置的控制行为不变。

## 参数与兼容性

新增 ROS 参数 `forward_prediction_horizon`，单位为秒。

- 正值：平移、偏航的 Follower 前向预测均使用该时域 `H`；Leader 的总预测时域为启用的 Artstein 纯滞后补偿 `Td` 加 `H`。
- 零或负值：自动使用原先的 `max(tau, tau_yaw)`，因此现有 YAML、launch 参数和未显式传参的用户保持原有行为。
- `enable_forward_prediction:=false`：不执行额外前向预测；`forward_prediction_horizon` 不影响输出。

`tau` 和 `tau_yaw` 继续只描述平移和偏航各自的一阶速度响应模型，不再隐式决定已显式配置的预测时域。

## 预测模型

给定一阶速度响应模型、当前速度 `v`、保持不变的命令 `u`、时间常数 `tau` 和前向时域 `H`，预测器使用：

```
decay = exp(-H / tau)
v(H) = u + decay * (v - u)
p(H) = p + u * H + tau * (1 - decay) * (v - u)
```

保留现有 Artstein 历史积分和 `Td` 外推的执行顺序；上式仅替换原先固定为一个 `tau` 的额外前向部分。

## 改动范围

- 扩展 `ArtsteinPredictorNd::predict()` 以接收前向时域，保留原有调用的默认语义。
- 在 `FormationController6DMapHpcArtstein` 中解析回退后的有效时域，并传给 Leader 与 Follower 预测路径。
- 在 YAML、普通 launch 和 mocap launch 中公开该参数并注明回退规则。
- 更新相关说明文档。

不修改 4D Artstein、普通 6D Artstein Disc 或 HOCBF 节点。

## 验证

- 单元测试证明指定 `H` 时一阶位置、速度预测符合解析公式。
- 单元测试证明使用默认/回退时域时结果与旧的单 `tau` 预测一致。
- Python 配置测试确认 YAML 和两个 launch 文件均传递新参数。
- 构建目标包并执行受影响的 CTest。
