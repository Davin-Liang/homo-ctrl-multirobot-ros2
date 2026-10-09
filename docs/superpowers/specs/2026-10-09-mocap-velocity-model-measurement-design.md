# 动捕速度响应模型测试设计

## 目标

提供一个 ROS 2 Python 实车测试脚本，通过重复阶跃实验比较 mini_omni 的速度响应更符合一阶滞后模型还是固定加速度模型。脚本使用动捕速度作为测量真值，同时保留全部原始数据以供离线复核。

## 输入与状态源

脚本新增为 `measure_mocap_velocity_model.py`，默认测试 `/robot2`。

- 命令话题：`/<namespace>/cmd_vel`。
- 测量话题：`/<namespace>/mocap/twist`，类型为 `geometry_msgs/msg/TwistStamped`，其中线速度处于 map 全局坐标系。
- `--namespace` 可选择待测机器人，默认 `/robot2`。
- `--mode move|turn` 必填选择测试类型。
- `--target` 必填指定目标幅值；`move` 的单位为 m/s，`turn` 的单位为 rad/s。

`move` 不区分前进、后退或横移：脚本只发布 `linear.x=target`，并用全局动捕速度的大小 `sqrt(vx_map^2 + vy_map^2)` 作为标量响应。`turn` 发布 `angular.z=target`，直接使用动捕的 `angular.z`。

## 实验序列与安全

每个试次执行：确认静止 → 发布目标阶跃并保持 → 发布零速度并等待静止。默认值应保守：`move` 目标建议 0.15 m/s、`turn` 目标建议 0.30 rad/s，均可经 `--target` 覆盖。

- `--trials` 指定重复次数，默认 5。
- `--hold-seconds`、`--settle-seconds` 分别指定目标保持和停稳等待时长。
- 动捕超过 `--mocap-timeout` 未更新时，立即停止当前试验，发布零速度并以非零状态退出。
- Ctrl-C、异常与所有正常结束路径均发布零 `Twist`。
- 单次测试只激励一个自由度；不同时测试平移和偏航。

## 数据和模型比较

每次运行创建独立目录，默认位于：

```
homo_multirobot_formation_control/robot_traj/real/model_identification/<timestamp>/
```

`--out-dir` 可指定另一输出根目录。目录包含：

- `samples.csv`：试次编号、阶段、单调相对时间、ROS 接收时间、动捕 header 时间、命令 `vx/vy/wz`、原始 `vx_map/vy_map/wz` 以及用于拟合的标量响应。
- `fit_summary.csv`：每个试次上升段和下降段分别对应的一阶模型、固定加速度模型的参数、SSE、RMSE、MAE、AIC 与较优模型。
- `metadata.json`：命令行参数、话题、开始时间、软件版本信息和完成状态。
- `comparison.png`：每个有效试次的观测速度、命令、一阶拟合与固定加速度拟合叠加图。

一阶模型针对阶跃输入拟合时间常数 `tau`；固定加速度模型拟合加速或减速的常量斜率。两个模型在同一有效样本上比较，按较小 AIC 标记单段优选模型。该结果只描述当前 `mode`、`target` 与场地条件下的响应，不自动修改控制器参数。

## 验证

- 纯 Python 单元测试覆盖平移合速度提取、两种拟合函数、AIC 选择、CSV 字段与异常样本处理；不要求 ROS 或实车。
- CLI 测试覆盖 `move`/`turn` 参数、目标幅值和默认输出目录。
- 静态检查确认所有退出路径调用零速度发布。
- ROS 构建后可使用 `--help` 验证 CLI；真实命令仅在用户选择并清空测试区域后运行。
