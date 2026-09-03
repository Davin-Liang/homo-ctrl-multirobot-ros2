# Task 1 报告：4D 仅前向预测测试基线

## 范围

本任务仅建立测试基线并将测试注册到 CTest。未实现 production 脚本中的任何新预测功能；未修改 ROS/C++、launch 或既有 results。

## TDD 记录

### RED

新增 `homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py` 后运行：

```text
python3 -m pytest -q homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py
```

结果：`2 failed, 1 passed`。

两个失败均为预期的后续功能缺失：

1. `predict_follower_state_first_order` 尚不存在，触发 `AttributeError`。
2. `plot_circle_compare` 当前仍接受四个位置参数，三组绘图接口调用触发参数数量 `TypeError`。

第二个测试通过，说明现有延迟 plant 样本基线可加载，并且 `forward_prediction_only` 当前尚未改变真实 plant 的延迟采样约束。

直接运行 `pytest ...` 失败是环境原因（`pytest: command not found`）；使用 `python3 -m pytest` 成功执行测试并得到上述预期 RED。

## 改动

- 新增三个行为测试：一阶闭式预测、仅预测组延迟 plant/采样、三组 summary/plot 接口。
- 在 `homo_multirobot_formation_control/CMakeLists.txt` 的 `BUILD_TESTING` 区域注册 `test_sim_4d_hpc_artstein_compare`。

## 自审

`git diff --check` 无输出；改动仅涉及上述测试文件与 CMake 测试注册，未纳入工作区中其他未跟踪的分析结果、文档或编辑器文件。

## 后续

Task 2 应实现测试要求的接口后，再运行同一测试文件验证 GREEN；本任务不应提前添加生产实现。

---

## 审查补充：覆盖逐样本延迟 plant、prediction-only 隔离与既有输出兼容性

### 范围

仅扩展 `homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py` 的 Task 1 测试；没有修改 production 脚本、ROS/C++、launch、CMake 或 results。CTest 注册已由原 Task 1 基线提供，无需改动测试依赖。

### 新增约束

1. `test_real_follower_applies_every_command_after_delay_and_motor_lag`：对 `simulate_delay_case` 和 `simulate_circle_case` 的 `forward_prediction_only` 全部样本重放 `cmd_vel -> Td delay line -> tau` 一阶电机更新，并逐样本核对记录的真实 follower 状态。初始命令为零，延迟长度为 `ceil(Td / h)`。
2. `test_prediction_only_uses_each_measured_state_without_artstein_or_td`：用非零位置/速度噪声和固定 seed 重建每帧 Leader/Follower 测量。逐帧断言 `x1_ctrl` 等于测得 Leader，且 `x2_ctrl` 仅为 `predict_follower_state_first_order(measured_follower, last_command, tau)`；预期值不含 `Td` 或 Artstein 历史项。
3. `test_existing_csv_case_names_and_plot_labels_are_preserved`：执行主入口并断言原始/Artstein 的六个既有 CSV case 名、三个既有图例标签以及四个既有 PNG 文件名仍存在。

### RED 验证

运行：

```text
python3 -m pytest -q homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py
```

输出摘要：

```text
F..FF.                                                                   [100%]
3 failed, 3 passed, 1 warning in 2.30s
```

预期 RED 失败为：

1. `test_first_order_prediction_matches_closed_form`：`AttributeError`，缺少 `predict_follower_state_first_order`。
2. `test_prediction_only_uses_each_measured_state_without_artstein_or_td`：同一缺失接口导致 `AttributeError`；接口实现后将逐样本检查测量状态与纯 `tau` 预测。
3. `test_three_group_summary_and_plot_include_prediction_only`：`plot_circle_compare()` 仍只接受原有四个位置参数，触发 `TypeError`。

通过的三项包括原有 delayed-plant 首样本基线、本次逐样本真实 follower 重放、以及既有 CSV/图例/文件名兼容性保护。唯一 warning 是环境中 Matplotlib 的 `Axes3D` 导入告警，与本测试断言无关。

### 后续

Task 2 必须在不改变原始/Artstein CSV case 名和输出标签的前提下，提供一阶预测接口与三组绘图/summary 行为；随后使用同一命令验证 GREEN。

---

## 第二轮审查补充：队尾排空、双场景反馈隔离与既有组不变性

### 范围

仅修改 `homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py`，并续写本报告。没有修改 production Python 脚本、ROS/C++、launch、CMake 或 results。

### 新增/强化约束

1. `test_real_follower_drains_every_command_through_delay_and_motor_lag`：对 `simulate_delay_case` 和 `simulate_circle_case` 的 `forward_prediction_only`，在 `Td=0.22`、`h=0.01` 下运行 `Td + 0.30 = 0.52 s`。逐样本重放真实 plant 的 `cmd_vel -> Td delay line -> tau` 一阶电机更新并比对真实 follower；然后排空余下的 `Td` 命令队列，断言每一条已生成命令恰好按原顺序进入链路，不留未验证队尾。
2. `test_delay_prediction_only_uses_measurements_and_first_order_follower_prediction` 与 `test_circle_prediction_only_uses_each_measured_state_without_artstein_or_td`：分别重建 delay 和 circle 场景每一帧的反馈。前者验证 Leader 反馈是当前真实测量状态；后者用固定噪声 seed 重建 Leader/Follower 测量。两者均逐帧要求 Follower 反馈严格等于 `predict_follower_state_first_order(measured_follower, last_command, tau)`，不包含 Artstein 状态、命令历史或 `Td`。
3. `test_original_and_compensated_short_runs_are_numerically_unchanged`：为 delay/circle 两个短场景中原始组、Artstein 补偿组的末样本建立固定数值回归（真实 follower、两路控制反馈与命令），防止三组扩展意外改动既有结果。
4. `test_existing_csv_case_names_and_plot_legends_are_preserved`：从对应 plot 函数的实际 axes/legend 读取标签，分别保护 delay/circle 图中原始和补偿标签所在坐标轴；CSV 断言 case 名唯一，且过滤新 prediction-only 行后既有行相对顺序严格保持。

### RED 验证

运行：

```text
python3 -m pytest -q homo_multirobot_formation_control/test/test_sim_4d_hpc_artstein_compare.py
```

结果：`4 failed, 4 passed, 1 warning in 2.37s`。

四项失败均为后续 production 工作尚未完成，且与测试断言直接对应：

1. `test_first_order_prediction_matches_closed_form` 缺少 `predict_follower_state_first_order`。
2. delay 场景逐帧 feedback 隔离测试因同一缺失接口失败。
3. circle 场景逐帧 feedback 隔离测试因同一缺失接口失败。
4. 三组 circle plot 测试调用五个位置参数，而当前 `plot_circle_compare` 仍只支持两组比较，触发 `TypeError`。

其余四项通过，包含新的 0.52 s 逐样本延迟/电机重放与队尾排空、既有组数值回归、以及 axes/legend + CSV 兼容性保护。唯一 warning 是环境中的 Matplotlib `Axes3D` 导入告警，和断言无关。

### 后续

Task 2 应实现一阶 Follower 闭式预测，将仅预测组用于两个 simulation 场景，并扩展三组 plot/summary；实现不得改变本报告所固定的 original/compensated 数值、标签和 CSV 既有行相对顺序。完成后使用上述命令验证 GREEN。
