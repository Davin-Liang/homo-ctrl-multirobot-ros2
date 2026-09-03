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
