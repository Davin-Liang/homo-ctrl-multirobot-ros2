# 无前馈敏感性实验统计 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 从 30 份无前馈实物轨迹生成可复算的逐次与组级统计，并写入实验统计表。

**Architecture:** 新增独立 Python 分析脚本；它内置两类参数组、加载 CSV 和元数据、输出逐次指标与三次试验的均值 ± 样本标准差。统计表从脚本的 Markdown 输出更新，确保可复算。

**Tech Stack:** Python 3、csv、math、statistics、PyYAML、pytest、Markdown。

## Global Constraints

- 仅接受 `mode: real` 与 `controller_parameters.enable_leader_cmd_feedforward: false` 的记录。
- 每次期望间距读取 `recording.ideal_radius_m`，必须为正有限数。
- 末 10 s 为 `time_s >= max(time_s) - 10.0`。
- 组统计为逐次指标的算术均值和 `statistics.stdev`，不池化采样点。
- 距离和速度输出固定四位小数，轨迹原始文件不改写。

---

### Task 1: 创建可测试的敏感性统计器

**Files:**
- Create: `homo_multirobot_formation_control/scripts/analyze_no_feedforward_sensitivity.py`
- Create: `homo_multirobot_formation_control/test/test_analyze_no_feedforward_sensitivity.py`

**Interfaces:**
- Consumes: `raw.csv` 的 `time_s`、`distance_m`、`leader_vx_map_ms`、`leader_vy_map_ms`、`follower_vx_map_ms`、`follower_vy_map_ms`，以及 `metadata.yaml`。
- Produces: `compute_trial_metrics(rows, radius_m)` 返回六项确认指标；`aggregate_trials(trials)` 返回每项的均值、样本标准差；CLI 输出 Markdown。

- [ ] **Step 1: 写出失败测试**

```python
def test_compute_trial_metrics_uses_absolute_error_and_last_ten_seconds():
    rows = [
        {"time_s": 0.0, "distance_m": 0.8, "leader_vx_map_ms": 1.0, "leader_vy_map_ms": 0.0, "follower_vx_map_ms": 0.0, "follower_vy_map_ms": 0.0},
        {"time_s": 5.0, "distance_m": 1.0, "leader_vx_map_ms": 1.0, "leader_vy_map_ms": 0.0, "follower_vx_map_ms": 1.0, "follower_vy_map_ms": 0.0},
        {"time_s": 10.0, "distance_m": 1.2, "leader_vx_map_ms": 0.0, "leader_vy_map_ms": 1.0, "follower_vx_map_ms": 0.0, "follower_vy_map_ms": 1.0},
        {"time_s": 15.0, "distance_m": 1.1, "leader_vx_map_ms": 0.0, "leader_vy_map_ms": 1.0, "follower_vx_map_ms": 0.0, "follower_vy_map_ms": 2.0},
    ]
    got = module.compute_trial_metrics(rows, 1.0)
    assert got["mean_abs_distance_error_m"] == pytest.approx(0.125)
    assert got["rms_distance_error_m"] == pytest.approx(0.158113883)
    assert got["tail_mean_abs_distance_error_m"] == pytest.approx(0.1)
    assert got["tail_distance_error_std_m"] == pytest.approx(0.081649658)
    assert got["mean_relative_velocity_error_mps"] == pytest.approx(0.5)
    assert got["tail_mean_relative_velocity_error_mps"] == pytest.approx(1.0 / 3.0)
```

- [ ] **Step 2: 运行测试确认失败**

Run: `pytest homo_multirobot_formation_control/test/test_analyze_no_feedforward_sensitivity.py -q`

Expected: FAIL，模块或 `compute_trial_metrics` 不存在。

- [ ] **Step 3: 实现最小指标计算与输入校验**

实现 `compute_trial_metrics`：距离误差为 `distance_m - radius_m`，平均距离误差和末 10 s 距离误差取绝对值均值，RMS 对带符号误差计算，末窗标准差使用 `statistics.pstdev`，相对速度误差使用 map 系二维速度差模长。加载器拒绝缺列、非有限值、时长不足 10 s、错误模式、前馈开启、无效半径和分组参数不符的轨迹；错误消息含目录和原因。

- [ ] **Step 4: 增加汇总与前馈校验测试**

```python
def test_aggregate_trials_uses_sample_standard_deviation():
    summary = module.aggregate_trials([{"mean_abs_distance_error_m": 0.1}, {"mean_abs_distance_error_m": 0.2}, {"mean_abs_distance_error_m": 0.3}])
    assert summary["mean_abs_distance_error_m"] == pytest.approx((0.2, 0.1))
```

再以临时目录的 `enable_leader_cmd_feedforward: true` 元数据调用加载器，并断言 `ValueError`。

- [ ] **Step 5: 通过测试并提交**

Run: `pytest homo_multirobot_formation_control/test/test_analyze_no_feedforward_sensitivity.py -q`

Expected: PASS。

Commit: `git add homo_multirobot_formation_control/scripts/analyze_no_feedforward_sensitivity.py homo_multirobot_formation_control/test/test_analyze_no_feedforward_sensitivity.py && git commit -m '新增无前馈敏感性统计工具'`

### Task 2: 计算真实数据并更新统计文档

**Files:**
- Modify: `homo_multirobot_formation_control/doc/experimental_statistics.md`

**Interfaces:**
- Consumes: Task 1 CLI 的 Markdown 输出。
- Produces: 齐次度和控制频率各一份逐次明细表与组级汇总表。

- [ ] **Step 1: 运行完整分析**

Run: `python3 homo_multirobot_formation_control/scripts/analyze_no_feedforward_sensitivity.py --format markdown`

Expected: 30 次记录、6 个齐次度组、4 个频率组，全部每组 3 次且校验通过。

- [ ] **Step 2: 将生成表写入文档**

在现有清单末尾追加 `## 无前馈敏感性统计结果`，含“齐次度逐次明细”“齐次度组级汇总”“控制频率逐次明细”“控制频率组级汇总”四张表。列为六个用户确认指标；组级单元格形如 `0.1234 ± 0.0123`。

- [ ] **Step 3: 验证与提交**

Run: `pytest homo_multirobot_formation_control/test/test_analyze_no_feedforward_sensitivity.py -q && git diff --check`

Expected: PASS，且无空白错误。人工复核每个组的目录数为 3，数值来自脚本输出。

Commit: `git add homo_multirobot_formation_control/doc/experimental_statistics.md && git commit -m '补充无前馈参数敏感性统计结果'`
