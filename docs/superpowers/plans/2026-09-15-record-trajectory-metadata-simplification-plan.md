# 轨迹记录元数据精简 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 删除轨迹记录器重复的 `platform` 与 `experiment_id` 参数和新元数据字段。

**Architecture:** `mode` 继续表示运行环境与输出子目录，目录标签继续表示实验组，`trial_id` 保留表示重复试验。仅修改记录器参数默认集合、运行时读取和 metadata 序列化；历史轨迹文件不改写。

**Tech Stack:** ROS 2 Python (`rclpy`)、PyYAML、pytest、Markdown。

## Global Constraints

- 不修改 `robot_traj` 下的历史 CSV、PNG 或 metadata。
- 只移除 `platform` 与 `experiment_id`，不重构其他记录器接口。
- 使用现有 `test_record_trajectory_yaml.py` 的动态导入方式验证。

---

### Task 1: 删除记录器参数和 metadata 字段

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/record_trajectory.py:56-69,248-253,569-575`
- Modify: `homo_multirobot_formation_control/config/record_trajectory.yaml:16-19`
- Test: `homo_multirobot_formation_control/test/test_record_trajectory_yaml.py:16-28`

**Interfaces:**
- Produces: 新记录器不提供 `platform`、`experiment_id` 默认参数，新生成 metadata 顶层不写两个键。

- [ ] **Step 1: 写入失败测试**

```python
def test_recorder_defaults_do_not_include_redundant_metadata_parameters():
    defaults = module.load_recorder_parameters(
        PACKAGE_DIR / "config" / "record_trajectory.yaml")
    assert "platform" not in defaults
    assert "experiment_id" not in defaults
    assert "platform" not in module.RECORDER_PARAMETER_DEFAULTS
    assert "experiment_id" not in module.RECORDER_PARAMETER_DEFAULTS
```

- [ ] **Step 2: 确认测试失败**

Run: `source /opt/ros/humble/setup.bash && pytest -q homo_multirobot_formation_control/test/test_record_trajectory_yaml.py::test_recorder_defaults_do_not_include_redundant_metadata_parameters`

Expected: FAIL，因为两个旧参数仍存在。

- [ ] **Step 3: 最小实现**

从 `RECORDER_PARAMETER_DEFAULTS`、构造函数参数读取和 `_save_metadata()` 顶层映射删除 `platform` 与 `experiment_id`；从 YAML 删除对应键。

- [ ] **Step 4: 确认记录器测试通过**

Run: `source /opt/ros/humble/setup.bash && pytest -q homo_multirobot_formation_control/test/test_record_trajectory_yaml.py`

Expected: PASS。

### Task 2: 清理使用文档

**Files:**
- Modify: `homo_multirobot_formation_control/README.md:643-678`
- Modify: `homo_multirobot_formation_control/doc/chapter3_experiment_plot_plan.md:532-537,610-616`
- Test: `homo_multirobot_formation_control/test/test_record_trajectory_yaml.py`

**Interfaces:**
- Produces: README 和实验规划仅描述仍可使用、仍会写入的字段。

- [ ] **Step 1: 写入失败测试**

```python
def test_recorder_readme_does_not_document_removed_metadata_parameters():
    readme = (PACKAGE_DIR / "README.md").read_text(encoding="utf-8")
    assert "-p experiment_id:=" not in readme
    assert "-p platform:=" not in readme
    assert "| `experiment_id`" not in readme
    assert "| `platform`" not in readme
```

- [ ] **Step 2: 确认测试失败**

Run: `source /opt/ros/humble/setup.bash && pytest -q homo_multirobot_formation_control/test/test_record_trajectory_yaml.py::test_recorder_readme_does_not_document_removed_metadata_parameters`

Expected: FAIL，因为 README 仍展示旧接口。

- [ ] **Step 3: 最小文档修改**

删除 README 示例中的两个 `-p` 参数及参数表的两行；从实验绘图计划的 metadata 字段清单删除 `platform`。

- [ ] **Step 4: 确认测试通过**

Run: `source /opt/ros/humble/setup.bash && pytest -q homo_multirobot_formation_control/test/test_record_trajectory_yaml.py`

Expected: PASS。

### Task 3: 完整验证和提交

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/record_trajectory.py`
- Modify: `homo_multirobot_formation_control/config/record_trajectory.yaml`
- Modify: `homo_multirobot_formation_control/test/test_record_trajectory_yaml.py`
- Modify: `homo_multirobot_formation_control/README.md`
- Modify: `homo_multirobot_formation_control/doc/chapter3_experiment_plot_plan.md`

- [ ] **Step 1: 搜索旧字段引用**

Run: `rg -n "experiment_id|platform" homo_multirobot_formation_control -g '*.py' -g '*.yaml' -g '*.md'`

Expected: 不显示记录器、配置、README、测试或实验规划中的旧接口引用。

- [ ] **Step 2: 运行关联测试**

Run: `source /opt/ros/humble/setup.bash && pytest -q homo_multirobot_formation_control/test/test_record_trajectory_yaml.py homo_multirobot_formation_control/test/test_launch_yaml_defaults.py`

Expected: PASS。

- [ ] **Step 3: 检查变更范围**

Run: `git diff --check`

Expected: 无空白错误，且不包含 `robot_traj` 历史文件修改。

- [ ] **Step 4: 提交实现**

Run: `git add docs/superpowers/plans/2026-09-15-record-trajectory-metadata-simplification-plan.md homo_multirobot_formation_control/scripts/record_trajectory.py homo_multirobot_formation_control/config/record_trajectory.yaml homo_multirobot_formation_control/test/test_record_trajectory_yaml.py homo_multirobot_formation_control/README.md homo_multirobot_formation_control/doc/chapter3_experiment_plot_plan.md && git commit -m "精简轨迹记录元数据"`
