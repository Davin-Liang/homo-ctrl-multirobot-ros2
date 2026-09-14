# 控制器参数元数据快照 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让轨迹记录器在 `metadata.yaml` 自动保存目标控制器节点全部当前生效的 ROS 参数。

**Architecture:** 记录器以 `list_parameters` 枚举参数名称，再以 `get_parameters` 读取值。独立转换函数把 `ParameterValue` 变为 YAML 原生值；延迟节点的既有查询不变，PNG 不显示参数摘要。

**Tech Stack:** Python 3、ROS 2 Humble `rcl_interfaces` 参数服务、PyYAML、pytest。

## Global Constraints

- 仅影响今后记录生成的目录，不修改已有 `metadata.yaml`。
- `controller_parameters` 含控制器公开的全部当前参数，支持标量和数组。
- 控制器服务不可用时沿用现有等待/重试逻辑；最终仍以空快照继续记录。
- PNG 不显示控制器参数摘要。

---

### Task 1: 参数枚举和值转换

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/record_trajectory.py:25-159`
- Test: `homo_multirobot_formation_control/test/test_record_trajectory_yaml.py`

**Interfaces:**
- Produces: `parameter_value_to_python(parameter_value) -> bool | int | float | str | list | None`。
- Produces: `wait_for_controller_parameter_names(node, client, service_name, logger) -> list[str] | None`。
- Consumes: `rcl_interfaces.srv.ListParameters`。

- [ ] **Step 1: Write failing conversion tests**

```python
@pytest.mark.parametrize(("parameter_value", "expected"), [
    (SimpleNamespace(type=1, bool_value=True), True),
    (SimpleNamespace(type=2, integer_value=20), 20),
    (SimpleNamespace(type=3, double_value=0.43), 0.43),
    (SimpleNamespace(type=4, string_value="mocap"), "mocap"),
    (SimpleNamespace(type=6, bool_array_value=[True, False]), [True, False]),
    (SimpleNamespace(type=7, integer_array_value=[1, 4]), [1, 4]),
    (SimpleNamespace(type=8, double_array_value=[0.1, 0.2]), [0.1, 0.2]),
    (SimpleNamespace(type=9, string_array_value=["a", "b"]), ["a", "b"]),
])
def test_parameter_value_to_python(parameter_value, expected):
    assert module.parameter_value_to_python(parameter_value) == expected
```

Also test type `0` yields `None`.

- [ ] **Step 2: Run the test and verify RED**

Run: `python3 -m pytest -q homo_multirobot_formation_control/test/test_record_trajectory_yaml.py -k parameter_value_to_python`

Expected: FAIL because the conversion function is undefined.

- [ ] **Step 3: Implement the minimal helpers**

Import `ListParameters` and use this explicit field map:

```python
PARAMETER_VALUE_FIELDS = {
    1: "bool_value", 2: "integer_value", 3: "double_value", 4: "string_value",
    5: "byte_array_value", 6: "bool_array_value", 7: "integer_array_value",
    8: "double_array_value", 9: "string_array_value",
}
```

`parameter_value_to_python` returns the named scalar or `list(value)` for types 5-9. The list helper sends `ListParameters.Request()` with no prefixes and depth zero, uses the existing bounded async wait/retry pattern, and returns sorted unique response names.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run: `python3 -m pytest -q homo_multirobot_formation_control/test/test_record_trajectory_yaml.py -k parameter_value_to_python`

Expected: PASS.

- [ ] **Step 5: Commit helper work**

```bash
git add homo_multirobot_formation_control/scripts/record_trajectory.py homo_multirobot_formation_control/test/test_record_trajectory_yaml.py
git commit -m '支持转换控制器全量参数值'
```

### Task 2: 全量控制器快照与图表摘要移除

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/record_trajectory.py:200-350, 590-670`
- Test: `homo_multirobot_formation_control/test/test_record_trajectory_yaml.py`

**Interfaces:**
- Consumes: Task 1 的两个 helper。
- Produces: `_query_controller_params()` 返回所有非 `PARAMETER_NOT_SET` 的 `{str: YAML-safe value}`。

- [ ] **Step 1: Write failing query tests**

Use fake list response names `['use_hpc', 'tau', 'state_source']` and matching bool/double/string values; assert:

```python
{"state_source": "ekf_tf", "tau": 0.43, "use_hpc": True}
```

Also assert type `0` is omitted and an empty name list returns `{}` without calling `get_parameters`.

- [ ] **Step 2: Run the tests and verify RED**

Run: `python3 -m pytest -q homo_multirobot_formation_control/test/test_record_trajectory_yaml.py -k query_controller_params`

Expected: FAIL because fixed `CTRL_PARAM_NAMES` are still requested.

- [ ] **Step 3: Replace the whitelist query and delete plot summary**

Create a `ListParameters` client at `node_path + '/list_parameters'`, wait for it, enumerate names, then invoke the existing get-parameters client with those names. Zip names to response values, call `parameter_value_to_python`, and retain only non-`None` values. Remove `CTRL_PARAM_NAMES`, `self.ctrl_title`, `_build_params_title`, and the `fig.suptitle(...)` block. Keep auto-tag generation, which reads its selected fields from the complete map.

- [ ] **Step 4: Run all recorder tests and verify GREEN**

Run: `python3 -m pytest -q homo_multirobot_formation_control/test/test_record_trajectory_yaml.py`

Expected: PASS.

- [ ] **Step 5: Run syntax and package build validation**

```bash
python3 -m py_compile homo_multirobot_formation_control/scripts/record_trajectory.py
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

Expected: Python succeeds and colcon reports one finished package.

- [ ] **Step 6: Commit feature work**

```bash
git add homo_multirobot_formation_control/scripts/record_trajectory.py homo_multirobot_formation_control/test/test_record_trajectory_yaml.py
git commit -m '完整记录控制器参数元数据'
```

### Task 3: Final verification

**Files:**
- Verify: `homo_multirobot_formation_control/scripts/record_trajectory.py`
- Verify: `homo_multirobot_formation_control/test/test_record_trajectory_yaml.py`

- [ ] **Step 1: Review the final feature diff**

Run: `git diff HEAD~1 -- homo_multirobot_formation_control/scripts/record_trajectory.py homo_multirobot_formation_control/test/test_record_trajectory_yaml.py`

Expected: only parameter enumeration/conversion and plot-summary removal.

- [ ] **Step 2: Re-run recorder tests**

Run: `python3 -m pytest -q homo_multirobot_formation_control/test/test_record_trajectory_yaml.py`

Expected: PASS.

- [ ] **Step 3: Confirm the runtime metadata contract**

After launching a controller and recorder, inspect a new `metadata.yaml`: `controller_parameters` contains keys including `use_hpc`, `state_source`, `wheel_radius`, and `enable_leader_cmd_feedforward`; its `check.png` lacks a top parameter summary.
