# 轨迹记录器 YAML 参数 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 使 `record_trajectory.py` 从 ROS 2 标准 YAML 读取记录参数，并允许显式 ROS 参数覆盖。

**Architecture:** 将 YAML 解析、结构校验和覆盖合并提取为不依赖 ROS 图通信的辅助函数。节点先读取 `config_file` 的 ROS 参数，然后加载 YAML；每个记录器参数取 YAML 默认值，若该参数存在于 rclpy 的 `_parameter_overrides` 则取覆盖值，最后以 `ignore_override=True` 声明最终值，避免二次覆盖。默认 YAML 随 `config/` 安装。

**Tech Stack:** Python 3、rclpy、PyYAML、ament_cmake、unittest。

## Global Constraints

- YAML 格式必须为 `/**` → `ros__parameters`，与 `formation_single_follower_4d_artstein.yaml` 一致。
- `config_file` 为空时读取包内 `config/record_trajectory.yaml`；非空时读取指定路径。
- YAML 默认值可被显式 `--ros-args -p <name>:=<value>` 覆盖。
- 无效 YAML 必须输出明确错误并退出；不得改变轨迹订阅、数据保存、绘图、控制器查询或输出格式。
- 在工作空间根目录 `/home/l1anggmgo/ros-projects/homo_multirobot_ws` 构建。

---

### Task 1: 实现 YAML 参数加载与回归覆盖

**Files:**
- Create: `homo_multirobot_formation_control/config/record_trajectory.yaml`
- Create: `homo_multirobot_formation_control/test/test_record_trajectory_yaml.py`
- Modify: `homo_multirobot_formation_control/scripts/record_trajectory.py:1-94`
- Modify: `homo_multirobot_formation_control/CMakeLists.txt:245-250`
- Modify: `homo_multirobot_formation_control/README.md:596-635`

**Interfaces:**
- Consumes: a YAML file whose root contains `/**: {ros__parameters: {...}}`.
- Produces: `load_recorder_parameters(path: str) -> dict[str, object]` and `merge_parameter_overrides(defaults: dict[str, object], overrides: dict[str, object]) -> dict[str, object]`.
- Runtime parameter: `config_file` (`str`), empty for packaged default or a user-supplied path.

- [ ] **Step 1: Write failing tests**

Create `test/test_record_trajectory_yaml.py` that imports the script with `importlib.util.spec_from_file_location` and tests the pure helpers:

```python
def test_default_yaml_has_all_recorder_parameters():
    defaults = module.load_recorder_parameters(PACKAGE_DIR / "config" / "record_trajectory.yaml")
    assert defaults["leader_ns"] == "/robot1"
    assert defaults["duration"] == 30.0
    assert defaults["trial_id"] == "trial_01"
    assert set(defaults) == set(module.RECORDER_PARAMETER_DEFAULTS)


def test_explicit_overrides_win_over_yaml_defaults():
    values = module.merge_parameter_overrides(
        {"duration": 30.0, "mode": "sim"}, {"duration": 60.0})
    assert values == {"duration": 60.0, "mode": "sim"}


def test_invalid_yaml_structure_raises_value_error(tmp_path):
    path = tmp_path / "invalid.yaml"
    path.write_text("ros__parameters: {}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="/\\*\\*/ros__parameters"):
        module.load_recorder_parameters(path)
```

- [ ] **Step 2: Run the tests to verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest homo_multirobot_formation_control/test/test_record_trajectory_yaml.py -q
```

Expected: FAIL because the module does not expose the helpers and the default YAML does not exist.

- [ ] **Step 3: Implement the minimal YAML integration**

1. Import `yaml` and `get_package_share_directory`.
2. Add `RECORDER_PARAMETER_DEFAULTS` containing exactly these existing parameters and current defaults:

```python
{
    "leader_ns": "/robot1", "follower_ns": "/robot2", "duration": 30.0,
    "out_dir": "", "radius": 0.0, "mode": "sim", "tag": "",
    "controller_node_name": "formation_control_node", "experiment_id": "",
    "trial_id": "trial_01", "platform": "", "controller": "",
}
```

3. Implement `load_recorder_parameters(path)` with `yaml.safe_load`; it raises `ValueError` when the file cannot yield a mapping at `/**.ros__parameters`, when that section is not a mapping, when keys differ from `RECORDER_PARAMETER_DEFAULTS`, or when parsing fails. Error text identifies the file and structural reason.
4. Implement `merge_parameter_overrides(defaults, overrides)` as a shallow copy updated only with override keys that are already in `defaults`.
5. In `TrajectoryRecorder.__init__`, declare `config_file` before all other recorder parameters. Resolve an empty value to `get_package_share_directory("homo_multirobot_formation_control")/config/record_trajectory.yaml`; load defaults; merge `self._parameter_overrides` values for the recorder parameter names; then declare each final recorder parameter with `ignore_override=True`. Preserve the existing assignments and all behavior after them.
6. Create `config/record_trajectory.yaml` with the required ROS 2 wrapper and all values from `RECORDER_PARAMETER_DEFAULTS`.
7. Register the test in CMake inside `if(BUILD_TESTING)`:

```cmake
add_test(NAME test_record_trajectory_yaml
  COMMAND ${Python3_EXECUTABLE} -m pytest -q
  ${CMAKE_CURRENT_SOURCE_DIR}/test/test_record_trajectory_yaml.py)
```

8. Update README with a default-YAML command and a custom YAML command:

```bash
ros2 run homo_multirobot_formation_control record_trajectory.py
ros2 run homo_multirobot_formation_control record_trajectory.py \
  --ros-args -p config_file:=/abs/path/record.yaml -p duration:=60.0
```

State that the second command overrides the file's `duration`.

- [ ] **Step 4: Run focused verification**

Run:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest homo_multirobot_formation_control/test/test_record_trajectory_yaml.py -q
python3 -m py_compile homo_multirobot_formation_control/scripts/record_trajectory.py
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=OFF
test -f install/homo_multirobot_formation_control/share/homo_multirobot_formation_control/config/record_trajectory.yaml
```

Expected: all commands succeed; the test confirms default parsing, override precedence, and invalid-structure rejection; installed configuration exists.

- [ ] **Step 5: Commit**

```bash
git add homo_multirobot_formation_control/config/record_trajectory.yaml \
  homo_multirobot_formation_control/test/test_record_trajectory_yaml.py \
  homo_multirobot_formation_control/scripts/record_trajectory.py \
  homo_multirobot_formation_control/CMakeLists.txt \
  homo_multirobot_formation_control/README.md
git commit -m "轨迹记录器支持YAML参数"
```
