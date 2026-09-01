# 编队控制 Launch YAML 参数加载 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为五个指定编队控制 launch 提供固定 YAML 默认参数文件，且原有命令行 launch 参数可覆盖 YAML 值。

**Architecture:** 每个 launch 在生成描述时读取其固定 YAML，并将 `ros__parameters` 值作为同名 launch argument 的默认值。控制器节点的 `parameters` 列表再先传入 YAML 路径、后传入由 `LaunchConfiguration` 构成的参数字典；这使未指定命令行参数时采用 YAML 默认值，而 `name:=value` 可覆盖它。YAML 只承担默认值，不引入可选参数文件或新的运行时接口。

**Tech Stack:** ROS 2 Humble launch/launch_ros、ROS 2 参数 YAML、Python `unittest`、ament_cmake 安装规则。

## Global Constraints

- 固定 YAML 映射，不新增 `params_file` launch 参数。
- 保持现有 launch argument 名称、描述和默认行为。
- YAML 初始值必须逐项等于改动前对应 launch 默认值。
- `parameters` 顺序必须为 YAML、再为 launch 参数字典。
- 仅改动 `homo_multirobot_formation_control` 与本计划/测试文件；不触碰已有未跟踪结果。

---

### Task 1: 添加覆盖顺序与参数一致性测试

**Files:**
- Create: `homo_multirobot_formation_control/test/test_launch_yaml_defaults.py`
- Test: `homo_multirobot_formation_control/test/test_launch_yaml_defaults.py`

**Interfaces:**
- Consumes: 五个 launch 文件与预期 YAML 文件名。
- Produces: `test_yaml_keys_match_declared_launch_arguments()` 与 `test_launch_loads_yaml_before_launch_argument_overrides()`。

- [ ] **Step 1: Write the failing test**

```python
LAUNCH_TO_YAML = {
    "formation_single_follower.launch.py": "formation_single_follower.yaml",
    "formation_single_follower_4d_artstein.launch.py": "formation_single_follower_4d_artstein.yaml",
    "formation_single_follower_6d_disc.launch.py": "formation_single_follower_6d_disc.yaml",
    "formation_single_follower_6d_artstein_disc_hocbf.launch.py": "formation_single_follower_6d_artstein_disc_hocbf.yaml",
    "formation_single_follower_6d_map_hpc_artstein.launch.py": "formation_single_follower_6d_map_hpc_artstein.yaml",
}

def test_yaml_keys_match_declared_launch_arguments():
    for launch_name, yaml_name in LAUNCH_TO_YAML.items():
        assert (CONFIG_DIR / yaml_name).is_file()
        assert declared_argument_names(LAUNCH_DIR / launch_name) == yaml_parameter_names(CONFIG_DIR / yaml_name)

def test_launch_uses_yaml_for_argument_defaults_and_node_parameters():
    for launch_name, yaml_name in LAUNCH_TO_YAML.items():
        source = (LAUNCH_DIR / launch_name).read_text()
        assert "yaml.safe_load" in source
        assert source.index(yaml_name) < source.index("parameters=[")
        assert source.index("parameters=[") < source.index("{")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest homo_multirobot_formation_control/test/test_launch_yaml_defaults.py -v`

Expected: FAIL because the five YAML files do not exist and the launch files do not reference them.

- [ ] **Step 3: Implement minimal parser helpers in the test**

```python
def yaml_parameter_names(path):
    lines = path.read_text().splitlines()
    return {
        line.split(":", 1)[0].strip()
        for line in lines
        if line.startswith("    ") and not line.lstrip().startswith("#")
    }

def declared_argument_names(path):
    return set(re.findall(r'DeclareLaunchArgument\("([^\"]+)"', path.read_text()))
```

Use a dedicated source-order assertion that looks for the YAML filename and verifies it occurs before the `parameters=[` list. The implementation uses the already available ROS 2 Python `yaml` module; do not add a new package dependency.

- [ ] **Step 4: Run test to verify it still fails only for missing implementation**

Run: `python3 -m unittest homo_multirobot_formation_control/test/test_launch_yaml_defaults.py -v`

Expected: five subtests fail because YAML files, YAML loading, and references remain absent.

### Task 2: 创建 YAML 默认参数文件

**Files:**
- Create: `homo_multirobot_formation_control/config/formation_single_follower.yaml`
- Create: `homo_multirobot_formation_control/config/formation_single_follower_4d_artstein.yaml`
- Create: `homo_multirobot_formation_control/config/formation_single_follower_6d_disc.yaml`
- Create: `homo_multirobot_formation_control/config/formation_single_follower_6d_artstein_disc_hocbf.yaml`
- Create: `homo_multirobot_formation_control/config/formation_single_follower_6d_map_hpc_artstein.yaml`

**Interfaces:**
- Consumes: 当前各 launch 的 `DeclareLaunchArgument(... default_value=...)`。
- Produces: `/**: ros__parameters:` 下的完整默认参数集合。

- [ ] **Step 1: Write each YAML using ROS 2 parameter-file structure**

```yaml
/**:
  ros__parameters:
    leader_ns: "/robot1"
    follower_ns: "/robot2"
    use_sim_time: true
```

Populate all remaining keys from the matching launch with their current values. Preserve booleans and numbers as YAML native scalar values and quote namespace/topic strings.

- [ ] **Step 2: Run the test to verify the YAML key set is correct**

Run: `python3 -m unittest homo_multirobot_formation_control/test/test_launch_yaml_defaults.py -v`

Expected: YAML-key tests may pass; reference-order tests fail until Task 3.

### Task 3: 从固定 YAML 提供 launch 默认值并保留命令行覆盖

**Files:**
- Modify: `homo_multirobot_formation_control/launch/formation_single_follower.launch.py`
- Modify: `homo_multirobot_formation_control/launch/formation_single_follower_4d_artstein.launch.py`
- Modify: `homo_multirobot_formation_control/launch/formation_single_follower_6d_disc.launch.py`
- Modify: `homo_multirobot_formation_control/launch/formation_single_follower_6d_artstein_disc_hocbf.launch.py`
- Modify: `homo_multirobot_formation_control/launch/formation_single_follower_6d_map_hpc_artstein.launch.py`

**Interfaces:**
- Consumes: `get_package_share_directory("homo_multirobot_formation_control")` and fixed YAML filenames.
- Produces: A `config_file` path passed as the first `Node.parameters` entry.

- [ ] **Step 1: Add YAML default-value loading to each launch**

```python
from ament_index_python.packages import get_package_share_directory
import os
import yaml

config_file = os.path.join(
    get_package_share_directory("homo_multirobot_formation_control"),
    "config",
    "formation_single_follower.yaml",
)

with open(config_file, encoding="utf-8") as stream:
    defaults = yaml.safe_load(stream)["/**"]["ros__parameters"]
```

Use the matching fixed filename in each launch. Replace every literal launch-argument default with `str(defaults["argument_name"])`, preserving its existing description. Place imports with the existing standard/third-party import grouping.

- [ ] **Step 2: Place YAML before the launch parameter dictionary**

```python
parameters=[
    config_file,
    {
        "leader_ns": leader_ns,
        # retain every existing launch-controlled parameter
    },
],
```

Keep existing remappings, conditions, node names, and delay-node parameter handling unchanged. In the HOCBF launch, convert its current `parameters=[parameters]` to `parameters=[config_file, parameters]`.

- [ ] **Step 3: Run focused test to verify it passes**

Run: `python3 -m unittest homo_multirobot_formation_control/test/test_launch_yaml_defaults.py -v`

Expected: PASS; all five YAML files exist, have exactly the declared argument names, are loaded as launch defaults, and are placed before overrides.

- [ ] **Step 4: Run static syntax validation**

Run: `python3 -m py_compile homo_multirobot_formation_control/launch/formation_single_follower.launch.py homo_multirobot_formation_control/launch/formation_single_follower_4d_artstein.launch.py homo_multirobot_formation_control/launch/formation_single_follower_6d_disc.launch.py homo_multirobot_formation_control/launch/formation_single_follower_6d_artstein_disc_hocbf.launch.py homo_multirobot_formation_control/launch/formation_single_follower_6d_map_hpc_artstein.launch.py`

Expected: exit code 0.

- [ ] **Step 5: Build and inspect launch arguments from workspace root**

Run: `source /opt/ros/humble/setup.bash && colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=OFF && source install/setup.bash && ros2 launch homo_multirobot_formation_control formation_single_follower_4d_artstein.launch.py --show-args`

Expected: build exits 0 and `--show-args` includes the existing `radius`, `leader_ns`, and `tau` arguments.

- [ ] **Step 6: Commit implementation**

```bash
git add homo_multirobot_formation_control/config homo_multirobot_formation_control/launch homo_multirobot_formation_control/test/test_launch_yaml_defaults.py
git commit -m "支持编队launch从YAML加载参数"
```
