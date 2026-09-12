# 动捕双车 RViz2 自动启动 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 启动 `mocap_two_robots.launch.py` 时自动启动 RViz2，并从定位包中加载双车动捕配置。

**Architecture:** 在定位包新增一个仅描述动捕可视化的 RViz 配置，固定坐标系为 `map`，显示双车 TF 与机器人模型。Launch 使用现有 `FindPackageShare` 的包共享路径，为无条件创建的 RViz2 节点传入该文件；CMake 安装规则将 `rviz/` 一并安装。

**Tech Stack:** ROS 2 Humble launch Python、RViz2、ament_cmake。

## Global Constraints

- 不增加 `use_rviz` 或其他 RViz 开关。
- 不修改 VRPN bridge、动捕状态适配器、话题名或 TF 命名。
- RViz 配置必须随 `homo_multirobot_localization` 安装到 `share/homo_multirobot_localization/rviz/`。
- 构建只能从工作空间根目录 `/home/l1anggmgo/ros-projects/homo_multirobot_ws` 执行。

---

### Task 1: 添加并集成双车动捕 RViz 配置

**Files:**
- Create: `homo_multirobot_localization/rviz/mocap_two_robots.rviz`
- Create: `homo_multirobot_localization/test/test_mocap_two_robots_rviz.py`
- Modify: `homo_multirobot_localization/launch/mocap_two_robots.launch.py:1-45`
- Modify: `homo_multirobot_localization/CMakeLists.txt:16-18`

**Interfaces:**
- Consumes: `FindPackageShare("homo_multirobot_localization")` and its `share` path.
- Produces: an RViz2 `Node` with `package="rviz2"`, `executable="rviz2"`, and `arguments=["-d", <package-share>/rviz/mocap_two_robots.rviz]`.

- [ ] **Step 1: Write the failing test**

Create `homo_multirobot_localization/test/test_mocap_two_robots_rviz.py`:

```python
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "launch" / "mocap_two_robots.launch.py"
RVIZ = ROOT / "rviz" / "mocap_two_robots.rviz"
CMAKE = ROOT / "CMakeLists.txt"


def test_mocap_launch_starts_rviz_with_packaged_config():
    launch_source = LAUNCH.read_text()
    assert 'package="rviz2"' in launch_source
    assert 'executable="rviz2"' in launch_source
    assert '"-d"' in launch_source
    assert '"rviz", "mocap_two_robots.rviz"' in launch_source


def test_mocap_rviz_config_is_packaged_and_targets_two_robots():
    config_source = RVIZ.read_text()
    assert "Fixed Frame: map" in config_source
    assert "/robot1/robot_description" in config_source
    assert "/robot2/robot_description" in config_source
    assert "DIRECTORY launch config rviz" in CMAKE.read_text()
```

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
python3 -m pytest homo_multirobot_localization/test/test_mocap_two_robots_rviz.py -q
```

Expected: FAIL because the RViz node, configuration file, and `rviz` installation directory do not yet exist.

- [ ] **Step 3: Implement the minimal integration**

1. Create `rviz/mocap_two_robots.rviz` with `map` as the fixed frame and enabled `TF`, `RobotModel robot1`, and `RobotModel robot2` displays. The RobotModel topics are `/robot1/robot_description` and `/robot2/robot_description`.
2. Define `rviz_config = PathJoinSubstitution([share, "rviz", "mocap_two_robots.rviz"])` in `generate_launch_description()`.
3. Append one `Node(package="rviz2", executable="rviz2", name="rviz2", output="screen", arguments=["-d", rviz_config])` to the returned `LaunchDescription`.
4. Change CMake's install line from `DIRECTORY launch config` to `DIRECTORY launch config rviz`.

- [ ] **Step 4: Run focused verification**

Run:

```bash
python3 -m pytest homo_multirobot_localization/test/test_mocap_two_robots_rviz.py -q
python3 -m py_compile homo_multirobot_localization/launch/mocap_two_robots.launch.py
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select homo_multirobot_localization --symlink-install --cmake-args -DBUILD_TESTING=OFF
test -f install/homo_multirobot_localization/share/homo_multirobot_localization/rviz/mocap_two_robots.rviz
```

Expected: all commands succeed and the installed RViz configuration exists.

- [ ] **Step 5: Commit**

```bash
git add homo_multirobot_localization/CMakeLists.txt \
  homo_multirobot_localization/launch/mocap_two_robots.launch.py \
  homo_multirobot_localization/rviz/mocap_two_robots.rviz \
  homo_multirobot_localization/test/test_mocap_two_robots_rviz.py
git commit -m "动捕双车启动RViz2"
```
