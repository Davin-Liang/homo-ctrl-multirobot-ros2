# Leader Circle Tangent Heading Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an optional forward-facing tangential-heading mode to the delay-aware closed-loop Leader circle node while retaining fixed heading as the default.

**Architecture:** Add one pure `reference_heading()` helper that selects a configured fixed yaw or the angle of the already-computed map-frame reference velocity. The ROS node reads a `heading_mode` parameter and sends the selected target yaw through the existing wrapped P yaw controller and its rate limits; the position predictor and translational controller remain unchanged.

**Tech Stack:** Python 3, NumPy, ROS 2 Humble `rclpy`, pytest.

## Global Constraints

- Change only the circle node, its focused pytest module, and its README section.
- `heading_mode` accepts `fixed` (default) and `tangent`; commands without it retain fixed-heading behavior.
- In tangent mode yaw is `atan2(reference_velocity.y, reference_velocity.x)`, so the robot faces its CW/CCW forward travel direction.
- An unknown mode logs a warning and falls back to `fixed`.
- Preserve the existing yaw controller and angular velocity/acceleration limits.
- Do not modify unrelated working-tree changes.

---

### Task 1: Add and test the pure heading-reference selector

**Files:**
- Create: `homo_multirobot_formation_control/test/test_leader_circle_closed_loop.py`
- Modify: `homo_multirobot_formation_control/scripts/leader_circle_closed_loop.py`

**Interfaces:**
- Produces: `reference_heading(heading_mode: str, fixed_heading: float, reference_velocity: np.ndarray) -> float`.
- `fixed` returns `fixed_heading`; `tangent` returns `math.atan2(reference_velocity[1], reference_velocity[0])`.

- [ ] **Step 1: Write the failing test**

Create the test module with this import and tests:

```python
import importlib.util
import math
from pathlib import Path
import sys
import numpy as np

SCRIPT_PATH = Path(__file__).parents[1] / 'scripts' / 'leader_circle_closed_loop.py'
SPEC = importlib.util.spec_from_file_location('leader_circle_closed_loop', SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)

def test_reference_heading_keeps_fixed_heading_by_default():
    assert MODULE.reference_heading('fixed', 0.73, np.array([0.0, 0.2])) == 0.73

def test_reference_heading_faces_ccw_and_cw_tangent_directions():
    ccw = MODULE.reference_heading('tangent', 0.0, np.array([-0.2, 0.0]))
    cw = MODULE.reference_heading('tangent', 0.0, np.array([0.2, 0.0]))
    assert math.isclose(ccw, math.pi)
    assert math.isclose(cw, 0.0)
```

- [ ] **Step 2: Run the focused test and verify red**

Run:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest homo_multirobot_formation_control/test/test_leader_circle_closed_loop.py -v
```

Expected: FAIL with `AttributeError` because `reference_heading` is absent.

- [ ] **Step 3: Implement the minimum helper**

Add after `circle_reference()`:

```python
def reference_heading(heading_mode: str, fixed_heading: float,
                      reference_velocity: np.ndarray) -> float:
    """Return fixed yaw or the forward tangent yaw of a circle reference."""
    if heading_mode == 'tangent':
        return math.atan2(reference_velocity[1], reference_velocity[0])
    return fixed_heading
```

- [ ] **Step 4: Run the focused test and verify green**

Run the Step 2 command. Expected: 2 passed.

- [ ] **Step 5: Commit the helper and test**

```bash
git add homo_multirobot_formation_control/scripts/leader_circle_closed_loop.py homo_multirobot_formation_control/test/test_leader_circle_closed_loop.py
git commit -m "增加圆轨迹切线航向参考"
```

### Task 2: Wire `heading_mode` into the ROS node

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/leader_circle_closed_loop.py`
- Test: `homo_multirobot_formation_control/test/test_leader_circle_closed_loop.py`

**Interfaces:**
- Consumes: ROS parameter `heading_mode`, default `fixed`.
- Produces: `self.heading_mode` constrained to `fixed` or `tangent`.
- Consumes: `reference_velocity` returned by the current call to `circle_reference()`.

- [ ] **Step 1: Write the failing node-wiring test**

Append:

```python
def test_node_declares_and_uses_heading_mode():
    source = SCRIPT_PATH.read_text(encoding='utf-8')
    assert "self.declare_parameter('heading_mode', 'fixed')" in source
    assert 'self.heading_mode = str(self.get_parameter(' in source
    assert 'reference_heading(self.heading_mode, self.heading,' in source
```

- [ ] **Step 2: Run it to verify the expected current failure**

Run:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest homo_multirobot_formation_control/test/test_leader_circle_closed_loop.py::test_node_declares_and_uses_heading_mode -v
```

Expected: FAIL because the node has not declared or used `heading_mode` yet.

- [ ] **Step 3: Implement parameter validation and target selection**

Declare `heading_mode` next to `heading`, retrieve it after `self.heading`, then validate it:

```python
self.declare_parameter('heading_mode', 'fixed')
# after reading self.heading
self.heading_mode = str(self.get_parameter('heading_mode').value)
if self.heading_mode not in ('fixed', 'tangent'):
    self.get_logger().warn(f'unknown heading_mode {self.heading_mode!r}; using fixed')
    self.heading_mode = 'fixed'
```

After `circle_reference()` returns `reference_velocity` in `timer_callback`, replace the fixed target with:

```python
target_heading = reference_heading(self.heading_mode, self.heading, reference_velocity)
yaw_error = wrap_angle(target_heading - self.yaw)
```

Keep the existing P, velocity-limit, and acceleration-limit code unchanged. Add `heading_mode` to the startup log.

- [ ] **Step 4: Run tests and syntax validation**

Run:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest homo_multirobot_formation_control/test/test_leader_circle_closed_loop.py -v
python3 -m py_compile homo_multirobot_formation_control/scripts/leader_circle_closed_loop.py
```

Expected: 3 passed and compilation exits 0.

- [ ] **Step 5: Commit the ROS parameter integration**

```bash
git add homo_multirobot_formation_control/scripts/leader_circle_closed_loop.py homo_multirobot_formation_control/test/test_leader_circle_closed_loop.py
git commit -m "支持闭环圆轨迹切线航向"
```

### Task 3: Document and build the feature

**Files:**
- Modify: `homo_multirobot_formation_control/README.md:552-585`
- Test: `homo_multirobot_formation_control/test/test_leader_circle_closed_loop.py`

**Interfaces:**
- Documents `heading_mode:=fixed|tangent`; tangent means the robot faces CW/CCW forward travel.

- [ ] **Step 1: Update the README**

State that `heading_mode:=fixed` holds `heading`, while `heading_mode:=tangent` tracks the forward circle tangent. Add this command after the current example:

```bash
ros2 run homo_multirobot_formation_control leader_circle_closed_loop.py \
  --ros-args -r __ns:=/robot1 \
  -p radius:=1.0 -p speed:=0.25 -p direction:=ccw \
  -p heading_mode:=tangent -p Td:=0.22 -p tau_v:=0.43
```

Add this row after `heading` in the parameter table:

```markdown
| `heading_mode` | `fixed` | `fixed`=保持 `heading`；`tangent`=车头沿 CW/CCW 运动前进切线 |
```

- [ ] **Step 2: Run focused tests and build at the colcon workspace root**

Run:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest homo_multirobot_formation_control/test/test_leader_circle_closed_loop.py -v
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

Expected: 3 pytest tests pass and colcon exits 0.

- [ ] **Step 3: Inspect scoped changes and commit documentation**

Run:

```bash
git diff --check
git diff -- homo_multirobot_formation_control/scripts/leader_circle_closed_loop.py homo_multirobot_formation_control/test/test_leader_circle_closed_loop.py homo_multirobot_formation_control/README.md
git add homo_multirobot_formation_control/README.md
git commit -m "补充圆轨迹切线航向说明"
```

Expected: no whitespace errors and the final commit contains only the README change.
