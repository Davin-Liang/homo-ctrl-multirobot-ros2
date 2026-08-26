# Delay-Aware Closed-Loop Leader Circle Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a fixed-yaw, EKF/odom-feedback, delay-aware closed-loop circular Leader trajectory node for physical and simulated omni-directional robots.

**Architecture:** The new Python ROS 2 node records the first valid odometry pose as the reference start point, generates a future map-frame circle reference, and predicts the delayed first-order translational state from measured odometry and final command history. A map-frame PD outer loop produces a bounded velocity command, which is transformed by measured yaw into body-frame `cmd_vel`; a separate P outer loop holds the requested fixed heading.

**Tech Stack:** Python 3, ROS 2 Humble `rclpy`, `nav_msgs/Odometry`, `geometry_msgs/Twist`, CMake install rules.

## Global Constraints

- Create `homo_multirobot_formation_control/scripts/leader_circle_closed_loop.py`.
- Subscribe to relative `odometry/filtered` by default and publish relative `cmd_vel`.
- Use the odometry message frame consistently for feedback and reference; do not add TF transforms.
- Preserve `leader_circle.py` as the existing open-loop baseline.
- Hold yaw at `heading` using measured orientation and publish bounded `angular.z`.
- Use final bounded map-frame velocity commands in the prediction history.
- Default physical parameters: (R=2.0) m, (v=0.2) m/s, rate (20) Hz, (T_d=0.22) s, (	au_v=0.43) s.

---

### Task 1: Add pure trajectory and control helpers

**Files:**
- Create: `homo_multirobot_formation_control/scripts/leader_circle_closed_loop.py`
- Test: inline Python assertions importing the script with `runpy.run_path`

**Interfaces:**
- Produces: `circle_reference(p0, radius, speed, omega, elapsed) -> (pd, vd)`.
- Produces: `body_to_map(v_body, yaw) -> np.ndarray` and `map_to_body(v_map, yaw) -> np.ndarray`.
- Produces: `predict_delayed_state(p, v_map, command_history, dt, td, tau_v) -> (p_pred, v_pred)`.

- [ ] **Step 1: Verify the helper API is absent**

Run:

```bash
python3 - <<'PY'
import runpy
module = runpy.run_path('homo_multirobot_formation_control/scripts/leader_circle_closed_loop.py')
assert 'circle_reference' in module
PY
```

Expected: file-not-found failure.

- [ ] **Step 2: Implement pure helpers**

Implement the reference equations:

```python
center = p0 + np.array([-radius, 0.0])
phase = omega * elapsed
pd = center + radius * np.array([math.cos(phase), math.sin(phase)])
vd = speed * np.array([-math.sin(phase), math.cos(phase)])
```

Implement map/body rotations, command-history Artstein integral, back mapping, and one-`tau_v` forward prediction. For `td == 0.0`, the integral is a zero vector.

- [ ] **Step 3: Verify helper behavior**

Run:

```bash
python3 - <<'PY'
import runpy
import numpy as np
module = runpy.run_path('homo_multirobot_formation_control/scripts/leader_circle_closed_loop.py')
pd, vd = module['circle_reference'](np.array([2.0, 3.0]), 2.0, 0.2, 0.1, 0.0)
assert np.allclose(pd, [2.0, 3.0])
assert np.allclose(vd, [0.0, 0.2])
v = np.array([0.3, -0.2])
assert np.allclose(module['map_to_body'](module['body_to_map'](v, 0.7), 0.7), v)
PY
```

Expected: exit code 0.

### Task 2: Implement the ROS 2 closed-loop node

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/leader_circle_closed_loop.py`
- Modify: `homo_multirobot_formation_control/CMakeLists.txt:293-304`

**Interfaces:**
- Consumes: `nav_msgs/msg/Odometry` from `odom_topic`.
- Produces: `geometry_msgs/msg/Twist` on `cmd_vel`.
- Parameters: `radius`, `speed`, `heading`, `direction`, `rate`, `odom_topic`, `Td`, `tau_v`, `kp`, `kv`, `k_yaw`, `max_linear_vel`, `max_linear_accel`, `max_angular_vel`, `max_angular_accel`.

- [ ] **Step 1: Implement initialization from the first valid odometry**

Subscribe to `odom_topic`; extract yaw from the odometry quaternion, transform odometry body velocity into map frame, and save the first position as `p0`. Publish zero commands until initialization completes.

- [ ] **Step 2: Implement delay-aware map-frame PD control**

At every timer tick, calculate:

```python
lookahead = td + tau_v
pd, vd = circle_reference(p0, radius, speed, omega, elapsed + lookahead)
v_map_cmd = vd - kp * (p_pred - pd) - kv * (v_pred - vd)
```

Limit the map-frame velocity norm and map-frame command increment. Convert the final map command to body frame with measured yaw.

- [ ] **Step 3: Implement fixed-yaw control and history write-back**

Calculate wrapped yaw error and apply angular velocity and angular acceleration bounds. Publish body-frame `Twist`. Rotate the final published linear command back to map frame and append it to command history.

- [ ] **Step 4: Add the script to package installation**

Add:

```cmake
scripts/leader_circle_closed_loop.py
```

to the existing `install(PROGRAMS ...)` list.

- [ ] **Step 5: Verify syntax and parameter declarations**

Run:

```bash
python3 -m py_compile homo_multirobot_formation_control/scripts/leader_circle_closed_loop.py
rg -n 'leader_circle_closed_loop.py' homo_multirobot_formation_control/CMakeLists.txt
```

Expected: both commands exit 0.

### Task 3: Document and smoke-test the node

**Files:**
- Modify: `homo_multirobot_formation_control/README.md:548-588`
- Test: direct script import and ROS package build.

**Interfaces:**
- Documents: launch command, fixed-yaw semantics, feedback topic, delay parameters, and low-speed physical defaults.

- [ ] **Step 1: Document launch and parameters**

Add a `leader_circle_closed_loop` subsection after the open-loop `leader_circle` subsection with:

```bash
ros2 run homo_multirobot_formation_control leader_circle_closed_loop.py \
  --ros-args -r __ns:=/robot1 \
  -p radius:=2.0 -p speed:=0.2 -p heading:=0.0 \
  -p Td:=0.22 -p tau_v:=0.43
```

State that the reference starts at the first odometry pose, yaw is held at `heading`, and `odometry/filtered` must share the reference coordinate frame.

- [ ] **Step 2: Run pure helper smoke checks**

Run the Task 1 helper assertions again and:

```bash
python3 -m py_compile homo_multirobot_formation_control/scripts/leader_circle_closed_loop.py
```

Expected: exit code 0.

- [ ] **Step 3: Build the package from the workspace root**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

Expected: exit code 0.

- [ ] **Step 4: Commit the scoped implementation**

```bash
git add homo_multirobot_formation_control/scripts/leader_circle_closed_loop.py \
  homo_multirobot_formation_control/CMakeLists.txt \
  homo_multirobot_formation_control/README.md
git commit -m "新增延迟感知闭环领航圆轨迹"
```
