# Map 系闭环 8 字领航轨迹 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新增一个使用 map 系闭环反馈、同时支持 Gazebo 定位与动捕输入的 8 字领航轨迹节点。

**Architecture:** 以 `leader_circle_closed_loop_map.py` 为状态输入、安全停车、预测和指令约束的实现模板，新增独立脚本以避免改变圆轨迹或开环 8 字行为。新脚本仅将圆参考替换成峰值速度驱动的 8 字参考，并通过独立单元测试验证解析参考函数。

**Tech Stack:** ROS 2 Humble Python (`rclpy`)、NumPy、`tf2_ros`、pytest、ROS 2 launch。

## Global Constraints

- 必须支持 `state_source:=odom_tf` 与 `state_source:=mocap`，参数名和安全行为与 `leader_circle_closed_loop_map.py` 一致。
- 所有闭环位置、速度参考和控制计算必须在 `map_frame` 下进行。
- `speed` 是参考线速度峰值；不公开 `period` 参数。
- `omega = speed / sqrt(amplitude_x**2 + 4 * amplitude_y**2)`；`speed=0` 时参考停在原点。
- 不改动 `leader_eight.py`、`leader_circle_closed_loop_map.py` 或用户已有的未跟踪分析结果。

---

### Task 1: 8 字解析参考及测试

**Files:**
- Create: `homo_multirobot_formation_control/scripts/leader_eight_closed_loop_map.py`
- Create: `homo_multirobot_formation_control/test/test_leader_eight_closed_loop_map.py`

**Interfaces:**
- Produces: `reference_omega(amplitude_x: float, amplitude_y: float, speed: float) -> float`.
- Produces: `eight_reference(p0: np.ndarray, amplitude_x: float, amplitude_y: float, speed: float, elapsed: float) -> Tuple[np.ndarray, np.ndarray]`.

- [ ] **Step 1: Write the failing test**

```python
def test_eight_reference_starts_and_returns_to_origin():
    p0 = np.array([1.5, -0.25])
    omega = MODULE.reference_omega(2.0, 1.0, 0.4)
    period = 2.0 * math.pi / omega
    np.testing.assert_allclose(MODULE.eight_reference(p0, 2.0, 1.0, 0.4, 0.0)[0], p0)
    np.testing.assert_allclose(MODULE.eight_reference(p0, 2.0, 1.0, 0.4, period)[0], p0)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `python3 -m pytest -q homo_multirobot_formation_control/test/test_leader_eight_closed_loop_map.py`

Expected: FAIL because the script/module does not exist.

- [ ] **Step 3: Write the minimal reference implementation**

```python
def reference_omega(ax, ay, speed):
    return 0.0 if speed == 0.0 else speed / math.hypot(ax, 2.0 * ay)

def eight_reference(p0, ax, ay, speed, elapsed):
    omega = reference_omega(ax, ay, speed)
    phase = omega * elapsed
    return (p0 + np.array([ax * math.sin(phase), ay * math.sin(2.0 * phase)]),
            np.array([ax * omega * math.cos(phase),
                      2.0 * ay * omega * math.cos(2.0 * phase)]))
```

- [ ] **Step 4: Add and run speed-limit tests**

```python
def test_eight_reference_peak_speed_and_zero_speed():
    p0, speed, ax, ay = np.zeros(2), 0.4, 2.0, 1.0
    period = 2.0 * math.pi / MODULE.reference_omega(ax, ay, speed)
    norms = [np.linalg.norm(MODULE.eight_reference(p0, ax, ay, speed, time)[1])
             for time in np.linspace(0.0, period, 1001)]
    assert max(norms) <= speed + 1e-12
    position, velocity = MODULE.eight_reference(p0, ax, ay, 0.0, 99.0)
    np.testing.assert_allclose(position, p0)
    np.testing.assert_allclose(velocity, [0.0, 0.0])
```

Run the same pytest command; expected PASS.

- [ ] **Step 5: Commit**

```bash
git add homo_multirobot_formation_control/scripts/leader_eight_closed_loop_map.py homo_multirobot_formation_control/test/test_leader_eight_closed_loop_map.py
git commit -m "新增闭环8字参考轨迹"
```

### Task 2: Map/动捕闭环节点与安装

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/leader_eight_closed_loop_map.py`
- Modify: `homo_multirobot_formation_control/CMakeLists.txt:232-247`

**Interfaces:**
- Consumes: Task 1 `eight_reference` and `reference_omega`.
- Produces: executable ROS node `leader_eight_closed_loop_map.py` publishing `cmd_vel`.

- [ ] **Step 1: Implement node by retaining Map-circle behavior**

Copy `leader_circle_closed_loop_map.py` into the new script, rename the class/node to `LeaderEightClosedLoopMap` / `leader_eight_closed_loop_map`, replace `radius`, `direction`, and `start_side` declarations with `amplitude_x:=2.0` and `amplitude_y:=1.0`, and retain `speed:=0.2`. Keep its TF conversion, mocap callbacks, stale-data zero command, predictor, map/body conversion, yaw loop, and limits unchanged. In `timer_callback`, replace `circle_reference(...)` with `eight_reference(self.p0, self.amplitude_x, self.amplitude_y, self.speed, elapsed + lookahead)`. Add `scripts/leader_eight_closed_loop_map.py` under `install(PROGRAMS ...)`.

- [ ] **Step 2: Run tests and syntax verification**

Run: `python3 -m pytest -q homo_multirobot_formation_control/test/test_leader_eight_closed_loop_map.py && python3 -m py_compile homo_multirobot_formation_control/scripts/leader_eight_closed_loop_map.py`

Expected: PASS and no compiler output.

- [ ] **Step 3: Commit**

```bash
git add homo_multirobot_formation_control/scripts/leader_eight_closed_loop_map.py homo_multirobot_formation_control/CMakeLists.txt homo_multirobot_formation_control/test/test_leader_eight_closed_loop_map.py
git commit -m "新增Map系闭环8字领航节点"
```

### Task 3: Launch 与使用说明

**Files:**
- Create: `homo_multirobot_formation_control/launch/leader_eight_closed_loop_map.launch.py`
- Modify: `homo_multirobot_formation_control/README.md:518-557`
- Modify: `homo_multirobot_formation_control/test/test_leader_eight_closed_loop_map.py`

**Interfaces:**
- Consumes: Task 2 executable and parameters.
- Produces: map-closed-loop launch entry point and Gazebo/mocap usage documentation.

- [ ] **Step 1: Write the failing launch test**

```python
def test_eight_launch_forwards_map_and_mocap_parameters():
    launch = PACKAGE_DIR / "launch" / "leader_eight_closed_loop_map.launch.py"
    source = launch.read_text(encoding="utf-8")
    for value in ("amplitude_x", "amplitude_y", "speed", "state_source",
                  "mocap_pose_topic", "mocap_twist_topic", "map_frame",
                  "leader_eight_closed_loop_map.py"):
        assert value in source
```

- [ ] **Step 2: Run it to verify it fails**

Run: `python3 -m pytest -q homo_multirobot_formation_control/test/test_leader_eight_closed_loop_map.py`

Expected: FAIL because the launch file does not exist.

- [ ] **Step 3: Add launch and README section**

Copy the argument and parameter forwarding structure from `leader_circle_closed_loop_map.launch.py`; remove `radius`, `direction`, and `start_side`; add `amplitude_x`, `amplitude_y`, and `speed`; set executable/name to `leader_eight_closed_loop_map.py` / `leader_eight_closed_loop_map`. Add README commands for `odom_tf` Gazebo and `mocap` use, documenting peak-speed semantics and stale-mocap safety stop.

- [ ] **Step 4: Run all local checks and workspace build**

Run: `python3 -m pytest -q homo_multirobot_formation_control/test/test_leader_eight_closed_loop_map.py && python3 -m py_compile homo_multirobot_formation_control/scripts/leader_eight_closed_loop_map.py && cd ../.. && source /opt/ros/humble/setup.bash && colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=OFF`

Expected: pytest PASS; syntax check exits 0; colcon exits 0.

- [ ] **Step 5: Commit**

```bash
git add homo_multirobot_formation_control/launch/leader_eight_closed_loop_map.launch.py homo_multirobot_formation_control/README.md homo_multirobot_formation_control/test/test_leader_eight_closed_loop_map.py
git commit -m "补充闭环8字轨迹启动说明"
```
