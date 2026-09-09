# Motion-Capture State Source for Trajectory Recorder Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a `state_source:=mocap` mode to `record_trajectory.py` that records the map-frame pose and velocity published by the motion-capture adapter.

**Architecture:** Keep one recorder node and select its subscriptions from the new `state_source` parameter. The existing `ekf_tf` source continues to transform filtered odometry into `map`; the new `mocap` source caches each robot's `PoseStamped` and `TwistStamped`, starts only after all four streams have arrived, and records directly in map coordinates. Configuration accepts older YAML files without the new key by injecting the default, while newly documented YAML carries the explicit key.

**Tech Stack:** ROS 2 Humble (`rclpy`, `geometry_msgs`), Python, PyYAML, pytest, colcon.

## Global Constraints

- `state_source` accepts exactly `ekf_tf` and `mocap`; its default is `ekf_tf`.
- `ekf_tf` must preserve the existing `/<ns>/odometry/filtered` plus `map -> <ns>_odom` TF behavior.
- `mocap` consumes `/<ns>/mocap/pose` (`PoseStamped`) and `/<ns>/mocap/twist` (`TwistStamped`) as already map-frame state.
- In mocap mode, do not start the duration clock or append samples until both pose and twist have arrived for both leader and follower.
- Preserve the CSV columns, plots, controller-parameter waiting behavior, and existing metadata fields; only add source/topic metadata.
- Do not stage unrelated user edits in the repository's dirty configuration and RViz files.

---

## File Structure

- `homo_multirobot_formation_control/scripts/record_trajectory.py`: validates source selection, builds source-specific subscriptions, caches and records mocap state, and writes source-aware metadata.
- `homo_multirobot_formation_control/config/record_trajectory.yaml`: documents the new parameter in the normal recorder configuration (the currently user-customized value remains unstaged).
- `homo_multirobot_formation_control/test/test_record_trajectory_yaml.py`: verifies YAML compatibility, source validation, and direct mocap sample extraction.
- `homo_multirobot_formation_control/README.md`: documents mocap launch order and recorder invocation.

### Task 1: Define and validate recorder state-source configuration

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/record_trajectory.py:32-95`
- Modify: `homo_multirobot_formation_control/config/record_trajectory.yaml:3-18`
- Test: `homo_multirobot_formation_control/test/test_record_trajectory_yaml.py:14-36`

**Interfaces:**
- Produces: `VALID_STATE_SOURCES = ('ekf_tf', 'mocap')` and `validate_state_source(value) -> str`.
- Produces: `load_recorder_parameters(path) -> dict` which supplies `state_source: 'ekf_tf'` if a legacy configuration omits it.
- Consumes: `RECORDER_PARAMETER_DEFAULTS`, including `state_source: 'ekf_tf'`.

- [ ] **Step 1: Write failing configuration tests**

```python
def test_legacy_yaml_without_state_source_gets_ekf_tf_default(tmp_path):
    parameters = dict(module.RECORDER_PARAMETER_DEFAULTS)
    parameters.pop('state_source')
    path = tmp_path / 'legacy.yaml'
    path.write_text('/**:\n  ros__parameters:\n' + yaml.safe_dump(parameters), encoding='utf-8')

    assert module.load_recorder_parameters(path)['state_source'] == 'ekf_tf'


def test_validate_state_source_rejects_unknown_value():
    with pytest.raises(ValueError, match='state_source'):
        module.validate_state_source('wheel_odom')
```

- [ ] **Step 2: Run the focused tests to verify failure**

Run: `python3 -m pytest homo_multirobot_formation_control/test/test_record_trajectory_yaml.py -q`

Expected: FAIL because `state_source` is absent from defaults and `validate_state_source` is undefined.

- [ ] **Step 3: Add the minimal configuration implementation**

```python
VALID_STATE_SOURCES = ('ekf_tf', 'mocap')
RECORDER_PARAMETER_DEFAULTS = {
    # existing entries...
    'state_source': 'ekf_tf',
}


def validate_state_source(value):
    if value not in VALID_STATE_SOURCES:
        raise ValueError(
            f"state_source 必须是 {', '.join(VALID_STATE_SOURCES)}，当前为 {value!r}")
    return value
```

Before comparing YAML keys, copy the mapping and call `parameters.setdefault('state_source', 'ekf_tf')`; then retain the exact-key validation. In `TrajectoryRecorder.__init__`, assign `self.state_source = validate_state_source(self.get_parameter('state_source').value)`. Add `state_source: "ekf_tf"` to the packaged example configuration without overwriting the user's selected value in the dirty working file.

- [ ] **Step 4: Run the focused tests to verify success**

Run: `python3 -m pytest homo_multirobot_formation_control/test/test_record_trajectory_yaml.py -q`

Expected: PASS, including the old configuration compatibility case and invalid-value rejection.

- [ ] **Step 5: Commit the configuration slice**

```bash
git add homo_multirobot_formation_control/scripts/record_trajectory.py \
  homo_multirobot_formation_control/test/test_record_trajectory_yaml.py
git commit -m "支持轨迹记录器状态源配置"
```

Do not add `config/record_trajectory.yaml` if it contains unrelated user parameters; leave its one-line state-source edit unstaged.

### Task 2: Subscribe to and record coherent mocap state

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/record_trajectory.py:16-24, 145-210, 286-315`
- Test: `homo_multirobot_formation_control/test/test_record_trajectory_yaml.py:37-70`

**Interfaces:**
- Consumes: `self.state_source`, `PoseStamped`, `TwistStamped`, and the existing `t1_*` / `t2_*` arrays.
- Produces: `mocap_sample(pose: PoseStamped, twist: TwistStamped) -> tuple[float, float, float, float]` containing `(x, y, vx, vy)`.
- Produces: `self._record_mocap(pose, twist, xl, yl, tl, vxl, vyl, vl)`, which uses direct map-frame values.

- [ ] **Step 1: Write failing direct-mocap extraction test**

```python
from geometry_msgs.msg import PoseStamped, TwistStamped


def test_mocap_sample_uses_map_pose_and_linear_twist_directly():
    pose = PoseStamped()
    pose.pose.position.x = 1.25
    pose.pose.position.y = -0.75
    twist = TwistStamped()
    twist.twist.linear.x = 0.4
    twist.twist.linear.y = -0.2

    assert module.mocap_sample(pose, twist) == (1.25, -0.75, 0.4, -0.2)
```

- [ ] **Step 2: Run the focused test to verify failure**

Run: `python3 -m pytest homo_multirobot_formation_control/test/test_record_trajectory_yaml.py::test_mocap_sample_uses_map_pose_and_linear_twist_directly -q`

Expected: FAIL because `mocap_sample` is undefined.

- [ ] **Step 3: Implement source-specific subscriptions and recording**

Import `PoseStamped` and `TwistStamped`, then add the pure helper:

```python
def mocap_sample(pose, twist):
    return (
        pose.pose.position.x,
        pose.pose.position.y,
        twist.twist.linear.x,
        twist.twist.linear.y,
    )
```

When `self.state_source == 'ekf_tf'`, retain the two existing odometry subscriptions unchanged. When it is `mocap`, initialize four latest-message attributes to `None` and create four subscriptions:

```python
self.create_subscription(PoseStamped, self.leader_ns + '/mocap/pose', self.cb_leader_mocap_pose, 10)
self.create_subscription(TwistStamped, self.leader_ns + '/mocap/twist', self.cb_leader_mocap_twist, 10)
self.create_subscription(PoseStamped, self.follower_ns + '/mocap/pose', self.cb_follower_mocap_pose, 10)
self.create_subscription(TwistStamped, self.follower_ns + '/mocap/twist', self.cb_follower_mocap_twist, 10)
```

Each callback updates its cache. Pose callbacks invoke a common method only if all four cached messages are non-`None`; that method calls `_record_mocap` for the matching robot. `_record_mocap` mirrors `_record` except it calls `mocap_sample` rather than `_odom_to_map`, appends the direct x/y/vx/vy values and `math.hypot(vx, vy)`, and initializes `self.t0` on the first eligible record. This ensures no timer start or sample exists before both robots' state pairs are present. Keep the wall-clock time basis and `check_done` unchanged.

- [ ] **Step 4: Run all recorder unit tests**

Run: `python3 -m pytest homo_multirobot_formation_control/test/test_record_trajectory_yaml.py -q`

Expected: PASS; the existing service-wait tests and the new direct-mocap extraction test pass.

- [ ] **Step 5: Commit the subscription and recording slice**

```bash
git add homo_multirobot_formation_control/scripts/record_trajectory.py \
  homo_multirobot_formation_control/test/test_record_trajectory_yaml.py
git commit -m "支持动捕轨迹状态订阅"
```

### Task 3: Record source metadata and document mocap use

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/record_trajectory.py:375-405`
- Modify: `homo_multirobot_formation_control/README.md:596-640`
- Test: `homo_multirobot_formation_control/test/test_record_trajectory_yaml.py:37-70`

**Interfaces:**
- Consumes: `self.state_source`, namespace fields, and the unchanged metadata schema.
- Produces: `metadata.yaml` `recording.state_source`; mocap runs additionally contain `leader_twist_topic` and `follower_twist_topic`.

- [ ] **Step 1: Write a failing source-topic helper test**

```python
def test_mocap_recording_topics_include_pose_and_twist():
    assert module.recording_topics('/robot1', '/robot2', 'mocap') == {
        'leader_topic': '/robot1/mocap/pose',
        'follower_topic': '/robot2/mocap/pose',
        'leader_twist_topic': '/robot1/mocap/twist',
        'follower_twist_topic': '/robot2/mocap/twist',
    }
```

- [ ] **Step 2: Run the focused test to verify failure**

Run: `python3 -m pytest homo_multirobot_formation_control/test/test_record_trajectory_yaml.py::test_mocap_recording_topics_include_pose_and_twist -q`

Expected: FAIL because `recording_topics` is undefined.

- [ ] **Step 3: Implement metadata topic selection and README guidance**

Add a pure `recording_topics(leader_ns, follower_ns, state_source)` helper. It returns the existing `odometry/filtered` leader/follower topics for `ekf_tf`; for `mocap`, it returns pose entries under the existing `leader_topic` / `follower_topic` keys plus twist-topic entries. In `_save_metadata`, merge that mapping into `recording` and add `state_source: self.state_source`.

In the README, add a mocap section with this sequence: launch `mocap_two_robots.launch.py`, launch `formation_single_follower_4d_artstein_mocap.launch.py`, then run the recorder with `-p state_source:=mocap`. State that `mocap` records adapter output directly in map coordinates and begins only after both robots have delivered pose and twist.

- [ ] **Step 4: Run full source-level verification**

Run:

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_record_trajectory_yaml.py -q
python3 -m py_compile homo_multirobot_formation_control/scripts/record_trajectory.py
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

Expected: all pytest tests pass, Python compilation succeeds, and the targeted colcon build finishes successfully.

- [ ] **Step 5: Commit documentation and metadata changes**

```bash
git add homo_multirobot_formation_control/scripts/record_trajectory.py \
  homo_multirobot_formation_control/test/test_record_trajectory_yaml.py \
  homo_multirobot_formation_control/README.md
git commit -m "记录动捕状态源元数据"
```
