# Trajectory Recorder Attitude Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Record Leader/Follower yaw and angular velocity, convert EKF twist to map-frame, and emit every trajectory-check plot as its own PNG.

**Architecture:** ROS callbacks collect complete planar states in per-robot arrays. Pure helpers extract quaternion yaw, transform body velocity into map coordinates, and unwrap yaw. The existing six-axis Matplotlib canvas becomes seven standalone figures.

**Tech Stack:** Python 3, ROS 2 Humble, Matplotlib, pytest.

## Global Constraints

- Modify only homo_multirobot_formation_control/scripts/record_trajectory.py and its focused Python test.
- CSV raw yaw remains in [-pi, pi]; unwrap only when rendering yaw.png.
- CSV linear velocities are map-frame for ekf_tf and mocap.
- Do not change controllers, launch files, YAML configuration, or user-owned experiment data.
- Run colcon only from /home/l1anggmgo/ros-projects/homo_multirobot_ws.

---

### Task 1: Add testable planar-state helpers

**Files:**

- Modify: homo_multirobot_formation_control/scripts/record_trajectory.py, helpers section.
- Test: homo_multirobot_formation_control/test/test_record_trajectory_yaml.py.

**Interfaces:**

- Consumes quaternion x/y/z/w, body vx/vy, and yaw.
- Produces yaw_from_quaternion(quaternion), body_velocity_to_map(vx_body, vy_body, yaw), and mocap_sample(pose, twist) returning x, y, yaw, vx_map, vy_map, omega.

- [ ] **Step 1: Write failing helper tests**

~~~python
def test_yaw_and_map_velocity_helpers_preserve_planar_state():
    pose = PoseStamped()
    pose.pose.orientation.z = math.sqrt(0.5)
    pose.pose.orientation.w = math.sqrt(0.5)
    assert module.yaw_from_quaternion(pose.pose.orientation) == pytest.approx(math.pi / 2)
    assert module.body_velocity_to_map(1.0, 0.0, math.pi / 2) == pytest.approx((0.0, 1.0))

def test_mocap_sample_includes_yaw_and_angular_velocity():
    pose = PoseStamped()
    pose.pose.orientation.w = 1.0
    twist = TwistStamped()
    twist.twist.linear.x, twist.twist.linear.y, twist.twist.angular.z = 0.4, -0.2, 0.3
    assert module.mocap_sample(pose, twist) == pytest.approx((0.0, 0.0, 0.0, 0.4, -0.2, 0.3))
~~~

- [ ] **Step 2: Verify RED**

Run: cd /home/l1anggmgo/ros-projects/homo_multirobot_ws && source /opt/ros/humble/setup.bash && pytest src/homo-ctrl-multirobot-ros2/homo_multirobot_formation_control/test/test_record_trajectory_yaml.py -k 'yaw_and_map_velocity or mocap_sample_includes' -v

Expected: FAIL because the helper functions and expanded sample result do not exist.

- [ ] **Step 3: Implement the smallest helpers**

~~~python
def yaw_from_quaternion(quaternion):
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2)
    return math.atan2(siny, cosy)

def body_velocity_to_map(vx_body, vy_body, yaw):
    return (vx_body * math.cos(yaw) - vy_body * math.sin(yaw),
            vx_body * math.sin(yaw) + vy_body * math.cos(yaw))
~~~

Use the helpers in mocap_sample; keep its map velocity unchanged and append yaw plus twist.angular.z.

- [ ] **Step 4: Verify GREEN**

Run the command from Step 2. Expected: PASS.

- [ ] **Step 5: Commit**

~~~bash
git add homo_multirobot_formation_control/scripts/record_trajectory.py homo_multirobot_formation_control/test/test_record_trajectory_yaml.py
git commit -m "增加轨迹姿态状态提取"
~~~

### Task 2: Record map velocity, yaw, and angular velocity in CSV

**Files:**

- Modify: homo_multirobot_formation_control/scripts/record_trajectory.py, recorder arrays, callbacks, and _save_csv.
- Test: homo_multirobot_formation_control/test/test_record_trajectory_yaml.py.

**Interfaces:**

- Consumes Task 1 helpers and Odometry/PoseStamped/TwistStamped messages.
- Produces aligned t1_yaw/t1_omega/t2_yaw/t2_omega arrays and CSV fields leader_yaw_rad, leader_omega_rads, follower_yaw_rad, follower_omega_rads.

- [ ] **Step 1: Write a failing CSV contract test**

~~~python
def test_save_csv_writes_map_velocity_and_attitude_columns(tmp_path):
    recorder = make_recorder_with_one_leader_and_follower_sample()
    recorder._save_csv(tmp_path)
    header = (tmp_path / 'raw.csv').read_text(encoding='utf-8').splitlines()[0].split(',')
    assert header == [
        'time_s', 'leader_x_m', 'leader_y_m',
        'leader_vx_map_ms', 'leader_vy_map_ms', 'leader_v_ms',
        'leader_yaw_rad', 'leader_omega_rads',
        'follower_x_m', 'follower_y_m',
        'follower_vx_map_ms', 'follower_vy_map_ms', 'follower_v_ms',
        'follower_yaw_rad', 'follower_omega_rads', 'distance_m',
    ]
~~~

- [ ] **Step 2: Verify RED**

Run: cd /home/l1anggmgo/ros-projects/homo_multirobot_ws && source /opt/ros/humble/setup.bash && pytest src/homo-ctrl-multirobot-ros2/homo_multirobot_formation_control/test/test_record_trajectory_yaml.py -k save_csv -v

Expected: FAIL because current CSV has body-frame names and no attitude fields.

- [ ] **Step 3: Implement state collection and serialization**

Create t1_yaw/t1_omega/t2_yaw/t2_omega in the initializer. Extend _record, _record_mocap, and every callback with those arrays. In _record, derive yaw from msg.pose.pose.orientation, use body_velocity_to_map for twist.linear.x/y, and append twist.angular.z. In mocap recording, use its expanded sample result. Write the exact Step 1 header and serialize nearest leader yaw/omega with follower yaw/omega. Make velocity_frame_label return Map-frame.

- [ ] **Step 4: Verify GREEN**

Run: cd /home/l1anggmgo/ros-projects/homo_multirobot_ws && source /opt/ros/humble/setup.bash && pytest src/homo-ctrl-multirobot-ros2/homo_multirobot_formation_control/test/test_record_trajectory_yaml.py -v

Expected: PASS.

- [ ] **Step 5: Commit**

~~~bash
git add homo_multirobot_formation_control/scripts/record_trajectory.py homo_multirobot_formation_control/test/test_record_trajectory_yaml.py
git commit -m "记录轨迹角度与角速度"
~~~

### Task 3: Replace the combined plot with independent figures

**Files:**

- Modify: homo_multirobot_formation_control/scripts/record_trajectory.py, _save_metadata and _plot_and_save.
- Test: homo_multirobot_formation_control/test/test_record_trajectory_yaml.py.

**Interfaces:**

- Consumes state arrays from Task 2.
- Produces trajectory.png, distance.png, map_velocity.png, speed.png, x.png, y.png, yaw.png, and metadata files.plots with those filenames and no check_plot.

- [ ] **Step 1: Write failing plot and metadata tests**

~~~python
def test_unwrap_yaw_series_removes_pi_boundary_jump():
    values = module.unwrap_yaw_series([math.radians(179), math.radians(-179)])
    assert values == pytest.approx([math.radians(179), math.radians(181)])

def test_metadata_lists_individual_plot_files(tmp_path):
    recorder = make_minimal_recorder()
    recorder._save_metadata(tmp_path)
    metadata = yaml.safe_load((tmp_path / 'metadata.yaml').read_text(encoding='utf-8'))
    assert metadata['files']['plots'] == [
        'trajectory.png', 'distance.png', 'map_velocity.png', 'speed.png',
        'x.png', 'y.png', 'yaw.png',
    ]
    assert 'check_plot' not in metadata['files']
~~~

- [ ] **Step 2: Verify RED**

Run: cd /home/l1anggmgo/ros-projects/homo_multirobot_ws && source /opt/ros/humble/setup.bash && pytest src/homo-ctrl-multirobot-ros2/homo_multirobot_formation_control/test/test_record_trajectory_yaml.py -k 'unwrap_yaw or metadata_lists' -v

Expected: FAIL because no unwrap helper exists and metadata still declares check.png.

- [ ] **Step 3: Implement standalone plotting**

~~~python
def unwrap_yaw_series(yaws):
    if not yaws:
        return []
    result = [yaws[0]]
    for yaw in yaws[1:]:
        delta = (yaw - result[-1] + math.pi) % (2.0 * math.pi) - math.pi
        result.append(result[-1] + delta)
    return result
~~~

Replace the six-axis canvas with a local save_figure(filename, draw) routine that creates, lays out, saves, and closes one figure. Port existing axes to trajectory.png, distance.png, map_velocity.png, speed.png, x.png, y.png. Add Leader/Follower omega curves to map_velocity.png. Add yaw.png using independently unwrapped Leader/Follower yaw arrays. Record the exact seven-image list in metadata.

- [ ] **Step 4: Verify GREEN**

Run: cd /home/l1anggmgo/ros-projects/homo_multirobot_ws && source /opt/ros/humble/setup.bash && python3 -m py_compile src/homo-ctrl-multirobot-ros2/homo_multirobot_formation_control/scripts/record_trajectory.py && pytest src/homo-ctrl-multirobot-ros2/homo_multirobot_formation_control/test/test_record_trajectory_yaml.py -v

Expected: both commands exit 0.

- [ ] **Step 5: Build and commit**

Run: cd /home/l1anggmgo/ros-projects/homo_multirobot_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=OFF

Expected: build exits 0.

~~~bash
git add homo_multirobot_formation_control/scripts/record_trajectory.py homo_multirobot_formation_control/test/test_record_trajectory_yaml.py
git commit -m "拆分轨迹记录检查图"
~~~

