# 6D Map-HPC Polygon Mocap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace fixed map offset formation in 6D Map-HPC Artstein with a map-fixed discrete polygon and add mocap input mode.

**Architecture:** The Map-HPC core owns `m_p` map-fixed offsets, tracks a selected target index, and rebuilds its gain/HPC state when hysteresis permits a switch. The ROS node selects EKF/TF or mocap state input and converts only mocap map velocity to the 6D model's body velocity.

**Tech Stack:** C++17, ROS 2 Humble, Eigen, rclcpp, geometry_msgs, nav_msgs, tf2.

## Global Constraints

- Remove fixed `offset_map_x/y`; use `m_p`, `radius`, `tol`.
- Candidate offsets are fixed in map and do not rotate with Leader yaw.
- Preserve 6D Disc mocap support and do not modify HOCBF.
- Default mode remains `state_source=ekf_tf`; mocap timeout is 0.10 s.
- Map-HPC target builds by default.

---

### Task 1: Implement and test map-fixed polygon selection

**Files:**
- Modify: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/homo_controller_6d_map_hpc_artstein.hpp`
- Create: `homo_multirobot_formation_control/test/test_6d_map_hpc_polygon.cpp`
- Modify: `homo_multirobot_formation_control/CMakeLists.txt`

- [ ] Write failing tests for four radius-2 map offsets and tolerance-protected target switching.
- [ ] Replace fixed offset constructor input with `m_p`, `radius`, `tol`; expose selected index.
- [ ] Select closest map-fixed offset and rebuild gain/HPC only after a valid switch.
- [ ] Build and run polygon unit tests.

### Task 2: Add Map-HPC mocap state mode

**Files:**
- Modify: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_6d_map_hpc_artstein.hpp`
- Modify: `homo_multirobot_formation_control/src/formation_control_node_6d_map_hpc_artstein.cpp`

- [ ] Add `state_source` and `mocap_state_timeout`; retain existing odometry/TF path as default.
- [ ] Subscribe to map-frame mocap pose/twist in mocap mode and map global velocity to body velocity once.
- [ ] On missing/stale data, publish zero cmd_vel and reset initialization/history.
- [ ] Use receipt-time diagnostics in mocap mode.

### Task 3: Replace launch/config interface and verify

**Files:**
- Modify: `homo_multirobot_formation_control/config/formation_single_follower_6d_map_hpc_artstein.yaml`
- Modify: `homo_multirobot_formation_control/launch/formation_single_follower_6d_map_hpc_artstein.launch.py`
- Create: `homo_multirobot_formation_control/launch/formation_single_follower_6d_map_hpc_artstein_mocap.launch.py`
- Modify: `docs/mocap/README.md`

- [ ] Remove offset arguments; add `m_p`, `radius`, `tol`, `state_source`, and `mocap_state_timeout`.
- [ ] Add mocap wrapper launch with `state_source=mocap`, `use_sim_time=false`, and no motor-delay node.
- [ ] Build without opt-in flags; verify target and launch arguments.
- [ ] Use VRPN test server plus two adapters for nonzero cmd_vel, then stop mocap chain for zero-command verification.
