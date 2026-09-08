# 6D Artstein Disc Mocap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a pure-mocap input mode to the existing 6D Artstein Disc controller while preserving its default EKF/TF behavior.

**Architecture:** The controller selects `ekf_tf` or `mocap` at launch. Mocap mode consumes map-frame pose/twist from the existing adapters, maps only linear velocity to body coordinates for the 6D model, and otherwise reuses the existing predictors, controller core, constraints, and command output.

**Tech Stack:** C++17, ROS 2 Humble, rclcpp, geometry_msgs, nav_msgs, tf2, Eigen.

## Global Constraints

- Change only `formation_control_node_6d_artstein_disc`; do not modify HOCBF or other 6D nodes.
- `state_source=ekf_tf` remains default and behavior-compatible.
- Mocap input is `/<robot_ns>/mocap/pose` and `/<robot_ns>/mocap/twist`, both in `map`.
- Mocap velocity is global map velocity and must be converted once to body velocity for the 6D state.
- Any pose/twist timeout over `0.10 s` publishes zero cmd_vel and resets controller initialization.
- Build from `/home/l1anggmgo/ros-projects/homo_multirobot_ws`.

---

### Task 1: Add testable 6D mocap-state conversion

**Files:**
- Modify: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_6d_artstein_disc.hpp`
- Modify: `homo_multirobot_formation_control/src/formation_control_node_6d_artstein_disc.cpp`
- Create: `homo_multirobot_formation_control/test/test_6d_artstein_disc_mocap.cpp`

- [ ] Add a failing gtest for `yaw=pi/2`, `v_map=(1,0)`, expecting `v_body=(0,-1)`.
- [ ] Run the new test target and confirm failure before implementation.
- [ ] Add reusable map-to-body state construction and test it.
- [ ] Build and run the package tests; expect zero failures.

### Task 2: Add mocap subscriptions and safety handling

**Files:**
- Modify: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_6d_artstein_disc.hpp`
- Modify: `homo_multirobot_formation_control/src/formation_control_node_6d_artstein_disc.cpp`

- [ ] Add `state_source` and `mocap_state_timeout` parameters, validating `ekf_tf|mocap` and positive timeout.
- [ ] Keep existing Odometry/TF subscriptions only for `ekf_tf`; create SensorDataQoS pose/twist subscriptions for `mocap`.
- [ ] In `mocap`, build `State6D` directly from map pose/yaw and map twist after converting map velocity to body velocity.
- [ ] On absent or stale leader/follower mocap data, publish zero cmd_vel, clear controller initialization and both command histories.
- [ ] Use mocap receipt timestamps in diagnostics for mocap mode.

### Task 3: Add launch and end-to-end verification

**Files:**
- Modify: `homo_multirobot_formation_control/launch/formation_single_follower_6d_artstein_disc.launch.py`
- Create: `homo_multirobot_formation_control/launch/formation_single_follower_6d_artstein_disc_mocap.launch.py`
- Modify: `docs/mocap/README.md`

- [ ] Add `state_source` and `mocap_state_timeout` launch arguments, defaulting to `ekf_tf` and `0.10`.
- [ ] Add mocap wrapper launch that fixes `state_source=mocap`, `use_sim_time=false`, and `use_motor_delay=false`.
- [ ] Run `vrpn_test_server`, `mocap_two_robots.launch.py` with both adapters mapped to `robot1`, and the 6D mocap launch; verify nonzero `/robot2/cmd_vel`.
- [ ] Stop mocap launch and verify zero `/robot2/cmd_vel` within 0.10 s.
- [ ] Commit only 6D Artstein Disc mocap files and documentation.
