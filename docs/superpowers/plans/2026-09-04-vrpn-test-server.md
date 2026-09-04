# VRPN Test Server Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Build a version-controlled VRPN Tracker server that publishes a deterministic moving `robot1` rigid body for laptop bridge testing.

**Architecture:** A standalone ament CMake package builds one C++ executable linked to VRPN. A small pure helper owns option parsing and trajectory math; `main()` owns the VRPN connection and publishes reports at the configured rate.

**Tech Stack:** C++17, ament_cmake, VRPN from `ros-humble-vrpn`, ROS 2 Humble, colcon.

## Global Constraints

- Create only `homo_multirobot_mocap_tools/`; do not modify the bridge, EKF, TF, or controller packages.
- Defaults: TCP port `3883`, tracker `robot1`, radius `1.0 m`, speed `0.5 m/s`, rate `100 Hz`.
- Reject port outside `1..65535`, non-positive radius/rate, and negative speed.
- Publish pose, global linear velocity, z-axis angular increment, and acceleration for one tracker.
- Build only from `/home/l1anggmgo/ros-projects/homo_multirobot_ws`.

---

### Task 1: Scaffold and test trajectory helpers

**Files:**
- Create: `homo_multirobot_mocap_tools/package.xml`
- Create: `homo_multirobot_mocap_tools/CMakeLists.txt`
- Create: `homo_multirobot_mocap_tools/include/homo_multirobot_mocap_tools/vrpn_test_server.hpp`
- Create: `homo_multirobot_mocap_tools/test/test_vrpn_test_server.cpp`

**Interfaces:**
- Produces `ServerOptions`, `parse_options(int, char**)`, and `circle_state(double, const ServerOptions&)`.

- [x] Write a failing gtest for defaults, invalid `--port 0`, and the state at `t=0`: position `(radius,0)`, velocity `(0,speed)`, acceleration `(-speed*speed/radius,0)`, yaw `pi/2`.
- [x] Run `source /opt/ros/humble/setup.bash && colcon test --packages-select homo_multirobot_mocap_tools`; expect package-not-found failure.
- [x] Add ament metadata, C++17 CMake configuration, gtest target, and minimal parser/state helper implementation.
- [x] Run `colcon build --packages-select homo_multirobot_mocap_tools --symlink-install --cmake-args -DBUILD_TESTING=ON`, then `colcon test --packages-select homo_multirobot_mocap_tools`; expect zero failures.

### Task 2: Implement VRPN server

**Files:**
- Create: `homo_multirobot_mocap_tools/src/vrpn_test_server.cpp`
- Modify: `homo_multirobot_mocap_tools/CMakeLists.txt`
- Modify: `homo_multirobot_mocap_tools/test/test_vrpn_test_server.cpp`

**Interfaces:**
- Consumes helper types from Task 1.
- Produces executable `vrpn_test_server`, accepting `--port`, `--tracker-name`, `--radius`, `--speed`, and `--rate`.

- [x] Add failing test for rejected `--rate 0`.
- [x] Run the package test and confirm the new assertion fails.
- [x] Implement parser validation and executable: create server connection, create `vrpn_Tracker_Server`, calculate circle state at each period, publish pose/velocity/acceleration, and exit cleanly on SIGINT/SIGTERM.
- [x] Rebuild and run package tests; expect build success and zero test failures.

### Task 3: Validate local end-to-end discovery

**Files:**
- Modify: none

- [x] Start `ros2 run homo_multirobot_mocap_tools vrpn_test_server -- --rate 50`; expect tracker and port in startup output.
- [x] In another sourced terminal start `ros2 launch vrpn_listener sync_entity_state.launch`; expect `robot1` discovery.
- [x] Run `ros2 topic list | rg '^/vrpn/robot1/(pose|twist|accel)$'` and `ros2 topic echo /vrpn/robot1/pose --once`; expect all three topics and non-zero pose.
- [x] Commit the package and plan with `git commit -m '新增VRPN测试服务端'`.
