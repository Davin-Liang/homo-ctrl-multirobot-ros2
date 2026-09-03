# VRPN Client ROS 2 Deployment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans task-by-task.

**Goal:** Vendor the selected VRPN bridge and verify it on ROS 2 Humble.

**Architecture:** The source is pinned and stored at `third_party/vrpn_client_ros2`; it is built from the colcon workspace root, without any link to existing localization, TF, or controller code.

**Tech Stack:** Ubuntu 22.04, ROS 2 Humble, colcon, CMake, VRPN, Git.

## Global Constraints

- Pin `https://github.com/efc-robot/vrpn_client_ros2.git` to `8731c69ab76bf66cebe8a47d0489cf64b5162445`.
- Install `ros-humble-vrpn`; do not hard-code lab server settings.
- Do not modify robot packages, EKF, or TF publishers.
- Never run colcon in the repository root.

---

### Task 1: Install VRPN

**Files:** none.

- [x] Run `dpkg-query -W -f='${Status}\\n' ros-humble-vrpn 2>/dev/null || true` to check the dependency.
- [x] Run `sudo apt update && sudo apt install -y ros-humble-vrpn`.
- [x] Run `test -f /opt/ros/humble/include/vrpn_Tracker.h` and expect exit code 0.

### Task 2: Vendor the bridge source

**Files:**
- Create: `third_party/vrpn_client_ros2/**`

- [x] Clone upstream into a `mktemp -d` directory, detached-check out `8731c69ab76bf66cebe8a47d0489cf64b5162445`, and verify with `git rev-parse HEAD`.
- [x] Import the clone without nested Git metadata into `third_party/vrpn_client_ros2`; verify `test ! -e third_party/vrpn_client_ros2/.git`.
- [x] Inspect `third_party/vrpn_client_ros2/src/vrpn_listener/package.xml` and `CMakeLists.txt` with `rg -n '<name>|vrpn|ament|find_package'` to determine the package name and required build interface.
- [x] Commit only the new vendor tree with message `纳入VRPN ROS2桥接源码`.

### Task 3: Build and launch-check

**Files:** none, unless a minimal Humble compatibility patch is strictly necessary.

- [x] From `/home/l1anggmgo/ros-projects/homo_multirobot_ws`, source `/opt/ros/humble/setup.bash` and run `colcon build --packages-select vrpn_listener --symlink-install`.
- [x] Source `install/setup.bash`; run `ros2 pkg executables vrpn_listener` and `ros2 pkg prefix vrpn_listener`, expecting both to succeed.
- [x] Start the upstream launch file without starting existing robot packages. In a second sourced terminal, run `ros2 node list` and `ros2 topic list | rg 'vrpn|pose'`; the bridge node and `/vrpn/VRPN_Control/{pose,twist,accel}` appeared.
- [x] Commit the required Humble compatibility patch with message `修复VRPN桥接Humble兼容性`.
