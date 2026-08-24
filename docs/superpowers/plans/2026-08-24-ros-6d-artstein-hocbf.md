# ROS 6D Artstein Predictor-HOCBF 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新增只依赖 `/scan` 的静态圆柱、多约束 predictor-HOCBF ROS 2 编队控制节点。

**Architecture:** 复用既有 6D Artstein Disc 的预测器、HPC 与命令历史；以独立 `hocbf_safety_filter` 实现静态圆柱拟合、map 系 HOCBF 约束和二维枚举 QP。新节点在轮速约束前滤波平移 map 命令，随后回写实际发布命令到 Artstein 历史。

**Tech Stack:** ROS 2 Humble、rclcpp、tf2、LaserScan、Eigen、ament_cmake/C++17。

## Global Constraints

- 新增节点与 launch，不能改旧 6D Artstein Disc 或旧 OA 的行为。
- 不允许 launch 参数输入障碍物真值中心或半径。
- 第一阶段只支持 scan 可见的静态圆柱；scan 过期、拟合失败或 QP 无解时零平移。
- HOCBF 在 map 系预测状态运行；最终命令和 Artstein 历史使用实际发布值。

---

### Task 1: 可单测的圆柱感知与 HOCBF-QP 核心

**Files:**
- Create: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/hocbf_safety_filter.hpp`
- Create: `homo_multirobot_formation_control/src/hocbf_safety_filter.cpp`
- Create: `homo_multirobot_formation_control/test/test_hocbf_safety_filter.cpp`
- Modify: `homo_multirobot_formation_control/CMakeLists.txt`

**Interfaces:**
- Produces `CylinderObservation { Eigen::Vector2d center; double radius; }`.
- Produces `fit_cylinder_cluster(points)`, `hocbf_halfspace(...)`, and `solve_translation_qp(...)`.

- [ ] **Step 1: Write failing gtest cases**

```cpp
TEST(HocbfSafetyFilter, FitsCircleAndUsesEveryHalfspace) {
  auto cylinder = fit_cylinder_cluster(points_on_circle({2.0, -1.0}, .25));
  EXPECT_NEAR(cylinder.center.x(), 2.0, 1e-3);
  EXPECT_NEAR(cylinder.radius, .25, 1e-3);
  auto result = solve_translation_qp({.5, 0.}, {0., 0.}, two_halfspaces, 1., 2., .05);
  EXPECT_TRUE(result.feasible);
}
```

- [ ] **Step 2: Build test and verify red**

Run from workspace root: `colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=ON --cmake-target test_hocbf_safety_filter`

Expected: target/source absent.

- [ ] **Step 3: Implement minimum geometry, HOCBF and 2D candidate QP**

Implement algebraic least-squares circle fitting, residual/radius validation, the numerical-model HOCBF affine halfspace, and candidate enumeration (nominal, each boundary projection, pair intersection, box boundaries). Return `feasible=false` for an empty candidate set.

- [ ] **Step 4: Build and run green test**

Run: `ctest --test-dir build/homo_multirobot_formation_control -R test_hocbf_safety_filter --output-on-failure`

Expected: PASS.

### Task 2: Artstein-HOCBF ROS node

**Files:**
- Create: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_6d_artstein_disc_hocbf.hpp`
- Create: `homo_multirobot_formation_control/src/formation_control_node_6d_artstein_disc_hocbf.cpp`
- Create: `homo_multirobot_formation_control/src/main_6d_artstein_disc_hocbf.cpp`
- Modify: `homo_multirobot_formation_control/CMakeLists.txt`

**Interfaces:**
- Consumes `LaserScan`, timestamped `map <- scan.frame_id` TF, EKF odometry and Task 1 filter.
- Produces follower-relative `cmd_vel` and diagnostics.

- [ ] **Step 1: Add a failing compile target**

Add the new executable and include `hocbf_safety_filter.hpp`; build before source implementation.

- [ ] **Step 2: Verify red build**

Run from workspace root: `colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=OFF`

Expected: failure because node sources are absent.

- [ ] **Step 3: Implement the node by adapting only 6D Artstein Disc state/prediction flow**

Add scan callback: filter finite ranges, contiguous cluster, circle fit, scan-stamp transform to map, and conservative radius inflation. In the timer: calculate predicted follower state, generate nominal HPC body command, map-transform it, form each HOCBF constraint, solve QP, map-transform safe command back to body, apply kinematics, publish and record the actual map command. A stale scan, no valid scan geometry during required safety mode, or infeasible QP emits zero translation and a throttled warning.

- [ ] **Step 4: Build green**

Run from workspace root: `source /opt/ros/humble/setup.bash && colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=OFF`

Expected: exit 0 and new executable installed.

### Task 3: Launch, validation and commit

**Files:**
- Create: `homo_multirobot_formation_control/launch/formation_single_follower_6d_artstein_disc_hocbf.launch.py`
- Modify: `homo_multirobot_formation_control/CMakeLists.txt`
- Modify: `docs/superpowers/plans/2026-08-24-ros-6d-artstein-hocbf.md`

- [ ] **Step 1: Add launch arguments**

Expose namespaces, Artstein/kinematic parameters, scan topic, fit point/radius/residual thresholds, follower radius, clearance, perception margin and scan timeout. Do not add obstacle position or true-radius arguments.

- [ ] **Step 2: Verify launch syntax and build**

Run from workspace root: `source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch homo_multirobot_formation_control formation_single_follower_6d_artstein_disc_hocbf.launch.py --show-args`

Expected: shows only controller, perception and safety parameters.

- [ ] **Step 3: Run unit test and inspect diff**

Run: `ctest --test-dir build/homo_multirobot_formation_control --output-on-failure && git diff --check`.

- [ ] **Step 4: Commit source, test, launch and plan**

Run: `git add homo_multirobot_formation_control/include/homo_multirobot_formation_control/hocbf_safety_filter.hpp homo_multirobot_formation_control/src/hocbf_safety_filter.cpp homo_multirobot_formation_control/test/test_hocbf_safety_filter.cpp homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_6d_artstein_disc_hocbf.hpp homo_multirobot_formation_control/src/formation_control_node_6d_artstein_disc_hocbf.cpp homo_multirobot_formation_control/src/main_6d_artstein_disc_hocbf.cpp homo_multirobot_formation_control/launch/formation_single_follower_6d_artstein_disc_hocbf.launch.py homo_multirobot_formation_control/CMakeLists.txt docs/superpowers/plans/2026-08-24-ros-6d-artstein-hocbf.md && git commit -m "实现ROS 6D Artstein HOCBF避障"`.
