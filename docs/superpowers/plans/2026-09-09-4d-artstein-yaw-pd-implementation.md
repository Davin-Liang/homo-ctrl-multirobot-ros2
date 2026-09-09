# 4D Artstein Yaw PD Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace 4D Artstein yaw P-plus-feedforward control with a testable linear PD controller using yaw and angular-velocity errors.

**Architecture:** Add a small header-only yaw helper so the controller's angle wrapping, PD calculation, and angular-speed saturation are independently testable. The ROS node obtains existing Leader/Follower yaw and `angular.z` measurements from either state source, passes them to that helper, and keeps downstream wheel and acceleration constraints unchanged. Rename the public tuning parameter from `K_ff` to `Kd_yaw` everywhere this launchable controller exposes it.

**Tech Stack:** C++17, ROS 2 Humble (`rclcpp`), standalone C++ `assert` test executable, Python launch/YAML validation, colcon.

## Global Constraints

- Implement exactly `omega_cmd = sat(Kp_yaw * wrap(leader_yaw - follower_yaw) + Kd_yaw * (leader_angular_z - follower_angular_z))`.
- `wrap` uses `atan2(sin(error), cos(error))`, giving an error in \([-pi, pi]\).
- `Kp_yaw` remains `4.0`; replace `K_ff` with `Kd_yaw`, default `1.0`.
- Do not apply Artstein prediction to yaw or alter the 4D translation/HPC path.
- Preserve `max_angular_vel`, wheel-speed, and `max_angular_accel` constraints on the final command.
- Both `ekf_tf` and `mocap` sources use their existing measured Leader/Follower `angular.z` values.
- Do not stage unrelated user changes in dirty configuration/RViz files.

---

## File Structure

- `homo_multirobot_formation_control/include/homo_multirobot_formation_control/yaw_pd_controller.hpp`: pure yaw wrapping and saturated PD command helper.
- `homo_multirobot_formation_control/test/test_yaw_pd_controller.cpp`: numeric unit tests for the helper.
- `homo_multirobot_formation_control/src/formation_control_node_4d_artstein.cpp`: declares `Kd_yaw` and uses the helper in the yaw command path.
- `homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_4d_artstein.hpp`: renames the stored yaw derivative gain.
- `homo_multirobot_formation_control/launch/formation_single_follower_4d_artstein.launch.py`: exposes only `Kd_yaw` and forwards it to the node.
- `homo_multirobot_formation_control/config/formation_single_follower_4d_artstein.yaml`: changes the documented default from `K_ff` to `Kd_yaw`.
- `homo_multirobot_formation_control/scripts/record_trajectory.py`: requests `Kd_yaw` when collecting controller metadata.
- `homo_multirobot_formation_control/README.md`: describes 4D Artstein yaw as independent PD and uses the new parameter name.
- `homo_multirobot_formation_control/CMakeLists.txt`: registers the new GoogleTest target.

### Task 1: Create a tested yaw PD helper

**Files:**
- Create: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/yaw_pd_controller.hpp`
- Create: `homo_multirobot_formation_control/test/test_yaw_pd_controller.cpp`
- Modify: `homo_multirobot_formation_control/CMakeLists.txt:311-322`

**Interfaces:**
- Produces: `formation_control::wrap_yaw_error(double error) -> double`.
- Produces: `formation_control::yaw_pd_command(double leader_yaw, double follower_yaw, double leader_angular_z, double follower_angular_z, double kp, double kd, double max_angular_vel) -> double`.
- Consumes: no ROS node state; only scalar values, `<algorithm>`, and `<cmath>`.

- [ ] **Step 1: Write failing C++ behavior tests**

```cpp
#include <cassert>
#include "homo_multirobot_formation_control/yaw_pd_controller.hpp"

int main() {
  assert(formation_control::yaw_pd_command(
      0.5, 0.1, 0.7, 0.2, 4.0, 1.0, 10.0) == 2.1);
  const double command = formation_control::yaw_pd_command(
      -3.10, 3.10, 0.0, 0.0, 1.0, 0.0, 10.0);
  assert(std::abs(command - 0.083185) < 1e-5);
  assert(formation_control::yaw_pd_command(
      2.0, 0.0, 0.0, 0.0, 4.0, 1.0, 0.8) == 0.8);
  return 0;
}
```

- [ ] **Step 2: Register and run the test to verify failure**

Add this CMake target inside `if(BUILD_TESTING)`:

```cmake
add_executable(test_yaw_pd_controller test/test_yaw_pd_controller.cpp)
target_include_directories(test_yaw_pd_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(test_yaw_pd_controller Eigen3::Eigen)
add_test(NAME test_yaw_pd_controller COMMAND test_yaw_pd_controller)
```

Run from the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select homo_multirobot_formation_control --symlink-install
source install/setup.bash
ctest --test-dir build/homo_multirobot_formation_control -R test_yaw_pd_controller --output-on-failure
```

Expected: build fails because `yaw_pd_controller.hpp` does not exist.

- [ ] **Step 3: Implement the minimal header-only helper**

```cpp
#pragma once
#include <algorithm>
#include <cmath>

namespace formation_control {
inline double wrap_yaw_error(double error) {
  return std::atan2(std::sin(error), std::cos(error));
}

inline double yaw_pd_command(double leader_yaw, double follower_yaw,
                             double leader_angular_z, double follower_angular_z,
                             double kp, double kd, double max_angular_vel) {
  const double yaw_error = wrap_yaw_error(leader_yaw - follower_yaw);
  const double angular_velocity_error = leader_angular_z - follower_angular_z;
  return std::clamp(kp * yaw_error + kd * angular_velocity_error,
                    -max_angular_vel, max_angular_vel);
}
}  // namespace formation_control
```

- [ ] **Step 4: Build and run the helper test**

Run the same colcon/ctest commands from Step 2.

Expected: target builds and all three tests pass.

- [ ] **Step 5: Commit the helper slice**

```bash
git add homo_multirobot_formation_control/include/homo_multirobot_formation_control/yaw_pd_controller.hpp \
  homo_multirobot_formation_control/test/test_yaw_pd_controller.cpp \
  homo_multirobot_formation_control/CMakeLists.txt
git commit -m "新增偏航PD控制器"
```

### Task 2: Switch the 4D Artstein node to PD yaw control

**Files:**
- Modify: `homo_multirobot_formation_control/src/formation_control_node_4d_artstein.cpp:15-20, 104-110, 349-354`
- Modify: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_4d_artstein.hpp:35-41`

**Interfaces:**
- Consumes: `formation_control::yaw_pd_command(...)` from Task 1 and existing `leader_yaw`, `follower_yaw`, `leader_az`, `follower_az` local variables.
- Produces: `cmd.angular.z` using the PD law before the unchanged `constraint_.apply(...)` call.
- Produces: node parameter `Kd_yaw` with default `1.0`.

- [ ] **Step 1: Extend the PD unit test with a nonzero follower-rate case**

```cpp
assert(formation_control::yaw_pd_command(
    0.0, 0.0, 0.0, 0.3, 4.0, 1.0, 10.0) == -0.3);
```

- [ ] **Step 2: Run the targeted test to verify the derivative behavior**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select homo_multirobot_formation_control --symlink-install
source install/setup.bash
ctest --test-dir build/homo_multirobot_formation_control -R test_yaw_pd_controller --output-on-failure
```

Expected: PASS; this locks the required negative derivative command before the ROS node is wired to the helper.

- [ ] **Step 3: Wire the node to the helper and rename the stored gain**

Include `yaw_pd_controller.hpp`. Change the header member from `K_ff_` to `Kd_yaw_`; declare it in the constructor with:

```cpp
Kd_yaw_ = declare_parameter("Kd_yaw", 1.0);
```

Replace the current manual error and feedforward block with:

```cpp
cmd.angular.z = formation_control::yaw_pd_command(
    leader_yaw, follower_yaw, leader_az, follower_az,
    Kp_yaw_, Kd_yaw_, max_angular_vel_);
```

Do not modify `constraint_.apply`, the command history, or either state-source extraction block.

- [ ] **Step 4: Build the production node and rerun the targeted test**

Run the Task 2 build/ctest commands.

Expected: the formation controller target compiles and the PD tests pass.

- [ ] **Step 5: Commit node integration**

```bash
git add homo_multirobot_formation_control/src/formation_control_node_4d_artstein.cpp \
  homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_4d_artstein.hpp \
  homo_multirobot_formation_control/test/test_yaw_pd_controller.cpp
git commit -m "4D Artstein偏航改为PD控制"
```

### Task 3: Update the public parameter interface and documentation

**Files:**
- Modify: `homo_multirobot_formation_control/launch/formation_single_follower_4d_artstein.launch.py:45-47, 100-101, 167-170`
- Modify: `homo_multirobot_formation_control/config/formation_single_follower_4d_artstein.yaml:25-28`
- Modify: `homo_multirobot_formation_control/scripts/record_trajectory.py:34-40`
- Modify: `homo_multirobot_formation_control/README.md:30-35, 656-660`
- Test: `homo_multirobot_formation_control/test/test_launch_yaml_defaults.py`

**Interfaces:**
- Consumes: YAML key `Kd_yaw` and launch `LaunchConfiguration('Kd_yaw')`.
- Produces: no `K_ff` declaration or forwarding in the 4D Artstein launch path.
- Produces: trajectory recorder metadata includes `Kd_yaw` if the controller exposes it while retaining `K_ff` collection for the other controllers that still use yaw feedforward.

- [ ] **Step 1: Write failing public-interface assertions**

Add to the existing Python launch/YAML test:

```python
def test_4d_artstein_yaw_pd_parameter_interface(self):
    yaml_path = CONFIG_DIR / "formation_single_follower_4d_artstein.yaml"
    launch_path = LAUNCH_DIR / "formation_single_follower_4d_artstein.launch.py"
    names = yaml_parameter_names(yaml_path)
    source = launch_path.read_text(encoding="utf-8")

    self.assertIn("Kd_yaw", names)
    self.assertNotIn("K_ff", names)
    self.assertIn('LaunchConfiguration("Kd_yaw")', source)
    self.assertIn('"Kd_yaw": Kd_yaw', source)
    self.assertNotIn("K_ff", source)
```

- [ ] **Step 2: Run the Python interface test to verify failure**

Run: `python3 -m pytest homo_multirobot_formation_control/test/test_launch_yaml_defaults.py -q`

Expected: FAIL because the YAML and launch currently expose `K_ff`.

- [ ] **Step 3: Update config, launch, recorder metadata list, and README**

Replace the YAML `K_ff: 1.0` plus “偏航前馈增益” comment with `Kd_yaw: 1.0` and “偏航微分增益（相对角速度误差）”. Rename the launch local variable, node parameter map entry, and `DeclareLaunchArgument`; describe it as “Yaw derivative gain on leader-follower angular velocity error”. In `record_trajectory.py`, add `Kd_yaw` to `CTRL_PARAM_NAMES` and the title list without removing `K_ff`, because other controller variants still expose the latter. Update README's 4D Artstein yaw description from “独立 P+前馈” to “独立线性 PD”, and update the automatic parameter list.

- [ ] **Step 4: Run source-level checks and targeted build**

Run:

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_launch_yaml_defaults.py -q
python3 -m pytest homo_multirobot_formation_control/test/test_record_trajectory_yaml.py -q
python3 -m py_compile homo_multirobot_formation_control/launch/formation_single_follower_4d_artstein.launch.py \
  homo_multirobot_formation_control/scripts/record_trajectory.py
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select homo_multirobot_formation_control --symlink-install
source install/setup.bash
ctest --test-dir build/homo_multirobot_formation_control --output-on-failure
```

Expected: Python tests pass, Python files parse, the package builds, and its CTest suite passes.

- [ ] **Step 5: Commit public interface changes**

```bash
git add homo_multirobot_formation_control/launch/formation_single_follower_4d_artstein.launch.py \
  homo_multirobot_formation_control/config/formation_single_follower_4d_artstein.yaml \
  homo_multirobot_formation_control/scripts/record_trajectory.py \
  homo_multirobot_formation_control/README.md \
  homo_multirobot_formation_control/test/test_launch_yaml_defaults.py
git commit -m "更新4D Artstein偏航PD参数接口"
```
