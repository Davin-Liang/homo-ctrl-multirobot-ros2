# Codex Handoff: 4D Artstein + Prediction Formation Controller

## 1. Context

Workspace:

```bash
/home/l1anggmgo/ros-projects/homo_multirobot_ws
```

Main package:

```text
src/homo-ctrl-multirobot-ros2/homo_multirobot_formation_control
```

Target architecture:

```text
Original 4D double-integrator homogeneous predictive controller
+ Artstein input-delay compensation
+ first-order motor forward state prediction
```

Design goal:

```text
Keep the upper HPC as the original 4D nilpotent double-integrator system A_h^2=0.
Do not augment -1/tau motor dynamics into the HPC state matrix.
Handle real actuator delay in an external prediction/mapping layer.
```

## 2. Important Files

Core C++ files:

```text
include/homo_multirobot_formation_control/homo_controller.hpp
include/homo_multirobot_formation_control/homo_controller_4d_artstein.hpp
include/homo_multirobot_formation_control/formation_control_node_4d_artstein.hpp
src/formation_control_node_4d_artstein.cpp
launch/formation_single_follower_4d_artstein.launch.py
```

Implementation notes:

- `LpcController` exposes `accel_calculate(...)`.
- `accel_calculate` returns the force-like/equivalent HPC input `u_hpc`; actual acceleration is `u_hpc / mass` through `B_h`.
- `formation_control_node_4d_artstein.cpp` adds `cmd_integrator_base`.
- `cmd_integrator_base:=pred` is the current recommended mode: `v_base = v_pred`.
- `cmd_integrator_base:=cmd` uses previous `cmd_vel`; it tested worse in the current circular tracking scenario.
- `formation_single_follower_4d_artstein.launch.py` forwards `cmd_integrator_base`, `initial_min_lambda`, `switch_min_lambda`, `hpc_c_min`, and `leader_vel_lpf_tau`.

Scripts:

```text
scripts/sim_4d_hpc_artstein_compare.py
scripts/record_velocity_diagnostics.py
scripts/sim_motor_delay.py
```

`record_velocity_diagnostics.py` records:

```text
/robot2/cmd_vel_raw       controller raw output
/robot2/cmd_vel           final delayed/executed command
/robot2/odometry/filtered follower EKF velocity
/robot1/odometry/filtered leader EKF velocity
```

Documentation:

```text
doc/4d_artstein_prediction_theory.md
doc/4d_artstein_prediction_simulation.md
doc/artstein_reduction.md
```

## 3. Build

Build from the workspace root, not from the source repository directory:

```bash
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

## 4. Gazebo Baseline

Recommended circular tracking baseline with delay simulation:

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_4d_artstein.launch.py \
  leader_ns:=/robot1 follower_ns:=/robot2 \
  tau:=0.43 Td:=0.22 control_rate:=20.0 \
  mass:=2.0 hpc_c_min:=0.1 \
  initial_min_lambda:=1.5 switch_min_lambda:=4.0 \
  min_cmd_vel:=0.0 max_linear_accel:=0.5 \
  use_motor_delay:=true motor_tau:=0.43 transport_delay:=0.22 delay_max_accel:=0.5 \
  cmd_integrator_base:=pred leader_vel_lpf_tau:=0.0
```

Record trajectory:

```bash
ros2 run homo_multirobot_formation_control record_trajectory.py \
  --ros-args -p mode:=sim -p duration:=60.0 \
  -p controller_node_name:=formation_control_node_4d_artstein
```

Velocity-chain diagnostic:

```bash
ros2 run homo_multirobot_formation_control record_velocity_diagnostics.py \
  --ros-args \
  -p leader_ns:=/robot1 \
  -p follower_ns:=/robot2 \
  -p mode:=sim \
  -p duration:=60.0 \
  -p tag:=vel_diag_artstein_pred
```

## 5. Real-Robot Baseline

Terminal 1, virtual leader:

```bash
ros2 run homo_multirobot_formation_control virtual_leader_circle.py \
  --ros-args -r __ns:=/virtual_leader \
  -p center_x:=1.5 -p center_y:=0.0 \
  -p radius:=0.5 -p speed:=0.25 -p direction:=ccw
```

Terminal 2, 4D Artstein controller:

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_4d_artstein.launch.py \
  leader_ns:=/virtual_leader follower_ns:=/robot2 \
  tau:=0.43 Td:=0.22 control_rate:=20.0 \
  mass:=2.0 radius:=1.0 hpc_c_min:=0.1 \
  initial_min_lambda:=1.5 switch_min_lambda:=4.0 \
  min_cmd_vel:=0.03 max_linear_accel:=0.25 \
  use_motor_delay:=false \
  cmd_integrator_base:=pred leader_vel_lpf_tau:=0.0
```

Record real trajectory:

```bash
ros2 run homo_multirobot_formation_control record_trajectory.py \
  --ros-args -p mode:=real -p duration:=30.0 \
  -p leader_ns:=/virtual_leader -p follower_ns:=/robot2 \
  -p radius:=1.0 -p controller_node_name:=formation_control_node_4d_artstein
```

## 6. Experimental Observations

Gazebo circular leader with delay enabled was tested around:

```text
control_rate=20Hz
tau=0.43
Td=0.22
motor_tau=0.43
transport_delay=0.22
max_linear_accel=0.5
delay_max_accel=0.5
```

Findings:

- `cmd_integrator_base:=pred` is better than `cmd`.
- `leader_vel_lpf_tau` is not helpful for circular tracking by default; it can smooth steady state but introduces leader-prediction phase lag and worse initial convergence.
- `leader_vel_lpf_tau:=0.0` should mean pass-through, not frozen filtering.
- `hpc_c_min:=0.2` did not significantly reduce raw speed oscillation compared with `0.1`.
- Lowering `initial_min_lambda` and `switch_min_lambda` reduced raw command amplitude.
- With `transport_delay=0` and `motor_tau=0.43`, tracking can look better because first-order lag is easier to predict than pure dead time.
- At `0.5m/s` leader speed, acceleration limits and delay dominate more clearly; tracking lag grows.
- 20Hz control frequency should be preserved for real-robot relevance.

## 7. Known Documentation/Tooling Issues

- `record_velocity_diagnostics.py` once had Windows/BOM/CRLF or shebang/permission issues and produced `OSError: [Errno 8] Exec format error`.
- Fix Python scripts by ensuring first line is `#!/usr/bin/env python3`, UTF-8 no BOM, LF line endings, and executable permission.
- README 4D Artstein sections were previously corrupted by mixed Windows/WSL Markdown writes; future edits should stay in WSL UTF-8 tooling.

## 8. Follow-Up Checks

- Confirm C++ Artstein wrapper still matches `doc/4d_artstein_prediction_theory.md`.
- Keep Python numerical simulation and ROS C++ parameter semantics aligned.
- Recheck `record_trajectory.py` PNG labels versus CSV recomputation when comparing real-robot distance statistics.
- Do strict delay-predictor ablations by synchronizing controller parameters and injected plant parameters, not only `tau:=0.0 Td:=0.0`.
