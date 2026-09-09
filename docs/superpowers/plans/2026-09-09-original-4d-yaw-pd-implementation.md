# 原始 4D 偏航 PD 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将原始 4D 节点的 yaw 改为使用 Leader/Follower 相对角速度的线性 PD 控制。

**Architecture:** 复用已验证的 `yaw_pd_command`；节点分别读取两台车的角速度，在最终轮速约束前生成 yaw 命令。原始 4D 的 launch/YAML 参数由 `K_ff` 改为 `Kd_yaw`。

**Tech Stack:** ROS 2 Humble、C++17、Python unittest、colcon。

## Global Constraints

- 使用 `Kp_yaw * wrap(yaw_l-yaw_f) + Kd_yaw * (omega_l-omega_f)`，并受 `max_angular_vel` 限幅。
- 不修改原始 4D 平移 HPC、轮速或加速度约束。
- 修正 EKF 路径的 Follower 角速度输出变量；mocap 路径同样填写它。
- 只更新原始 4D 的 `K_ff` 接口；其他控制器的兼容采集保持不变。

### Task 1: 接入 PD 与修正状态数据

**Files:**
- Modify: `homo_multirobot_formation_control/src/formation_control_node.cpp`
- Modify: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node.hpp`

- [ ] **Step 1: 写入失败的源码接口测试**

在 `test/test_launch_yaml_defaults.py` 增加断言，要求 `formation_control_node.cpp` 同时声明 `leader_az, follower_az` 并调用 `yaw_pd_command`。

- [ ] **Step 2: 运行测试确认失败**

Run: `python3 -m pytest homo_multirobot_formation_control/test/test_launch_yaml_defaults.py -q`

Expected: FAIL，当前节点没有 `follower_az` 与 PD helper 调用。

- [ ] **Step 3: 最小实现**

包含 `yaw_pd_controller.hpp`；在两条状态源路径分别填充 `leader_az`、`follower_az`；将 `K_ff_` 改为 `Kd_yaw_` 并以 `yaw_pd_command(...)` 生成 `cmd.angular.z`。

- [ ] **Step 4: 运行测试与构建**

Run:

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_launch_yaml_defaults.py -q
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/homo_multirobot_formation_control -R test_yaw_pd_controller --output-on-failure
```

- [ ] **Step 5: Commit**

```bash
git add homo_multirobot_formation_control/src/formation_control_node.cpp \
  homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node.hpp \
  homo_multirobot_formation_control/test/test_launch_yaml_defaults.py
git commit -m "原始4D偏航改为PD控制"
```

### Task 2: 更新原始 4D 参数接口

**Files:**
- Modify: `homo_multirobot_formation_control/launch/formation_single_follower.launch.py`
- Modify: `homo_multirobot_formation_control/config/formation_single_follower.yaml`
- Modify: `homo_multirobot_formation_control/README.md`

- [ ] **Step 1: 扩展失败测试**

断言原始 4D YAML 和 launch 存在 `Kd_yaw`、不存在 `K_ff`。

- [ ] **Step 2: 运行测试确认失败**

Run: `python3 -m pytest homo_multirobot_formation_control/test/test_launch_yaml_defaults.py -q`

- [ ] **Step 3: 最小实现**

将 YAML、launch 局部变量、节点参数映射和声明的 `K_ff` 替换为 `Kd_yaw: 1.0`；README 原始 4D yaw 描述更新为独立线性 PD。

- [ ] **Step 4: 验证与提交**

Run 上述 pytest、colcon 与 CTest 命令；通过后提交接口文件，且不暂存不相关用户修改。
