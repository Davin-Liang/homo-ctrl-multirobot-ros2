# ROS 2 6D Map HPC Artstein Disc 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**目标：** 实现独立的 6D Map HPC Artstein Disc ROS 2 节点。

**架构：** 新核心负责多边形选点、map 系误差和正则化 HPC；节点复用现有 6D Artstein Disc 的 TF、预测、约束与命令历史链路。

**技术栈：** ROS 2 Humble、rclcpp、Eigen、tf2、ament_cmake、Python launch。

## 全局约束

- 不修改现有 `formation_control_node_6d_artstein_disc`。
- 多边形语义使用 `radius`、`m_p`、`tol` 的最近点与迟滞切换。
- 命令历史保存约束后的最终命令，map 平移和 yaw-rate 均 newest-first。
- 三频 yaw 抖动仅为离线数值仿真场景，不能加入 ROS 参数或节点逻辑。
- 新 6D 目标通过 `BUILD_6D_MAP_HPC_ARTSTEIN=ON` 显式启用，避免增加 ARM 默认编译负担。

### Task 1: 控制器核心与单元测试

**文件：**
- Create: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/homo_controller_6d_map_hpc_artstein.hpp`
- Create: `homo_multirobot_formation_control/test/test_6d_map_hpc_artstein_controller.cpp`
- Modify: `homo_multirobot_formation_control/CMakeLists.txt`

**接口：** `MapHpcController6DArtsteinDisc::command(const Eigen::VectorXd& leader, const Eigen::VectorXd& follower)` 返回 map 系 `Eigen::Vector3d`；`target_idx()` 返回当前多边形点。

- [ ] 写 `test_6d_map_hpc_artstein_controller.cpp`：Leader 为全零、Follower 为 `[-radius,0,0,0,0,0]` 时命令为零、目标为点 0；Leader yaw 为 `pi/2` 时点 0 的 map 偏移为 `[0,-radius]`；候选点只有超过 `tol` 才切换；关闭 HPC 时结果为基础线性反馈。
- [ ] 运行 `cd ../.. && source /opt/ros/humble/setup.bash && colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=ON -DBUILD_6D_MAP_HPC_ARTSTEIN=ON`，确认因头文件/目标缺失失败。
- [ ] 实现核心：以 `R(theta_leader)d_i` 形成 Leader 相对 map 目标；计算 `[e_p,e_theta,e_v_map,e_omega]`；使用数值仿真的 `K=-diag(m,m,I)[kp I,kv I]`、`Gd`、Lyapunov `P` 与 `hpc_c_min` 求正则化 HPC；生成 map 系速度命令。
- [ ] 在 CMake 的 `BUILD_TESTING` 块注册测试，并重新运行上述构建，确认测试通过。
- [ ] 提交：`git add homo_multirobot_formation_control/include/homo_multirobot_formation_control/homo_controller_6d_map_hpc_artstein.hpp homo_multirobot_formation_control/test/test_6d_map_hpc_artstein_controller.cpp homo_multirobot_formation_control/CMakeLists.txt && git commit -m "新增6D Map HPC控制器核心"`。

### Task 2: ROS 节点、预测和命令历史

**文件：**
- Create: `homo_multirobot_formation_control/include/homo_multirobot_formation_control/formation_control_node_6d_map_hpc_artstein.hpp`
- Create: `homo_multirobot_formation_control/src/formation_control_node_6d_map_hpc_artstein.cpp`
- Create: `homo_multirobot_formation_control/src/main_6d_map_hpc_artstein.cpp`
- Modify: `homo_multirobot_formation_control/CMakeLists.txt`

**接口：** 可执行程序为 `formation_control_node_6d_map_hpc_artstein`；参数为既有 namespace、Artstein、约束参数和 `mu/kp/kv/hpc_c_min`。

- [ ] 在 CMake 先声明三个不存在的新源文件，运行 `colcon build`，确认 CMake 因文件缺失失败。
- [ ] 从现有 6D Artstein Disc 节点复制 `ArtsteinPredictorNd`、TF/odometry 转换、`predict_leader_state` 和 `predict_follower_state`；类名改为 `FormationController6DMapHpcArtstein`，控制核心改为 Task 1 接口。
- [ ] 每周期按 `Td + max(tau,tau_yaw)` 预测 Leader 和 Follower；把核心 map 命令转为 Follower body 命令，经过现有标量限幅、最小速度和 `KinematicConstraint` 后发布。
- [ ] 只将最终发布的 body 命令用实际 Follower yaw 反变换为 map 速度后写入平移历史；最终 angular.z 写入 yaw 历史；保持 newest-first 与预测器缓冲长度一致。
- [ ] 运行 `cd ../.. && source /opt/ros/humble/setup.bash && colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=OFF -DBUILD_6D_MAP_HPC_ARTSTEIN=ON && test -x install/homo_multirobot_formation_control/lib/homo_multirobot_formation_control/formation_control_node_6d_map_hpc_artstein`，确认通过后提交节点文件。

### Task 3: Launch 与完整验证

**文件：**
- Create: `homo_multirobot_formation_control/launch/formation_single_follower_6d_map_hpc_artstein.launch.py`
- Modify: `homo_multirobot_formation_control/CMakeLists.txt`
- Test: `homo_multirobot_formation_control/test/test_sim_6d_map_hpc_artstein.py`

- [ ] 先运行 `python3 -c 'from pathlib import Path; assert Path("homo_multirobot_formation_control/launch/formation_single_follower_6d_map_hpc_artstein.launch.py").exists()'`，确认新 launch 尚不存在而失败。
- [ ] 以 `formation_single_follower_6d_artstein_disc.launch.py` 创建新 launch：替换 executable/name；保留 namespace、Artstein、约束和可选 `sim_motor_delay.py`；移除旧 LPC 参数，增加 `mu=-0.25`、`kp=1.2`、`kv=2.0`；不得添加 yaw 抖动参数。
- [ ] 运行 `cd ../.. && source /opt/ros/humble/setup.bash && colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=ON -DBUILD_6D_MAP_HPC_ARTSTEIN=ON && source install/setup.bash && python3 -m unittest src/homo-ctrl-multirobot-ros2/homo_multirobot_formation_control.test.test_sim_6d_map_hpc_artstein -v && ros2 launch homo_multirobot_formation_control formation_single_follower_6d_map_hpc_artstein.launch.py --show-args`。
- [ ] 确认构建、C++ 测试、20 项 Python 数值回归和 launch 参数检查全部成功；提交 launch/CMake 改动并用 `git status --short` 确认没有本次工程文件遗留未提交。
