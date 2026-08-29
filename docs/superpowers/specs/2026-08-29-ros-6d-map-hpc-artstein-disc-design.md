# ROS 2 6D Map HPC Artstein Disc 工程设计

**目标：** 新增一个独立的 ROS 2 6D Artstein 控制器：采用 map 系固定偏移、4D 同构
`k_lin → lpc2hpc` 的控制链路与 Artstein 延迟补偿。

## 范围

- 新增可执行程序、节点类、控制器核心和 launch 文件；不修改
  `formation_control_node_6d_artstein_disc`。
- 使用 `offset_map_x/y` 的单一固定 map 编队偏移；不使用 `radius`、`m_p`、`tol`，
  Leader 自转不会带动 Follower 绕圈。
- 三频未知 Leader yaw 抖动仅是离线数值仿真的扰动场景；ROS 工程不生成这类虚拟
  抖动，也不增加专用 Leader 运动模型。

## 架构与数据流

新节点遵循现有 6D Artstein Disc 节点的 ROS 接口和运行流程：

```
Leader/Follower 的 /odometry/filtered 与 map TF
  → 获取 map 位姿，并完成 body/map 速度转换
  → Artstein 预测 Follower 状态；以常 twist 预测 Leader 状态
  → 计算固定 map 偏移误差
  → map-frame 正则化 6D HPC
  → map 系速度命令转换为 Follower body 系 cmd_vel
  → 限幅、轮速/加速度约束、发布，并把最终命令回写历史
```

Follower 预测沿用既有的“map 平移 4D + yaw 2D”Artstein 分离结构。命令历史保存
经过全部约束后的最终命令：平移部分以 map 系速度保存、转动部分以 yaw-rate 保存，
并按“最新命令在队首”排列；这与已验证的数值仿真积分顺序一致。

对于固定 map 偏移 `d_map`，误差为：

```
e_p     = p_f - p_l - d_map
e_theta = wrap(theta_f - theta_l)
e_v     = v_f_map - v_l_map
e_omega = omega_f - omega_l
```

新控制器初始化时按 4D 同构规则计算三通道 `k_lin`，随后用 `lpc2hpc_nd` 同步生成
`P`、`Gd`、`nu`。固定偏移无切换事件，运行期保持该组参数不变。

## 新增或修改的文件

- `include/homo_multirobot_formation_control/homo_controller_6d_map_hpc_artstein.hpp`：
  map 系正则化 HPC 与多边形选点。
- `include/homo_multirobot_formation_control/formation_control_node_6d_map_hpc_artstein.hpp`：
  节点状态、Artstein 预测器、ROS 接口和诊断。
- `src/formation_control_node_6d_map_hpc_artstein.cpp`、
  `src/main_6d_map_hpc_artstein.cpp`：节点实现和程序入口。
- `launch/formation_single_follower_6d_map_hpc_artstein.launch.py`：启动参数和可选的
  仿真电机延迟注入。
- `CMakeLists.txt`：新增可执行程序和安装目标。
- `test/test_6d_map_hpc_artstein_controller.cpp`：不依赖 ROS/Gazebo 的确定性单元测试，
  覆盖零误差、map 系误差/选点、因果 Leader 预测和命令历史语义。

## 验收条件

1. 现有 6D Artstein Disc 节点及其 launch 文件保持不变。
2. 在工作空间根目录可成功编译 formation-control 包。
3. 单元测试验证选点、map 系误差和因果预测行为，且不依赖 ROS/Gazebo 运行时。
4. 新 launch 暴露现有执行器/约束参数及新的 map-HPC 增益参数，并发布 Follower
   body 系 `cmd_vel`。
5. 完整 Python 数值仿真回归测试保持通过。
