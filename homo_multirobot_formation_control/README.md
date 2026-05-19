# homo_multirobot_formation_control

基于**齐次控制（Homogeneous Control）** 的 Leader-Follower 编队算法（C++ / Eigen），
适配项目的 slam_toolbox / AMCL + EKF 定位体系。

提供两套控制器：**4D 质点模型**（原版论文算法）和 **6D 运动学模型**（考虑车身朝向 + 全向轮约束）。

## 目录

- [控制器版本](#控制器版本)
- [算法原理 (4D)](#算法原理-4d)
- [算法原理 (6D)](#算法原理-6d)
- [数据输入](#数据输入)
- [参数详解](#参数详解)
- [运动学约束参数](#运动学约束参数)
- [编译与启动](#编译与启动)
- [完整联调](#完整联调)
- [验证](#验证)

## 控制器版本

| 版本 | Launch 文件 | 可执行文件 | 状态模型 | 编队策略 | yaw 控制 |
|------|------------|-----------|---------|---------|---------|
| **4D (原版)** | `formation_single_follower.launch.py` | `formation_control_node` | 双积分器 `[p_x,p_y,v_x,v_y]` (map 系) | 离散多边形 + tol 切换 | 独立 P+前馈 |
| **6D (运动学)** | `formation_single_follower_6d.launch.py` | `formation_control_node_6d` | 混合系 `[p_x,p_y,θ,v_x^b,v_y^b,ω]` | 连续边界投影 | 集成于 6D 主回路 |

两套控制器共享以下模块（不修改原 4D 代码）：
- `kinematic_constraint.hpp` — 全向轮轮速/加速度约束
- `types_nd.hpp`, `hnorm_nd.hpp`, `lpc2hpc_nd.hpp` — N-D 泛化齐次控制工具库

## 算法原理 (4D)

1. 将机器人建模为**双重积分器**（4 阶状态：位置 x, y + 速度 vx, vy），
   系统矩阵 `A=[0,0,1,0; 0,0,0,1; 0,0,0,0; 0,0,0,0]`，输入矩阵 `B=[0,0; 0,0; 1/m,0; 0,1/m]`
2. 对线性状态反馈控制器（LPC）进行**齐次升级**（HPC），通过 `lpc2hpc` 算法
   引入 homogeneity degree `nu` 和 dilation generator `Gd`
3. 运行时用二分法计算**齐次范数** `hnorm(e, Gd, P)`（~35 次 `expm(4x4)` per tick），
   通过矩阵指数 `expm(Gd * (1-log(c)))` 对误差做非线性 warping：
   `u = c^(1+nu) * k_lin * expm(Gd * (1-log(c))) * e`
4. **编队几何**：`m_p` 个安全编队点均匀分布在以领航者为中心、`radius` 为半径的圆上，
   跟随者自动选择最近的编队点，并在领航者移动时动态切换（切换阈值为 `tol`）
5. **偏航控制**：比例 + 前馈，角度误差归一化到 [-π, π]

## 算法原理 (6D)

详细的数学推导见 `doc/kinematic_homogeneous_control.md`。核心要点：

1. **6D 混合系状态**：$[p_x, p_y, \theta, v_x^b, v_y^b, \omega]^\top$，位置/朝向在 map 系，
   速度在车体系，输出天然对应 `cmd_vel`
2. **误差在 leader 车体系下计算**：follower 速度按 $\Delta\theta$ 旋转后求差，
   控制力再旋转回 follower 车体系做前向欧拉积分
3. **边界投影编队**：$d = r_s \cdot (\mathbf{p}_f - \mathbf{p}_l)/\|\mathbf{p}_f - \mathbf{p}_l\|$，
   连续光滑，无离散切换
4. **时变 $A_l$ 矩阵**：含 leader 速度耦合项 $(\omega_l, v_{x,l}^b, v_{y,l}^b)$，
   每周期更新；HPC 参数在 leader 速度或 $\Delta\theta$ 变化超过阈值时重算
5. **yaw 控制集成**：$\theta/\omega$ 作为 3×6 增益矩阵的第三通道，临界阻尼双极点设计

## 数据输入

本包通过以下通道获取机器人状态：

| 数据 | 来源 | 4D 坐标系 | 6D 坐标系 |
|------|------|----------|----------|
| 位置 | TF `map → <prefix>_base_footprint` | map | map |
| 偏航角 | TF `map → <prefix>_base_footprint` 旋转 | map | map |
| 线速度 | EKF `odometry/filtered` | 旋转到 map | 车体系（不旋转） |
| 角速度 | EKF `odometry/filtered` | body | body |

> 6D 版本中车体系速度直接取自 EKF 消息 `twist.twist.linear.x/y`，不做旋转，
> 消除了 4D 版本中 map 系速度与 `cmd_vel` 车体系语义不匹配的问题。

## 参数详解

### 4D 控制器模型参数（launch 可改）

| 参数 | 类型 | 默认值 | 作用 | 调大效果 | 调小效果 |
|------|------|--------|------|----------|----------|
| `mass` | double | 8.0 | 双重积分器模型的等效质量 | 增益增大，响应更快 | 增益减小，响应更慢 |
| `m_p` | int | 4 | 安全编队点数量 | 更多编队位置可选 | 编队选择少 |
| `radius` | double | 2.0 | 编队圆半径 (m) | 跟随距离增大 | 跟随更近 |
| `tol` | double | 0.1 | 编队点切换容差 (m) | 不易频繁切换 | 切换更灵敏 |

### 4D 偏航控制参数（launch 可改）

| 参数 | 类型 | 默认值 | 作用 |
|------|------|--------|------|
| `Kp_yaw` | double | 4.0 | 偏航比例增益 |
| `K_ff` | double | 1.0 | 偏航前馈增益 |

### 6D 控制器参数（launch 可改）

| 参数 | 类型 | 默认值 | 作用 |
|------|------|--------|------|
| `radius` | double | 2.0 | 安全圆半径 (m)。边界投影的目标距离 |
| `mass` | double | 8.0 | 平移通道质量调谐参数 |
| `I` | double | 1.0 | 偏航通道转动惯量调谐参数 |
| `omega_d` | double | 1.5 | 位置通道临界阻尼带宽 |
| `omega_d_theta` | double | 1.5 | 偏航通道临界阻尼带宽 |
| `hpc_vel_threshold` | double | 0.3 | leader 速度变化触发 HPC 重算的阈值 |

> 6D 版本中没有 `m_p`、`tol`（边界投影无需离散点）、`Kp_yaw`、`K_ff`（yaw 集成于主回路）。

## 运动学约束参数

两个版本共享（基于 URDF 三全向轮几何，L=0.11m, r=0.03m）：

| 参数 | 类型 | 默认值 | 作用 |
|------|------|--------|------|
| `wheel_radius` | double | 0.03 | 轮半径 (m) |
| `base_radius` | double | 0.11 | 底盘半径 (m) |
| `wheel_max_omega` | double | 20.0 | 最大轮角速度 (rad/s)，超限等比缩放 |
| `max_linear_accel` | double | 2.0 | 线加速度 slew rate 限幅 (m/s²) |
| `max_angular_accel` | double | 4.0 | 角加速度 slew rate 限幅 (rad/s²) |

> 约束日志：轮速约束触发时每 2s 打印 `[WARN] 轮速约束触发: scale=X.XX`。

### 控制频率（launch 可改）

| 参数 | 类型 | 默认值 | 作用 |
|------|------|--------|------|
| `control_rate` | double | 20.0 | 控制循环频率 (Hz) |

### 4D 进阶参数（代码内硬编码）

以下参数在 `homo_controller.hpp` 中：

| 参数 | 默认值 | 作用 |
|------|--------|------|
| `omega_d` | 1.5 | 期望阻尼带宽 |
| `c` clamp 下界 | 0.5 | 齐次范数下限（原版 Python 0.1，提升以抑制噪声放大） |
| `h`（前向积分步长） | 0.1 | 前向欧拉步长 |

### 典型调参建议

| 场景 | 4D 版 | 6D 版 |
|------|------|------|
| slam_toolbox 定位 | `mass:=8.0 Kp_yaw:=4.0` | `mass:=8.0 I:=1.0 radius:=2.0` |
| 响应太慢 | 增大 `mass` 到 15.0 | 增大 `mass` 或降低 `omega_d` |
| 偏航跟不上 | 增大 `Kp_yaw`/`K_ff` | 增大 `omega_d_theta` 或 `I` |
| 编队距离不对 | 调 `radius` | 调 `radius` |

## 依赖

- `rclcpp`、`geometry_msgs`、`nav_msgs`
- `tf2_ros`、`tf2_geometry_msgs`
- `Eigen 3.4`（`libeigen3-dev`）+ unsupported 模块（KroneckerProduct, MatrixFunctions）
- `eigen3_cmake_module`

## 编译与启动

### 编译

```bash
colcon build --packages-select homo_multirobot_formation_control --symlink-install
```

### 启动（4D 单 follower）

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower.launch.py
```

带参数：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower.launch.py \
  mass:=2.0 radius:=2.0 Kp_yaw:=4.0 K_ff:=1.0
```

### 启动（6D 单 follower）

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_6d.launch.py
```

带参数：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_6d.launch.py \
  radius:=1.0 mass:=8.0 I:=1.0 wheel_max_omega:=10.0
```

### 启动（双 follower，4D 版）

```bash
ros2 launch homo_multirobot_formation_control formation_two_followers.launch.py
```

## 完整联调

```bash
# 1. Gazebo 双机仿真 + 里程计链路 (rf2o + EKF)
ros2 launch homo_multirobot_localization sim_rf2o_ekf_two_robots.launch.py \
  use_rviz:=false robot2_x:=4.0 robot2_yaw:=1.57

# 2. 地图 + slam_toolbox 定位
ros2 launch homo_multirobot_nav slam_toolbox_loc_two_robots.launch.py \
  robot2_map_start_x:=4.0 robot2_map_start_yaw:=1.57

# 3a. 编队控制 — 4D 版
ros2 launch homo_multirobot_formation_control formation_single_follower.launch.py

# 3b. 编队控制 — 6D 版
ros2 launch homo_multirobot_formation_control formation_single_follower_6d.launch.py \
  radius:=1.0 wheel_max_omega:=10.0

# 4. 键盘遥控领航者
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel
```

## 验证

```bash
# 检查 cmd_vel 连续发布
ros2 topic hz /robot2/cmd_vel

# 查看 TF 树
ros2 run tf2_tools view_frames

# 验证 map → base_footprint TF
ros2 run tf2_ros tf2_echo map robot2_base_footprint
```
