# homo_multirobot_formation_control

基于**齐次控制（Homogeneous Control）** 的 Leader-Follower 编队算法（C++ / Eigen），
适配项目的 slam_toolbox / AMCL + EKF 定位体系。

## 目录

- [算法原理](#算法原理)
- [数据输入](#数据输入)
- [参数详解](#参数详解)
- [编译与启动](#编译与启动)
- [完整联调](#完整联调)
- [验证](#验证)

## 算法原理

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

## 数据输入

本包通过以下通道获取机器人状态（不修改齐次控制算法本身）：

| 数据 | 来源 | 坐标系 |
|------|------|--------|
| 位置 | TF `map → <prefix>_base_footprint` | map |
| 偏航角 | TF `map → <prefix>_base_footprint` 旋转 | map |
| 速度 | EKF `odometry/filtered` 本体速度，通过 EKF yaw + TF yaw 旋转到 map | map |
| 角速度 | EKF `odometry/filtered` | body |

> 位置和速度来自不同的源但经过 TF 统一变换到同一个 map 帧。这是 Gazebo `/odom`
> （自带位置+速度+偏航）在真实定位体系下的最佳等效替代。

## 参数详解

### 控制器模型参数（launch 可改）

| 参数 | 类型 | 默认值 | 作用 | 调大效果 | 调小效果 |
|------|------|--------|------|----------|----------|
| `mass` | double | 8.0 | 双重积分器模型的等效质量，决定 `B` 矩阵的 `1/m` 以及 `_calculate_klin` 中的增益带宽 `omega_d * mass` | 增益增大，响应更快，编队收敛更猛 | 增益减小，响应变慢，更平滑但可能跟不上 |
| `m_p` | int | 4 | 安全编队点数量。均匀分布在以领航者为中心的圆上 | 更多编队位置可选，适合多 follower | 编队选择少，但切换更少 |
| `radius` | double | 2.0 | 编队圆半径 (m)。跟随者与领航者的期望距离 | 跟随距离增大 | 跟随更近 |
| `tol` | double | 0.1 | 编队点切换容差 (m)。只有当另一个编队点比当前点近 `tol` 以上时才切换 | 不易频繁切换，编队更稳定 | 切换更灵敏，但可能频繁跳变 |

### 偏航控制参数（launch 可改）

| 参数 | 类型 | 默认值 | 作用 | 调大效果 | 调小效果 |
|------|------|--------|------|----------|----------|
| `Kp_yaw` | double | 4.0 | 偏航比例增益。`angular_cmd = norm_error * Kp_yaw + leader_angular_z * K_ff` | 偏航纠正更快 | 偏航更平稳，避免振荡 |
| `K_ff` | double | 1.0 | 偏航前馈增益。直接跟随领航者的角速度 | 领航者转弯时跟随更及时 | 减少前馈，但转弯时可能滞后 |

### 控制频率（launch 可改）

| 参数 | 类型 | 默认值 | 作用 |
|------|------|--------|------|
| `control_rate` | double | 20.0 | 控制循环频率 (Hz) |

### 进阶参数（代码内硬编码，需改代码调整）

以下参数定义在 `include/homo_multirobot_formation_control/homo_controller.hpp` 中：

| 参数 | 位置 | 默认值 | 作用 |
|------|------|--------|------|
| `omega_d` | `calculate_klin()` 函数 | 1.5 | 期望的系统阻尼带宽。`a = max(val_a, omega_d * mass/b>` 是增益下界 |
| `c` clamp 下界 | `lpc_calculate()` 函数 | 0.5 | 齐次范数的下限。在齐次控制律中 `c` 越小 `expm` 放大倍数越大。原版 Python 用 0.1（Gazebo 完美数据下），C++ 用 0.5（抑制定位噪声放大） |
| `c` clamp 上界 | `lpc_calculate()` 函数 | 1.0 | 齐次范数的上限，对应最小放大 |
| `val_a / val_b` clamp | `calculate_klin()` 函数 | `omega_d * mass` | 位置误差/速度误差比值的 clamp 范围。防止微小位置误差下增益爆炸。可改为 `omega_d * mass * N` 放松限制 |
| `h`（前向积分步长） | `lpc_calculate()` 函数 | 0.1 | `goal_x2 = x2 + h * (A*x2 + B*u2)` 中的步长。影响输出速度的幅值 |
| `alpha_lpf` | `lpc_calculate()` 函数 | 0.3 | 低通滤波系数（当前在返回原始值时未启用） |

### 代码中的 clamp 说明

控制律 `u = c^(1+nu) * k_lin * expm(Gd * (1-log(c))) * e` 中，`c` 来自 `hnorm(e, Gd, P)` 的二分法结果，并被 clamp 到 `[0.5, 1.0]`。

- **c = 1.0**：无放大，`expm(Gd * 1.0)` 接近恒等。对应系统已接近编队点
- **c = 0.5**：中等放大，`expm(Gd * 1.693)` 对误差产生数倍放大。对应需要较强纠正
- **c = 0.1**：Python 原版下界，`expm(Gd * 3.303)` 产生数十倍放大。Gazebo 完美数据下只在严重偏离时触发，AMCL/slam_toolbox 噪声下会频繁触发导致振荡

### 控制器增益计算流程

```
e = x2 - x1 - d                          // 误差向量 [ex, ey, evx, evy]
↓
val_a = -mass * evx / ex          (位置误差非零时)
val_b = -mass * evy / ey
↓
val_a = clamp(val_a, -max_ratio, max_ratio)    // max_ratio = omega_d * mass
val_b = clamp(val_b, -max_ratio, max_ratio)
↓
a = max(val_a, omega_d * mass)
b = max(val_b, omega_d * mass)
↓
K = [[-a^2/m, 0, -2a, 0],           // k1_00, k2_00 作用于 X 轴
     [0, -b^2/m, 0, -2b]]           // k1_11, k2_11 作用于 Y 轴
↓
u = c^(1+nu) * K * expm(Gd * (1-log(c))) * e
↓
goal_x2 = x2 + h * (A * x2 + B * u)
cmd_vel = [goal_x2[2], goal_x2[3]]           // 期望速度
```

### 典型调参建议

| 场景 | 推荐参数 |
|------|---------|
| slam_toolbox 定位（推荐） | `mass:=8.0 Kp_yaw:=4.0 K_ff:=1.0` |
| 响应太慢（跟不上领航者） | 增大 `mass`（如 `mass:=15.0`） |
| follower 小幅抖动 | 增大 c clamp 下界（代码改为 0.6~0.7），或降低 `Kp_yaw` |
| 偏航跟不上 | 增大 `Kp_yaw` 或 `K_ff` |
| 编队距离不对 | 调 `radius` |
| 编队点跳动太频繁 | 增大 `tol` 到 0.2~0.3 |
| 实机测试 | 建议先降 `mass` 到 2.0~4.0，降 `Kp_yaw` 到 1.0 |

## 依赖

- `rclcpp`、`geometry_msgs`、`nav_msgs`
- `tf2_ros`、`tf2_geometry_msgs`
- `Eigen 3.4`（`libeigen3-dev`）
- `eigen3_cmake_module`

## 编译与启动

### 编译

```bash
colcon build --packages-select homo_multirobot_formation_control --symlink-install
```

### 启动（单 follower）

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower.launch.py
```

带参数：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower.launch.py \
  mass:=2.0 radius:=2.0 Kp_yaw:=4.0 K_ff:=1.0
```

### 启动（双 follower）

```bash
ros2 launch homo_multirobot_formation_control formation_two_followers.launch.py
```

## 完整联调

```bash
# 1. Gazebo 双机仿真 + 里程计链路 (rf2o + EKF)
ros2 launch homo_multirobot_localization sim_rf2o_ekf_two_robots.launch.py world_name:=sim_room1.world

# 2. 地图 + slam_toolbox 定位
ros2 launch homo_multirobot_nav slam_toolbox_loc_two_robots.launch.py \
  robot1_map_start_x:=0.0 robot1_map_start_y:=0.0 robot1_map_start_yaw:=0.0 \
  robot2_map_start_x:=2.0 robot2_map_start_y:=0.0 robot2_map_start_yaw:=0.0

# 3. 编队控制
ros2 launch homo_multirobot_formation_control formation_single_follower.launch.py

# 4. 领航者轨迹（正弦参考）
ros2 run homo_multirobot_formation_control leader_control.py --ros-args -r __ns:=/robot1
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
