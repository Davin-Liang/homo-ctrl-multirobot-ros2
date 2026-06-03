# homo_multirobot_formation_control

基于**齐次控制（Homogeneous Control）** 的 Leader-Follower 编队算法（C++ / Eigen），
适配项目的 slam_toolbox / AMCL + EKF 定位体系。

提供四套控制器：**4D 质点模型**（原版论文算法）、**6D 运动学模型**（考虑车身朝向 + 全向轮约束）、
**6D+OA 运动学 + 避障模型**（在 6D 基础上集成 QP 避障融合）、
以及 **MPC 6D 运动学模型预测控制**（顺序线性化 + OSQP 求解，作为 HPC 的对照组）。

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
| **4D Cont (连续边界投影)** | `formation_single_follower_4d_cont.launch.py` | `formation_control_node_4d_cont` | 同 4D | 连续边界投影（无 tol/m_p） | 独立 P+前馈 |
| **6D (运动学)** | `formation_single_follower_6d.launch.py` | `formation_control_node_6d` | 混合系 `[p_x,p_y,θ,v_x^b,v_y^b,ω]` | 连续边界投影 | 集成于 6D 主回路 |
| **6D+OA (运动学+避障)** | `formation_single_follower_6d_oa.launch.py` | `formation_control_node_6d_oa` | 同 6D | 同 6D | 同 6D |
| **MPC 6D (模型预测控制)** | `formation_single_follower_mpc_6d.launch.py` | `formation_control_node_mpc_6d` | 同 6D | 固定偏移（Leader 车体系） | 集成于 6D 主回路 |

**MPC 6D** 是基于顺序线性化 + OSQP 求解的模型预测编队控制器，作为 HPC 齐次控制的对照组。
采用单点局部线性化（整个预测时域用同一组 $A_d, B_d, C_d$），QP 规模 366 变量，求解时间 ~1-5ms。
当前默认使用 Leader 车体系固定偏移编队（位置/速度参考一致），边界投影模式需多轮 SQP 迭代，待后续完善。

6D+OA 在 6D 基础上新增基于单线激光雷达的避障功能：通过 `/scan` 话题感知障碍物，
将障碍物距离约束以软约束形式融入 QP 优化框架，求解最优速度指令平衡编队跟踪与避障。
**适用于圆柱体等光滑曲面障碍物，不支持正方体等多面体。**

三套控制器共享以下模块（不修改原 4D/6D 代码）：
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

1. **6D 混合系状态**：$[p_x, p_y, \theta, v_x^b, v_y^b, \omega]^{\mathsf{T}}$，位置/朝向在 map 系，
   速度在车体系，输出天然对应 `cmd_vel`
2. **误差在 leader 车体系下计算**：follower 速度按 $\Delta\theta$ 旋转后求差，
   控制力再旋转回 follower 车体系做前向欧拉积分
3. **边界投影编队**：$d = r_s \cdot (\mathbf{p}_f - \mathbf{p}_l)/\|\mathbf{p}_f - \mathbf{p}_l\|$，
   连续光滑，无离散切换
4. **时变 $A_l$ 矩阵**：含 leader 速度耦合项 $(\omega_l, v_{x,l}^b, v_{y,l}^b)$，
   每周期更新；HPC 参数在 leader 速度或 $\Delta\theta$ 变化超过阈值时重算
5. **yaw 控制集成**：$\theta/\omega$ 作为 3×6 增益矩阵的第三通道，临界阻尼双极点设计

## 算法原理 (6D+OA)

6D+OA 复用了 6D 的 HPC 核心算法（`homo_controller_6d.hpp` 等），在 HPC 期望速度输出后插入避障融合模块。
架构如下：

```
HPC 期望力 → 坐标系旋转 → 前向欧拉积分 → 候选速度 v_hpc
                                                 ↓
/scan → 点云滤波 → 欧几里得聚类 → 障碍物列表 → QP 融合求解 → 运动学约束 → cmd_vel
```

### 激光处理

1. 滤除无效点（inf/nan/超量程），转为 2D 笛卡尔坐标（车身系）
2. 欧几里得聚类：相邻点距离 ≤ `cluster_tolerance` 的归为一簇
3. 每个簇取**最近点**（离机器人最近的点）作为障碍物位置，半径上限 0.5m
4. 多帧最近邻匹配 + 低通滤波，跟踪障碍物 ID 并估计速度

### QP 优化问题

决策变量 $v = [v_x, v_y, \omega] \in \mathbb{R}^3$（车体系速度指令）：

$$\min_v \quad \|v - v_{\text{hpc}}\|^2 + \sum_i w_i \cdot \phi_{\text{smooth}}(v \cdot n_i - v_{\text{safe},i})^2$$

$$\text{s.t.} \quad v_{\min} \le v \le v_{\max}, \quad |v - v_{\text{prev}}| \le a_{\max} \cdot dt$$

其中 $\phi_{\text{smooth}}(x) = \frac{1}{2}(x + \sqrt{x^2 + \varepsilon^2})$ 为光滑 max(0,x) 近似，
$n_i$ 为机器人指向障碍物表面的单位向量，
$w_i$ 为近距离双曲线增长（上限 8x）的障碍物有效权重。

安全速度 $v_{\text{safe},i}$：
- 障碍物在安全距离外：$v_{\text{safe}} = \max(0, \text{clearance}/T)$，限制靠近速度
- 进入安全距离内：$v_{\text{safe}} < 0$（负值），要求机器人主动后退

求解方法：投影梯度下降（Eigen，无外部 QP 求解器依赖），Armijo 回溯线搜索。

### 避障参数（launch 可改）

| 参数 | 类型 | 默认值 | 作用 |
|------|------|--------|------|
| `scan_topic` | string | `scan` | 激光雷达话题（相对 follower 命名空间） |
| `safety_distance` | double | 0.5 | 安全距离阈值 (m)，进入该范围触发后退 |
| `obstacle_weight` | double | 1.0 | 避障代价权重，越大越保守 |
| `time_horizon` | double | 0.5 | 碰撞预测时域 (s) |
| `max_obstacles` | int | 10 | 最大考虑障碍物数量 |
| `cluster_tolerance` | double | 0.1 | 聚类距离阈值 (m) |
| `min_cluster_size` | int | 5 | 聚类最少点数 |

### 已知局限

- **适用**：圆柱体、球体等光滑曲面障碍物
- **不适用**：正方体、长方体等多面体——最近点会在面间跳变，导致 QP 反复拉锯

## 算法原理 (MPC 6D)

MPC 6D 将编队控制描述为一个有限时域最优控制问题，每步求解 QP（二次规划）得到最优加速度。

### 模型

状态 $x = [p_x, p_y, \theta, v_x^b, v_y^b, \omega]^T$（与 6D HPC 相同），
输入 $u = [a_x^b, a_y^b, \alpha]^T$（车体系加速度）。MPC 内部不使用 mass/I，输入直接解释为加速度。

非线性模型在每个控制周期于当前跟随者状态处做**单点局部线性化**，离散化（前向 Euler）后得到仿射线性模型：

$$A_d = I + dt \cdot A_c, \quad B_d = dt \cdot B, \quad C_d = dt \cdot (f(x_0,0) - A_c \cdot x_0)$$

### QP 形式

非紧凑形式，决策变量 $z = [x_0, u_0, x_1, u_1, \dots, x_{N-1}, u_{N-1}, x_N]$，$N=40$（2.0s 时域），共 366 个变量。

$$\min \sum_{k=0}^{N-1} \left[(x_k - x_{\text{ref},k})^T Q (x_k - x_{\text{ref},k}) + u_k^T R u_k\right] + (x_N - x_{\text{ref},N})^T Q_f (x_N - x_{\text{ref},N})$$

约束包括：动力学等式、输入限幅、车体速度限幅（从 $x_3$ 开始，避免 $x_0$ 超限不可行）、轮速通过 `KinematicConstraint` 后处理。

### 求解器

使用 OSQP（Operator Splitting QP），通过 `ros-humble-osqp-vendor` 安装。每步完整 rebuild + solve，未使用 warm-start。

### Leader 预测与参考轨迹

Leader 跟踪采用恒定车体速度积分（含旋转积分公式），参考位置为 Leader 车体系固定偏移量。
参考速度使用跟随者实际朝向 $R(\theta_f)^T$ 旋转到车体系，保证侧向跟踪对称性。
参考角度使用 $(\theta_L - \theta_f)$ unwrap 连续化处理。

### 已知局限

- 单点局部线性化在大角度 / 高速运动时预测精度有限
- 固定偏移编队与 HPC 的边界投影策略不同，后续可升级为沿预测轨迹时变线性化
- 未使用 warm-start，求解器每次完整 rebuild

### MPC 参数（launch 可改）

| 参数 | 类型 | 默认值 | 作用 |
|------|------|--------|------|
| `mpc_horizon` | int | 40 | 预测时域步数 N |
| `formation_radius` | double | 2.0 | 编队圆半径（边界投影模式） |
| `formation_offset_x` | double | -2.0 | Leader 车体系 x 偏移（固定偏移模式） |
| `formation_offset_y` | double | 0.0 | Leader 车体系 y 偏移 |
| `mpc_q_px` / `mpc_q_py` | double | 5.0 | 位置跟踪权重 |
| `mpc_q_theta` | double | 20.0 | 朝向跟踪权重 |
| `mpc_q_vx` / `mpc_q_vy` | double | 0.5 | 速度阻尼权重 |
| `mpc_q_omega` | double | 2.0 | 角速度阻尼权重 |
| `mpc_r_ax` / `mpc_r_ay` | double | 0.01 | 线加速度惩罚 |
| `mpc_r_alpha` | double | 0.01 | 角加速度惩罚 |
| `mpc_terminal_factor` | double | 10.0 | 终端代价倍数 |
| `max_linear_vel` | double | 1.0 | 线速度上限 (m/s) |
| `max_angular_vel` | double | 2.0 | 角速度上限 (rad/s) |
| `max_linear_accel` | double | 2.0 | 线加速度上限 (m/s²) |
| `max_angular_accel` | double | 6.0 | 角加速度上限 (rad/s²) |

### 典型调参建议

| 场景 | 操作 |
|------|------|
| 保持力不够 | 增大 `mpc_q_px/py` 到 20.0 |
| 控制太激进 | 增大 `mpc_r_ax/ay` 到 0.1 |
| 航向转太慢 | 增大 `mpc_q_theta` |
| 侧向跟踪弱 | 增大 `mpc_q_py`，降低 `mpc_r_ay` 到 0.005 |
| 求解太慢 | 减小 `mpc_horizon` 到 20 |

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
| `omega_d` | double | 1.5 | 期望阻尼带宽，决定最小收敛速度 | 响应更快但可能震荡 | 更平滑但跟踪滞后 |
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

### 消融实验参数（两套共享）

| 参数 | 类型 | 默认值 | 作用 |
|------|------|--------|------|
| `use_hpc` | bool | true | 启用齐次升级（false 退化为纯线性比例控制 LPC，用于消融对照） |

> 论文消融实验矩阵：
> - 4D + HPC（原版 baseline）、4D + LPC（对照组）
> - 6D + HPC（本文方法）、6D + LPC（消融组）

### 控制频率（launch 可改）

| 参数 | 类型 | 默认值 | 作用 |
|------|------|--------|------|
| `control_rate` | double | 20.0 | 控制循环频率 (Hz) |

### 4D 进阶参数（代码内硬编码）

以下参数在 `homo_controller.hpp` 中（不暴露为 launch 参数）：

| 参数 | 默认值 | 作用 |
|------|--------|------|
| `c` clamp 下界 | 0.5 | 齐次范数下限（原版 Python 0.1，提升以抑制噪声放大） |
| `h`（前向积分步长） | 0.1 | 前向欧拉步长 |

> `omega_d` 已从硬编码升级为 launch 参数（默认 1.5），可在命令行直接调参。

### 典型调参建议

| 场景 | 4D 版 | 6D 版 |
|------|------|------|
| slam_toolbox 定位 | `mass:=8.0 Kp_yaw:=4.0` | `mass:=8.0 I:=1.0 radius:=2.0` |
| 响应太慢 | 增大 `mass` 或 `omega_d` | 增大 `mass` 或 `omega_d` |
| 边界震荡 | 降低 `omega_d` 或 `mass` | 降低 `omega_d` 或 `mass` |
| 偏航跟不上 | 增大 `Kp_yaw`/`K_ff` | 增大 `omega_d_theta` 或 `I` |
| 编队距离不对 | 调 `radius` | 调 `radius` |

## 依赖

- `rclcpp`、`geometry_msgs`、`nav_msgs`、`sensor_msgs`
- `tf2_ros`、`tf2_geometry_msgs`
- `Eigen 3.4`（`libeigen3-dev`）+ unsupported 模块（KroneckerProduct, MatrixFunctions）
- `eigen3_cmake_module`
- `osqp_vendor`（`ros-humble-osqp-vendor`，MPC 求解器依赖）

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

# LPC 消融对照（关闭齐次升级）
ros2 launch homo_multirobot_formation_control formation_single_follower.launch.py \
  use_hpc:=false

# 提高带宽追移动目标
ros2 launch homo_multirobot_formation_control formation_single_follower.launch.py \
  omega_d:=3.0 mass:=10.0
```

### 启动（4D Cont 连续边界投影）

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_4d_cont.launch.py
```

带参数：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_4d_cont.launch.py \
  radius:=2.0 mass:=8.0 omega_d:=1.5

# LPC 消融对照
ros2 launch homo_multirobot_formation_control formation_single_follower_4d_cont.launch.py \
  use_hpc:=false
```

### 启动（6D 单 follower）

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_6d.launch.py
```

带参数：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_6d.launch.py \
  radius:=1.0 mass:=8.0 I:=1.0 wheel_max_omega:=10.0

# LPC 消融对照（关闭齐次升级）
ros2 launch homo_multirobot_formation_control formation_single_follower_6d.launch.py \
  use_hpc:=false
```

### 启动（6D+OA 单 follower，带避障）

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_oa.launch.py
```

带参数：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_oa.launch.py \
  safety_distance:=0.6 radius:=1.0 obstacle_weight:=1.5
```

### 启动（MPC 6D 单 follower）

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_mpc_6d.launch.py
```

带参数：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_mpc_6d.launch.py \
  formation_offset_x:=-1.0 \
  mpc_q_px:=20.0 mpc_q_py:=20.0 \
  mpc_r_ax:=0.005 mpc_r_ay:=0.005 \
  max_linear_vel:=2.0
```

### 启动（双 follower，4D 版）

```bash
ros2 launch homo_multirobot_formation_control formation_two_followers.launch.py
```

## 领航者轨迹脚本

本包提供两个领航者开环控制脚本，用于编队测试：

### leader_circle — 圆轨迹

```bash
ros2 run homo_multirobot_formation_control leader_circle.py --ros-args -r __ns:=/robot1

# 带参数
ros2 run homo_multirobot_formation_control leader_circle.py --ros-args -r __ns:=/robot1 \
  -p radius:=2.0 -p speed:=0.5 -p direction:=cw
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `radius` | 1.0 | 圆半径 (m) |
| `speed` | 0.3 | 切向线速度 (m/s) |
| `heading` | 0.0 | 车体航向角 (度) |
| `direction` | ccw | ccw=逆时针, cw=顺时针 |
| `rate` | 20.0 | 发布频率 (Hz) |

### leader_eight — 8 字轨迹

```bash
ros2 run homo_multirobot_formation_control leader_eight.py --ros-args -r __ns:=/robot1

# 带参数：大 8 字 + 慢速
ros2 run homo_multirobot_formation_control leader_eight.py --ros-args -r __ns:=/robot1 \
  -p amplitude_x:=3.0 -p amplitude_y:=1.5 -p period:=15.0
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `amplitude_x` | 2.0 | X 方向半幅 (m) |
| `amplitude_y` | 1.0 | Y 方向半幅 (m) |
| `period` | 10.0 | 一个 8 字周期 (s) |
| `heading` | 0.0 | 车体航向角 (度) |
| `rate` | 20.0 | 发布频率 (Hz) |

> 两个脚本均为纯开环速度指令，无位置反馈。`period` 控制指令频率而非实际轨迹周期。
> Y 通道频率为 2ω（X 通道的 2 倍），对控制器带宽要求更高，需适当提高 `omega_d`。

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

# 3c. 编队控制 — 6D+OA 版（带避障）
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_oa.launch.py \
  safety_distance:=0.6 radius:=1.0

# 3d. 编队控制 — MPC 6D 版（对照组）
ros2 launch homo_multirobot_formation_control formation_single_follower_mpc_6d.launch.py \
  formation_offset_x:=-1.0 mpc_q_px:=20.0 mpc_q_py:=20.0

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
