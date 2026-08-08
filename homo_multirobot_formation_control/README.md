# homo_multirobot_formation_control

基于**齐次控制（Homogeneous Control）** 的 Leader-Follower 编队算法（C++ / Eigen），
适配项目的 slam_toolbox / AMCL + EKF 定位体系。

提供十套控制器：**4D 质点模型**（原版论文算法）、**4D Artstein-预测补偿**、**4D Cont 连续边界投影**、
**6D 运动学模型**（考虑车身朝向 + 全向轮约束 + 边界投影编队）、
**6D Disc 离散多边形编队**（6D 模型 + 离散多边形策略）、
**6D Artstein Disc**（map 系平移预测 + yaw 预测 + 6D Disc HPC 核心）、
**6D Bearing 方位角约束编队**（6D 模型 + 固定方位角，无切换平滑弧线）、
**6D Motor 电机感知模型**（执行器一阶滞后显式增广，面向实物大延迟场景）、
**6D+OA 运动学 + 避障模型**（在 6D 基础上集成 QP 避障融合）、
以及 **MPC 6D 运动学模型预测控制**（顺序线性化 + OSQP 求解，作为 HPC 的对照组）。

## 目录

- [控制器版本](#控制器版本)
- [算法原理 (4D)](#算法原理-4d)
- [算法原理 (4D Artstein)](#算法原理-4d-artstein)
- [算法原理 (6D Artstein Disc)](#算法原理-6d-artstein-disc)
- [算法原理 (6D)](#算法原理-6d)
- [算法原理 (6D Motor)](#算法原理-6d-motor)
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
| **4D Artstein (预测补偿)** | `formation_single_follower_4d_artstein.launch.py` | `formation_control_node_4d_artstein` | 双积分器 `[p_x,p_y,v_x,v_y]` (map 系)，输入前做延迟/电机预测映射 | 离散多边形 + tol 切换 | 独立 P+前馈 |
| **4D Cont (连续边界投影)** | `formation_single_follower_4d_cont.launch.py` | `formation_control_node_4d_cont` | 同 4D | 连续边界投影（无 tol/m_p） | 独立 P+前馈 |
| **6D (运动学, 边界投影)** | `formation_single_follower_6d.launch.py` | `formation_control_node_6d` | 混合系 `[p_x,p_y,θ,v_x^b,v_y^b,ω]` | 连续边界投影 | 集成于 6D 主回路 |
| **6D Disc (运动学, 离散多边形)** | `formation_single_follower_6d_disc.launch.py` | `formation_control_node_6d_disc` | 同 6D | 离散多边形 + tol 切换 | 集成于 6D 主回路 |
| **6D Artstein Disc (预测补偿)** | `formation_single_follower_6d_artstein_disc.launch.py` | `formation_control_node_6d_artstein_disc` | 同 6D，进入 HPC 前做平移/yaw 预测 | 离散多边形 + tol 切换 | 2D Artstein 预测后集成于 6D 主回路 |
| **6D Bearing (运动学, 方位角约束)** | `formation_single_follower_6d_bearing.launch.py` | `formation_control_node_6d_bearing` | 同 6D | 固定方位角 $\phi_d$，无切换 | 集成于 6D 主回路 |
| **6D Motor (电机感知模型)** | `formation_single_follower_6d_motor.launch.py` | `formation_control_node_6d_motor` | `[p_x,p_y,v_x^c,v_y^c,v_x^r,v_y^r]` (map 系, cmd/real 拆分) | 离散多边形 + tol 切换 | 独立 P+前馈 |
| **6D+OA (运动学+避障)** | `formation_single_follower_6d_oa.launch.py` | `formation_control_node_6d_oa` | 同 6D | 同 6D | 同 6D |
| **MPC 6D (模型预测控制)** | `formation_single_follower_mpc_6d.launch.py` | `formation_control_node_mpc_6d` | 同 6D | 固定偏移（Leader 车体系） | 集成于 6D 主回路 |

**MPC 6D** 是基于顺序线性化 + OSQP 求解的模型预测编队控制器，作为 HPC 齐次控制的对照组。
采用单点局部线性化（整个预测时域用同一组 $A_d, B_d, C_d$），QP 规模 366 变量，求解时间 ~1-5ms。
当前默认使用 Leader 车体系固定偏移编队（位置/速度参考一致），边界投影模式需多轮 SQP 迭代，待后续完善。

6D+OA 在 6D 基础上新增基于单线激光雷达的避障功能：通过 `/scan` 话题感知障碍物，
将障碍物距离约束以软约束形式融入 QP 优化框架，求解最优速度指令平衡编队跟踪与避障。
**适用于圆柱体等光滑曲面障碍物，不支持正方体等多面体。**

**6D Motor** 面向实物大延迟场景（实测电机 T_90% ≈ 1.24s）：把 STM32 执行器一阶滞后
显式增广进状态方程 `[p, v_cmd, v_real]`（每轴三阶链），使 HPC 天然感知
"指令速度 ≠ 实际速度"，消除过度补偿震荡。v_cmd 为控制器内部积分状态
（发布后用最终 cmd_vel 回写，抗饱和），v_real 来自 EKF。编队点策略与 4D 相同
（离散多边形 + tol），可直接与 4D baseline 对比。关键参数 `tau`（默认 0.5，
实测 ~0.43，须 ≥ 0.1）。详见 `doc/motor_homogeneous_control_full.md`（正式设计文档）和 `doc/6d_motor_model_design.md`（原始方案草稿）。自适应 τ（tau_min/tau_max/v_tau_trans）匹配实物变加速度特性；Smith 预估器（smith_Td=0.22）补偿 ~220ms 死区。后续 8D Pade 全链路模型见 `doc/pade_deadtime_full.md`。

**4D Artstein** 是原始 4D 双积分 HPC 的输入延迟与电机响应预测补偿版本。上层 HPC 核心仍保持原始 4D 双积分器结构和 `A_h^2=0` 幂零性质；`Td` 只表示纯输入/传输延迟补偿参数，由 Artstein 输入时延补偿处理；`tau` 表示一阶电机响应预测参数，用于把测得的 Follower 状态向前预测到等效双积分状态。详见 `doc/4d_artstein_prediction_theory.md`；数值仿真说明见 `doc/4d_artstein_prediction_simulation.md`；早期 Artstein 约简草稿见 `doc/artstein_reduction.md`。

**6D Artstein Disc** 是方向 A 的 6D 延迟补偿实现。它不把 `R(theta)` 引入一个全局常值 6x3 Artstein kernel，
而是在进入 6D Disc HPC 前分别做：
- map 系平移通道 4D Artstein 预测；
- yaw 通道 2D Artstein 预测；
- 预测后的 map 速度再按预测 yaw 转回 body 系，组成 `[p_x,p_y,theta,v_x^b,v_y^b,omega]`。

这样 6D Disc 的车体级 HPC 核心、离散编队点和全向轮约束保持不变，理论结论只按 nominal/local 预测补偿表述。
详见 `doc/6d_artstein_disc_theory.md`，数值仿真脚本为 `scripts/sim_6d_disc_artstein_compare.py`。

约束和齐次控制基础模块：
三套控制器共享以下模块（不修改原 4D/6D 代码）：
- `kinematic_constraint.hpp` — 全向轮轮速/加速度约束
- `types_nd.hpp`, `hnorm_nd.hpp`, `lpc2hpc_nd.hpp` — N-D 泛化齐次控制工具库

论文写作与课题规划参考：
- `doc/thesis_structure_recommendation.md` — 硕士论文推荐章节结构、参考文献、创新性定位、算法对比矩阵和风险表述。

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

## 算法原理 (4D Artstein)

4D Artstein-预测补偿控制器的数据流为：

```text
measured follower [p, v_real]
    -> Artstein input-delay compensation (Td)
    -> first-order motor forward prediction (tau)
    -> x_h=[p_pred, v_pred]
    -> original 4D double-integrator HPC
    -> cmd_vel
```

该版本不把 `-1/tau` 电机极点增广进 HPC 系统矩阵，而是在进入 HPC 前完成状态预测映射。因此 HPC 仍直接使用原始 4D 双积分模型、`A_h^2=0` 幂零结构和 4D 齐次权重。`Td` 是纯输入/传输延迟补偿参数；`tau` 是一阶电机响应预测参数。做严格消融实验时，若要测试“不使用延迟预测器”，不能只设置 `tau:=0.0 Td:=0.0`，还必须让仿真注入参数同步关断，例如 `transport_delay:=0.0`，必要时 `motor_tau` 也要对应设置。

## 算法原理 (6D Artstein Disc)

6D Artstein Disc 的数据流为：

```text
measured follower [p_map, theta, v_body, omega]
    -> v_body 转 map 系
    -> map-frame 4D Artstein 平移预测 + 2D Artstein yaw 预测
    -> 预测速度按预测 yaw 转回 body 系
    -> predicted 6D Disc state [p_x,p_y,theta,v_x^b,v_y^b,omega]
    -> 6D Disc HPC core
    -> 全向轮/速度/加速度约束
    -> cmd_vel
```

Leader 侧按当前 body twist 做常 twist 外推到 `Td + tau`，用于和预测后的 follower 状态对齐。
固定航向 leader 绕圆时，Disc 编队偏移在 map 系近似为常值偏移，因此 follower 轨迹应接近 leader 圆轨迹的平移版本，便于和 4D Artstein 对比。

当前实现中，`initial_min_lambda` 与 `switch_min_lambda` 的含义已和 4D Artstein 对齐：
它们直接表示闭环极点尺度下界，而不是 `mass * min_lambda`。HPC 模式下 `K_lin`
只在初始化、编队点切换、或 leader twist/相对 yaw 触发 HPC 重建时同步更新，避免
`K_lin` 与 `G0/P/Gd` 使用不同线性化条件。`use_motor_delay:=true` 时，
延迟注入节点以 100 Hz 运行，和 4D Artstein 的延迟仿真链路保持一致。

需要注意：Artstein 预测只能补偿模型内的输入延迟和一阶执行器滞后，不能突破速度、轮速和加速度饱和。
Gazebo 或实物中如果 `max_linear_vel/max_angular_vel/max_linear_accel/max_angular_accel` 设得过低，轨迹误差会主要由物理约束决定。
`delay_max_accel` 只作用于 `sim_motor_delay.py` 延迟注入节点，不是控制器侧加速度上限；控制器侧上限必须单独设置 `max_linear_accel/max_angular_accel`。
当前控制器和延迟注入节点的限幅均为分量限幅，不是二维速度/加速度模长限幅。
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

## 算法原理 (6D Motor)

详细的数学推导见 `doc/motor_homogeneous_control_full.md`。核心要点：

1. **6D 电机感知状态**：$[p_x, p_y, v_x^{\mathrm{cmd}}, v_y^{\mathrm{cmd}}, v_x^{\mathrm{real}}, v_y^{\mathrm{real}}]^{\mathsf{T}}$（map 系），
   将执行器一阶滞后 $\dot{v}^{\mathrm{real}} = (v^{\mathrm{cmd}} - v^{\mathrm{real}})/\tau$ 显式增广进系统矩阵，
   使 HPC 天然感知 "指令速度 ≠ 实际速度"
2. **三阶自适应极点配置**：每轴从 4D 的二阶链 $[p, v]$ 变为三阶链 $[p, v^{\mathrm{cmd}}, v^{\mathrm{real}}]$，
   对 $(s+\lambda)^3$ 三重极点配置给出解析解（定理 1），$\lambda = a/m$ 与 4D 自适应逻辑兼容
3. **v_cmd 内部积分**：初始化对齐 EKF，每周期 $v^{\mathrm{cmd}} \gets v^{\mathrm{cmd}} + h \cdot \mathbf{u}/m$，
   发布后以最终 cmd_vel 回写（抗饱和）；Leader 取 $v^{\mathrm{cmd}} = v^{\mathrm{real}}$（稳态假设）
4. **齐次链深度与翘曲放大**：6D 三阶链权重 $[2,1,0]$（4D 为 $[1,0]$），
   近目标区域翘曲放大 ~30×（vs 4D 的 ~5×），必须提高 $c_{\min}$ 至 0.9 抑制弛豫振荡
5. **A 常值**：无需 6D 运动学模型的时变重算，HPC 仅在编队点切换时更新
6. **偏航独立**：P+前馈独立回路（与 4D 一致），偏航通道电机延迟通常显著小于线速度

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

### 启动（4D Artstein 预测补偿）

Gazebo 双车仿真，控制器预测参数与仿真注入延迟对齐：

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

只启动 Follower，Leader 使用 `/virtual_leader`，实物或外部延迟由底盘自身体现时关闭仿真注入：

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

### 启动（6D Motor 单 follower，电机感知模型）

```bash
# 默认参数（经仿真实物联合标定，对齐 0.25 m/s² 实物加速度）
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_motor.launch.py \
  leader_ns:=/robot1 follower_ns:=/robot2 use_motor_delay:=true
```

带参数：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_motor.launch.py \
  leader_ns:=/robot1 follower_ns:=/robot2 \
  use_motor_delay:=true motor_tau:=0.43 tau:=0.43 \
  mass:=2.0 omega_d:=0.7 hpc_c_min:=0.9 max_linear_accel:=0.25
```

关键参数：

| 参数 | 默认 | 含义 |
|------|------|------|
| `tau` | 0.43 | 电机时间常数（模型），越小响应越快 |
| `mass` | 2.0 | 控制力→加速度增益（6D Motor 专用，4D 用 8.0） |
| `omega_d` | 0.7 | 闭环带宽（须 ≤ 物理可达值，0.25 accel 时上限 ~1.0） |
| `hpc_c_min` | 0.9 | HPC warp clamp 下界（6D 三阶链须高于 4D 的 0.5） |
| `max_linear_accel` | 0.25 | 控制器侧加速度约束（对齐实物） |
| `motor_tau` | 0.43 | 仿真注入延迟的时间常数（实物不启 `use_motor_delay`） |
| `transport_delay` | 0.0 | 纯传输延迟 (s)，0=关断（v1 不建模死区） |
| `delay_max_accel` | 0.25 | 仿真注入延迟的加速度限幅 |
| `leader_vel_lpf_tau` | 0.0 | leader 速度低通 (s)，0=关断，噪声大时设 0.2–0.3 |

LPC 消融对照（关闭齐次升级）：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_motor.launch.py \
  leader_ns:=/robot1 follower_ns:=/robot2 use_motor_delay:=true use_hpc:=false
```

轨迹记录（需指定 `controller_node_name` 以读取 6D Motor 专属参数）：

```bash
ros2 run homo_multirobot_formation_control record_trajectory.py \
  --ros-args -p mode:=sim -p duration:=45.0 \
  -p controller_node_name:=formation_control_node_6d_motor
```

与 4D baseline 的区别：状态扩展为 $[p, v^{\mathrm{cmd}}, v^{\mathrm{real}}]$，执行器
滞后显式写入系统矩阵，v_cmd 是跨周期积分状态；偏航控制独立（P+前馈，与 4D 一致）。

### 启动（6D+OA 单 follower，带避障）

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_oa.launch.py
```

带参数：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_oa.launch.py \
  safety_distance:=0.6 radius:=1.0 obstacle_weight:=1.5
```

### 启动（6D Disc 单 follower，离散多边形）

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_disc.launch.py
```

带参数：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_disc.launch.py \
  radius:=1.0 m_p:=4 tol:=0.1 \
  mass:=1.5 I:=0.3 omega_d:=0.8 omega_d_theta:=0.8
```

与 6D 连续边界投影版本的区别：`m_p` 个编队点均匀分布在安全圆上，`tol` 提供切换迟滞，避免边界投影在小半径圆轨迹下的震荡。

### 启动（6D Artstein Disc 单 follower，延迟预测补偿）

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_artstein_disc.launch.py
```

Gazebo 双车仿真，开启延迟注入并让预测参数与注入参数对齐：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_artstein_disc.launch.py \
  leader_ns:=/robot1 follower_ns:=/robot2 \
  use_motor_delay:=true \
  motor_tau:=0.43 transport_delay:=0.22 delay_max_accel:=0.3 \
  radius:=2.5 mass:=2.0 I:=1.0 \
  m_p:=4 tol:=0.1 \
  tau:=0.43 tau_yaw:=0.43 Td:=0.22 \
  control_rate:=20.0 hpc_c_min:=0.7 \
  initial_min_lambda:=1.0 switch_min_lambda:=2.5 \
  max_linear_vel:=0.45 max_angular_vel:=0.3 \
  max_linear_accel:=0.3 max_angular_accel:=0.5
```

LPC 消融对照（关闭齐次升级，仍保留 Artstein 预测层）：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_artstein_disc.launch.py \
  leader_ns:=/robot1 follower_ns:=/robot2 use_motor_delay:=true use_hpc:=false
```

关键参数：

| 参数 | 默认 | 含义 |
|------|------|------|
| `tau` | 0.43 | 平移通道一阶执行器预测时间常数 |
| `tau_yaw` | 0.43 | yaw 通道一阶执行器预测时间常数 |
| `Td` | 0.22 | 纯输入/传输延迟补偿时间 |
| `hpc_c_min` | 0.5 | HPC 齐次范数下界，调大可减弱近目标 warp 放大 |
| `initial_min_lambda` | 1.0 | 初始目标点的最小线性闭环带宽 |
| `switch_min_lambda` | 4.0 | 编队点切换后的临时最小闭环带宽 |
| `hpc_vel_threshold` | 0.3 | leader twist 变化超过该阈值才重算 HPC |
| `hpc_yaw_threshold` | 0.3 | leader/follower 相对 yaw 变化超过该阈值才重算 HPC |
| `stability_margin` | 0.01 | `A+B*K` Hurwitz 检查裕度，不满足则复用上一组稳定 HPC 或退回线性控制 |
| `max_linear_accel` | 2.0 | 控制器侧 body x/y 分量加速度约束 |
| `max_angular_accel` | 4.0 | 控制器侧 yaw 加速度约束 |
| `delay_max_accel` | 2.0 | 仿真延迟节点侧分量加速度约束，仅 `use_motor_delay:=true` 时生效 |

调参时先让 `max_linear_vel/max_angular_vel` 满足 leader 速度和目标圆半径所需的稳态速度，再用
`max_linear_accel/max_angular_accel` 对齐实物能力；`delay_max_accel` 只用于模拟底盘响应，不会限制控制器内部命令。
若实物加速度约为 0.25-0.30 m/s²，leader 速度 0.5 m/s 会明显受约束影响，建议优先增大轨迹半径、降低 yaw 角速度需求，或降低闭环带宽。
控制器日志中的 `|v_raw|/|v_clamped|/|v_final|` 分别表示 6D Artstein 核心输出、速度分量限幅后、
以及轮速/加速度约束后的线速度模长；`YAW_DIAG` 用于判断 yaw 通道是否正在主导轮速缩放或角速度饱和。

### 启动（6D Bearing 单 follower，方位角约束）

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_bearing.launch.py
```

带参数：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_bearing.launch.py \
  radius:=2.0 phi_d:=3.1416 \
  mass:=1.5 I:=0.3 omega_d:=0.8 omega_d_theta:=0.8

# LPC 消融对照（关闭齐次升级）
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_bearing.launch.py \
  use_hpc:=false

# 右后方 45° 编队
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_bearing.launch.py \
  phi_d:=-0.785
```

**Bearing 特有的编队参数**：

| 参数 | 类型 | 默认值 | 作用 |
|------|------|--------|------|
| `phi_d` | double | 3.1416 ($\pi$) | Leader 车体系下目标方位角 (rad)。0=正前方，$\pi$=正后方，$\pi/2$=左侧，$-\pi/2$=右侧 |

**与 6D 离散编队点 (Disc) 的区别**：编队偏移固定于方位角 $\phi_d$ 处，无需 `m_p`/`tol` 参数和切换逻辑。Cartesian 位置误差同时编码径向距离误差和切向方位角误差——径向将 Follower 推/拉到安全圆，切向驱动沿圆弧滑向目标方位，轨迹为连续平滑弧线。

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

### virtual_leader_circle — 虚拟 Leader 绕圈

不依赖仿真/实车，直接发布 Odometry + 静态 TF，虚拟一个在 map 系绕圈运动的 leader：

```bash
ros2 run homo_multirobot_formation_control virtual_leader_circle.py \
  --ros-args -r __ns:=/virtual_leader

# 带参数
ros2 run homo_multirobot_formation_control virtual_leader_circle.py \
  --ros-args -r __ns:=/virtual_leader \
  -p center_x:=0.0 -p center_y:=0.0 -p radius:=2.0 -p speed:=0.5 -p direction:=ccw
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `center_x` | 0.0 | 圆心 X (map 系) |
| `center_y` | 0.0 | 圆心 Y (map 系) |
| `radius` | 2.0 | 圆半径 (m) |
| `speed` | 0.5 | 切向线速度 (m/s) |
| `direction` | ccw | ccw=逆时针, cw=顺时针 |
| `rate` | 50.0 | 发布频率 (Hz) |

与 `leader_circle.py`（开环 cmd_vel，依赖 Gazebo 提供里程计）不同，
`virtual_leader_circle.py` 直接发布 `<ns>/odometry/filtered` 和 `map → <prefix>_odom` 静态 TF，
**完全取代 leader 仿真/实车**，编队控制器通过 `leader_ns:=/virtual_leader` 即可对接。

### record_trajectory — 轨迹记录与画图

```bash
# 仿真（自动读控制器参数生成标签，无需手动指定 tag）
ros2 run homo_multirobot_formation_control record_trajectory.py \
  --ros-args -p mode:=sim -p duration:=30.0

# 实物 + 自定义标签
ros2 run homo_multirobot_formation_control record_trajectory.py \
  --ros-args -p mode:=real -p tag:=hpc_mass8_r2 \
  -p leader_ns:=/virtual_leader -p follower_ns:=/robot2 \
  -p radius:=2.0 -p duration:=30.0
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `leader_ns` | /robot1 | Leader 命名空间 |
| `follower_ns` | /robot2 | Follower 命名空间 |
| `duration` | 30.0 | 记录时长 (s) |
| `mode` | sim | `sim` 或 `real`，决定输出子目录 |
| `tag` | 自动生成 | 文件名标签，留空则从控制器参数自动生成 |
| `radius` | 0.0 | 编队理想半径，>0 时在距离图上画参考虚线 |
| `out_dir` | 包内 robot_traj/ | 输出根目录 |

**输出**：
- `{out_dir}/{mode}/{mode}_{tag}_{timestamp}.png` — 四子图（轨迹、X时序、Y时序、编队距离）
- `{out_dir}/{mode}/{mode}_{tag}_{timestamp}.csv` — MATLAB 可直接 readtable 的原始数据

**自动参数读取**：如果不指定 `tag`，脚本从 follower 命名空间下的 `formation_control_node`
自动读取 `mass, radius, omega_d, control_rate, m_p, Kp_yaw, K_ff, tol` 并：
- 生成文件名标签（如 `sim_m8_r2_od1.5_f35_20260710_153000.png`）
- 在图上方黄框中显示完整参数组合

CSV 格式（`time_s, leader_x_m, leader_y_m, follower_x_m, follower_y_m, distance_m`），
以 follower 时间为基准对齐 leader 数据点。

### record_velocity_diagnostics — 速度诊断记录

记录控制器原始速度指令、最终速度指令和 Leader/Follower EKF 速度，用于分析预测补偿前后的限幅、延迟和实际速度响应。

```bash
ros2 run homo_multirobot_formation_control record_velocity_diagnostics.py \
  --ros-args \
  -p leader_ns:=/robot1 \
  -p follower_ns:=/robot2 \
  -p mode:=sim \
  -p duration:=60.0 \
  -p tag:=vel_diag_artstein_pred
```

输出图中：
- `cmd_vel_raw |V|` = 控制器原始速度指令模长
- `cmd_vel |V|` = 约束/限幅后的最终速度指令模长
- `follower odom |V|` = Follower EKF 实际速度模长
- `leader odom |V|` = Leader EKF 速度模长

## 诊断工具脚本

### measure_motor_latency — 电机响应延迟测试

测量 cmd_vel 发出到轮子实际转动的时间差（实物用）：

```bash
# 实物：对比 raw odom vs EKF 延迟
python3 measure_motor_latency.py --ns /robot2 --raw-odom-topic /odom --trials 10

# 仿真：只测 EKF 链路
python3 measure_motor_latency.py --ns /robot1 --trials 5
```

每轮测试：静止检测 → 阶跃 cmd_vel → 检测 odom 速度越过阈值 → 倒车复位。
同时测量 `/odom`（串口直出）和 `/odometry/filtered`（EKF 滤波）两路延迟，
差值即为 EKF 滤波开销。

### measure_cross_machine_delay — 跨机器话题延迟

测量 WiFi 环境下 ROS 2 话题的端到端延迟（在 Follower 车上运行）：

```bash
python3 measure_cross_machine_delay.py --topic /robot1/odometry/filtered --duration 60 --csv /tmp/delay.csv
```

输出 avg/P50/P95/P99 延迟统计，CSV 可导出 MATLAB 画图。

> 依赖两台机器时钟同步（chrony），否则 header.stamp 与 receive time 不在同一时间基准。

### 系统延迟链路图

```
实物: 编码器 → STM32(20Hz) → 串口 → /odom(20Hz) → EKF(实际20Hz) → /odometry/filtered → 控制器(20Hz)
                                                                         ↑ avg_ekf_age
                                                                                        → cmd_vel → 串口 → STM32 → 电机
                                                                                                        ↑ motor_latency
Leader: .../odometry/filtered → DDS → WiFi → follower 回调 → timer 取用
                                                    ↑ avg_leader_age
```

| 延迟段 | 测量工具 | 仿真典型值 | 实物待测 |
|--------|---------|-----------|---------|
| 网络（Leader→Follower） | `ros2 topic delay` + DIAGavg_leader_age | ~7ms | 待测 |
| 电机响应（cmd_vel→轮转） | `measure_motor_latency.py` | ~80-100ms(rf2o) | 待测 |
| EKF 滤波 | raw - ekf 差值 | ~14ms | 待测 |
| 数据源频率上限 | `ros2 topic hz /odom` | ~10Hz(rf2o) | 20Hz(STM32固件) |

> **实物 /odom = 20Hz 来自 STM32 固件**，`wheeltec_robot` 驱动无频率设置。
> EKF 与控制频率不应超过此硬件上限，否则纯预测无测量更新。

## 完整联调

### 标准双机联调（robot1=Leader 仿真, robot2=Follower 仿真）

```bash
# 1. Gazebo 双机仿真 + 里程计链路 (rf2o + EKF)
ros2 launch homo_multirobot_localization sim_rf2o_ekf_two_robots.launch.py \
  use_rviz:=false

# 2. 地图 + slam_toolbox 定位
ros2 launch homo_multirobot_nav slam_toolbox_loc_two_robots.launch.py \
  robot1_map_start_x:=0.0 robot1_map_start_y:=0.0 robot1_map_start_yaw:=0.0 \
  robot2_map_start_x:=2.0 robot2_map_start_y:=0.0 robot2_map_start_yaw:=0.0

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

# 3e. 编队控制 — 6D Artstein Disc 版（延迟预测补偿）
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_artstein_disc.launch.py \
  leader_ns:=/robot1 follower_ns:=/robot2 \
  use_motor_delay:=true tau:=0.43 tau_yaw:=0.43 Td:=0.22

# 4. 键盘遥控领航者
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel
```

### 虚拟 Leader 联调（只跑 Follower 仿真/实车，Leader 由程序虚拟）

```bash
# ===== 终端 1：只启动 Follower 的 Gazebo 仿真 + 定位链路 =====
ros2 launch homo_multirobot_localization sim_rf2o_ekf_single_robot.launch.py \
  robot_namespace:=/robot2 robot_prefix:=robot2_ \
  robot_x:=2.0 robot_y:=0.0 robot_yaw:=0.0

# ===== 终端 2：Follower 已知地图定位 =====
ros2 launch homo_multirobot_nav slam_toolbox_loc_single_robot.launch.py \
  namespace:=/robot2 prefix:=robot2_ \
  map_name:=sim_room1_map \
  map_start_x:=2.0 map_start_y:=0.0 map_start_yaw:=0.0

# ===== 终端 3：虚拟 Leader 绕圈 =====
ros2 run homo_multirobot_formation_control virtual_leader_circle.py \
  --ros-args -r __ns:=/virtual_leader \
  -p center_x:=0.0 -p center_y:=0.0 -p radius:=2.0 -p speed:=0.5

# ===== 终端 4：编队控制（robot2 跟随 virtual_leader）=====
ros2 launch homo_multirobot_formation_control formation_single_follower_6d.launch.py \
  leader_ns:=/virtual_leader follower_ns:=/robot2
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
