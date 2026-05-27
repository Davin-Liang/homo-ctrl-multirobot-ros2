# BUG_RECORD — `homo_multirobot_formation_control` 联调记录

本文档记录将齐次控制编队算法从 Python/Gazebo 里程计移植到 C++/slam_toolbox 定位体系
过程中遇到的问题与处理方式。

---

## 总览

| # | 问题 | 分类 | 处理 |
|---|------|------|------|
| 1 | Gazebo odom ↔ slam_toolbox 定位数据不一致 → 振荡 | 架构 | EKF odom + TF 变换通道 |
| 2 | 话题相对路径导致命名空间双重解析 | ROS 2 | 统一使用绝对路径 |
| 3 | `MatrixExponential` / `MatrixSquareRoot` API 不存在 | C++ | 改用 `.exp()` / `.sqrt()` |
| 4 | `M += M.transpose()` 触发 aliasing 断言 | C++ | 临时变量求值 |
| 5 | `trans_con` 固定尺寸矩阵死循环 | C++ | 改用 `MatrixXd` |
| 6 | `HpcResult.K0` 类型 4×4 → 2×4 | C++ | 改 `Mat24d` |
| 7 | `expm` 非线性放大 AMCL 噪声 | 控制 | c clamp 0.1→0.5 |
| 8 | `val_a/val_b` 比值在噪声下爆炸 | 控制 | 比值 clamp |
| 9 | 死区导致"开-关"抽搐 | 控制 | 移除死区 |
| 10 | 速度归零导致算法退化 | 控制 | 不采用，保留完整模型 |
| 11 | 6D 控制器速度误差跨坐标系直接相减 → 发散 | 控制 | follower 速度旋转到 leader 车体系 |
| 12 | 边界投影符号错误 → 两车相撞 | 控制 | d = +radius（非 -radius）|
| 13 | `trans_con_nd` 可控性矩阵尺寸硬编码 | C++ | 改用动态 `MatrixXd(N, N*M)` |
| 14 | `sim_rf2o_ekf` launch 未转发 spawn 位姿参数 | 架构 | 添加 robot*_x/y/z/yaw 参数转发 |
| 15 | 避障参数重复声明导致节点崩溃 | OA | has_parameter 检查后按需 declare |
| 16 | scan 时间戳与 rclcpp::Time 类型不兼容 | OA | rclcpp::Time(msg->header.stamp) 转换 |
| 17 | v_hpc 过大（>4 m/s）导致梯度爆炸 | OA | QP 求解前裁剪到加速度盒内 |
| 18 | severity 无上限导致径向力远超横向力 | OA | severity 上限设为 8x |
| 19 | 最近点表示在正方体棱边跳变导致振荡 | OA | 圆柱体适用，正方体不支持 |
| 20 | MPC QP 速度硬约束在 x1 不可行 → 求解失败 | MPC | 从 x3 开始施加约束，留缓冲步数 |
| 21 | MPC 参考速度坐标系错误 → 侧向跟踪不对称 | MPC | 改用跟随者朝向 R(θ_f)^T |
| 22 | MPC 边界投影位置/速度参考不一致 → 侧向弱 | MPC | 固定偏移模式 + 动态 ω×d 偏移 |
| 23 | MPC 求解器 API 字段名不兼容 | MPC | warm_start/polish |
| 24 | MPC QP 条件数极端 → 全部求解失败 | MPC | R 设下限 + eps 放松 |

---

## 1) 原始 Gazebo `/odom` 与定位体系的数据不一致

**现象**

Python 原版在 Gazebo 中工作正常（订阅 Gazebo 仿真内部 `/odom`），移植到
slam_toolbox 定位体系后 follower 持续大幅振荡（leader 静止时 cmd_vel 振幅 > 0.5 m/s），
无论 mass 取何值都无改善。

**原因**

Gazebo `/odom` 同一消息内提供位置、速度、偏航角、角速度，全部来自物理引擎，
天然一致。而 slam_toolbox/AMCL 定位体系中，位置和速度来自不同估计器：

- 位置: 定位算法（~10Hz），在 map 帧
- 速度: EKF 里程计（~50Hz），在 body 帧

两者帧率不同、坐标系不同、无时间同步。双重积分器模型将位置误差和速度误差
耦合在同一误差向量中，不一致的数据直接导致控制器把噪声放大为振荡。

**处理**

改用 **EKF odometry/filtered + TF 变换** 的数据通道：

- 从 EKF 里程计取位置（odom 帧）+ 速度（body 帧）——同一消息，自身一致
- 通过 slam_toolbox 的 `map → odom` TF 将位置变换到 map 帧
- 通过 TF yaw + EKF yaw 的总偏航角将速度旋转到 map 帧
- 角速度直接取自 EKF 里程计

位置和速度来自同一个 EKF 消息，只有变换依赖 TF，而 TF 是平滑的。
最终效果等价于将 Gazebo 的原生 odom 通过定位修正后投到 map 帧使用。

---

## 2) 订阅话题使用相对路径导致命名空间双重解析

**现象**

节点在 `/robot2` 命名空间下运行，实际订阅了 `/robot2/robot1/odometry/filtered`
而非 `/robot1/odometry/filtered`，数据收不到。

**原因**

话题构造时用 `strip_slash` 去掉了前导 `/`，ROS 2 将话题名解析为相对路径，
在节点命名空间下自动补全为 `/namespace/<relative_topic>`。

**处理**

所有跨命名空间的话题统一使用以 `/` 开头的绝对路径。

---

## 3) `MatrixExponential` / `MatrixSquareRoot` API 不匹配

**现象**

编译报错 `'MatrixExponential' is not a member of 'Eigen'`。

**原因**

Eigen 3.4 的 unsupported 模块将 `expm` 和 `sqrtm` 暴露为 `MatrixBase` 的方法
（`.exp()` / `.sqrt()`），而非独立的 class template。

**处理**

- `Eigen::MatrixExponential<Mat4d>(arg).compute()` → `arg.exp()`
- `Eigen::MatrixSquareRoot<Mat4d>(P).compute()` → `P.sqrt()`

---

## 4) `M += M.transpose()` 触发 Eigen aliasing 断言

**现象**

运行时断言 `aliasing detected during transposition` 并中止。

**原因**

Eigen 3.4 在赋值运算中检测到 `M` 和 `M.transpose()` 共享内存，拒绝执行。

**处理**

```cpp
Mat4d M_sym = M + M.transpose();  // 先求值到临时变量
M = M_sym;
```

---

## 5) `trans_con` 使用固定尺寸矩阵导致死循环

**现象**

程序在 `lpc2hpc → trans_con` 中卡死，不返回。

**原因**

Python 版 `trans_con` 每轮迭代后 Ak 和 Bk 维度实际缩小（4→2→1），
C++ 版使用固定 `Mat4d` 和 `Mat42d`，导致 `Ak.rows()` 始终返回 4，
循环终止条件永远为假。

**处理**

将工作矩阵改为动态尺寸 `Eigen::MatrixXd`，使其维度随迭代正确收缩。

---

## 6) `HpcResult.K0` 类型定义错误

**现象**

编译报 `YOU_MIXED_MATRICES_OF_DIFFERENT_SIZES`。

**原因**

`K0 = K0_new @ T` 结果为 2×4（2 输入 × 4 状态），但 `HpcResult::K0` 被定义为 `Mat4d`。

**处理**

改为 `Mat24d`。

---

## 7) 定位噪声被 `expm` 非线性放大

**现象**

即使数据通道一致后，leader 静止时 follower 仍有小幅振荡。

**原因**

控制律中 `expm(Gd * (1 - log(c)))` 在 c 取小值时放大倍数极高：

| c | `1 - log(c)` | 放大效果 |
|---|-------------|---------|
| 1.0 | 1.0 | 接近恒等，无放大 |
| 0.5 | 1.693 | 数倍放大 |
| 0.1 | 3.303 | 10~100 倍放大 |

Python 原版使用 `c = clamp(nx, 0.1, 1.0)`。Gazebo 完美数据下 c=0.1 只在
严重偏离编队点时触发，但在定位噪声下 hnorm 频繁给出小值、误判为需要强纠正，
指数级放大噪声导致振荡。

**处理**

将 c clamp 下限从 0.1 提高到 0.5，降低 warping 放大的上限。

---

## 8) `val_a / val_b` 比值在噪声下爆炸

**现象**

位置误差极小时（e[0] ≈ 0.01~0.05 m），`val_a = -m * e[2] / e[0]` 的分母接近零，
比值达到数百甚至上千，增益矩阵特征值远超 `omega_d * m` 的设计下界。

**原理**

在双积分器模型中，当位置误差和速度误差方向相反时，比值变为正数且幅值极大。
这在 Gazebo 完美数据下是期望的防超调行为，但在噪声下变成增益爆炸。

**处理**

```cpp
double max_ratio = omega_d * mass_;
val_a = std::clamp(val_a, -max_ratio, max_ratio);
val_b = std::clamp(val_b, -max_ratio, max_ratio);
```

---

## 9) 死区（deadband）导致"开-关"式抽搐

**现象**

添加死区（`|dist - 2.0| < threshold`）后，follower 在边界反复进出，
表现为"加速→进死区→停止→滑出→加速→..."的抽搐。

**处理**

移除死区。齐次控制本身的非线性 gain scheduling 在靠近编队点时自动降低增益，
不需要额外的开关逻辑。

---

## 10) 速度归零导致算法退化 — 不采用

**尝试**

曾尝试将双积分器状态的速度分量强制置零，控制器降级为纯位置环。

**结论**

丢失 K2 阻尼项，明显改变算法特性，与 Python 原版不等价。不采用。

**最终方案**

通过 EKF odom + TF 变换通道提供完整 4 维状态（位置 + 速度），控制器逻辑等价于原版。

---

## 11) 6D 控制器速度误差跨坐标系直接相减 → 发散

**现象**

6D 控制器（Layer 3）初次运行时两车相撞，cmd_vel 输出恒定方向不收敛。

**原因**

6D 状态中 $v_x^b, v_y^b$ 定义在各机器人**自身车体系**下。当 leader yaw=0°、follower yaw=90° 时，
直接做 $v_{x,f}^b - v_{x,l}^b$ 是把不同坐标系下的速度相减，无物理意义。
follower 的"前进"（body +X）是 map +Y 方向，leader 的"前进"是 map +X 方向，
两者相减不能反映真实速度误差。

**处理**

在误差计算时将 follower 速度旋转到 leader 车体系：

$$v_{x,f}^L = v_{x,f}^b \cos\Delta\theta - v_{y,f}^b \sin\Delta\theta$$
$$v_{y,f}^L = v_{x,f}^b \sin\Delta\theta + v_{y,f}^b \cos\Delta\theta$$

其中 $\Delta\theta = \theta_f - \theta_l$。对应地，控制力从 leader 车体系旋转回 follower 车体系进行前向欧拉积分。

**修改文件**: `homo_controller_6d.hpp` → `compute_error()`, `lpc_calculate()`

---

## 12) 边界投影符号错误 → 两车相撞

**现象**

修复 #11 后两车仍然相撞。调试日志显示 follower 从右侧逼近 leader 并直接穿过到达左侧。

**原因**

边界投影公式 $d = -r_s \cdot \text{direction}$ 中的负号把编队点放在了 leader 的**反方向**。
follower 在 leader 右侧时，编队点被置于 leader 左侧，follower 必须穿越 leader 才能到达。

**处理**

改为 $d = +r_s \cdot \text{direction}$，编队点落在 leader→follower 连线与安全圆的交点（两者之间）：

$$d_{\text{pos}} = r_s \cdot \frac{\mathbf{p}_f - \mathbf{p}_l}{\|\mathbf{p}_f - \mathbf{p}_l\|}$$

**修改文件**: `homo_controller_6d.hpp` → `compute_error()`

---

## 13) `trans_con_nd` 可控性矩阵尺寸硬编码

**现象**

首次编译 `lpc2hpc_nd.hpp` 时，可控性矩阵直接使用了原 4D 版本的 `Matrix<double, 4, 8>`。

**原因**

原版 `trans_con` 的可控性矩阵针对 4 状态 2 输入设计（4×8）。6D 系统有 6 状态 3 输入，
可控性矩阵应为 6×18。硬编码导致编译错误。

**处理**

改为动态尺寸 `MatrixXd(N, N*M)`，由运行时维度决定。

**修改文件**: `lpc2hpc_nd.hpp` → `trans_con_nd()`

---

## 14) `sim_rf2o_ekf` launch 未转发 spawn 位姿参数

**现象**

通过 `sim_rf2o_ekf_two_robots.launch.py` 传入 `robot2_x:=4.0 robot2_yaw:=1.57` 不生效，
两车仍 spawn 在默认位置。

**原因**

`sim_rf2o_ekf_two_robots.launch.py` 内嵌了 `sim_two_robots.launch.py`（Gazebo spawn），
但 `robot*_x/y/z/yaw` 等参数未在 `IncludeLaunchDescription` 的 `launch_arguments` 中转发。

**处理**

在 launch 文件中声明并转发所有 8 个 spawn 位姿参数。

**修改文件**: `homo_multirobot_localization/launch/sim_rf2o_ekf_two_robots.launch.py`

---

## 15) 避障参数 `max_linear_accel` 重复声明 → 节点崩溃

**现象**

`ros2 launch formation_single_follower_6d_oa.launch.py` 后进程立即退出：
`ParameterAlreadyDeclaredException: parameter 'max_linear_accel' has already been declared`

**原因**

`FormationController6DOA` 节点和 `ObstacleAvoider` 分别在各自构造函数中
declare 了同名的 `max_linear_accel` 和 `max_angular_accel` 参数。ROS 2 不允许
同一节点内重复声明同名参数。

**处理**

在 `ObstacleAvoider` 中使用 `has_parameter` 检查：
- 已存在 → `get_parameter` 读取
- 不存在 → `declare_parameter` 声明后读取

**修改文件**: `obstacle_avoider.cpp` → 构造函数 `get_or_declare_double` lambda

---

## 16) scan 时间戳 `builtin_interfaces::msg::Time` 与 `rclcpp::Time` 类型不兼容

**现象**

编译报错：`no match for 'operator-' (operand types are 'builtin_interfaces::msg::Time' and 'rclcpp::Time')`

**原因**

`msg->header.stamp` 类型为 `builtin_interfaces::msg::Time`，`last_scan_stamp_` 为 `rclcpp::Time`，
两者不能直接做减法运算。

**处理**

```cpp
rclcpp::Time now(msg->header.stamp);
double dt = (now - last_scan_stamp_).seconds();
```

**修改文件**: `obstacle_avoider.cpp` → `scan_callback()`

---

## 17) v_hpc 远大于加速度可行盒导致梯度爆炸，梯度下降不收敛

**现象**

日志显示 `v_hpc=[-4.4,-0.4]`（4.4 m/s），但加速度约束仅允许 ±0.1 m/s 变化。
QP 目标函数 `||v - v_hpc||²` 的梯度约为 `2×(4.4) ≈ 8.8`，
避障梯度仅 ~0.5，被编队力完全压制。梯度下降每次迭代都撞到加速度盒边界
并被投影回来，20 次迭代不收敛，输出速度接近零。

**原因**

HPC 控制器在大编队误差下输出极高速度（非线性增益放大），
但在 QP 中没有将编队目标裁剪到加速度可达范围。

**处理**

求解前将 v_hpc 裁剪到加速度盒内：`v_target = clip(v_hpc, lb, ub)`，
让编队力只在可达范围内竞争。后续版本中此逻辑被移除（清理正方体代码时），
保留此记录供未来调优参考。

**修改文件**: `obstacle_avoider.cpp` → `solve()`（已被后续清理移除）

---

## 18) 障碍物权重 severity 无上限，靠近表面时径向力爆炸

**现象**

`severity = safety_distance / d_surface` 公式在 `d_surface ≈ 0.01m` 时可达 150x，
径向排斥力为横向绕行力的 50 倍以上，优化器只能后退无法绕行。

**处理**

severity 上限设为 8x：`severity = clamp(1.5, severity, 8.0)`。

**修改文件**: `obstacle_avoider.cpp` → `obstacle_effective_weight()`

---

## 19) 最近点表示在正方体棱边跳变 → 左右振荡

**现象**

正方体作为障碍物时，follower 在棱边处来回振荡，无法绕行。
圆柱体正常（光滑曲面，最近点连续）。

**原因**

正方体有多个离散面，当机器人经过棱边时，最近点从一个面跳到另一个面，
排斥方向 `n` 突变，QP 代价函数结构跳变，导致速度指令帧间不连续 → 振荡。
多次尝试通过位置平滑、横向激励、动态编队权重等手段修复，均无法完全消除。

**结论**

基于最近点的障碍物表示**仅适用于光滑曲面（圆柱体、球体等）**，
多面体需用质心圆或其他表示方式。此局限已写入 README 已知局限。

---

## 20) MPC QP 速度硬约束在 x1 不可行 → 求解失败

**现象**

MPC 启动后频繁出现 OSQP status=-3（primal infeasible），连续求解失败后 fallback 零速度。

**原因**

速度约束从 $x_1$ 开始施加，$|v_1| \le v_{\max}$。但 $x_1 = x_0 + dt \cdot u_0$，
且 $u_0$ 受加速度限幅 $|u| \le a_{\max}$ 约束。
当当前速度 $v_0$ 远超 $v_{\max}$（如初始时刻或大角速度 $>2$ rad/s），即使最大减速也
无法在一个步长内回到限制范围内，导致 $x_1$ 约束直接不可行。

**处理**

速度约束改为从 $x_3$（k=3）开始施加，留出 $3 \cdot dt \cdot a_{\max}$ 的缓冲空间。
此修改将约束不可行的概率大幅降低。若当前速度严重超限（>$v_{\max} + 3 \cdot dt \cdot a_{\max}$），
仍需 fallback 零速度作为兜底。

---

## 21) MPC 参考速度坐标系错误 → 侧向跟踪不对称

**现象**

Leader 前后移动时 follower 能保持编队，Leader 侧移时 follower 反应迟钝、保持不住距离。

**原因**

参考车体速度 $v_{\text{ref}}^{\text{body}}$ 的计算使用了参考朝向 $\theta_{\text{ref}}$（Leader 朝向）
进行旋转，而非跟随者的实际朝向 $\theta_f$：
$$v_{\text{ref}}^{\text{body}} = R(\theta_{\text{ref}})^T \cdot v_{\text{ref}}^{\text{map}}$$

当两车朝向不一致时（如初始 90° 偏差），Leader 车体系的"侧移"对应跟随者车体系的"前后移"，
速度参考与位置参考矛盾，导致侧向响应显著弱于前后向。

**处理**

改用跟随者的实际朝向进行旋转变换：
$$v_{\text{ref}}^{\text{body}} = R(\theta_f)^T \cdot v_{\text{ref}}^{\text{map}}$$

修改后前后和侧向跟踪对称性恢复。

---

## 22) MPC 边界投影位置/速度参考不一致 → 侧向保持力弱

**现象**

使用边界投影编队时，Leader 侧移时 follower 保持力弱于前后移动。

**原因**

边界投影参考位置为 $p_{\text{ref}} = p_L - r \cdot (p_L - p_f)/\|p_L - p_f\|$。
当 Leader 侧移 $\Delta y$ 时，参考位置仅移动约 $(r/d) \cdot \Delta y$（$d$ 为两车距离），
位置误差被人为缩小。但速度参考 $v_{\text{ref}}$ 直接使用 Leader 速度，保持不变。
位置和速度参考不一致——位置参考说"你还行"，速度参考说"快去追"——MPC 跟着偏弱的位置误差走，响应迟缓。

**处理**

（1）改为固定偏移编队模式，位置和速度参考完全同步移动，作为默认策略。
（2）速度参考的 $\omega \times d$ 偏移项同步改用边界投影动态偏移量（非固定偏移），
消除位置/速度公式层面的不一致。纯边界投影的完整一致性需要多轮 SQP 迭代，留待后续升级。

---

## 23) MPC 求解器 API 字段名不兼容

**现象**

编译时报错 `OSQPSettings has no member named 'warm_starting'`、`'polishing'`。

**原因**

`ros-humble-osqp-vendor` 提供的 OSQP 版本字段名为 `warm_start`、`polish`，
而非旧版 API 的 `warm_starting`、`polishing`。`osqp.h` 位于 `/opt/ros/humble/include/osqp/`，
需通过 `target_link_libraries(… osqp::osqp)` 链接。

**处理**

修正字段名为 `warm_start`、`polish`；CMakeLists.txt 中通过 `find_package(osqp_vendor)` +
`target_link_libraries(… osqp::osqp)` 正确引入依赖。

---

## 24) MPC QP 条件数极端 → 全部求解失败

**现象**

将 `q_theta=20.0` 与 `r_alpha=0.001` 组合使用时，QP 100% 返回 infeasible/max_iter，
连续 141 次求解均失败，节点进入安全停车状态。

**原因**

Q/R 权重比达到 20000:1，QP 的 Hessian 矩阵条件数极端差，OSQP 在默认精度
（`eps_abs/rel=1e-4`）下无法收敛。极端权重组合对数值求解器不友好。

**处理**

（1）R 权重设下限 0.01，避免 Q/R 比超过 2000:1。
（2）OSQP 精度放松至 `eps_abs/rel=1e-3`，提升数值稳定性。
（3）极端调参需求通过提高 Q 权重（而非降低 R）来满足。

