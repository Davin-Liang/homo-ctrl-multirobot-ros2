# BUG_RECORD — `homo_multirobot_formation_control` 联调记录

本文档记录将齐次控制编队算法从 Python/Gazebo 里程计移植到 C++/slam_toolbox 定位体系
过程中遇到的问题与处理方式。

---

## 总览

| # | 问题 | 分类 | 处理 |
|---|------|------|------|
| 28 | `ParameterValue` 对象无 `name` 属性 → record_trajectory 崩溃 | Python | zip 参数名与返回值 |
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
| 25 | omega_d·mass 下限过高 → 边界极限环震荡 | 控制 | 调低 mass 或 omega_d |
| 26 | 连续边界投影切线方向无恢复力 → 单轴漂移 | 控制 | 结构特性，非 bug |
| 27 | 8 字轨迹 Y 通道频率 2ω > ω_d → Y 轴跟踪差 | 控制 | 提高 omega_d 或放慢 8 字 |

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

---

## 25) omega_d·mass 下限过高 → 边界极限环震荡

**现象**

4D 连续边界投影控制器（`LpcController4DCont`）在 `wheel_max_omega` 调高后，
follower 在安全圆边界附近持续进出震荡，表现为 cmd_vel 正负交替。

**原因**

`calculate_klin()` 中自适应增益下限为 `a ≥ omega_d * mass`。默认 `omega_d=1.5, mass=8.0`
时下限为 12，而 MATLAB 原版在 `lpc_hpc_distance_square.m` 中使用硬编码下限 1（初始）
和 4（切换后）。

| 版本 | 下限 | k1 (mass=8) | 闭环极点 |
|------|------|------------|---------|
| MATLAB 原版 | 1~4 | -0.125~-2 | -(0.125~4)/m |
| C++ 修正版 | omega_d·m = 12 | -18 | -12/8 = -1.5 |

C++ 版边界附近弹簧刚度是 MATLAB 的 9~144 倍。在离散控制相位滞后、
加速度饱和、速度硬限幅三个非线性环节串联下，高增益形成极限环震荡。

**处理**

降低 `mass` 或 `omega_d` 可软化边界，牺牲收敛速度换稳定性：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_4d_cont.launch.py \
  mass:=3.0 omega_d:=2.0  # 自适应增益下限 = 6
```

MATLAB 原版的硬编码下限 1~4 与论文理论一致，`omega_d * mass` 是 C++ 移植时引入的工程修正，
目的是解耦 mass 与收敛速度。代价是默认值下边界过于刚性。

---

## 26) 连续边界投影切线方向无恢复力 → 单轴漂移

**现象**

使用连续边界投影时，Y 轴坐标偏差显著大于 X 轴，即使 leader 做匀速圆周运动。

**原因**

连续边界投影的误差 `e = d × (1 - R/|d|)` 始终平行于相对位置向量 `d`，只有径向分量。
切线方向完全没有恢复力。离散多边形约束的是空间中一个固定点（位置 X + Y 全约束），
连续投影只约束到圆的半径（1 维约束），切线方向自由度不受控。

**力学类比**

| 编队策略 | 约束维度 | 力学类比 |
|---------|---------|---------|
| 离散多边形 | 2（位置全约束） | 刚性杆，两端固定 |
| 连续边界投影 | 1（仅径向） | 杆 + 铰链，可绕圈滑动 |

这是连续边界投影的结构特性，不是 bug。切线漂移的程度取决于初始条件、
leader 运动轨迹和控制器参数。对于编队应用，通常不需要严格约束圆周上的具体方位，
切线自由度是设计让步。

---

## 27) 8 字轨迹 Y 通道频率 2ω > ω_d → Y 轴跟踪差

**现象**

leader 做 8 字运动（`leader_eight.py`）时，follower Y 轴跟踪显著差于 X 轴，
X 轴可以紧密跟随，Y 轴有明显相位滞后和幅值衰减。

**原因**

`leader_eight.py` 的轨迹参数方程为：

$$x(t) = A_x \sin(\omega t), \quad y(t) = A_y \sin(2\omega t)$$

Y 通道频率是 X 通道的 2 倍。默认 `period=10s → ω=0.628 → 2ω=1.257`。
默认 `omega_d=1.5`，Y 通道频率已逼近带宽，闭环系统无法跟上。

对于圆轨迹（`leader_circle.py`），X 和 Y 频率相同，跟踪效果对称。

**处理**

```bash
# 方案1：提高控制器带宽
ros2 launch homo_multirobot_formation_control formation_single_follower_4d_cont.launch.py \
  omega_d:=3.0

# 方案2：放慢 8 字
ros2 run homo_multirobot_formation_control leader_eight.py --ros-args -r __ns:=/robot1 \
  -p period:=20.0

# 方案3：减小 Y 幅值
ros2 run homo_multirobot_formation_control leader_eight.py --ros-args -r __ns:=/robot1 \
  -p amplitude_y:=0.5
```

这与连续边界投影无关——离散多边形方案在相同条件下也会出现 Y 轴滞后（同频率）。
根本原因是 8 字轨迹 Y 通道的固有高频特性，需要足够的控制器带宽来跟踪。

---

## 28) `record_trajectory.py` 读取控制器参数时 `AttributeError: 'ParameterValue' object has no attribute 'name'`

**现象**

```bash
ros2 run homo_multirobot_formation_control record_trajectory.py --ros-args -p mode:=sim
```
报错 `AttributeError: 'ParameterValue' object has no attribute 'name'`，进程退出。

**原因**

ROS 2 Humble 的 `rcl_interfaces/msg/ParameterValue` 消息不包含 `name` 字段。
参数名来自请求数组的顺序，与返回值中的 values 数组按位置一一对应。

原代码 `for pv in result.values: params[pv.name] = val` 假设 `ParameterValue` 有 `name`
属性，但该属性不存在。

**处理**

改为用 `zip(CTRL_PARAM_NAMES, result.values)` 按顺序匹配参数名与值：

```python
for name, pv in zip(CTRL_PARAM_NAMES, result.values):
    if pv.type == 3:       # PARAMETER_DOUBLE
        val = pv.double_value
    elif pv.type == 2:     # PARAMETER_INTEGER
        val = pv.integer_value
    else:
        continue
    params[name] = val
```

**修改文件**: `record_trajectory.py` → `_query_controller_params()`


---

## 29) rf2o 硬编码 vy=0 → EKF 融合后全向底盘横向速度反馈失效

**现象**: 6D Motor 控制器持续输出 ±0.6 横向指令，但 `/odometry/filtered` vy 恒为零。
leader 0.2 m/s 绕圈时 follower 编队距离震荡幅度大、周期慢。

**原因**:
1. 上游 rf2o 只考虑差速车，发布时 `odom.twist.twist.linear.y = 0.0` 硬编码，
   但其核心算法 `kai_loc_ = [vx, vy, ω]` 实际估计了完整平面速度
2. EKF 配置 `odom0_config` vy=true 将此恒零假测量当真值融合，
   `/odometry/filtered` 的 vy 被强行压零

**处理**: 补丁 third_party/rf2o_laser_odometry 三处：
- `CLaserOdometry2D.hpp`: 新增成员 `lin_speed_y`
- `CLaserOdometry2D.cpp`: `lin_speed_y = acu_trans(1,2)/dt`
- `CLaserOdometry2DNode.cpp`: `odom.twist.twist.linear.y = rf2o_ref.lin_speed_y`

**影响面**: 全系依赖 EKF vy 的控制器（4D/6D 全系列）；实车轮式里程计模式不受影响

---

## 30) v_cmd 积分步长与控制周期不一致 → 闭环极点失真、欠阻尼慢震荡

**现象**: ω_d=0.8 时编队距离缓慢震荡（周期 ~3-4s），表现为"靠近→停止→拉开→重复"。

**原因**: `lpc_calculate()` 中前向欧拉步长写死 `h=0.1`（照抄 4D），但 4D 的 h 只是
输出线性整形系数（它的 v 每周期从 EKF 重新测量）；6D Motor 的 v_cmd 是跨周期
积分状态，`goal = v_cmd + h·u/m` 中的 h 必须等于真实控制周期 0.05s。
h=0.1 时等效 B 乘以 2 → 三阶极点 (s+λ)³ 变成 -0.33 实极点 + ζ≈0.6 的欠阻尼复极对；
配合加速度限幅相位损失，产生弛豫振荡。

**处理**: 构造器新增 `control_period` 参数 = 1/control_rate，节点构造时传入；
`lpc_calculate` 用 `h_`（成员变量）替代硬编码 0.1。

---

## 31) HPC 的 c-clamp 下界 0.5 对 6D 三阶链的翘曲放大 ~30×（协同因素，非主因）

**现象**: LPC-only 不震荡，开 HPC 后靠近目标时加速/震荡明显；
omega_d 或 leader 速度越高越剧烈。

**原因**: 6D Motor 齐次权重 [2,1,0] 比 4D [1,0] 深，c_min=0.5 时
expm(Gd·1.69) 对位置通道放大约 30×（vs 4D 的 5×）。
与 accel≤0.25 的慢执行器组合形成弛豫振荡。

**重要更正**: 经后续隔离分析，震荡的主因是 ω_d 偏高（1.2–1.5）超过
0.25 accel 物理上限 + h 步长 bug（等效 B 矩阵 ×2）+ rf2o vy=0 反馈盲区。
c_min 的 30× 翘曲是在这些因素叠加下将信号进一步放大——在 ω_d=0.7
（物理可达范围）+ h 修复 + rf2o 补丁后，c_min=0.5 不再导致震荡。

**处理**: 新增 `hpc_c_min` 参数。在 ω_d 物理可达范围内，c_min=0.5（4D 默认）
或 0.9 均可稳定运行；当前工程默认保留 0.9（保守侧）。
纯 LPC 模式 (use_hpc=false) 不受此参数影响。

---

## 32) leader_vel_lpf_tau:=0 时滤波器冻结 leader 速度（而非关断）

**现象**: `leader_vel_lpf_tau:=0` 后 follower 完全跟不上 leader。

**原因**: alpha = 20/(20+1/0) = 0 → 低通滤波器冻结在初始测量值，
leader 速度被永远锁死在启动瞬间的数值上。

**处理**: `leader_vel_lpf_tau_ ≤ 0` 时跳过滤波、直通原始测量。
默认值改为 0.0（关断），需要时设 0.2-0.3。

---

## 33) 三阶链 lambda 语义与 4D 对齐 + 自适应逻辑保留

**说明**: 4D 的 `calculate_klin` 中 `a` 不是极点，闭环极点 = a/m（λ=a/m, λ≥ωd）。
6D Motor 直接采用 λ 作为极点参数，`compute_channel_3rd` 内部先换算 λ=a/m，
保证与 4D 的自适应逻辑（e_v 用 v_real 误差、clamp 到 ±ωd·M）完全兼容。

---

## 34) 死区 Td 初步通过 Smith 外挂补偿 → 后续改为 Artstein 模型约简

**背景**: 实物实测确认 ~220ms 指令死区（`measure_motor_latency.py`，5% @0.3m/s 阶跃
的 EKF 延迟 P50=271ms，扣除 EKF 48ms 后 ≈ 220ms）。死区导致 4-5 个控制周期
的虚假"无响应"信号，控制器在这些周期内过度补偿。

**当前方案 (v1)**: Smith 预估器外挂（`motor_predictor.hpp`，τ+Td 双模型），
`comp_vx/comp_vy` 加在 v_real 测量值上。

**后续方案**: 采用 Artstein 模型约简——等价变换 $B_{\mathrm{eff}}=e^{-A T_d}B$，
死区从外挂补偿升级为内部模型的等价修正，不增维、不动 HPC。详见
`doc/artstein_reduction.md`。

---

## 35) record_velocity_diagnostics.py Exec format error

**现象**: 执行

```bash
ros2 run homo_multirobot_formation_control record_velocity_diagnostics.py
```

报错：

```text
OSError: [Errno 8] Exec format error
```

**原因**: 脚本文件曾出现 Windows/BOM/换行或 shebang/可执行权限问题，导致 Linux
不能按 Python 脚本执行。

**处理**: 确保文件首行是：

```python
#!/usr/bin/env python3
```

并转换为 UTF-8 no BOM + LF，执行 `chmod +x`，重新 `colcon build` 或
`source install/setup.bash`。

---

## 36) README 乱码

**现象**: README 中 4D Artstein 和 `record_velocity_diagnostics` 段落出现 `?????` 乱码。

**原因**: 可能是通过 Windows PowerShell/WSL 混合命令写入中文 Markdown 时编码不一致，
或反引号被 shell 解释造成内容损坏。

**处理**: 后续文档修改统一在 WSL 内使用 UTF-8 编辑，避免 PowerShell here-doc
直接写含中文和反引号的 Markdown。

---

## 37) record_trajectory 实物 PNG 统计与 CSV 复算不一致

**现象**: 实物图
`real_m2_r1_od0.7_f20_tau0.4_cmin0.1_Td0.2_20000101_081420.png`
中距离统计看起来和 CSV 复算结果不完全一致。

**已复算结果**:

```text
all: distance 0.998 ± 0.056 m, range [0.895, 1.101]
t>5s: distance 1.004 ± 0.055 m, range [0.918, 1.101]
t>10s: distance 0.995 ± 0.056 m, range [0.918, 1.101]
t>15s: distance 1.010 ± 0.055 m, range [0.921, 1.101]
```

**待查**: `record_trajectory.py` 是否存在绘图标签使用旧参数、旧数据、不同截断窗口，
或 PNG/CSV 文件不匹配的问题。

---

## 38) 6D Artstein Disc 开启延迟注入后控制器有输出但机器人不动

**现象**: 启动

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_artstein_disc.launch.py \
  leader_ns:=/robot1 follower_ns:=/robot2 use_motor_delay:=true
```

控制器日志持续输出非零命令，例如：

```text
6D_ART pred_shift=(-0.135, -0.113, -0.142) cmd=(-0.372,+0.455,-0.455)
DIAG: freq=20.2Hz avg_leader_age=32ms avg_ekf_age=32ms vcmd_map=(-0.459,-0.370) wcmd=-0.460
```

但 Gazebo 中 follower 不移动，或 `/robot2/cmd_vel` 没有最终速度命令。

**原因**: `formation_single_follower_6d_artstein_disc.launch.py` 最初把控制器输出重映射到
`cmd_vel_raw`，同时尝试用 remap 方式连接 `sim_motor_delay.py` 的
`cmd_vel_in/cmd_vel_out`。实际 `sim_motor_delay.py` 不使用这两个 remap 名称，而是通过参数
`input_topic` 和 `output_topic` 决定订阅/发布话题。结果是控制器发布到了
`/robot2/cmd_vel_raw`，延迟节点没有正确订阅并转发到 `/robot2/cmd_vel`，底盘插件收不到命令。

**处理**: 延迟节点改为显式传参：

```python
"input_topic": "cmd_vel_raw",
"output_topic": "cmd_vel",
```

当 `use_motor_delay:=true` 时链路为：

```text
/robot2/formation_control_node_6d_artstein_disc -> /robot2/cmd_vel_raw
/robot2/sim_motor_delay                         -> /robot2/cmd_vel
Gazebo/实物底盘                                  <- /robot2/cmd_vel
```

当 `use_motor_delay:=false` 时控制器直接发布 `/robot2/cmd_vel`。

**排查命令**:

```bash
ros2 topic echo /robot2/cmd_vel_raw --once
ros2 topic echo /robot2/cmd_vel --once
ros2 topic info -v /robot2/cmd_vel
ros2 topic info -v /robot2/cmd_vel_raw
```

若 `/robot2/cmd_vel_raw` 有值但 `/robot2/cmd_vel` 无值，优先检查延迟节点是否启动、
`input_topic/output_topic` 参数是否正确，以及 `use_motor_delay` 是否为 `true`。
