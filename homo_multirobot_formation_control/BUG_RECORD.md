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
