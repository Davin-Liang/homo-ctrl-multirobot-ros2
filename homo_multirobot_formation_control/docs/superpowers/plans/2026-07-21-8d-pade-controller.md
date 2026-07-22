# 8D Pade 死区增广齐次编队控制器 — 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将执行器纯死区（~220ms）通过 Pade(1,1) 近似显式增广为状态维度，构建 8D 全链路齐次编队控制器，替换 6D Motor + Smith 外挂方案。

**Architecture:** 照搬 `homo_controller_6d_motor.hpp` + `formation_control_node_6d_motor.cpp` 结构，状态从 6 维扩到 8 维（每轴增加 Pade 死区记忆状态 ω），每轴链从三阶升为四阶，极点配置从 `compute_channel_3rd` 升为 `compute_channel_4th`（Ackermann 数值求解）。移除 Smith 预估器和 motor delay 节点依赖。

**Tech Stack:** C++17, Eigen 3, ROS 2 Humble (rclcpp, tf2, nav_msgs, geometry_msgs)

**理论文档:** `doc/pade_deadtime_full.md`（已审查通过，5 处错误已修正）

---

## 文件结构

| 文件 | 操作 | 职责 |
|------|------|------|
| `include/.../homo_controller_8d_motor.hpp` | 新建 | LpcController8DMotor：A(8×8), B(8×2), compute_channel_4th, HPC |
| `include/.../formation_control_node_8d_motor.hpp` | 新建 | 节点头文件 |
| `src/formation_control_node_8d_motor.cpp` | 新建 | 节点实现：TF+EKF 管线, 8D 状态组装, ω/v_cmd 回写 |
| `src/main_8d_motor.cpp` | 新建 | 入口 |
| `launch/formation_single_follower_8d_motor.launch.py` | 新建 | Launch（去 Smith, 去 delay node, 加 Td） |
| `CMakeLists.txt` | 修改 | 新增 8D target |

**不修改的文件（照搬复用）:** `types_nd.hpp`, `lpc2hpc_nd.hpp`, `hnorm_nd.hpp`, `kinematic_constraint.hpp`

---

### Task 1: 创建控制器头文件骨架 + compute_channel_4th

**Files:**
- Create: `include/homo_multirobot_formation_control/homo_controller_8d_motor.hpp`

- [ ] **Step 1: 创建头文件骨架（include guard, namespace, 类声明）**

```cpp
#pragma once

/// @file 8D Pade 死区增广齐次编队控制器。
///
/// 状态 x = [px, py, vx_cmd, vy_cmd, ωx, ωy, vx_real, vy_real]^T（全部 map 系）
///   - px, py:            位置（TF + EKF）
///   - vx_cmd, vy_cmd:    指令速度——控制器内部积分状态
///   - ωx, ωy:            Pade 死区记忆状态——内部积分，不可测量
///   - vx_real, vy_real:  电机实际速度（EKF 测量）
///
/// 系统方程（死区 Pade(1,1) + 电机滞后增广）:
///   dp/dt      = v_real
///   dv_cmd/dt  = u / mass
///   dω/dt      = -(2/Td)·ω + v_cmd
///   dv_real/dt = (1/τ)·((4/Td)·ω - v_cmd - v_real)
///
/// 退化: Td→0 时退化为 6D Motor; Td→0 且 τ→0+ 时退化为 4D 双积分器。
/// 齐次权重: [3,3,2,2,1,1,0,0]（每轴四阶链 [3,2,1,0]）。
///
/// 编队点逻辑与 6D Motor 相同（离散多边形 + tol 滞后切换）。
/// 不接 Smith 预估器——死区内嵌在 A 中。

#include <cmath>
#include <tuple>
#include <vector>
#include <limits>
#include <algorithm>
#include <stdexcept>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "homo_multirobot_formation_control/types_nd.hpp"
#include "homo_multirobot_formation_control/lpc2hpc_nd.hpp"
#include "homo_multirobot_formation_control/hnorm_nd.hpp"

namespace formation_control {

class LpcController8DMotor {
public:
  LpcController8DMotor(int m_p = 4, double radius = 2.0, double tol = 0.1,
                       double mass = 2.0, double tau = 0.43, double Td = 0.22,
                       double omega_d = 0.7, bool use_hpc = true,
                       double control_period = 0.05, double hpc_c_min = 0.95);

  void controller_initial(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2);
  std::vector<double> lpc_calculate(const Eigen::VectorXd& x1,
                                    const Eigen::VectorXd& x2);
  double calculate_distance(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2);

private:
  void check_and_switch_target(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2);
  Eigen::MatrixXd calculate_klin(const Eigen::VectorXd& e);

  /// 四阶极点配置（Ackermann 数值求解），每轴独立
  static std::tuple<double, double, double, double> compute_channel_4th(
      double e_p, double e_v, double M, double tau, double Td, double wd);

  int    m_p_;       double radius_;    double tol_;
  double mass_;      double tau_;       double Td_;
  double omega_d_;   double h_;         bool   use_hpc_;
  double hpc_c_min_;

  Eigen::MatrixXd A_;   // 8×8
  Eigen::MatrixXd B_;   // 8×2
  Eigen::Matrix<double, 8, Eigen::Dynamic> dl_;  // 编队偏移向量集 (8 × m_p)
  Eigen::VectorXd d_;         // 当前目标偏移 (8)
  Eigen::MatrixXd k_lin_;     // 线性反馈增益 (2×8)
  Eigen::MatrixXd P_;         // Lyapunov 矩阵 (8×8)
  Eigen::MatrixXd G0_;        // 齐次生成元 (8×8)
  Eigen::MatrixXd Gd_;        // 膨胀生成元 (8×8)
  double nu_;
};

}  // namespace formation_control
```

- [ ] **Step 2: 实现 compute_channel_4th — Ackermann 公式四阶极点配置**

在类声明之后（`};` 之前），添加静态方法实现：

```cpp
// --------------------------------------------------------------------------
// 单轴四阶极点配置（p → v_real → ω → v_cmd 链）。
//
// 子系统 Ax(4×4), Bx(4×1):
//   dp/dt      = v_real
//   dv_cmd/dt  = u/M
//   dω/dt      = v_cmd - (2/Td)·ω
//   dv_real/dt = (-v_cmd + (4/Td)·ω - v_real) / τ
//
// 使用 Ackermann 公式将闭环极点配置到 -λ（四重根，λ = a/M ≥ wd）:
//   α_c(s) = (s+λ)^4
//   K_ack = e4^T · C^{-1} · α_c(Ax)    (negative feedback convention)
//   K = -K_ack                           (positive feedback convention)
//
// a 的自适应逻辑沿用 compute_channel_3rd 模式：
//   a = clamp(-M·e_v/e_p, -wd·M, wd·M), lower bounded by wd·M
// --------------------------------------------------------------------------
static std::tuple<double, double, double, double> compute_channel_4th(
    double e_p, double e_v, double M, double tau, double Td, double wd)
{
  // 自适应 a（同 6D Motor compute_channel_3rd，取 v_real 误差分量）
  double val = (std::abs(e_p) > 1e-6) ? -M * e_v / e_p : 0.0;
  double max_ratio = wd * M;
  val = std::clamp(val, -max_ratio, max_ratio);
  double a = std::max(val, wd * M);
  double lambda = a / M;

  // 构建四阶子系统 Ax(4×4)
  Eigen::Matrix4d Ax;
  Ax << 0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, -2.0 / Td, 0.0,
        0.0, -1.0 / tau, 4.0 / (tau * Td), -1.0 / tau;

  // Bx(4×1)
  Eigen::Vector4d Bx;
  Bx << 0.0, 1.0 / M, 0.0, 0.0;

  // 可控性矩阵 C = [Bx, Ax·Bx, Ax²·Bx, Ax³·Bx]
  Eigen::Matrix4d C;
  C.col(0) = Bx;
  C.col(1) = Ax * C.col(0);
  C.col(2) = Ax * C.col(1);
  C.col(3) = Ax * C.col(2);

  // 期望闭环特征多项式 α_c(s) = (s+λ)^4
  Eigen::Matrix4d I4 = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d Ax2 = Ax * Ax;
  Eigen::Matrix4d Ax3 = Ax2 * Ax;
  Eigen::Matrix4d Ax4 = Ax3 * Ax;
  Eigen::Matrix4d alpha_Ax = Ax4
      + 4.0 * lambda * Ax3
      + 6.0 * lambda * lambda * Ax2
      + 4.0 * lambda * lambda * lambda * Ax
      + lambda * lambda * lambda * lambda * I4;

  // Ackermann: 解 C^T · z = e4 → z^T = e4^T · C^{-1}
  Eigen::Vector4d e4;
  e4 << 0.0, 0.0, 0.0, 1.0;

  // 用 QR 分解求解（比直接求逆更稳定）
  Eigen::Vector4d z = C.transpose()
                          .colPivHouseholderQr()
                          .solve(e4);

  // K_ack = z^T · α_c(Ax)（负反馈约定）
  // K = -K_ack（正反馈约定，与 compute_channel_3rd 一致）
  Eigen::RowVector4d Kx_raw = -z.transpose() * alpha_Ax;

  return {Kx_raw(0), Kx_raw(1), Kx_raw(2), Kx_raw(3)};
}
```

> **备用方案**: 若 colPivHouseholderQr::solve 对接近奇异的 C 报错（controllable 系统理论上不会），改用 `C.fullPivLu().solve(e4)`。

- [ ] **Step 3: 验证 compute_channel_4th 特征值正确性**

在 `compute_channel_4th` 末尾添加断言（仅在控制周期内验证一次）。后续 `calculate_klin` 调用后可选输出 RCLCPP_DEBUG。

验证逻辑（不写入头文件，手动测试用）：
```cpp
// 测试代码（单独编译验证，不嵌入最终代码）:
// M=2.0, τ=0.43, Td=0.22, wd=0.7, e_p=1.0, e_v=0.0 → λ=0.7
// 期望: eig(Ax + Bx*Kx) ≈ [-0.7, -0.7, -0.7, -0.7]
```

---

### Task 2: 实现 LpcController8DMotor 构造函数 + A/B 矩阵

**Files:**
- Modify: `include/homo_multirobot_formation_control/homo_controller_8d_motor.hpp`

- [ ] **Step 1: 添加构造函数实现（在类声明内或类外 inline）**

```cpp
LpcController8DMotor(int m_p = 4, double radius = 2.0, double tol = 0.1,
                     double mass = 2.0, double tau = 0.43, double Td = 0.22,
                     double omega_d = 0.7, bool use_hpc = true,
                     double control_period = 0.05, double hpc_c_min = 0.95)
  : m_p_(m_p), radius_(radius), tol_(tol), mass_(mass),
    tau_(tau), Td_(Td), omega_d_(omega_d),
    h_(control_period), use_hpc_(use_hpc),
    hpc_c_min_(hpc_c_min)
{
  if (Td_ < 0.01) {
    throw std::invalid_argument("LpcController8DMotor: Td 不得小于 0.01 s"
                                "（接近 0 时 1/Td 发散，退化为 6D Motor）");
  }

  const int n = 8, m = 2;

  // 8D Pade 死区-电机全链路模型 A 矩阵（状态顺序同式(3)）
  // [px, py, vx_cmd, vy_cmd, ωx, ωy, vx_real, vy_real]
  A_.resize(n, n);
  A_ << 0.0, 0.0,  0.0,        0.0,        0.0,           0.0,           1.0,         0.0,
        0.0, 0.0,  0.0,        0.0,        0.0,           0.0,           0.0,         1.0,
        0.0, 0.0,  0.0,        0.0,        0.0,           0.0,           0.0,         0.0,
        0.0, 0.0,  0.0,        0.0,        0.0,           0.0,           0.0,         0.0,
        0.0, 0.0,  1.0,        0.0,       -2.0 / Td_,     0.0,           0.0,         0.0,
        0.0, 0.0,  0.0,        1.0,        0.0,          -2.0 / Td_,     0.0,         0.0,
        0.0, 0.0, -1.0 / tau_,  0.0,        4.0/(tau_*Td_), 0.0,        -1.0 / tau_,  0.0,
        0.0, 0.0,  0.0,       -1.0 / tau_,  0.0,           4.0/(tau_*Td_), 0.0,        -1.0 / tau_;

  B_.resize(n, m);
  B_ << 0.0, 0.0,
        0.0, 0.0,
        1.0 / mass_, 0.0,
        0.0, 1.0 / mass_,
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0;

  d_.resize(n);
  d_.setZero();
  k_lin_.resize(m, n);
  k_lin_.setZero();
  P_.resize(n, n);
  P_.setIdentity();
  G0_.resize(n, n);
  G0_.setZero();
  Gd_.resize(n, n);
  Gd_.setIdentity();
  nu_ = 0.0;
}
```

- [ ] **Step 2: 无需 update_A_tau（第一版不做自适应 τ）**

第一版 τ 和 Td 均为常值，不需要 `update_A_tau()`。后续叠加自适应 τ 时再加（与 6D Motor 同模式，修改 A_ 第 6/7 行 0-indexed 含 1/τ 的 6 个元素）。

---

### Task 3: 实现 controller_initial + check_and_switch_target

**Files:**
- Modify: `include/homo_multirobot_formation_control/homo_controller_8d_motor.hpp`

- [ ] **Step 1: 实现 controller_initial**

```cpp
void controller_initial(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2)
{
  dl_.resize(8, m_p_);
  dl_.setZero();
  for (int i = 0; i < m_p_; ++i) {
    double angle = 2.0 * M_PI * i / m_p_;
    dl_(0, i) = -radius_ * std::cos(angle);
    dl_(1, i) = -radius_ * std::sin(angle);
  }

  int best_idx = 0;
  double best_dist = std::numeric_limits<double>::max();
  for (int i = 0; i < m_p_; ++i) {
    double dist = (x2 - x1 - dl_.col(i)).norm();
    if (dist < best_dist) { best_dist = dist; best_idx = i; }
  }
  d_ = dl_.col(best_idx);

  Eigen::VectorXd e = x2 - x1 - d_;
  k_lin_ = calculate_klin(e);

  if (use_hpc_) {
    auto res = lpc2hpc_nd(A_, B_, k_lin_);
    if (res.G0.isZero(1e-12)) {
      throw std::runtime_error("8D Pade 控制器初始化失败: lpc2hpc 返回零结果。");
    }
    G0_ = res.G0;
    P_  = res.P;
    nu_ = res.nu_min;
    Gd_ = Eigen::MatrixXd::Identity(8, 8) + nu_ * G0_;
  }
}
```

- [ ] **Step 2: 实现 check_and_switch_target**

```cpp
void check_and_switch_target(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2)
{
  double min_dist = std::numeric_limits<double>::max();
  int best_idx = 0;
  for (int i = 0; i < m_p_; ++i) {
    double dist = (x2 - x1 - dl_.col(i)).norm();
    if (dist < min_dist) { min_dist = dist; best_idx = i; }
  }

  double current_dist = (x2 - x1 - d_).norm();
  if (min_dist + tol_ < current_dist) {
    std::cout << "[LpcController8DMotor] 编队点切换 -> idx " << best_idx
              << " (err " << current_dist << " -> " << min_dist << " m)" << std::endl;
    d_ = dl_.col(best_idx);

    Eigen::VectorXd e = x2 - x1 - d_;
    k_lin_ = calculate_klin(e);

    if (use_hpc_) {
      auto res = lpc2hpc_nd(A_, B_, k_lin_);
      if (!res.G0.isZero(1e-12)) {
        G0_ = res.G0;
        P_  = res.P;
        nu_ = res.nu_min;
        Gd_ = Eigen::MatrixXd::Identity(8, 8) + nu_ * G0_;
      }
    }
  }
}
```

---

### Task 4: 实现 calculate_klin + lpc_calculate

**Files:**
- Modify: `include/homo_multirobot_formation_control/homo_controller_8d_motor.hpp`

- [ ] **Step 1: 实现 calculate_klin — 2×8 K 矩阵（x/y 解耦）**

```cpp
Eigen::MatrixXd calculate_klin(const Eigen::VectorXd& e)
{
  // e(0)=e_px, e(1)=e_py, e(6)=e_vx_real, e(7)=e_vy_real
  auto [k1_x, k2_x, k3_x, k4_x] =
      compute_channel_4th(e(0), e(6), mass_, tau_, Td_, omega_d_);
  auto [k1_y, k2_y, k3_y, k4_y] =
      compute_channel_4th(e(1), e(7), mass_, tau_, Td_, omega_d_);

  Eigen::MatrixXd K(2, 8);
  //   k1   0     k2   0     k3   0     k4   0
  //   0     k1    0    k2    0    k3    0    k4
  K << k1_x, 0.0,   k2_x, 0.0,   k3_x, 0.0,   k4_x, 0.0,
       0.0,   k1_y, 0.0,   k2_y, 0.0,   k3_y, 0.0,   k4_y;
  return K;
}
```

- [ ] **Step 2: 实现 lpc_calculate — 控制量计算（20Hz 每周期）**

```cpp
std::vector<double> lpc_calculate(const Eigen::VectorXd& x1,
                                  const Eigen::VectorXd& x2)
{
  check_and_switch_target(x1, x2);

  Eigen::VectorXd e = x2 - x1 - d_;

  Eigen::Vector2d u2;
  if (use_hpc_) {
    double nx = hnorm_nd(e, Gd_, P_);
    double c = std::clamp(nx, hpc_c_min_, 1.0);
    double log_c = std::log(c);
    Eigen::MatrixXd expm_g = (Gd_ * (1.0 - log_c)).exp();
    Eigen::VectorXd warped_e = expm_g * e;
    u2 = std::pow(c, 1.0 + nu_) * (k_lin_ * warped_e);
  } else {
    u2 = k_lin_ * e;
  }

  // 前向欧拉：v_cmd 自演化（不读测量速度）
  double goal_vx_cmd = x2(2) + h_ * u2(0) / mass_;
  double goal_vy_cmd = x2(3) + h_ * u2(1) / mass_;

  return {goal_vx_cmd, goal_vy_cmd};
}
```

- [ ] **Step 3: 实现 calculate_distance**

```cpp
double calculate_distance(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2)
{
  double min_dist = std::numeric_limits<double>::max();
  for (int i = 0; i < m_p_; ++i) {
    double dist = (x2 - x1 - dl_.col(i)).norm();
    min_dist = std::min(min_dist, dist);
  }
  return min_dist;
}
```

---

### Task 5: 创建 Node 头文件

**Files:**
- Create: `include/homo_multirobot_formation_control/formation_control_node_8d_motor.hpp`

- [ ] **Step 1: 完整头文件**

```cpp
#pragma once

/// @file 8D Pade 死区增广编队控制 ROS 2 节点 — TF + EKF 数据管线。
///
/// 与 6D Motor 节点（formation_control_node_6d_motor.hpp）的差异:
///   - 状态 8 维: [px, py, vx_cmd, vy_cmd, ωx, ωy, vx_real, vy_real]（map 系）
///   - ω 是新增内部积分状态（Pade 死区记忆）
///   - 不接 Smith 预估器（死区已内嵌在 A 矩阵中）
///   - 不接 sim_motor_delay 节点（cmd_vel 直接发布）

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include "homo_multirobot_formation_control/homo_controller_8d_motor.hpp"
#include "homo_multirobot_formation_control/kinematic_constraint.hpp"

class FormationController8DMotor : public rclcpp::Node
{
public:
  FormationController8DMotor();

private:
  void timer_cb();

  // 参数
  std::string leader_ns_, follower_ns_;
  double Kp_yaw_, K_ff_;
  double max_linear_vel_, max_angular_vel_;
  double min_cmd_vel_ = 0.03;
  double control_rate_;
  double Td_;

  // 控制器 + 约束
  std::unique_ptr<formation_control::LpcController8DMotor> ctrl_;
  formation_control::KinematicConstraint constraint_;

  // v_cmd 内部状态（map 系）
  double vx_cmd_map_ = 0.0;
  double vy_cmd_map_ = 0.0;

  // ω 内部状态（map 系）—— Pade 死区记忆
  double omega_x_map_ = 0.0;
  double omega_y_map_ = 0.0;

  // leader 速度低通滤波
  double leader_vel_lpf_tau_ = 0.3;
  double lpf_leader_vx_ = 0.0, lpf_leader_vy_ = 0.0;
  bool leader_vel_filtered_ = false;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // EKF 里程计订阅
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_sub_, follower_sub_;
  nav_msgs::msg::Odometry::SharedPtr leader_odom_, follower_odom_;

  // 发布
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 诊断
  rclcpp::Time leader_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time follower_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_diag_time_{0, 0, RCL_ROS_TIME};
  int diag_tick_ = 0;
  double sum_leader_age_ = 0.0;
  double sum_ekf_age_ = 0.0;

  bool leader_ok_ = false, follower_ok_ = false;
  bool controller_initialized_ = false;
};
```

---

### Task 6: 创建 Node 实现文件 — 构造器 + 订阅 + 定时器

**Files:**
- Create: `src/formation_control_node_8d_motor.cpp`

- [ ] **Step 1: 文件开头 + 头文件 + using**

```cpp
/// @file 8D Pade 死区增广编队控制节点实现。
///
/// 数据流:
///   EKF odometry/filtered ──→ 缓冲区（回调存储最新消息）
///   TF  map→X_odom        ──→ 定时器读取当前 TF
///
/// 定时器 (20 Hz):
///   1. 查找两机器人的 map → odom TF + EKF 位姿 → map 系位置/偏航/速度
///   2. 组装 8 维状态:
///        leader   x1 = [p, v_meas, (Td/2)*v_meas, v_meas]（稳态假设）
///        follower x2 = [p, v_cmd(内部), ω(内部), v_meas]
///   3. LpcController8DMotor::lpc_calculate → goal_v_cmd (map 系)
///   4. 旋转到车体系 → clamp → 轮速约束 → 发布 cmd_vel
///   5. 回写 v_cmd 和 ω（抗饱和/死区记忆一致性）

#include "homo_multirobot_formation_control/formation_control_node_8d_motor.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace formation_control;
```

- [ ] **Step 2: 辅助函数（照搬 6D Motor，一字不改）**

```cpp
static double tf2_yaw(const tf2::Quaternion& q)
{
  double r, p, y;
  tf2::Matrix3x3(q).getRPY(r, p, y);
  return y;
}

static double msg_yaw(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
```

- [ ] **Step 3: 构造函数 — 参数声明**

```cpp
FormationController8DMotor::FormationController8DMotor()
: rclcpp::Node("formation_control_node_8d_motor")
{
  leader_ns_   = declare_parameter("leader_ns",   "/robot1");
  follower_ns_ = declare_parameter("follower_ns", "/robot2");
  int m_p      = declare_parameter("m_p",      4);
  double radius = declare_parameter("radius",  2.0);
  double tol    = declare_parameter("tol",     0.1);
  double mass   = declare_parameter("mass",    2.0);
  double tau    = declare_parameter("tau",     0.43);
  Td_           = declare_parameter("Td",      0.22);
  double omega_d = declare_parameter("omega_d", 0.7);
  Kp_yaw_       = declare_parameter("Kp_yaw",  4.0);
  K_ff_         = declare_parameter("K_ff",    1.0);
  control_rate_ = declare_parameter("control_rate", 20.0);

  double wheel_radius    = declare_parameter("wheel_radius",    0.03);
  double base_radius     = declare_parameter("base_radius",     0.11);
  double wheel_max_omega = declare_parameter("wheel_max_omega", 20.0);
  double max_linear_accel  = declare_parameter("max_linear_accel",  2.0);
  double max_angular_accel = declare_parameter("max_angular_accel", 4.0);

  max_linear_vel_  = declare_parameter("max_linear_vel",  1.0);
  max_angular_vel_ = declare_parameter("max_angular_vel", 0.5);

  bool use_hpc = declare_parameter("use_hpc", true);
  double hpc_c_min = declare_parameter("hpc_c_min", 0.95);
  leader_vel_lpf_tau_ = declare_parameter("leader_vel_lpf_tau", 0.0);
  min_cmd_vel_ = declare_parameter("min_cmd_vel", 0.03);

  ctrl_ = std::make_unique<LpcController8DMotor>(
      m_p, radius, tol, mass, tau, Td_,
      omega_d, use_hpc, 1.0 / control_rate_, hpc_c_min);

  constraint_ = KinematicConstraint(wheel_radius, base_radius,
                                    wheel_max_omega,
                                    max_linear_accel, max_angular_accel);

  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  auto qos = rclcpp::SensorDataQoS();
  leader_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      leader_ns_ + "/odometry/filtered", qos,
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        leader_odom_ = m; leader_ok_ = true;
        leader_odom_stamp_ = m->header.stamp;
      });
  follower_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      follower_ns_ + "/odometry/filtered", qos,
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        follower_odom_ = m; follower_ok_ = true;
        follower_odom_stamp_ = m->header.stamp;
      });

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  last_diag_time_ = get_clock()->now();

  int ms = static_cast<int>(1000.0 / control_rate_);
  timer_ = create_wall_timer(std::chrono::milliseconds(ms),
                             [this]() { timer_cb(); });

  RCLCPP_INFO(get_logger(),
      "8D Pade 死区增广编队控制节点已启动 (tau=%.2fs, Td=%.2fs, hpc_c_min=%.2f)。",
      tau, Td_, hpc_c_min);
  RCLCPP_INFO(get_logger(), "  领航者: %s, 跟随者: %s",
      leader_ns_.c_str(), follower_ns_.c_str());
}
```

---

### Task 7: 实现 ekf_to_map + timer_cb

**Files:**
- Modify: `src/formation_control_node_8d_motor.cpp`（续 Task 6）

- [ ] **Step 1: ekf_to_map（照搬 6D Motor，一字不改）**

```cpp
static bool ekf_to_map(tf2_ros::Buffer& tf, const std::string& ns,
                       const nav_msgs::msg::Odometry::SharedPtr& odom,
                       double& px, double& py, double& vx_meas, double& vy_meas,
                       double& map_yaw, double& angular_z)
{
  if (!odom) return false;

  std::string odom_frame = ns;
  if (!odom_frame.empty() && odom_frame[0] == '/')
    odom_frame = odom_frame.substr(1);
  odom_frame += "_odom";

  try {
    auto t = tf.lookupTransform("map", odom_frame, tf2::TimePoint());
    double tf_x   = t.transform.translation.x;
    double tf_y   = t.transform.translation.y;
    double tf_yaw = tf2_yaw(tf2::Quaternion(
        t.transform.rotation.x, t.transform.rotation.y,
        t.transform.rotation.z, t.transform.rotation.w));

    double ekf_px  = odom->pose.pose.position.x;
    double ekf_py  = odom->pose.pose.position.y;
    double ekf_yaw = msg_yaw(odom->pose.pose.orientation);

    px = tf_x + ekf_px * std::cos(tf_yaw) - ekf_py * std::sin(tf_yaw);
    py = tf_y + ekf_px * std::sin(tf_yaw) + ekf_py * std::cos(tf_yaw);

    double total_yaw = tf_yaw + ekf_yaw;
    double vx_body = odom->twist.twist.linear.x;
    double vy_body = odom->twist.twist.linear.y;
    vx_meas = vx_body * std::cos(total_yaw) - vy_body * std::sin(total_yaw);
    vy_meas = vx_body * std::sin(total_yaw) + vy_body * std::cos(total_yaw);

    map_yaw   = total_yaw;
    angular_z = odom->twist.twist.angular.z;
    return true;
  } catch (const tf2::TransformException&) {
    return false;
  }
}
```

- [ ] **Step 2: timer_cb — 主控制循环**

```cpp
void FormationController8DMotor::timer_cb()
{
  if (!leader_ok_ || !follower_ok_) return;

  double l_px, l_py, l_vx, l_vy, leader_yaw, leader_az;
  double f_px, f_py, f_vx, f_vy, follower_yaw, follower_az;
  if (!ekf_to_map(*tf_buffer_, leader_ns_, leader_odom_,
                  l_px, l_py, l_vx, l_vy, leader_yaw, leader_az)) return;
  if (!ekf_to_map(*tf_buffer_, follower_ns_, follower_odom_,
                  f_px, f_py, f_vx, f_vy, follower_yaw, follower_az)) return;

  // Leader 速度低通（同 6D Motor）
  double l_vx_f, l_vy_f;
  if (leader_vel_lpf_tau_ <= 0.0) {
    l_vx_f = l_vx; l_vy_f = l_vy;
  } else {
    if (!leader_vel_filtered_) {
      lpf_leader_vx_ = l_vx; lpf_leader_vy_ = l_vy;
      leader_vel_filtered_ = true;
    } else {
      double alpha = control_rate_ / (control_rate_ + 1.0 / leader_vel_lpf_tau_);
      lpf_leader_vx_ += alpha * (l_vx - lpf_leader_vx_);
      lpf_leader_vy_ += alpha * (l_vy - lpf_leader_vy_);
    }
    l_vx_f = lpf_leader_vx_; l_vy_f = lpf_leader_vy_;
  }

  // 8D 状态组装
  Eigen::VectorXd x1(8), x2(8);

  // Leader: v_cmd = v_real = 测量速度, ω = (Td/2)*v_cmd（稳态假设）
  x1 << l_px, l_py,
        l_vx_f, l_vy_f,                // v_cmd
        (Td_ / 2.0) * l_vx_f, (Td_ / 2.0) * l_vy_f,  // ω（稳态解）
        l_vx_f, l_vy_f;                // v_real

  // 延迟初始化
  if (!controller_initialized_) {
    vx_cmd_map_ = f_vx;
    vy_cmd_map_ = f_vy;
    omega_x_map_ = (Td_ / 2.0) * f_vx;
    omega_y_map_ = (Td_ / 2.0) * f_vy;

    x2 << f_px, f_py, vx_cmd_map_, vy_cmd_map_,
          omega_x_map_, omega_y_map_, f_vx, f_vy;
    try {
      ctrl_->controller_initial(x1, x2);
      controller_initialized_ = true;
      RCLCPP_INFO(get_logger(), "8D Pade 控制器初始化完成。");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "初始化失败: %s", e.what());
      return;
    }
  }

  // Follower: v_cmd 和 ω 来自内部状态, v_real 来自 EKF
  x2 << f_px, f_py, vx_cmd_map_, vy_cmd_map_,
        omega_x_map_, omega_y_map_, f_vx, f_vy;

  // 齐次控制律 → map 系指令速度
  auto out = ctrl_->lpc_calculate(x1, x2);

  // map 系 → 车体系旋转
  double vx_body =  out[0] * std::cos(follower_yaw) + out[1] * std::sin(follower_yaw);
  double vy_body = -out[0] * std::sin(follower_yaw) + out[1] * std::cos(follower_yaw);

  // clamp + min_cmd_vel
  geometry_msgs::msg::Twist cmd;
  double vx_clamped = std::clamp(vx_body, -max_linear_vel_, max_linear_vel_);
  double vy_clamped = std::clamp(vy_body, -max_linear_vel_, max_linear_vel_);

  if (min_cmd_vel_ > 0.0) {
    double raw_mag = std::hypot(vx_body, vy_body);
    double cmd_mag = std::hypot(vx_clamped, vy_clamped);
    if (raw_mag > 0.001 && cmd_mag > 0.0 && cmd_mag < min_cmd_vel_) {
      double scale = min_cmd_vel_ / cmd_mag;
      vx_clamped *= scale;
      vy_clamped *= scale;
    }
  }
  cmd.linear.x = vx_clamped;
  cmd.linear.y = vy_clamped;

  // 偏航控制（与 6D Motor 相同）
  double raw_err   = leader_yaw - follower_yaw;
  double norm_err  = std::atan2(std::sin(raw_err), std::cos(raw_err));
  cmd.angular.z = std::clamp(norm_err * Kp_yaw_ + leader_az * K_ff_,
                              -max_angular_vel_, max_angular_vel_);

  // 全向轮运动学约束
  double dt = 1.0 / control_rate_;
  double wheel_scale = constraint_.apply(cmd.linear.x, cmd.linear.y,
                                          cmd.angular.z, dt);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "raw=(%+6.3f,%+6.3f) clamped=(%+6.3f,%+6.3f) final=(%+6.3f,%+6.3f) scale=%.2f "
      "vcmd=(%+6.3f,%+6.3f) vreal=(%+6.3f,%+6.3f) omega=(%+6.3f,%+6.3f)",
      vx_body, vy_body, vx_clamped, vy_clamped,
      cmd.linear.x, cmd.linear.y, wheel_scale,
      vx_cmd_map_, vy_cmd_map_, f_vx, f_vy,
      omega_x_map_, omega_y_map_);

  // 诊断: 实际控制频率 + 数据新鲜度（每 5 秒输出）
  ++diag_tick_;
  auto now = get_clock()->now();
  sum_leader_age_ += (now - leader_odom_stamp_).seconds();
  sum_ekf_age_    += (now - follower_odom_stamp_).seconds();
  double diag_elapsed = (now - last_diag_time_).seconds();
  if (diag_elapsed >= 5.0) {
    double real_freq = diag_tick_ / diag_elapsed;
    double avg_leader_age_ms = sum_leader_age_ / diag_tick_ * 1000.0;
    double avg_ekf_age_ms    = sum_ekf_age_    / diag_tick_ * 1000.0;
    RCLCPP_INFO(get_logger(),
        "DIAG: freq=%.1fHz avg_leader_age=%.0fms avg_ekf_age=%.0fms "
        "vcmd_vs_vreal=(%+.3f,%+.3f) omega=(%+.3f,%+.3f)",
        real_freq, avg_leader_age_ms, avg_ekf_age_ms,
        vx_cmd_map_ - f_vx, vy_cmd_map_ - f_vy,
        omega_x_map_, omega_y_map_);
    diag_tick_ = 0;
    sum_leader_age_ = 0.0;
    sum_ekf_age_ = 0.0;
    last_diag_time_ = now;
  }

  if (wheel_scale < 0.99) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "轮速约束触发: scale=%.2f, 限幅后 cmd=(%.2f, %.2f, %.2f)",
        wheel_scale, cmd.linear.x, cmd.linear.y, cmd.angular.z);
  }

  cmd_pub_->publish(cmd);

  // ---- 回写: v_cmd + ω 内部状态（抗饱和 + 死区记忆一致性）---------------------
  // v_cmd 回写（clamp/轮速约束后的最终发布值，同 6D Motor）
  vx_cmd_map_ = cmd.linear.x * std::cos(follower_yaw)
              - cmd.linear.y * std::sin(follower_yaw);
  vy_cmd_map_ = cmd.linear.x * std::sin(follower_yaw)
              + cmd.linear.y * std::cos(follower_yaw);

  // ω 回写（前向欧拉，使用同一份发布值——进入传输管道的实际信号）
  omega_x_map_ += dt * (-(2.0 / Td_) * omega_x_map_ + vx_cmd_map_);
  omega_y_map_ += dt * (-(2.0 / Td_) * omega_y_map_ + vy_cmd_map_);
}
```

---

### Task 8: 创建入口文件

**Files:**
- Create: `src/main_8d_motor.cpp`

- [ ] **Step 1: 完整文件**

```cpp
/// 8D Pade 死区增广编队控制节点入口。
///
/// 状态 [px, py, vx_cmd, vy_cmd, ωx, ωy, vx_real, vy_real]，
/// 死区 Pade(1,1) + 电机滞后显式建模，详见 doc/pade_deadtime_full.md。
///
/// 使用: ros2 launch homo_multirobot_formation_control \
///         formation_single_follower_8d_motor.launch.py \
///         leader_ns:=/virtual_leader follower_ns:=/robot2

#include <rclcpp/rclcpp.hpp>
#include "homo_multirobot_formation_control/formation_control_node_8d_motor.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FormationController8DMotor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

---

### Task 9: 创建 Launch 文件

**Files:**
- Create: `launch/formation_single_follower_8d_motor.launch.py`

- [ ] **Step 1: 完整 launch 文件（照搬 6D Motor，去 Smith + delay node，加 Td）**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    leader_ns = LaunchConfiguration("leader_ns")
    follower_ns = LaunchConfiguration("follower_ns")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Formation geometry
    m_p = LaunchConfiguration("m_p")
    radius = LaunchConfiguration("radius")
    tol = LaunchConfiguration("tol")

    # Robot dynamics
    mass = LaunchConfiguration("mass")
    tau = LaunchConfiguration("tau")
    Td = LaunchConfiguration("Td")
    omega_d = LaunchConfiguration("omega_d")

    # Yaw control
    Kp_yaw = LaunchConfiguration("Kp_yaw")
    K_ff = LaunchConfiguration("K_ff")

    # Velocity limits
    max_linear_vel = LaunchConfiguration("max_linear_vel")
    max_angular_vel = LaunchConfiguration("max_angular_vel")

    # Kinematic constraints
    wheel_radius = LaunchConfiguration("wheel_radius")
    base_radius = LaunchConfiguration("base_radius")
    wheel_max_omega = LaunchConfiguration("wheel_max_omega")
    max_linear_accel = LaunchConfiguration("max_linear_accel")
    max_angular_accel = LaunchConfiguration("max_angular_accel")

    # Control rate
    control_rate = LaunchConfiguration("control_rate")

    formation_node = Node(
        package="homo_multirobot_formation_control",
        executable="formation_control_node_8d_motor",
        name="formation_control_node_8d_motor",
        namespace=follower_ns,
        output="screen",
        parameters=[{
            "leader_ns": leader_ns,
            "follower_ns": follower_ns,
            "use_sim_time": use_sim_time,
            "m_p": m_p,
            "radius": radius,
            "tol": tol,
            "mass": mass,
            "tau": tau,
            "Td": Td,
            "omega_d": omega_d,
            "Kp_yaw": Kp_yaw,
            "K_ff": K_ff,
            "wheel_radius": wheel_radius,
            "base_radius": base_radius,
            "max_linear_vel": max_linear_vel,
            "max_angular_vel": max_angular_vel,
            "wheel_max_omega": wheel_max_omega,
            "max_linear_accel": max_linear_accel,
            "max_angular_accel": max_angular_accel,
            "control_rate": control_rate,
            "use_hpc": LaunchConfiguration("use_hpc"),
            "hpc_c_min": LaunchConfiguration("hpc_c_min"),
            "leader_vel_lpf_tau": LaunchConfiguration("leader_vel_lpf_tau"),
            "min_cmd_vel": LaunchConfiguration("min_cmd_vel"),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("leader_ns", default_value="/robot1"),
        DeclareLaunchArgument("follower_ns", default_value="/robot2"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("m_p", default_value="4"),
        DeclareLaunchArgument("radius", default_value="2.0"),
        DeclareLaunchArgument("tol", default_value="0.1"),
        DeclareLaunchArgument("mass", default_value="2.0"),
        DeclareLaunchArgument("tau", default_value="0.43",
                              description="Motor time constant (s)"),
        DeclareLaunchArgument("Td", default_value="0.22",
                              description="Dead-time delay (s). "
                                          "Measured ~220ms on real robot. "
                                          "Scan 0.10–0.30 for optimal value."),
        DeclareLaunchArgument("omega_d", default_value="0.7"),
        DeclareLaunchArgument("Kp_yaw", default_value="4.0"),
        DeclareLaunchArgument("K_ff", default_value="1.0"),
        DeclareLaunchArgument("control_rate", default_value="20.0"),
        DeclareLaunchArgument("use_hpc", default_value="true"),
        DeclareLaunchArgument("hpc_c_min", default_value="0.95",
                              description="HPC warp clamp. 8D chain (weights "
                                          "[3,2,1,0]) amplifies ~54x at c=0.5. "
                                          "Default 0.95 after extrapolation from "
                                          "6D's 0.9. Adjust downward if HPC "
                                          "effect too weak, upward if oscillating."),
        DeclareLaunchArgument("leader_vel_lpf_tau", default_value="0.0"),
        DeclareLaunchArgument("min_cmd_vel", default_value="0.03"),
        DeclareLaunchArgument("wheel_radius", default_value="0.03"),
        DeclareLaunchArgument("base_radius", default_value="0.11"),
        DeclareLaunchArgument("wheel_max_omega", default_value="20.0"),
        DeclareLaunchArgument("max_linear_accel", default_value="0.25"),
        DeclareLaunchArgument("max_angular_accel", default_value="4.0"),
        DeclareLaunchArgument("max_linear_vel", default_value="1.0"),
        DeclareLaunchArgument("max_angular_vel", default_value="0.5"),
        formation_node,
    ])
```

---

### Task 10: 修改 CMakeLists.txt

**Files:**
- Modify: `CMakeLists.txt`

- [ ] **Step 1: 在 6D Motor target 之后添加 8D target**

在 `install(TARGETS formation_control_node_6d_motor ...)` 之后插入：

```cmake
# ---- 8D Pade 死区增广控制器（doc/pade_deadtime_full.md）-------------------
add_executable(formation_control_node_8d_motor
  src/main_8d_motor.cpp
  src/formation_control_node_8d_motor.cpp
)

target_include_directories(formation_control_node_8d_motor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(formation_control_node_8d_motor
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(formation_control_node_8d_motor
  Eigen3::Eigen
)

install(TARGETS formation_control_node_8d_motor
  DESTINATION lib/${PROJECT_NAME}
)
```

- [ ] **Step 2: 确认没有引入对 motor_predictor.hpp 的链接依赖**

8D 节点不依赖 `motor_predictor.hpp`（不 include，不 link），无需额外操作。

---

### Task 11: Build

- [ ] **Step 1: 构建**

```bash
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

预期: `[100%] Built target formation_control_node_8d_motor`，无编译错误。

- [ ] **Step 2: source**

```bash
source install/setup.bash
```

---

### Task 12: 仿真验证

- [ ] **Step 1: 启动 Gazebo 单机**

```bash
# 终端 A
source /opt/ros/humble/setup.bash
source ~/ros-projects/homo_multirobot_ws/install/setup.bash
ros2 launch homo_multirobot_localization sim_rf2o_ekf_single_robot.launch.py \
  robot_namespace:=/robot2 robot_prefix:=robot2_ \
  robot_x:=2.0 robot_y:=0.0 robot_yaw:=0.0
```

预期: Gazebo 窗口出现一台 mini_omni_robot；`ros2 topic list | grep robot2` 可见 `/robot2/scan`, `/robot2/imu`, `/robot2/odometry/filtered`

- [ ] **Step 2: 启动虚拟 Leader**

```bash
# 终端 B
source /opt/ros/humble/setup.bash
source ~/ros-projects/homo_multirobot_ws/install/setup.bash
ros2 run homo_multirobot_formation_control virtual_leader_circle.py \
  --ros-args -r __ns:=/virtual_leader -p radius:=2.0 -p speed:=0.5
```

预期: 虚线 Leader 开始绕圈，话题 `/virtual_leader/odometry/filtered` 有输出

- [ ] **Step 3: 启动 8D Pade 控制器**

```bash
# 终端 C
source /opt/ros/humble/setup.bash
source ~/ros-projects/homo_multirobot_ws/install/setup.bash
ros2 launch homo_multirobot_formation_control formation_single_follower_8d_motor.launch.py \
  leader_ns:=/virtual_leader follower_ns:=/robot2 Td:=0.22
```

预期: 终端输出 `8D Pade 死区增广编队控制节点已启动`，随后每 1s 输出 `raw=(...), clamped=(...)`。机器人开始跟踪虚拟 Leader 绕圈。

- [ ] **Step 4: 对比验证 — 6D Motor + Smith 基线**

```bash
# 终端 C（替换为 6D Motor 基线）
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_motor.launch.py \
  leader_ns:=/virtual_leader follower_ns:=/robot2 \
  use_smith_predictor:=true smith_Td:=0.22
```

对比两种方案的编队距离 RMSE 和 overshoot。记录 30s 轨迹: `ros2 run homo_multirobot_formation_control record_trajectory.py --ros-args -p mode:=sim -p duration:=30.0 -p controller_node_name:=formation_control_node_8d_motor`

- [ ] **Step 5: 参数扫描**

```bash
# Td 扫描 0.10, 0.15, 0.22, 0.30
for td in 0.10 0.15 0.22 0.30; do
  ros2 launch homo_multirobot_formation_control formation_single_follower_8d_motor.launch.py \
    leader_ns:=/virtual_leader follower_ns:=/robot2 Td:=$td
  sleep 35
  pkill -f formation_control_node_8d_motor
done

# LPC-only 消融对比
ros2 launch homo_multirobot_formation_control formation_single_follower_8d_motor.launch.py \
  leader_ns:=/virtual_leader follower_ns:=/robot2 use_hpc:=false
```

记录每个参数组合的稳态误差和振荡频率，最优 Td 应与实物 ~0.22 接近。

---

### Task 13: 保存计划到项目 docs

- [ ] **Step 1: 创建目录 + 复制计划**

```bash
mkdir -p /home/l1anggmgo/ros-projects/homo_multirobot_ws/src/homo-ctrl-multirobot-ros2/homo_multirobot_formation_control/docs/superpowers/plans
cp /home/l1anggmgo/.claude/plans/virtual-swimming-engelbart.md \
   /home/l1anggmgo/ros-projects/homo_multirobot_ws/src/homo-ctrl-multirobot-ros2/homo_multirobot_formation_control/docs/superpowers/plans/2026-07-21-8d-pade-controller.md
```

---

## 风险与缓解

| 风险 | 缓解 |
|------|------|
| Ackermann C 矩阵接近奇异（特定参数组合） | colPivHouseholderQr → fullPivLu 降级；必要时 sympy 导出闭式解 |
| hpc_c_min=0.95 时 HPC 效果不明显 | 消融对比 LPC-only，确认 HPC 增益；降到 0.90 测试 |
| Td=0.22 仿真表现差（仿真无死区） | 仿真测试主要验证模型退化正确性（Td→0 时收敛到 6D）；实物测试验证死区补偿 |
| 8D expm 比 6D 慢 2.4× | 8×8 矩阵指数在 20Hz 下开销约几十 μs，可忽略；若 ARM 瓶颈则只在编队点切换时重算 HPC |
