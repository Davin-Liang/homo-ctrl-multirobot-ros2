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
  /// @param m_p     安全编队点数量
  /// @param tau_nominal  电机时间常数基准值 (s)
  /// @param tau_min      低速段 τ 下限 (s)，|v_cmd| 小时电机无加速度限幅拖累
  /// @param tau_max      高速段 τ 上限 (s)，加速度限幅使等效 τ 变大
  /// @param v_tau_trans  自适应 τ 的过渡速度 (m/s)，低于此值用 tau_min
  LpcController8DMotor(int m_p = 4, double radius = 2.0, double tol = 0.1,
                       double mass = 2.0,
                       double tau_nominal = 0.43,
                       double Td = 0.22,
                       double omega_d = 0.7, bool use_hpc = true,
                       double control_period = 0.05, double hpc_c_min = 0.95,
                       double hpc_nu = 0.3,
                       double tau_min = 0.25, double tau_max = 0.55,
                       double v_tau_trans = 0.10)
    : m_p_(m_p), radius_(radius), tol_(tol), mass_(mass),
      tau_(tau_nominal), tau_nominal_(tau_nominal),
      tau_min_(tau_min), tau_max_(tau_max), v_tau_trans_(v_tau_trans),
      Td_(Td), omega_d_(omega_d),
      h_(control_period), use_hpc_(use_hpc),
      hpc_c_min_(hpc_c_min), hpc_nu_(hpc_nu)
  {
    if (Td_ < 0.01) {
      throw std::invalid_argument("LpcController8DMotor: Td 不得小于 0.01 s"
                                  "（接近 0 时 1/Td 发散，退化为 6D Motor）");
    }

    const int n = 8, m = 2;

    // 8D Pade 死区-电机全链路模型 A 矩阵（状态顺序同 doc/pade_deadtime_full.md 式(5)）
    // [px, py, vx_cmd, vy_cmd, ωx, ωy, vx_real, vy_real]
    A_.resize(n, n);
    A_ << 0.0, 0.0,  0.0,        0.0,        0.0,             0.0,             1.0,         0.0,
          0.0, 0.0,  0.0,        0.0,        0.0,             0.0,             0.0,         1.0,
          0.0, 0.0,  0.0,        0.0,        0.0,             0.0,             0.0,         0.0,
          0.0, 0.0,  0.0,        0.0,        0.0,             0.0,             0.0,         0.0,
          0.0, 0.0,  1.0,        0.0,       -2.0 / Td_,       0.0,             0.0,         0.0,
          0.0, 0.0,  0.0,        1.0,        0.0,            -2.0 / Td_,       0.0,         0.0,
          0.0, 0.0, -1.0 / tau_,  0.0,        4.0/(tau_*Td_), 0.0,            -1.0 / tau_, 0.0,
          0.0, 0.0,  0.0,       -1.0 / tau_,  0.0,             4.0/(tau_*Td_), 0.0,        -1.0 / tau_;

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

  // --------------------------------------------------------------------------
  // 一次性初始化: 创建编队点集 → 选最近编队点 → 计算 k_lin → 升级到 HPC。
  // 必须在 lpc_calculate 之前调用。x2 的 v_cmd/ω 分量此时应对齐稳态初值。
  // --------------------------------------------------------------------------
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
      // 8D 齐次链权重 [3,3,2,2,1,1,0,0]（每轴 [3,2,1,0] 四阶链）。
      // 硬编码 G0/P/ν，不调 lpc2hpc_nd——后者对每轴双阻尼（-2/Td, -1/τ）
      // 的 8D 系统生成错误参数，导致 HPC 控制方向反转。
      G0_ = Eigen::MatrixXd::Zero(8, 8);
      G0_.diagonal() << 3.0, 3.0, 2.0, 2.0, 1.0, 1.0, 0.0, 0.0;
      nu_ = hpc_nu_;
      Gd_ = Eigen::MatrixXd::Identity(8, 8) + nu_ * G0_;
      P_  = Eigen::MatrixXd::Identity(8, 8);
      // 给 P 的物理状态（位置）加权以适应 hnorm_nd 尺度
      P_(0,0) = P_(1,1) = 1.0;
    }
  }

  // --------------------------------------------------------------------------
  // 每周期控制量计算（20 Hz）。
  //
  // 返回 map 系 {goal_vx_cmd, goal_vy_cmd} (m/s)。偏航控制由调用方负责。
  // --------------------------------------------------------------------------
  std::vector<double> lpc_calculate(const Eigen::VectorXd& x1,
                                    const Eigen::VectorXd& x2)
  {
    check_and_switch_target(x1, x2);

    // 自适应 τ: |v_cmd| 小时的 τ_eff 小（~244ms），大时加速度限幅拖慢（~580ms）。
    // 用 x2(2:3) = v_cmd 分量，低通过渡（同 6D Motor）。
    double vc_mag = std::hypot(x2(2), x2(3));
    double ratio = std::clamp((vc_mag - v_tau_trans_) / (0.3 - v_tau_trans_), 0.0, 1.0);
    tau_ = tau_min_ + ratio * (tau_max_ - tau_min_);
    if (std::abs(tau_ - last_tau_) > 0.001) {
      update_A_tau();                     // A 含 1/τ 项（row 6/7 0-indexed，共 6 个元素）
      last_tau_ = tau_;
    }

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

    // 前向欧拉：v_cmd 自演化（不读测量速度——与 6D Motor 一致）
    double goal_vx_cmd = x2(2) + h_ * u2(0) / mass_;
    double goal_vy_cmd = x2(3) + h_ * u2(1) / mass_;

    return {goal_vx_cmd, goal_vy_cmd};
  }

  double calculate_distance(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2)
  {
    double min_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < m_p_; ++i) {
      double dist = (x2 - x1 - dl_.col(i)).norm();
      min_dist = std::min(min_dist, dist);
    }
    return min_dist;
  }

private:
  // --------------------------------------------------------------------------
  // 编队点切换（带 tol_ 滞后避免频繁跳动）
  // --------------------------------------------------------------------------
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
        // 编队点切换时 HPC 权重不变，G0/P/ν/Gd 保持硬编码值。
        // 只需确认 lpc2hpc_nd 不被调用——8D 双阻尼系统不走通用路径。
      }
    }
  }

  // --------------------------------------------------------------------------
  // 单轴四阶极点配置（p → v_real → ω → v_cmd 链）。
  //
  // 子系统 Ax(4×4), Bx(4×1):
  //   dp/dt      = v_real
  //   dv_cmd/dt  = u/M
  //   dω/dt      = v_cmd - (2/Td)·ω
  //   dv_real/dt = (-v_cmd + (4/Td)·ω - v_real) / τ
  //
  // 使用 Ackermann 公式配置分离快慢极点:
  //   - 两个慢极点 -λ    (物理状态 p, v_real)
  //   - 两个快极点 -10λ  (内部状态 ω, v_cmd)
  //   α_c(s) = (s+λ)²(s+10λ)² = s⁴ + 22λs³ + 141λ²s² + 220λ³s + 100λ⁴
  //
  // 不用四重根 (s+λ)⁴——四阶链物理增益跨度极大（Td/2 ≈ 0.11），
  // 四重根导致 k3/k1 ≈ 7000，ω 误差被极端放大，实物无法工作。
  // 10x 分离增益比 ~9，与 6D Motor 增益比 ~2.5 可比。
  //
  // a 的自适应逻辑沿用 compute_channel_3rd 模式：
  //   a = clamp(-M·e_v/e_p, -wd·M, wd·M), lower bounded by wd·M
  // --------------------------------------------------------------------------
  static std::tuple<double, double, double, double> compute_channel_4th(
      double e_p, double e_v, double M, double tau, double Td, double wd)
  {
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

    // 期望闭环特征多项式: α_c(s) = (s+λ)²(s+10λ)²
    // = s⁴ + 22λs³ + 141λ²s² + 220λ³s + 100λ⁴
    Eigen::Matrix4d I4 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d Ax2 = Ax * Ax;
    Eigen::Matrix4d Ax3 = Ax2 * Ax;
    Eigen::Matrix4d Ax4 = Ax3 * Ax;
    Eigen::Matrix4d alpha_Ax = Ax4
        + 22.0 * lambda * Ax3
        + 141.0 * lambda * lambda * Ax2
        + 220.0 * lambda * lambda * lambda * Ax
        + 100.0 * lambda * lambda * lambda * lambda * I4;

    // Ackermann: 解 C^T · z = e4 → z^T = e4^T · C^{-1}
    Eigen::Vector4d e4;
    e4 << 0.0, 0.0, 0.0, 1.0;

    Eigen::Vector4d z = C.transpose()
                            .colPivHouseholderQr()
                            .solve(e4);

    // K_ack = z^T · α_c(Ax)（负反馈约定）
    // K = -K_ack（正反馈约定，与 compute_channel_3rd 一致）
    Eigen::RowVector4d Kx_raw = -z.transpose() * alpha_Ax;

    return {Kx_raw(0), Kx_raw(1), Kx_raw(2), Kx_raw(3)};
  }

  // --------------------------------------------------------------------------
  // 自适应线性增益: K 为 2×8，x/y 两轴解耦，每轴四阶极点配置。
  // --------------------------------------------------------------------------
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

  // ---- 参数 ----------------------------------------------------------------
  int    m_p_;       double radius_;    double tol_;
  double mass_;      double tau_;             // 当前有效 τ（自适应变化）
  double tau_nominal_;                       // τ 基准值
  double tau_min_;    double tau_max_;       // 自适应 τ 上下限
  double v_tau_trans_;                       // τ 自适应过渡速度 (m/s)
  double last_tau_ = 0.0;                    // 避免每周期重建 A
  double Td_;
  double omega_d_;   double h_;         bool   use_hpc_;
  double hpc_c_min_;  double hpc_nu_;

  // ---- 系统模型 -------------------------------------------------------------
  Eigen::MatrixXd A_;   // 8×8
  Eigen::MatrixXd B_;   // 8×2

  // 自适应 τ: 更新 A 中 row 6/7（0-indexed，即第 7/8 行 v_real）含 1/τ 的 6 个元素
  void update_A_tau() {
    A_(6,2) = -1.0 / tau_;  A_(6,4) = 4.0 / (tau_ * Td_);  A_(6,6) = -1.0 / tau_;
    A_(7,3) = -1.0 / tau_;  A_(7,5) = 4.0 / (tau_ * Td_);  A_(7,7) = -1.0 / tau_;
  }

  // ---- 控制器状态 -----------------------------------------------------------
  Eigen::Matrix<double, 8, Eigen::Dynamic> dl_;  // 编队偏移向量集 (8 × m_p)
  Eigen::VectorXd d_;         // 当前目标偏移 (8)
  Eigen::MatrixXd k_lin_;     // 线性反馈增益 (2×8)
  Eigen::MatrixXd P_;         // Lyapunov 矩阵 (8×8)
  Eigen::MatrixXd G0_;        // 齐次生成元 (8×8)
  Eigen::MatrixXd Gd_;        // 膨胀生成元 (8×8)
  double nu_;                 // 齐次度
};

}  // namespace formation_control
