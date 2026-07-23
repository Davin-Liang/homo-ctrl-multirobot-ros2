#pragma once

/// @file 4D Artstein-HPC 齐次编队控制器。
///
/// 状态 x = [px, py, vx_real, vy_real]^T（全部 map 系）
///   - px, py:            位置（TF + EKF）
///   - vx_real, vy_real:  电机实际速度（EKF 测量）
///
/// 控制输入 u = v^cmd = [vx_cmd, vy_cmd]^T（map 系速度指令）。
/// v^cmd 不是状态——控制器直接输出速度指令，不经前向欧拉积分。
///
/// 系统方程（执行器死区 + 一阶滞后）:
///   dp/dt      = v_real
///   dv_real/dt = (v^cmd(t−Td) − v_real) / τ
///
/// Artstein 约简将输入时延系统等价变换为无时延系统:
///   z = x + ∫ e^{A(t−s−Td)} B v^cmd(s) ds
///   ż = A z + B_eff v^cmd(t),   B_eff = e^{−A Td} B
///
/// HPC 直接设计在等价无时延系统 (A, B_eff) 上。
/// 自适应 τ 变化时 B_eff 和 HPC 参数同步重算——无隐性近似。
///
/// 编队点逻辑与 4D/6D 控制器相同（离散多边形 + tol 滞后切换）。

#include <cmath>
#include <tuple>
#include <vector>
#include <deque>
#include <limits>
#include <algorithm>
#include <stdexcept>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "homo_multirobot_formation_control/types_nd.hpp"
#include "homo_multirobot_formation_control/lpc2hpc_nd.hpp"
#include "homo_multirobot_formation_control/hnorm_nd.hpp"

namespace formation_control {

class LpcController4DArtstein {
public:
  /// @param m_p        安全编队点数量
  /// @param radius     编队圆半径 (m)
  /// @param tol        切换滞后容差 (m)
  /// @param mass       速度通道等效增益（调参用，量纲归一，默认 1.0）
  /// @param tau_nominal  电机时间常数基准值 (s)
  /// @param omega_d    期望闭环带宽
  /// @param use_hpc    false 时退化为纯 LPC
  /// @param control_period  控制周期 (s)
  /// @param hpc_c_min  HPC warp clamp 下界
  /// @param tau_min    自适应 τ 下限 (s)
  /// @param tau_max    自适应 τ 上限 (s)
  /// @param v_tau_trans  自适应 τ 过渡速度 (m/s)
  /// @param Td         死区时延 (s)
  LpcController4DArtstein(int m_p = 4, double radius = 2.0, double tol = 0.1,
                          double mass = 1.0, double tau_nominal = 0.43,
                          double omega_d = 0.7, bool use_hpc = true,
                          double control_period = 0.05, double hpc_c_min = 0.9,
                          double tau_min = 0.25, double tau_max = 0.55,
                          double v_tau_trans = 0.10, double Td = 0.22)
    : m_p_(m_p), radius_(radius), tol_(tol), mass_(mass),
      tau_(tau_nominal), tau_nominal_(tau_nominal),
      tau_min_(tau_min), tau_max_(tau_max), v_tau_trans_(v_tau_trans),
      omega_d_(omega_d), h_(control_period), use_hpc_(use_hpc),
      hpc_c_min_(hpc_c_min), Td_(Td)
  {
    if (tau_ < 0.05) {
      throw std::invalid_argument("LpcController4DArtstein: tau 不得小于 0.05 s");
    }
    if (Td_ < 0.0) {
      throw std::invalid_argument("LpcController4DArtstein: Td 不得为负");
    }

    const int n = 4, m = 2;

    // ---- 4D 执行器模型 (A, B) ------------------------------------------------
    build_AB();

    // ---- Artstein 积分核预计算 -------------------------------------------------
    build_artstein_kernels();

    // ---- 控制器状态 -----------------------------------------------------------
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
  // Artstein 积分核缓冲区大小
  // --------------------------------------------------------------------------
  int artstein_buffer_size() const { return N_; }

  // --------------------------------------------------------------------------
  // 计算 Artstein 积分项 I(t) = Σ e^{A(kh−Td)} B · vcmd(t−kh) · w_k
  //
  // vcmd_history[0] = v^cmd(t)（最新），vcmd_history[N-1] = v^cmd(t−(N-1)h)（最旧）
  // 返回 4D 向量 I(t)
  // --------------------------------------------------------------------------
  Eigen::Vector4d compute_artstein_integral(
      const std::deque<Eigen::Vector2d>& vcmd_history) const
  {
    Eigen::Vector4d I = Eigen::Vector4d::Zero();
    int len = static_cast<int>(vcmd_history.size());
    for (int k = 0; k < N_ && k < len; ++k) {
      I += artstein_kernels_[k] * vcmd_history[k] * weights_[k];
    }
    return I;
  }

  // --------------------------------------------------------------------------
  // 一次性初始化: 创建编队点集 → 选最近编队点 → 计算 k_lin → 升级到 HPC。
  // 必须在 lpc_calculate 之前调用。
  // z1, z2 为 leader 和 follower 的 Artstein 预测状态 (4D)。
  // --------------------------------------------------------------------------
  void controller_initial(const Eigen::VectorXd& z1, const Eigen::VectorXd& z2)
  {
    // 领航者周围半径 radius_ 的圆上均匀分布 m_p_ 个编队偏移向量
    dl_.resize(4, m_p_);
    dl_.setZero();
    for (int i = 0; i < m_p_; ++i) {
      double angle = 2.0 * M_PI * i / m_p_;
      dl_(0, i) = -radius_ * std::cos(angle);
      dl_(1, i) = -radius_ * std::sin(angle);
    }

    // 选择距离当前跟随者位置最近的编队点
    int best_idx = 0;
    double best_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < m_p_; ++i) {
      double dist = (z2 - z1 - dl_.col(i)).norm();
      if (dist < best_dist) {
        best_dist = dist;
        best_idx = i;
      }
    }
    d_ = dl_.col(best_idx);

    Eigen::VectorXd e = z2 - z1 - d_;
    k_lin_ = calculate_klin(e);

    if (use_hpc_) {
      auto res = lpc2hpc_nd(A_, B_eff_, k_lin_);
      if (res.G0.isZero(1e-12)) {
        throw std::runtime_error("4D Artstein 控制器初始化失败: lpc2hpc 返回零结果。");
      }
      G0_ = res.G0;
      P_  = res.P;
      nu_ = res.nu_min;
      Gd_ = Eigen::MatrixXd::Identity(4, 4) + nu_ * G0_;

      // 诊断: 打印 HPC 参数
      std::cout << "[LpcController4DArtstein] HPC init: nu=" << nu_
                << " nu_max=" << res.nu_max
                << " |λ(G0)|_max=" << G0_.diagonal().cwiseAbs().maxCoeff()
                << " cond(P)="
                << Eigen::JacobiSVD<Eigen::MatrixXd>(P_).singularValues()(0)
                   / Eigen::JacobiSVD<Eigen::MatrixXd>(P_).singularValues()(3)
                << std::endl;
    }
  }

  // --------------------------------------------------------------------------
  // 每周期控制量计算 (~20 Hz)。
  //
  // 返回 map 系 {vx_cmd, vy_cmd} (m/s)。偏航控制由调用方负责。
  //
  // z1, z2 为 leader 和 follower 的 Artstein 预测状态 (4D)。
  //
  // 控制律（同 6D Motor，但 u 即为 v^cmd，不再积分）:
  //   e  = z2 − z1 − d
  //   c  = clamp(hnorm(e, Gd, P), c_min, 1)
  //   u  = c^(1+ν) · K · expm(Gd·(1−ln c)) · e
  //   v_cmd = u   （直接输出，不经前向欧拉积分）
  // --------------------------------------------------------------------------
  std::vector<double> lpc_calculate(const Eigen::VectorXd& z1,
                                    const Eigen::VectorXd& z2)
  {
    check_and_switch_target(z1, z2);

    // 自适应 τ: |v_cmd| 小时 τ_eff 小，大时加速度限幅拖慢。
    // 用 z2(2:3) = v_real (Artstein 预测的实际速度) 的幅值近似驱动。
    double vmag = std::hypot(z2(2), z2(3));
    double ratio = std::clamp((vmag - v_tau_trans_) / (0.3 - v_tau_trans_), 0.0, 1.0);
    tau_ = tau_min_ + ratio * (tau_max_ - tau_min_);
    if (std::abs(tau_ - last_tau_) > 0.001) {
      build_AB();                    // 更新 A, B, B_eff（build_AB 内部调用 build_beff）
      build_artstein_kernels();      // 重算积分核（用更新后的 A, B）

      // 重算 k_lin_（极点配置依赖 B_eff 的通道分量，τ 变 → B_eff 变 → K 变）
      Eigen::VectorXd e = z2 - z1 - d_;
      k_lin_ = calculate_klin(e);

      if (use_hpc_) {
        auto res = lpc2hpc_nd(A_, B_eff_, k_lin_);  // 重算 HPC
        if (!res.G0.isZero(1e-12)) {
          G0_ = res.G0;
          P_  = res.P;
          nu_ = res.nu_min;
          Gd_ = Eigen::MatrixXd::Identity(4, 4) + nu_ * G0_;
        }
      }
      last_tau_ = tau_;
    }

    Eigen::VectorXd e = z2 - z1 - d_;

    Eigen::Vector2d u;
    if (use_hpc_) {
      double nx = hnorm_nd(e, Gd_, P_);
      double c = std::clamp(nx, hpc_c_min_, 1.0);
      double log_c = std::log(c);
      Eigen::MatrixXd expm_g = (Gd_ * (1.0 - log_c)).exp();
      Eigen::VectorXd warped_e = expm_g * e;
      u = std::pow(c, 1.0 + nu_) * (k_lin_ * warped_e);
    } else {
      u = k_lin_ * e;  // 纯线性比例控制
    }

    // u 即为 map 系速度指令 v^cmd（不积分）
    return {u(0), u(1)};
  }

  // 跟随者到最近编队点的距离（调试/度量用）
  double calculate_distance(const Eigen::VectorXd& z1, const Eigen::VectorXd& z2)
  {
    double min_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < m_p_; ++i) {
      double dist = (z2 - z1 - dl_.col(i)).norm();
      min_dist = std::min(min_dist, dist);
    }
    return min_dist;
  }

private:
  // ---- 构建 4D 系统矩阵 (A, B) ------------------------------------------------
  void build_AB()
  {
    A_.resize(4, 4);
    A_ << 0, 0,  1,         0,
          0, 0,  0,         1,
          0, 0, -1.0/tau_,  0,
          0, 0,  0,        -1.0/tau_;

    B_.resize(4, 2);
    B_ << 0, 0,
          0, 0,
          1.0/tau_, 0,
          0, 1.0/tau_;

    build_beff();
  }

  // ---- 构建 B_eff = exp(-A * Td) * B -----------------------------------------
  void build_beff()
  {
    B_eff_ = (-A_ * Td_).exp() * B_;
  }

  // ---- 构建 Artstein 积分核矩阵 ------------------------------------------------
  void build_artstein_kernels()
  {
    N_ = std::max(1, static_cast<int>(std::ceil(Td_ / h_)));

    weights_.resize(N_);
    for (int k = 0; k < N_ - 1; ++k) {
      weights_[k] = h_;
    }
    weights_[N_ - 1] = Td_ - (N_ - 1) * h_;  // 截断权重

    artstein_kernels_.resize(N_);
    for (int k = 0; k < N_; ++k) {
      double arg = k * h_ - Td_;
      artstein_kernels_[k] = (A_ * arg).exp() * B_;  // 4×2
    }
  }

  // --------------------------------------------------------------------------
  // 编队点切换（带 tol_ 滞后避免频繁跳动）。切换后重算 k_lin + HPC（用 B_eff）。
  // --------------------------------------------------------------------------
  void check_and_switch_target(const Eigen::VectorXd& z1, const Eigen::VectorXd& z2)
  {
    double min_dist = std::numeric_limits<double>::max();
    int best_idx = 0;
    for (int i = 0; i < m_p_; ++i) {
      double dist = (z2 - z1 - dl_.col(i)).norm();
      if (dist < min_dist) {
        min_dist = dist;
        best_idx = i;
      }
    }

    double current_dist = (z2 - z1 - d_).norm();
    if (min_dist + tol_ < current_dist) {
      std::cout << "[LpcController4DArtstein] 编队点切换 -> idx " << best_idx
                << " (err " << current_dist << " -> " << min_dist << " m)" << std::endl;
      d_ = dl_.col(best_idx);

      Eigen::VectorXd e = z2 - z1 - d_;
      k_lin_ = calculate_klin(e);

      if (use_hpc_) {
        auto res = lpc2hpc_nd(A_, B_eff_, k_lin_);
        if (!res.G0.isZero(1e-12)) {
          G0_ = res.G0;
          P_  = res.P;
          nu_ = res.nu_min;
          Gd_ = Eigen::MatrixXd::Identity(4, 4) + nu_ * G0_;
        }
      }
    }
  }

  // --------------------------------------------------------------------------
  // 二阶极点配置（p → v_real 链 + τ 滞后），以 B_eff 为输入矩阵。
  //
  // 使用 B_eff = [[b1,0],[0,b1],[b3,0],[0,b3]] 的通道分量:
  //   每轴闭环: A_ch + [b1;b3]·[k1,k2]
  //   特征多项式 s² − tr·s + det = 0
  //     tr  = b1·k1 + b3·k2 − 1/τ
  //     det = −k1·(b1/τ + b3)
  //   对 (s+λ)² = s² + 2λs + λ² 配置:
  //     k1 = −λ² / (b1/τ + b3)
  //     k2 = (−2λ + 1/τ − b1·k1) / b3
  //
  // 注意: B_eff 的位置行分量 b1 < 0 (Td>0 时)，若用 B 的公式
  //   (k1=−λ²τ, k2=1−2λτ) 会导致 k1·b1 > 0 正反馈，τ 较小时闭环不稳定。
  //   修正后的公式直接对 (A, B_eff) 做极点配置，消除此问题。
  //
  // 返回 {k1, k2}
  // --------------------------------------------------------------------------
  static std::pair<double, double> compute_channel_2nd(
      double e_p, double e_v, double M, double tau, double wd,
      double b1, double b3)
  {
    // 防超调比值: a = −M · e_v / e_p，clamp 防止位置误差极小时增益爆炸
    double val = (std::abs(e_p) > 1e-6) ? -M * e_v / e_p : 0.0;
    double max_ratio = wd * M;
    val = std::clamp(val, -max_ratio, max_ratio);
    double a = std::max(val, wd * M);

    // 二重极点 (s+λ)², λ = a/M ≥ wd
    double lambda = a / M;

    // 修正后的极点配置公式（以 B_eff 为输入矩阵）
    double denom = b1 / tau + b3;
    if (std::abs(denom) < 1e-12) {
      // 退化情况 (Td=0 即 B_eff=B 时 b1=0, b3=1/τ, denom=1/τ)
      // 回退到 B 公式: k1=−λ²τ, k2=1−2λτ
      return {-lambda * lambda * tau, 1.0 - 2.0 * lambda * tau};
    }
    double k1 = -lambda * lambda / denom;
    double k2 = (-2.0 * lambda + 1.0 / tau - b1 * k1) / b3;
    return {k1, k2};
  }

  // 自适应线性增益: K 为 2×4，x/y 两轴解耦，每轴二阶极点配置。
  // 使用 B_eff 的通道分量进行精确极点配置（非 B 的近似公式）。
  Eigen::MatrixXd calculate_klin(const Eigen::VectorXd& e)
  {
    // B_eff 通道分量: x 通道用 (0,0) 和 (2,0), y 通道用 (1,1) 和 (3,1)
    double b1x = B_eff_(0,0), b3x = B_eff_(2,0);
    double b1y = B_eff_(1,1), b3y = B_eff_(3,1);

    auto [k1_x, k2_x] = compute_channel_2nd(e(0), e(2), mass_, tau_, omega_d_, b1x, b3x);
    auto [k1_y, k2_y] = compute_channel_2nd(e(1), e(3), mass_, tau_, omega_d_, b1y, b3y);

    Eigen::MatrixXd K(2, 4);
    K << k1_x, 0,    k2_x, 0,
         0,    k1_y, 0,    k2_y;
    return K;
  }

  // ---- 参数 ----------------------------------------------------------------
  int    m_p_;       // 编队点数量
  double radius_;    // 编队圆半径 (m)
  double tol_;       // 切换滞后容差 (m)
  double mass_;      // 速度通道等效增益（调参）
  double tau_;            // 当前有效 τ（自适应变化）
  double tau_nominal_;    // τ 基准值
  double tau_min_;        // 低速 τ 下限
  double tau_max_;        // 高速 τ 上限
  double v_tau_trans_;    // τ 自适应过渡速度 (m/s)
  double last_tau_ = 0.0; // 避免每周期重建
  double omega_d_;        // 期望阻尼带宽
  double h_;              // 控制周期 (s)
  double Td_;             // 死区时延 (s)

  // ---- 系统模型（4D 执行器）-------------------------------------------------
  Eigen::MatrixXd A_;        // 4×4
  Eigen::MatrixXd B_;        // 4×2
  Eigen::MatrixXd B_eff_;    // 4×2 = exp(-A·Td)·B

  // ---- Artstein 积分 -------------------------------------------------------
  int N_;                                     // 缓冲长度
  std::vector<double> weights_;               // 截断权重 w_k
  std::vector<Eigen::MatrixXd> artstein_kernels_;  // e^{A(kh−Td)}·B, 4×2 each

  // ---- 控制器状态 -----------------------------------------------------------
  Eigen::Matrix<double, 4, Eigen::Dynamic> dl_;  // 编队偏移向量集 (4 × m_p)
  Eigen::VectorXd d_;        // 当前目标偏移向量 (4D)
  Eigen::MatrixXd k_lin_;    // 线性反馈增益 (2×4)
  Eigen::MatrixXd P_;        // Lyapunov 矩阵 (4×4)
  Eigen::MatrixXd G0_;       // 齐次生成元 (4×4)
  Eigen::MatrixXd Gd_;       // 膨胀生成元 (4×4)
  double nu_;                // 齐次度
  bool   use_hpc_;           // false 时退化为纯 LPC
  double hpc_c_min_;         // hnorm clamp 下界
};

}  // namespace formation_control
