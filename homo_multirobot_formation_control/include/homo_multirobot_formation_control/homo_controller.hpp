#pragma once

/// @file 齐次控制器后端（C++ / Eigen）
/// 移植自 Python homo_ctrl_using_ann1.py。
/// 运行时使用二分法 hnorm 替代原始 ANN 逼近器（避免 cvxpy 依赖）。

#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "homo_multirobot_formation_control/types.hpp"
#include "homo_multirobot_formation_control/lpc2hpc.hpp"
#include "homo_multirobot_formation_control/hnorm.hpp"

namespace formation_control {

class LpcController {
public:
  // m_p: 安全编队点数量   radius: 编队圆半径 (m)
  // tol: 编队点切换容差 (m)   mass: 双重积分器模型质量（调参用，非物理质量）
  LpcController(int m_p = 4, double radius = 2.0, double tol = 0.1, double mass = 2.0,
               double omega_d = 1.5, bool use_hpc = true, double hpc_c_min = 0.5,
               double control_period = 0.1, double initial_min_lambda = 0.0,
               double switch_min_lambda = 0.0)
    : m_p_(m_p), radius_(radius), tol_(tol), mass_(mass), omega_d_(omega_d), use_hpc_(use_hpc),
      hpc_c_min_(hpc_c_min), h_(control_period),
      initial_min_lambda_(initial_min_lambda > 0.0 ? initial_min_lambda : omega_d * mass),
      switch_min_lambda_(switch_min_lambda > 0.0 ? switch_min_lambda : omega_d * mass)
  {
    // 2D 双重积分器: x = [px, py, vx, vy]
    A_ << 0, 0, 1, 0,
          0, 0, 0, 1,
          0, 0, 0, 0,
          0, 0, 0, 0;
    B_ << 0, 0,
          0, 0,
          1.0 / mass_, 0,
          0, 1.0 / mass_;

    d_.setZero();
    k_lin_.setZero();
    P_.setIdentity();
    nu_ = 0.0;
    Gd_.setIdentity();
    G0_.setZero();

    last_cmd_vel_ << 0.0, 0.0;
  }

  // --------------------------------------------------------------------------
  // 一次性初始化: 创建编队点集 → 选最近的编队点 → 计算 k_lin → 升级到 HPC。
  // 必须在 lpc_calculate 之前调用。
  // --------------------------------------------------------------------------
  void controller_initial(const Vec4d& x1, const Vec4d& x2)
  {
    // 在领航者周围半径为 radius_ 的圆上均匀分布 m_p_ 个编队偏移向量
    dl_.resize(4, m_p_);
    for (int i = 0; i < m_p_; ++i) {
      double angle = 2.0 * M_PI * i / m_p_;
      dl_(0, i) = -radius_ * std::cos(angle);
      dl_(1, i) = -radius_ * std::sin(angle);
      dl_(2, i) = 0.0;
      dl_(3, i) = 0.0;
    }

    // 选择距离当前跟随者位置最近的编队点
    int best_idx = 0;
    double best_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < m_p_; ++i) {
      double dist = (x2 - x1 - dl_.col(i)).norm();
      if (dist < best_dist) {
        best_dist = dist;
        best_idx = i;
      }
    }
    d_ = dl_.col(best_idx);

    Vec4d e = x2 - x1 - d_;
    k_lin_ = calculate_klin(e, initial_min_lambda_);

    if (use_hpc_) {
      auto res = lpc2hpc(A_, B_, k_lin_);
      if (res.G0.isZero(1e-12)) {
        throw std::runtime_error("控制器初始化失败: lpc2hpc 返回零结果。");
      }
      G0_ = res.G0;
      P_  = res.P;
      nu_ = res.nu_min;
      Gd_ = Mat4d::Identity() + nu_ * G0_;

      Eigen::EigenSolver<Eigen::Matrix4d> es_g0(G0_);
      std::cout << "[HPC 4D init] nu=" << nu_ << " nu_max=" << res.nu_max
                << " G0_eig=[" << es_g0.eigenvalues()(0).real()
                << "," << es_g0.eigenvalues()(1).real()
                << "," << es_g0.eigenvalues()(2).real()
                << "," << es_g0.eigenvalues()(3).real() << "]"
                << std::endl;
    }
  }

  // --------------------------------------------------------------------------
  // 每周期控制量计算（调用频率 20–50 Hz）。
  //
  // 返回 {vx_cmd, vy_cmd} (m/s)。偏航控制由调用方负责。
  //
  // 控制律:
  //   e  = x2 − x1 − d                     (编队误差)
  //   c  = clamp(hnorm(e, Gd, P), 0.5, 1)
  //   u2 = c^(1+nu) · K · expm(Gd·(1−ln c)) · e
  //   v  = v_current + h · u2 / mass       (前向欧拉积分)
  // --------------------------------------------------------------------------
  std::vector<double> lpc_calculate(const Vec4d& x1, const Vec4d& x2)
  {
    Eigen::Vector2d u2 = accel_calculate(x1, x2);

    // Forward Euler: v_desired = v_current + h * a
    Vec4d goal_x2 = x2 + h_ * (A_ * x2 + B_ * u2);

    // LPF is computed but not used, preserving the Python baseline behavior.
    double alpha_lpf = 0.3;
    double smooth_vx = (1.0 - alpha_lpf) * last_cmd_vel_(0) + alpha_lpf * goal_x2(2);
    double smooth_vy = (1.0 - alpha_lpf) * last_cmd_vel_(1) + alpha_lpf * goal_x2(3);
    last_cmd_vel_ << smooth_vx, smooth_vy;

    (void)smooth_vx;
    (void)smooth_vy;
    return {goal_x2(2), goal_x2(3)};
  }

  Eigen::Vector2d accel_calculate(const Vec4d& x1, const Vec4d& x2)
  {
    check_and_switch_target(x1, x2);

    Vec4d e = x2 - x1 - d_;

    Eigen::Vector2d u2;
    if (use_hpc_) {
      double nx = hnorm(e, Gd_, P_);
      double c = std::clamp(nx, hpc_c_min_, 1.0);
      double log_c  = std::log(c);
      Mat4d  expm_g = (Gd_ * (1.0 - log_c)).exp();
      Vec4d  warped_e = expm_g * e;
      u2 = std::pow(c, 1.0 + nu_) * (k_lin_ * warped_e);

      static int dbg_cnt_4d = 0;
      if (++dbg_cnt_4d % 20 == 0) {
        std::cout << "[HPC 4D diag] |e|=" << e.norm() << " nx=" << nx
                  << " c=" << c << " nu=" << nu_
                  << " warp_scale=" << std::pow(c, 1.0+nu_)
                  << " |we|/|e|=" << warped_e.norm()/std::max(e.norm(),1e-6)
                  << std::endl;
      }
    } else {
      u2 = k_lin_ * e;  // Pure linear proportional control.
    }

    return u2;
  }

  // 跟随者到最近编队点的距离（调试/度量用）
  double calculate_distance(const Vec4d& x1, const Vec4d& x2)
  {
    double min_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < m_p_; ++i) {
      double dist = (x2 - x1 - dl_.col(i)).norm();
      min_dist = std::min(min_dist, dist);
    }
    return min_dist;
  }

  int target_index() const
  {
    for (int i = 0; i < m_p_; ++i) {
      if ((d_ - dl_.col(i)).norm() < 1e-12) {
        return i;
      }
    }
    return 0;
  }

  Vec4d target_offset() const
  {
    return d_;
  }

  double current_distance(const Vec4d& x1, const Vec4d& x2) const
  {
    return (x2 - x1 - d_).norm();
  }

  double best_distance(const Vec4d& x1, const Vec4d& x2) const
  {
    double best_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < m_p_; ++i) {
      double dist = (x2 - x1 - dl_.col(i)).norm();
      if (dist < best_dist) {
        best_dist = dist;
      }
    }
    return best_dist;
  }

  Vec4d selected_error(const Vec4d& x1, const Vec4d& x2) const
  {
    return x2 - x1 - d_;
  }

private:
  // --------------------------------------------------------------------------
  // 编队点切换（带 tol_ 滞后避免频繁跳动）
  // --------------------------------------------------------------------------
  void check_and_switch_target(const Vec4d& x1, const Vec4d& x2)
  {
    double min_dist = std::numeric_limits<double>::max();
    int best_idx = 0;
    for (int i = 0; i < m_p_; ++i) {
      double dist = (x2 - x1 - dl_.col(i)).norm();
      if (dist < min_dist) {
        min_dist = dist;
        best_idx = i;
      }
    }

    double current_dist = (x2 - x1 - d_).norm();
    if (min_dist + tol_ < current_dist) {
      d_ = dl_.col(best_idx);

      // 切换到新编队点后重新计算增益和 HPC 参数
      Vec4d e = x2 - x1 - d_;
      k_lin_ = calculate_klin(e, switch_min_lambda_);

      if (use_hpc_) {
        auto res = lpc2hpc(A_, B_, k_lin_);
        if (!res.G0.isZero(1e-12)) {
          G0_ = res.G0;
          P_  = res.P;
          nu_ = res.nu_min;
          Gd_ = Mat4d::Identity() + nu_ * G0_;
        }
      }
    }
  }

  // --------------------------------------------------------------------------
  // 自适应线性增益计算（齐次控制论文 Lemma 1 的实现）。
  //
  // K = [K1, K2]，K1 作用于位置，K2 作用于速度。
  // 防超调设计: 特征值随 omega_d · mass 自适应缩放，
  // e_i_v / e_i_p 比值被 clamp 防止噪声下增益爆炸。
  // --------------------------------------------------------------------------
  Mat24d calculate_klin(const Vec4d& e, double min_lambda)
  {

    // 防超调比值: a_i = −m · e_i_v / e_i_p
    double val_a = (std::abs(e(0)) > 1e-6) ? -mass_ * e(2) / e(0) : 0.0;
    double val_b = (std::abs(e(1)) > 1e-6) ? -mass_ * e(3) / e(1) : 0.0;

    // 限制比值范围，防止位置误差极小时（AMCL 微扰动）增益爆炸
    double max_ratio = std::max(min_lambda, 1e-6);
    val_a = std::clamp(val_a, -max_ratio, max_ratio);
    val_b = std::clamp(val_b, -max_ratio, max_ratio);

    // 特征值至少为 min_lambda，和 Python 数值仿真的 min_lambda 对齐
    double a = std::max(val_a, min_lambda);
    double b = std::max(val_b, min_lambda);

    double k2_00 = -2.0 * a;
    double k2_11 = -2.0 * b;
    double k1_00 = a * (k2_00 + a) / mass_;
    double k1_11 = b * (k2_11 + b) / mass_;

    Mat24d result;
    result << k1_00, 0,      k2_00, 0,
              0,      k1_11, 0,      k2_11;
    return result;
  }

  // ---- 参数 ----------------------------------------------------------------
  int    m_p_;       // 编队点数量
  double radius_;    // 编队圆半径 (m)
  double tol_;       // 切换滞后容差 (m)
  double mass_;      // 模型质量（调参）
  double omega_d_;   // 期望阻尼带宽

  // ---- 系统模型（双重积分器） ----------------------------------------------
  Mat4d  A_;
  Mat42d B_;

  // ---- 控制器状态 -----------------------------------------------------------
  Eigen::Matrix<double, 4, Eigen::Dynamic> dl_;  // 编队偏移向量集 (4 × m_p)
  Vec4d         d_;           // 当前目标偏移向量
  Mat24d        k_lin_;       // 线性反馈增益
  Mat4d         P_;           // Lyapunov 矩阵
  Mat4d         G0_;          // 齐次生成元
  Mat4d         Gd_;          // 膨胀生成元 (I + nu * G0)
  double        nu_;          // 齐次度
  bool          use_hpc_;     // false 时退化为纯 LPC
  double        hpc_c_min_;   // hnorm clamp 下界
  double        h_;           // 控制周期 (s)
  double        initial_min_lambda_;  // 初始增益下界
  double        switch_min_lambda_;   // 编队点切换后增益下界

  Eigen::Vector2d last_cmd_vel_;
};

}  // namespace formation_control
