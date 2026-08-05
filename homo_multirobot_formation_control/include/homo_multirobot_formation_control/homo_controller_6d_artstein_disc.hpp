#pragma once

/// @file 6D Artstein Disc 的 HPC 核心。
///
/// 本类只实现 6D Disc 的齐次控制核心，不包含 Artstein 执行器预测。
/// 输入状态仍为 6D Disc 状态:
///   x = [px, py, theta, vx_body, vy_body, omega]^T
///
/// 与 LpcController6DDisc 的主要区别：
///   - 线性增益使用 4D 同款 initial/switch min_lambda 参数；
///   - 每次重算 HPC 前检查 A+B*K 的 Hurwitz 裕度；
///   - HPC 重算失败时保留上一组稳定参数，若没有稳定参数则退回线性控制。

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>

#include "homo_multirobot_formation_control/hnorm_nd.hpp"
#include "homo_multirobot_formation_control/lpc2hpc_nd.hpp"

namespace formation_control {

class LpcController6DArtsteinDisc {
public:
  LpcController6DArtsteinDisc(double radius = 2.0, double mass = 2.0,
                              double inertia = 1.0, int m_p = 4,
                              double tol = 0.1, bool use_hpc = true,
                              double control_period = 0.05,
                              double hpc_c_min = 0.5,
                              double initial_min_lambda = 1.0,
                              double switch_min_lambda = 4.0,
                              double hpc_vel_threshold = 0.3,
                              double hpc_yaw_threshold = 0.3,
                              double stability_margin = 0.01)
  : radius_(radius), mass_(mass), inertia_(inertia), m_p_(m_p), tol_(tol),
    use_hpc_(use_hpc), h_(control_period), hpc_c_min_(hpc_c_min),
    initial_min_lambda_(initial_min_lambda),
    switch_min_lambda_(switch_min_lambda),
    min_lambda_(initial_min_lambda),
    hpc_vel_threshold_(hpc_vel_threshold),
    hpc_yaw_threshold_(hpc_yaw_threshold),
    stability_margin_(stability_margin)
  {
    if (mass_ <= 0.0 || inertia_ <= 0.0) {
      throw std::invalid_argument("6D Artstein Disc: mass/inertia must be positive");
    }
    if (h_ <= 0.0) {
      throw std::invalid_argument("6D Artstein Disc: control_period must be positive");
    }
    if (hpc_c_min_ <= 0.0 || hpc_c_min_ > 1.0) {
      throw std::invalid_argument("6D Artstein Disc: hpc_c_min must be in (0, 1]");
    }
    if (m_p_ <= 0) {
      throw std::invalid_argument("6D Artstein Disc: m_p must be positive");
    }

    A_.resize(6, 6);
    build_A(0.0, 0.0, 0.0);

    B_.resize(6, 3);
    B_ << 0, 0, 0,
          0, 0, 0,
          0, 0, 0,
          1.0 / mass_, 0, 0,
          0, 1.0 / mass_, 0,
          0, 0, 1.0 / inertia_;

    k_lin_.resize(3, 6);
    k_lin_.setZero();
    P_.resize(6, 6);
    P_.setIdentity();
    G0_.resize(6, 6);
    G0_.setZero();
    Gd_.resize(6, 6);
    Gd_.setIdentity();

    dl_.resize(6, m_p_);
    d_.resize(6);
    d_.setZero();
    for (int i = 0; i < m_p_; ++i) {
      double angle = 2.0 * M_PI * i / static_cast<double>(m_p_);
      dl_(0, i) = -radius_ * std::cos(angle);
      dl_(1, i) = -radius_ * std::sin(angle);
      dl_(2, i) = 0.0;
      dl_(3, i) = 0.0;
      dl_(4, i) = 0.0;
      dl_(5, i) = 0.0;
    }
  }

  void controller_initial(const Eigen::VectorXd& x1,
                          const Eigen::VectorXd& x2)
  {
    select_nearest(x1, x2);
    update_A(x1);

    double dtheta, cos_dt, sin_dt;
    Eigen::VectorXd e = compute_error(x1, x2, dtheta, cos_dt, sin_dt);
    min_lambda_ = initial_min_lambda_;
    k_lin_ = calculate_klin(e);
    rebuild_hpc(true);

    last_hpc_leader_vel_ << x1(3), x1(4), x1(5);
    last_dtheta_ = dtheta;
    initialized_ = true;
  }

  std::vector<double> lpc_calculate(const Eigen::VectorXd& x1,
                                    const Eigen::VectorXd& x2)
  {
    if (!initialized_) {
      return {0.0, 0.0, 0.0};
    }

    update_A(x1);
    check_and_switch_target(x1, x2);

    double dtheta, cos_dt, sin_dt;
    Eigen::VectorXd e = compute_error(x1, x2, dtheta, cos_dt, sin_dt);

    Eigen::Vector3d leader_vel(x1(3), x1(4), x1(5));
    bool vel_changed = (leader_vel - last_hpc_leader_vel_).norm() > hpc_vel_threshold_;
    bool yaw_changed = std::abs(wrap_angle(dtheta - last_dtheta_)) > hpc_yaw_threshold_;
    if (!use_hpc_ || vel_changed || yaw_changed) {
      k_lin_ = calculate_klin(e);
    }
    if (use_hpc_ && (vel_changed || yaw_changed)) {
      rebuild_hpc(false);
      last_hpc_leader_vel_ = leader_vel;
      last_dtheta_ = dtheta;
    }

    Eigen::VectorXd u_L;
    if (use_hpc_ && hpc_valid_) {
      double nx = hnorm_nd(e, Gd_, P_);
      double c = std::clamp(nx, hpc_c_min_, 1.0);
      Eigen::MatrixXd expm_g = (Gd_ * (1.0 - std::log(c))).exp();
      u_L = std::pow(c, 1.0 + nu_) * (k_lin_ * expm_g * e);
    } else {
      u_L = k_lin_ * e;
    }

    double ux_f =  u_L(0) * cos_dt + u_L(1) * sin_dt;
    double uy_f = -u_L(0) * sin_dt + u_L(1) * cos_dt;

    double goal_vx = x2(3) + h_ * ux_f / mass_;
    double goal_vy = x2(4) + h_ * uy_f / mass_;
    double goal_omega = x2(5) + h_ * u_L(2) / inertia_;
    return {goal_vx, goal_vy, goal_omega};
  }

  double distance_to_current_target(const Eigen::VectorXd& x1,
                                    const Eigen::VectorXd& x2)
  {
    double dtheta, cos_dt, sin_dt;
    Eigen::VectorXd e = compute_error(x1, x2, dtheta, cos_dt, sin_dt);
    return e.head<2>().norm();
  }

  int target_idx() const { return target_idx_; }
  int hpc_fallback_count() const { return hpc_fallback_count_; }
  bool hpc_valid() const { return hpc_valid_; }
  double last_max_real_eig() const { return last_max_real_eig_; }

private:
  static double wrap_angle(double a)
  {
    return std::atan2(std::sin(a), std::cos(a));
  }

  void build_A(double vx_l, double vy_l, double omega_l)
  {
    A_ << 0,        omega_l, -vy_l, 1, 0, 0,
         -omega_l, 0,        vx_l, 0, 1, 0,
          0,       0,        0,    0, 0, 1,
          0,       0,        0,    0, 0, 0,
          0,       0,        0,    0, 0, 0,
          0,       0,        0,    0, 0, 0;
  }

  void update_A(const Eigen::VectorXd& x1)
  {
    build_A(x1(3), x1(4), x1(5));
  }

  Eigen::VectorXd compute_error(const Eigen::VectorXd& x1,
                                const Eigen::VectorXd& x2,
                                double& dtheta,
                                double& cos_dt,
                                double& sin_dt) const
  {
    return compute_error_for_target(x1, x2, d_, dtheta, cos_dt, sin_dt);
  }

  Eigen::VectorXd compute_error_for_target(const Eigen::VectorXd& x1,
                                           const Eigen::VectorXd& x2,
                                           const Eigen::VectorXd& target,
                                           double& dtheta,
                                           double& cos_dt,
                                           double& sin_dt) const
  {
    dtheta = wrap_angle(x2(2) - x1(2));
    cos_dt = std::cos(dtheta);
    sin_dt = std::sin(dtheta);

    double dpx = x2(0) - x1(0);
    double dpy = x2(1) - x1(1);
    double cos_tl = std::cos(x1(2));
    double sin_tl = std::sin(x1(2));
    double dex =  dpx * cos_tl + dpy * sin_tl;
    double dey = -dpx * sin_tl + dpy * cos_tl;

    double vx_f_in_L = x2(3) * cos_dt - x2(4) * sin_dt;
    double vy_f_in_L = x2(3) * sin_dt + x2(4) * cos_dt;

    Eigen::VectorXd e(6);
    e << dex - target(0),
         dey - target(1),
         wrap_angle(dtheta - target(2)),
         vx_f_in_L - x1(3) - target(3),
         vy_f_in_L - x1(4) - target(4),
         x2(5) - x1(5) - target(5);
    return e;
  }

  Eigen::VectorXd compute_error_with_target(const Eigen::VectorXd& x1,
                                            const Eigen::VectorXd& x2,
                                            int idx) const
  {
    double dtheta, cos_dt, sin_dt;
    return compute_error_for_target(x1, x2, dl_.col(idx), dtheta, cos_dt, sin_dt);
  }

  void select_nearest(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2)
  {
    int best = 0;
    double best_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < m_p_; ++i) {
      double dist = compute_error_with_target(x1, x2, i).norm();
      if (dist < best_dist) {
        best_dist = dist;
        best = i;
      }
    }
    target_idx_ = best;
    d_ = dl_.col(best);
  }

  void check_and_switch_target(const Eigen::VectorXd& x1,
                               const Eigen::VectorXd& x2)
  {
    int best = 0;
    double best_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < m_p_; ++i) {
      double dist = compute_error_with_target(x1, x2, i).norm();
      if (dist < best_dist) {
        best_dist = dist;
        best = i;
      }
    }

    double current_dist = compute_error_with_target(x1, x2, target_idx_).norm();
    if (best != target_idx_ && best_dist + tol_ < current_dist) {
      target_idx_ = best;
      d_ = dl_.col(best);
      min_lambda_ = switch_min_lambda_;

      double dtheta, cos_dt, sin_dt;
      Eigen::VectorXd e = compute_error(x1, x2, dtheta, cos_dt, sin_dt);
      k_lin_ = calculate_klin(e);
      rebuild_hpc(false);
      last_hpc_leader_vel_ << x1(3), x1(4), x1(5);
      last_dtheta_ = dtheta;
    }
  }

  Eigen::MatrixXd calculate_klin(const Eigen::VectorXd& e) const
  {
    auto compute_channel = [this](double e_p, double e_v, double M) {
      double val = (std::abs(e_p) > 1e-6) ? -M * e_v / e_p : 0.0;
      double max_ratio = std::max(min_lambda_, 1e-6);
      val = std::clamp(val, -max_ratio, max_ratio);
      double a = std::max(val, min_lambda_);
      double k2 = -2.0 * a;
      double k1 = a * (k2 + a) / M;
      return std::make_pair(k1, k2);
    };

    auto [k1_x, k2_x] = compute_channel(e(0), e(3), mass_);
    auto [k1_y, k2_y] = compute_channel(e(1), e(4), mass_);
    auto [k1_t, k2_t] = compute_channel(e(2), e(5), inertia_);

    Eigen::MatrixXd K(3, 6);
    K << k1_x, 0,    0,    k2_x, 0,    0,
         0,    k1_y, 0,    0,    k2_y, 0,
         0,    0,    k1_t, 0,    0,    k2_t;
    return K;
  }

  bool closed_loop_hurwitz()
  {
    Eigen::EigenSolver<Eigen::MatrixXd> es(A_ + B_ * k_lin_);
    double max_real = -std::numeric_limits<double>::max();
    for (int i = 0; i < es.eigenvalues().size(); ++i) {
      max_real = std::max(max_real, std::real(es.eigenvalues()(i)));
    }
    last_max_real_eig_ = max_real;
    return max_real < -stability_margin_;
  }

  void rebuild_hpc(bool throw_on_failure)
  {
    if (!use_hpc_) {
      hpc_valid_ = false;
      return;
    }

    if (!closed_loop_hurwitz()) {
      ++hpc_fallback_count_;
      if (throw_on_failure && !hpc_valid_) {
        throw std::runtime_error("6D Artstein Disc: A+B*K is not Hurwitz");
      }
      return;
    }

    auto res = lpc2hpc_nd(A_, B_, k_lin_);
    if (res.G0.isZero(1e-12)) {
      ++hpc_fallback_count_;
      if (throw_on_failure && !hpc_valid_) {
        throw std::runtime_error("6D Artstein Disc: lpc2hpc_nd failed");
      }
      return;
    }

    G0_ = res.G0;
    P_ = res.P;
    nu_ = res.nu_min;
    Gd_ = Eigen::MatrixXd::Identity(6, 6) + nu_ * G0_;
    hpc_valid_ = true;
  }

  double radius_, mass_, inertia_;
  int m_p_;
  double tol_;
  bool use_hpc_;
  double h_;
  double hpc_c_min_;
  double initial_min_lambda_;
  double switch_min_lambda_;
  double min_lambda_;
  double hpc_vel_threshold_;
  double hpc_yaw_threshold_;
  double stability_margin_;

  Eigen::MatrixXd A_, B_;
  Eigen::MatrixXd k_lin_, P_, G0_, Gd_;
  Eigen::MatrixXd dl_;
  Eigen::VectorXd d_;
  double nu_ = 0.0;
  bool hpc_valid_ = false;
  bool initialized_ = false;
  int hpc_fallback_count_ = 0;
  double last_max_real_eig_ = 0.0;
  int target_idx_ = 0;
  Eigen::Vector3d last_hpc_leader_vel_ = Eigen::Vector3d::Zero();
  double last_dtheta_ = 0.0;
};

}  // namespace formation_control
