#pragma once

/// @file 4D Artstein-LQR upper-layer controller.
///
/// The ROS node owns EKF/TF state construction, Artstein compensation,
/// actuator prediction, yaw control, and final cmd_vel limiting. This
/// controller owns only the delay-free 4D DARE-LQR law on x=[px,py,vx,vy]^T.

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace formation_control {

class LqrController4DArtstein {
public:
  struct Params {
    double dt = 0.05;
    double mass = 2.0;
    double formation_radius = 2.0;
    int m_p = 4;
    double tol = 0.1;

    double q_px = 40.0;
    double q_py = 40.0;
    double q_vx = 1.0;
    double q_vy = 1.0;
    double r_ux = 0.02;
    double r_uy = 0.02;

    int dare_max_iter = 10000;
    double dare_tol = 1e-12;
  };

  explicit LqrController4DArtstein(const Params& p)
    : p_(p)
  {
    if (p_.dt <= 0.0) {
      throw std::invalid_argument("LqrController4DArtstein: dt must be positive");
    }
    if (p_.mass <= 0.0) {
      throw std::invalid_argument("LqrController4DArtstein: mass must be positive");
    }
    if (p_.m_p < 1) {
      throw std::invalid_argument("LqrController4DArtstein: m_p must be positive");
    }
    if (p_.r_ux <= 0.0 || p_.r_uy <= 0.0) {
      throw std::invalid_argument("LqrController4DArtstein: R weights must be positive");
    }
    if (p_.dare_max_iter < 1) {
      throw std::invalid_argument("LqrController4DArtstein: dare_max_iter must be positive");
    }

    zoh_matrices(p_.mass, p_.dt, Ad_, Bd_);
    Q_ = Eigen::Vector4d(p_.q_px, p_.q_py, p_.q_vx, p_.q_vy).asDiagonal();
    R_ = Eigen::Vector2d(p_.r_ux, p_.r_uy).asDiagonal();
    solve_dare();
    build_formation_offsets();
  }

  static void zoh_matrices(double mass, double dt,
                           Eigen::Matrix4d& Ad,
                           Eigen::Matrix<double, 4, 2>& Bd)
  {
    Ad << 1.0, 0.0, dt,  0.0,
          0.0, 1.0, 0.0, dt,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0;

    Bd << 0.5 * dt * dt / mass, 0.0,
          0.0, 0.5 * dt * dt / mass,
          dt / mass, 0.0,
          0.0, dt / mass;
  }

  void init(const Eigen::Vector4d& leader_state,
            const Eigen::Vector4d& follower_state)
  {
    target_idx_ = best_target_idx(leader_state, follower_state);
    d_ = dl_.col(target_idx_);
    initialized_ = true;
  }

  /// Returns map-frame velocity command v_pred + dt * u_lqr / mass.
  Eigen::Vector2d compute_velocity_command(const Eigen::Vector4d& leader_state,
                                           const Eigen::Vector4d& follower_state)
  {
    if (!initialized_) {
      init(leader_state, follower_state);
    }
    switch_if_needed(leader_state, follower_state);
    last_error_ = selected_error(leader_state, follower_state);
    last_u_ = -K_ * last_error_;
    return follower_state.tail<2>() + p_.dt * (last_u_ / p_.mass);
  }

  double calculate_distance(const Eigen::Vector4d& leader_state,
                            const Eigen::Vector4d& follower_state) const
  {
    double min_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < p_.m_p; ++i) {
      const double dist = (follower_state - leader_state - dl_.col(i)).norm();
      min_dist = std::min(min_dist, dist);
    }
    return min_dist;
  }

  double best_distance(const Eigen::Vector4d& leader_state,
                       const Eigen::Vector4d& follower_state) const
  {
    return calculate_distance(leader_state, follower_state);
  }

  double current_distance(const Eigen::Vector4d& leader_state,
                          const Eigen::Vector4d& follower_state) const
  {
    return (follower_state - leader_state - d_).norm();
  }

  Eigen::Vector4d selected_error(const Eigen::Vector4d& leader_state,
                                 const Eigen::Vector4d& follower_state) const
  {
    return follower_state - leader_state - d_;
  }

  int target_index() const { return target_idx_; }
  const Eigen::Matrix4d& Ad() const { return Ad_; }
  const Eigen::Matrix<double, 4, 2>& Bd() const { return Bd_; }
  const Eigen::Matrix<double, 2, 4>& K() const { return K_; }
  const Eigen::Matrix4d& P() const { return P_; }
  const Eigen::Matrix4d& Q() const { return Q_; }
  const Eigen::Matrix2d& R() const { return R_; }
  const Eigen::Vector2d& last_u() const { return last_u_; }
  const Eigen::Vector4d& last_error() const { return last_error_; }

private:
  void solve_dare()
  {
    P_ = Q_;
    bool converged = false;

    for (int i = 0; i < p_.dare_max_iter; ++i) {
      const Eigen::Matrix2d S = R_ + Bd_.transpose() * P_ * Bd_;
      const Eigen::Matrix<double, 2, 4> gain =
          S.ldlt().solve(Bd_.transpose() * P_ * Ad_);
      const Eigen::Matrix4d next =
          Ad_.transpose() * P_ * Ad_ - Ad_.transpose() * P_ * Bd_ * gain + Q_;

      const double diff = (next - P_).cwiseAbs().maxCoeff();
      P_ = 0.5 * (next + next.transpose());
      if (diff < p_.dare_tol) {
        converged = true;
        break;
      }
    }

    if (!converged) {
      throw std::runtime_error("LqrController4DArtstein: DARE did not converge");
    }

    const Eigen::Matrix2d S = R_ + Bd_.transpose() * P_ * Bd_;
    K_ = S.ldlt().solve(Bd_.transpose() * P_ * Ad_);
  }

  void build_formation_offsets()
  {
    dl_.resize(4, p_.m_p);
    for (int i = 0; i < p_.m_p; ++i) {
      const double angle = 2.0 * M_PI * i / p_.m_p;
      dl_(0, i) = -p_.formation_radius * std::cos(angle);
      dl_(1, i) = -p_.formation_radius * std::sin(angle);
      dl_(2, i) = 0.0;
      dl_(3, i) = 0.0;
    }
    d_ = dl_.col(0);
  }

  int best_target_idx(const Eigen::Vector4d& leader_state,
                      const Eigen::Vector4d& follower_state) const
  {
    int best_idx = 0;
    double best_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < p_.m_p; ++i) {
      const double dist = (follower_state - leader_state - dl_.col(i)).norm();
      if (dist < best_dist) {
        best_dist = dist;
        best_idx = i;
      }
    }
    return best_idx;
  }

  void switch_if_needed(const Eigen::Vector4d& leader_state,
                        const Eigen::Vector4d& follower_state)
  {
    const int best_idx = best_target_idx(leader_state, follower_state);
    const double best_dist = (follower_state - leader_state - dl_.col(best_idx)).norm();
    const double current_dist = (follower_state - leader_state - d_).norm();
    if (best_dist + p_.tol < current_dist) {
      target_idx_ = best_idx;
      d_ = dl_.col(target_idx_);
    }
  }

  Params p_;
  Eigen::Matrix4d Ad_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 4, 2> Bd_ = Eigen::Matrix<double, 4, 2>::Zero();
  Eigen::Matrix4d Q_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix2d R_ = Eigen::Matrix2d::Identity();
  Eigen::Matrix4d P_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 2, 4> K_ = Eigen::Matrix<double, 2, 4>::Zero();

  Eigen::MatrixXd dl_;
  Eigen::Vector4d d_ = Eigen::Vector4d::Zero();
  int target_idx_ = 0;
  bool initialized_ = false;
  Eigen::Vector2d last_u_ = Eigen::Vector2d::Zero();
  Eigen::Vector4d last_error_ = Eigen::Vector4d::Zero();
};

}  // namespace formation_control
