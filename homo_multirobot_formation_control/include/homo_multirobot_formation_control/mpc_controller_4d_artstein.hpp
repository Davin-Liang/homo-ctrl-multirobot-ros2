#pragma once

/// @file 4D Artstein-MPC upper-layer controller.
///
/// The ROS node owns EKF/TF state construction, Artstein compensation, actuator
/// prediction, yaw control, and final cmd_vel limiting. This controller owns
/// only the delay-free 4D linear MPC on x=[px,py,vx,vy]^T.

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "homo_multirobot_formation_control/osqp_interface.hpp"

namespace formation_control {

class MpcController4DArtstein {
public:
  struct Params {
    int N = 30;
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
    double terminal_factor = 10.0;

    double max_linear_vel = 0.5;
    double max_linear_accel = 0.4;

    int osqp_max_iter = 4000;
    double osqp_eps_abs = 1e-3;
    double osqp_eps_rel = 1e-3;
    bool osqp_polish = true;
  };

  explicit MpcController4DArtstein(const Params& p);

  static void zoh_matrices(double mass, double dt,
                           Eigen::Matrix4d& Ad,
                           Eigen::Matrix<double, 4, 2>& Bd);

  void init(const Eigen::Vector4d& leader_state,
            const Eigen::Vector4d& follower_state);

  /// Returns map-frame velocity command x_{1|0}.tail<2>(), not u0.
  Eigen::Vector2d compute_velocity_command(const Eigen::Vector4d& leader_state,
                                           const Eigen::Vector4d& follower_state);

  double calculate_distance(const Eigen::Vector4d& leader_state,
                            const Eigen::Vector4d& follower_state) const;
  double best_distance(const Eigen::Vector4d& leader_state,
                       const Eigen::Vector4d& follower_state) const;
  double current_distance(const Eigen::Vector4d& leader_state,
                          const Eigen::Vector4d& follower_state) const;
  Eigen::Vector4d selected_error(const Eigen::Vector4d& leader_state,
                                 const Eigen::Vector4d& follower_state) const;

  int target_index() const { return target_idx_; }
  int last_status() const { return last_status_; }
  const char* last_status_string() const { return last_status_string_.c_str(); }
  double last_solve_time_ms() const { return last_solve_time_ms_; }
  Eigen::Vector2d last_u0() const { return last_u0_; }
  Eigen::Vector4d last_x1_pred() const { return last_x1_pred_; }
  Eigen::Vector4d last_ref0_error() const { return last_ref0_error_; }

private:
  static constexpr int n_ = 4;
  static constexpr int m_ = 2;

  static int x_idx(int k) { return k * (n_ + m_); }
  static int u_idx(int k) { return k * (n_ + m_) + n_; }
  int n_vars() const { return (p_.N + 1) * n_ + p_.N * m_; }
  int n_cons() const { return p_.N * n_ + n_ + p_.N * m_ + p_.N * m_; }

  void build_formation_offsets();
  int best_target_idx(const Eigen::Vector4d& leader_state,
                      const Eigen::Vector4d& follower_state) const;
  void switch_if_needed(const Eigen::Vector4d& leader_state,
                        const Eigen::Vector4d& follower_state);
  Eigen::Vector4d reference_state(const Eigen::Vector4d& leader_state, int k) const;
  void build_qp(const Eigen::Vector4d& leader_state,
                const Eigen::Vector4d& follower_state);

  Params p_;
  Eigen::Matrix4d Ad_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 4, 2> Bd_ = Eigen::Matrix<double, 4, 2>::Zero();

  Eigen::MatrixXd dl_;
  Eigen::Vector4d d_ = Eigen::Vector4d::Zero();
  int target_idx_ = 0;
  bool initialized_ = false;

  std::unique_ptr<OsqpSolver> solver_;
  Eigen::SparseMatrix<double> P_sparse_;
  Eigen::SparseMatrix<double> A_sparse_;
  Eigen::VectorXd q_;
  Eigen::VectorXd l_;
  Eigen::VectorXd u_;

  int last_status_ = -1;
  std::string last_status_string_ = "unavailable";
  double last_solve_time_ms_ = 0.0;
  Eigen::Vector2d last_u0_ = Eigen::Vector2d::Zero();
  Eigen::Vector4d last_x1_pred_ = Eigen::Vector4d::Zero();
  Eigen::Vector4d last_ref0_error_ = Eigen::Vector4d::Zero();
};

inline void MpcController4DArtstein::zoh_matrices(
    double mass, double dt, Eigen::Matrix4d& Ad,
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

inline MpcController4DArtstein::MpcController4DArtstein(const Params& p)
  : p_(p), solver_(std::make_unique<OsqpSolver>())
{
  if (p_.N < 1) {
    throw std::invalid_argument("MpcController4DArtstein: horizon N must be positive");
  }
  if (p_.dt <= 0.0) {
    throw std::invalid_argument("MpcController4DArtstein: dt must be positive");
  }
  if (p_.mass <= 0.0) {
    throw std::invalid_argument("MpcController4DArtstein: mass must be positive");
  }
  if (p_.m_p < 1) {
    throw std::invalid_argument("MpcController4DArtstein: m_p must be positive");
  }

  zoh_matrices(p_.mass, p_.dt, Ad_, Bd_);
  build_formation_offsets();
  solver_->configure(p_.osqp_max_iter, p_.osqp_eps_abs, p_.osqp_eps_rel, p_.osqp_polish);
}

inline void MpcController4DArtstein::build_formation_offsets()
{
  dl_.resize(n_, p_.m_p);
  for (int i = 0; i < p_.m_p; ++i) {
    const double angle = 2.0 * M_PI * i / p_.m_p;
    dl_(0, i) = -p_.formation_radius * std::cos(angle);
    dl_(1, i) = -p_.formation_radius * std::sin(angle);
    dl_(2, i) = 0.0;
    dl_(3, i) = 0.0;
  }
  d_ = dl_.col(0);
}

inline void MpcController4DArtstein::init(
    const Eigen::Vector4d& leader_state,
    const Eigen::Vector4d& follower_state)
{
  target_idx_ = best_target_idx(leader_state, follower_state);
  d_ = dl_.col(target_idx_);
  initialized_ = true;
}

inline int MpcController4DArtstein::best_target_idx(
    const Eigen::Vector4d& leader_state,
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

inline void MpcController4DArtstein::switch_if_needed(
    const Eigen::Vector4d& leader_state,
    const Eigen::Vector4d& follower_state)
{
  const int best_idx = best_target_idx(leader_state, follower_state);
  const double best_dist = (follower_state - leader_state - dl_.col(best_idx)).norm();
  const double current_dist = (follower_state - leader_state - d_).norm();
  if (best_dist + p_.tol < current_dist) {
    target_idx_ = best_idx;
    d_ = dl_.col(best_idx);
  }
}

inline Eigen::Vector4d MpcController4DArtstein::reference_state(
    const Eigen::Vector4d& leader_state, int k) const
{
  Eigen::Vector4d ref = leader_state;
  ref.head<2>() += static_cast<double>(k) * p_.dt * leader_state.tail<2>();
  return ref + d_;
}

inline double MpcController4DArtstein::calculate_distance(
    const Eigen::Vector4d& leader_state,
    const Eigen::Vector4d& follower_state) const
{
  double min_dist = std::numeric_limits<double>::max();
  for (int i = 0; i < p_.m_p; ++i) {
    const double dist = (follower_state - leader_state - dl_.col(i)).norm();
    min_dist = std::min(min_dist, dist);
  }
  return min_dist;
}

inline double MpcController4DArtstein::best_distance(
    const Eigen::Vector4d& leader_state,
    const Eigen::Vector4d& follower_state) const
{
  return calculate_distance(leader_state, follower_state);
}

inline double MpcController4DArtstein::current_distance(
    const Eigen::Vector4d& leader_state,
    const Eigen::Vector4d& follower_state) const
{
  return (follower_state - leader_state - d_).norm();
}

inline Eigen::Vector4d MpcController4DArtstein::selected_error(
    const Eigen::Vector4d& leader_state,
    const Eigen::Vector4d& follower_state) const
{
  return follower_state - leader_state - d_;
}

inline void MpcController4DArtstein::build_qp(
    const Eigen::Vector4d& leader_state,
    const Eigen::Vector4d& follower_state)
{
  using T = Eigen::Triplet<double>;
  std::vector<T> P_triplets;
  std::vector<T> A_triplets;

  const int N = p_.N;
  const int nz = n_vars();
  const int dyn_rows = N * n_;
  const int x0_row0 = dyn_rows;
  const int inp_row0 = x0_row0 + n_;
  const int vel_row0 = inp_row0 + N * m_;
  const int total_rows = n_cons();

  const Eigen::Vector4d Qd =
      (Eigen::Vector4d() << p_.q_px, p_.q_py, p_.q_vx, p_.q_vy).finished();
  const Eigen::Vector2d Rd =
      (Eigen::Vector2d() << p_.r_ux, p_.r_uy).finished();

  for (int k = 0; k < N; ++k) {
    const int xo = x_idx(k);
    const int uo = u_idx(k);
    for (int i = 0; i < n_; ++i) {
      P_triplets.emplace_back(xo + i, xo + i, 2.0 * Qd(i));
    }
    for (int i = 0; i < m_; ++i) {
      P_triplets.emplace_back(uo + i, uo + i, 2.0 * Rd(i));
    }
  }
  {
    const int xNo = x_idx(N);
    for (int i = 0; i < n_; ++i) {
      P_triplets.emplace_back(xNo + i, xNo + i, 2.0 * p_.terminal_factor * Qd(i));
    }
  }
  P_sparse_.resize(nz, nz);
  P_sparse_.setFromTriplets(P_triplets.begin(), P_triplets.end());

  q_.setZero(nz);
  for (int k = 0; k < N; ++k) {
    const Eigen::Vector4d ref = reference_state(leader_state, k);
    const int xo = x_idx(k);
    for (int i = 0; i < n_; ++i) {
      q_(xo + i) = -2.0 * Qd(i) * ref(i);
    }
  }
  {
    const Eigen::Vector4d ref = reference_state(leader_state, N);
    const int xNo = x_idx(N);
    for (int i = 0; i < n_; ++i) {
      q_(xNo + i) = -2.0 * p_.terminal_factor * Qd(i) * ref(i);
    }
  }

  for (int k = 0; k < N; ++k) {
    const int r0 = k * n_;
    const int xk = x_idx(k);
    const int uk = u_idx(k);
    const int xk1 = x_idx(k + 1);

    for (int r = 0; r < n_; ++r) {
      for (int c = 0; c < n_; ++c) {
        if (std::abs(Ad_(r, c)) > 1e-14) {
          A_triplets.emplace_back(r0 + r, xk + c, Ad_(r, c));
        }
      }
      for (int c = 0; c < m_; ++c) {
        if (std::abs(Bd_(r, c)) > 1e-14) {
          A_triplets.emplace_back(r0 + r, uk + c, Bd_(r, c));
        }
      }
      A_triplets.emplace_back(r0 + r, xk1 + r, -1.0);
    }
  }

  for (int r = 0; r < n_; ++r) {
    A_triplets.emplace_back(x0_row0 + r, r, 1.0);
  }

  for (int k = 0; k < N; ++k) {
    const int r0 = inp_row0 + k * m_;
    const int uo = u_idx(k);
    for (int c = 0; c < m_; ++c) {
      A_triplets.emplace_back(r0 + c, uo + c, 1.0);
    }
  }

  for (int k = 1; k <= N; ++k) {
    const int r0 = vel_row0 + (k - 1) * m_;
    const int xo = x_idx(k);
    A_triplets.emplace_back(r0 + 0, xo + 2, 1.0);
    A_triplets.emplace_back(r0 + 1, xo + 3, 1.0);
  }

  A_sparse_.resize(total_rows, nz);
  A_sparse_.setFromTriplets(A_triplets.begin(), A_triplets.end());

  l_.resize(total_rows);
  u_.resize(total_rows);

  for (int k = 0; k < N; ++k) {
    for (int r = 0; r < n_; ++r) {
      l_(k * n_ + r) = 0.0;
      u_(k * n_ + r) = 0.0;
    }
  }

  for (int r = 0; r < n_; ++r) {
    l_(x0_row0 + r) = follower_state(r);
    u_(x0_row0 + r) = follower_state(r);
  }

  const double umax = p_.mass * p_.max_linear_accel;
  for (int k = 0; k < N; ++k) {
    const int r0 = inp_row0 + k * m_;
    l_(r0 + 0) = -umax;
    u_(r0 + 0) = umax;
    l_(r0 + 1) = -umax;
    u_(r0 + 1) = umax;
  }

  for (int k = 1; k <= N; ++k) {
    const int r0 = vel_row0 + (k - 1) * m_;
    l_(r0 + 0) = -p_.max_linear_vel;
    u_(r0 + 0) = p_.max_linear_vel;
    l_(r0 + 1) = -p_.max_linear_vel;
    u_(r0 + 1) = p_.max_linear_vel;
  }
}

inline Eigen::Vector2d MpcController4DArtstein::compute_velocity_command(
    const Eigen::Vector4d& leader_state,
    const Eigen::Vector4d& follower_state)
{
  using namespace std::chrono;

  if (!initialized_) {
    init(leader_state, follower_state);
  }

  switch_if_needed(leader_state, follower_state);
  last_ref0_error_ = follower_state - reference_state(leader_state, 0);

  build_qp(leader_state, follower_state);
  solver_->setup(P_sparse_, q_, A_sparse_, l_, u_);

  const auto t0 = steady_clock::now();
  const bool ok = solver_->solve();
  const auto t1 = steady_clock::now();

  last_status_ = solver_->status();
  last_status_string_ = solver_->status_string();
  last_solve_time_ms_ = duration<double, std::milli>(t1 - t0).count();

  (void)ok;
  if (!solver_->has_usable_solution()) {
    last_u0_.setZero();
    last_x1_pred_ = follower_state;
    return follower_state.tail<2>();
  }

  const Eigen::VectorXd z = solver_->solution();
  last_u0_ = z.segment<m_>(u_idx(0));
  last_x1_pred_ = z.segment<n_>(x_idx(1));
  return last_x1_pred_.tail<2>();
}

}  // namespace formation_control
