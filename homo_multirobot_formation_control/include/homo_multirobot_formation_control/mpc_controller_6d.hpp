#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <cmath>
#include <memory>
#include <vector>

#include "homo_multirobot_formation_control/osqp_interface.hpp"

namespace formation_control {

/// Single-point linearized MPC for 6D kinematic formation control.
///
/// State:  x = [px, py, θ, vx_body, vy_body, ω]^T   (map pos/yaw + body velocities)
/// Input:  u = [a_x^b, a_y^b, α]^T                    (body accelerations)
class MpcController6D {
public:
  struct Params {
    int    N  = 40;  // 2.0s horizon @ 20Hz
    double dt = 0.05;
    double formation_radius = 2.0;  // safety-circle radius (same as HPC 6D)
    double formation_offset_x = -2.0;  // fixed offset in leader body-frame (for velocity consistency)
    double formation_offset_y =  0.0;

    // Q weights — position tracking
    double q_px = 5.0, q_py = 5.0, q_theta = 20.0;
    // Q weights — velocity damping (higher = smoother, less overshoot)
    double q_vx = 0.5, q_vy = 0.5, q_omega = 2.0;
    // R weights — input cost (lower = more aggressive correction)
    double r_ax = 0.01, r_ay = 0.01, r_alpha = 0.01;
    double terminal_factor = 10.0;

    // constraints
    double max_linear_accel  = 2.0;
    double max_angular_accel = 6.0;
    double max_linear_vel    = 1.0;
    double max_angular_vel   = 2.0;
  };

  explicit MpcController6D(const Params& p);

  /// @return u = [ax, ay, α]^T in body-frame. Returns zero vector on failure.
  Eigen::Vector3d compute_control(const Eigen::VectorXd& x_leader,
                                   const Eigen::VectorXd& x_follower);

  /// Exposed for logging.
  int last_solve_status() const { return last_status_; }
  int last_solve_iter()   const { return last_iter_;   }
  double last_solve_time_ms() const { return last_time_ms_; }

private:
  static constexpr int n_ = 6;
  static constexpr int m_ = 3;

  // ---- index helpers --------------------------------------------------------
  static int x_idx(int k) { return k * (n_ + m_); }
  static int u_idx(int k) { return k * (n_ + m_) + n_; }
  int n_vars()     const { return (p_.N + 1) * n_ + p_.N * m_; }
  int n_cons()     const { return p_.N * n_ + n_ + p_.N * m_; }

  // ---- math helpers ---------------------------------------------------------
  static double wrap(double angle);
  static double unwrap_ref(double theta_ref_raw, double theta_current);

  // ---- algorithmic steps ----------------------------------------------------
  void linearize(const Eigen::VectorXd& x_follower);
  Eigen::VectorXd predict_leader(const Eigen::VectorXd& xl, int k) const;
  Eigen::VectorXd reference_state(const Eigen::VectorXd& xl,
                                   const Eigen::VectorXd& x_follower, int k) const;
  void build_qp(const Eigen::VectorXd& x_leader,
                const Eigen::VectorXd& x_follower);

  Params p_;

  Eigen::MatrixXd A_d_, B_d_;
  Eigen::VectorXd C_d_;

  std::unique_ptr<OsqpSolver> solver_;
  Eigen::SparseMatrix<double> P_sparse_, A_sparse_;
  Eigen::VectorXd q_, l_, u_;

  int    last_status_ = -1;
  int    last_iter_   = 0;
  double last_time_ms_ = 0.0;
};

// ============================================================================
// Angle helpers
// ============================================================================
inline double MpcController6D::wrap(double a)
{
  return a - 2.0 * M_PI * std::floor((a + M_PI) / (2.0 * M_PI));
}

inline double MpcController6D::unwrap_ref(double theta_ref_raw, double theta_current)
{
  return theta_current + wrap(theta_ref_raw - theta_current);
}

// ============================================================================
// Index helpers (static, used in build_qp)
// ============================================================================

// x_idx and u_idx are defined inline in the class body above.

// ============================================================================
// Constructor
// ============================================================================
inline MpcController6D::MpcController6D(const Params& p)
  : p_(p)
  , A_d_(n_, n_), B_d_(n_, m_), C_d_(n_)
  , solver_(std::make_unique<OsqpSolver>())
{
  A_d_.setZero(); B_d_.setZero(); C_d_.setZero();
}

// ============================================================================
// Linearization
// ============================================================================
inline void MpcController6D::linearize(const Eigen::VectorXd& x)
{
  const double theta = x(2);
  const double vx    = x(3);
  const double vy    = x(4);
  const double c     = std::cos(theta);
  const double s     = std::sin(theta);

  // ---- A_c = ∂f/∂x ---------------------------------------------------------
  Eigen::MatrixXd A_c = Eigen::MatrixXd::Zero(n_, n_);
  A_c(0, 2) = -vx * s - vy * c;
  A_c(0, 3) =  c;
  A_c(0, 4) = -s;
  A_c(1, 2) =  vx * c - vy * s;
  A_c(1, 3) =  s;
  A_c(1, 4) =  c;
  A_c(2, 5) = 1.0;

  A_d_ = Eigen::MatrixXd::Identity(n_, n_) + p_.dt * A_c;

  // ---- B (constant) ---------------------------------------------------------
  Eigen::MatrixXd B_c = Eigen::MatrixXd::Zero(n_, m_);
  B_c(3, 0) = 1.0;
  B_c(4, 1) = 1.0;
  B_c(5, 2) = 1.0;
  B_d_ = p_.dt * B_c;

  // ---- affine term C_d = dt·(f(x,0) − A_c·x) --------------------------------
  Eigen::VectorXd f0(n_);
  f0 << vx * c - vy * s,
        vx * s + vy * c,
        x(5),
        0.0, 0.0, 0.0;
  C_d_ = p_.dt * (f0 - A_c * x);
}

// ============================================================================
// Leader prediction (constant body-velocity)
// ============================================================================
inline Eigen::VectorXd MpcController6D::predict_leader(const Eigen::VectorXd& xl,
                                                        int k) const
{
  const double t_k   = k * p_.dt;
  const double th0   = xl(2);
  const double vx    = xl(3);
  const double vy    = xl(4);
  const double omega = xl(5);
  const double th_t  = th0 + omega * t_k;

  Eigen::VectorXd x(n_);

  if (std::abs(omega) < 1e-8) {
    x(0) = xl(0) + t_k * (vx * std::cos(th0) - vy * std::sin(th0));
    x(1) = xl(1) + t_k * (vx * std::sin(th0) + vy * std::cos(th0));
  } else {
    const double ds = std::sin(th_t) - std::sin(th0);
    const double dc = std::cos(th_t) - std::cos(th0);
    x(0) = xl(0) + (vx * ds + vy * dc) / omega;
    x(1) = xl(1) + (vx * (-dc) + vy * ds) / omega;
  }

  x(2) = th_t;
  x(3) = vx;
  x(4) = vy;
  x(5) = omega;

  return x;
}

// ============================================================================
// Reference state (coord transform: map→body for velocity)
// ============================================================================
inline Eigen::VectorXd MpcController6D::reference_state(
    const Eigen::VectorXd& xl,
    const Eigen::VectorXd& x_follower,
    int k) const
{
  const Eigen::VectorXd xl_pred = predict_leader(xl, k);
  const double th_l = xl_pred(2);
  const double c_l  = std::cos(th_l);
  const double s_l  = std::sin(th_l);

  // ---- position: boundary projection (HPC strategy) ------------------------
  // Project follower onto the safety circle around the leader.
  const double dx = xl_pred(0) - x_follower(0);
  const double dy = xl_pred(1) - x_follower(1);
  const double dist = std::hypot(dx, dy);
  const double inv = (dist > 1e-6) ? (1.0 / dist) : 0.0;
  const double px_ref = xl_pred(0) - p_.formation_radius * dx * inv;
  const double py_ref = xl_pred(1) - p_.formation_radius * dy * inv;

  // Dynamic offset from leader to ref point (leader body frame), for velocity ref
  const double off_x_map = px_ref - xl_pred(0);
  const double off_y_map = py_ref - xl_pred(1);
  const double off_x_body =  c_l * off_x_map + s_l * off_y_map;
  const double off_y_body = -s_l * off_x_map + c_l * off_y_map;

  // ---- yaw: match leader heading, unwrapped to follower's branch -----------
  const double th_ref_raw = th_l;
  const double th_ref = unwrap_ref(th_ref_raw, x_follower(2));

  // ---- reference velocity in body frame ------------------------------------
  // Leader body velocity → map frame
  const double v_lx_map = c_l * xl_pred(3) - s_l * xl_pred(4);
  const double v_ly_map = s_l * xl_pred(3) + c_l * xl_pred(4);

  // ω_L × d: dynamic boundary-projection offset in leader body frame → map
  const double omg = xl_pred(5);
  const double rot_x = c_l * (-omg * off_y_body) - s_l * (omg * off_x_body);
  const double rot_y = s_l * (-omg * off_y_body) + c_l * (omg * off_x_body);

  const double v_ref_map_x = v_lx_map + rot_x;
  const double v_ref_map_y = v_ly_map + rot_y;

  // Map → follower body frame (NOT reference frame).
  // Use follower's actual heading so body-velocity reference tells the follower
  // what its own body axes must do to match the map-frame motion.
  const double c_f = std::cos(x_follower(2));
  const double s_f = std::sin(x_follower(2));
  const double v_ref_bx =  c_f * v_ref_map_x + s_f * v_ref_map_y;
  const double v_ref_by = -s_f * v_ref_map_x + c_f * v_ref_map_y;

  Eigen::VectorXd x_ref(n_);
  x_ref << px_ref, py_ref, th_ref, v_ref_bx, v_ref_by, omg;
  return x_ref;
}

// ============================================================================
// Shared QP build helper — P, A, l, u matrices (reference-independent)
// ============================================================================
inline void MpcController6D::build_qp(const Eigen::VectorXd& x_leader,
                                       const Eigen::VectorXd& x_follower)
{
  std::vector<Eigen::VectorXd> x_ref(p_.N + 1);
  for (int k = 0; k <= p_.N; ++k)
    x_ref[k] = reference_state(x_leader, x_follower, k);

  // ---- build QP from x_ref --------------------------------------------------
  using T = Eigen::Triplet<double>;
  std::vector<T> P_triplets, A_triplets;

  const int N     = p_.N;
  const int nz    = n_vars();
  const int nrows = n_cons();

  // ---- Q, R diagonals -------------------------------------------------------
  const Eigen::VectorXd Qd = (Eigen::VectorXd(n_) <<
    p_.q_px, p_.q_py, p_.q_theta, p_.q_vx, p_.q_vy, p_.q_omega).finished();
  const Eigen::VectorXd Rd = (Eigen::VectorXd(m_) <<
    p_.r_ax, p_.r_ay, p_.r_alpha).finished();

  // ---- P: block diagonal  2Q … 2R …  2·tf·Q ---------------------------------
  for (int k = 0; k < N; ++k) {
    const int xo = x_idx(k), uo = u_idx(k);
    for (int i = 0; i < n_; ++i) P_triplets.emplace_back(xo + i, xo + i, 2.0 * Qd(i));
    for (int i = 0; i < m_; ++i) P_triplets.emplace_back(uo + i, uo + i, 2.0 * Rd(i));
  }
  {
    const int xNo = x_idx(N);
    for (int i = 0; i < n_; ++i)
      P_triplets.emplace_back(xNo + i, xNo + i, 2.0 * p_.terminal_factor * Qd(i));
  }
  P_sparse_.resize(nz, nz);
  P_sparse_.setFromTriplets(P_triplets.begin(), P_triplets.end());

  // ---- q:  -2Q·x_ref  for states,  0 for inputs -----------------------------
  q_.resize(nz);
  for (int k = 0; k < N; ++k) {
    const int xo = x_idx(k), uo = u_idx(k);
    for (int i = 0; i < n_; ++i) q_(xo + i) = -2.0 * Qd(i) * x_ref[k](i);
    for (int i = 0; i < m_; ++i) q_(uo + i) = 0.0;
  }
  {
    const int xNo = x_idx(N);
    for (int i = 0; i < n_; ++i)
      q_(xNo + i) = -2.0 * p_.terminal_factor * Qd(i) * x_ref[N](i);
  }

  // ---- A: dynamics + x0 fix + input bounds + speed bounds -------------------
  //
  // Row layout:
  //   [0, N*n_)            dynamics:  x_{k+1} = A_d x_k + B_d u_k + C_d
  //   [N*n_, N*n_+n_)      x0 fixed:  x_0 = x_follower
  //   [dyn_end, dyn_end+N*m_)   input bounds:  |u_k| ≤ u_max
  //   [inp_end, inp_end+(N-1)*3) speed bounds:  |v_k| ≤ v_max  for k=1..N
  //
  const int dyn_rows   = N * n_;
  const int x0_row0    = dyn_rows;
  const int inp_row0   = x0_row0 + n_;
  const int spd_row0   = inp_row0 + N * m_;
  const int spd_rows   = (N - 1) * 3;   // skip x0, constrain x1..xN

  // ---- G1: dynamics ---------------------------------------------------------
  for (int k = 0; k < N; ++k) {
    const int r0  = k * n_;
    const int xk  = x_idx(k);
    const int uk  = u_idx(k);
    const int xk1 = x_idx(k + 1);

    for (int r = 0; r < n_; ++r) {
      for (int c = 0; c < n_; ++c)
        if (std::abs(A_d_(r, c)) > 1e-14)
          A_triplets.emplace_back(r0 + r, xk + c, A_d_(r, c));
      for (int c = 0; c < m_; ++c)
        if (std::abs(B_d_(r, c)) > 1e-14)
          A_triplets.emplace_back(r0 + r, uk + c, B_d_(r, c));
      A_triplets.emplace_back(r0 + r, xk1 + r, -1.0);
    }
  }

  // ---- G2: x0 = x_follower --------------------------------------------------
  for (int r = 0; r < n_; ++r)
    A_triplets.emplace_back(x0_row0 + r, r, 1.0);

  // ---- G3: input bounds -----------------------------------------------------
  for (int k = 0; k < N; ++k) {
    const int r0 = inp_row0 + k * m_;
    const int uo = u_idx(k);
    for (int c = 0; c < m_; ++c)
      A_triplets.emplace_back(r0 + c, uo + c, 1.0);
  }

  // ---- G4: speed bounds  (x_kSpeed … xN, skip early steps) -----------------
  // Allow a_m*dt buffer steps before enforcing speed constraints, preventing
  // primal infeasibility when current speed far exceeds v_max.
  static constexpr int k_speed_start = 3;
  int spd_r = spd_row0;
  for (int k = k_speed_start; k <= N; ++k) {
    const int xo = x_idx(k);
    A_triplets.emplace_back(spd_r++, xo + 3, 1.0);  // vx_b
    A_triplets.emplace_back(spd_r++, xo + 4, 1.0);  // vy_b
    A_triplets.emplace_back(spd_r++, xo + 5, 1.0);  // ω
  }
  const int total_rows = spd_r;

  A_sparse_.resize(total_rows, nz);
  A_sparse_.setFromTriplets(A_triplets.begin(), A_triplets.end());

  // ---- l / u bounds ---------------------------------------------------------
  l_.resize(total_rows);
  u_.resize(total_rows);

  // Dynamics: l = u = -C_d
  for (int k = 0; k < N; ++k) {
    for (int r = 0; r < n_; ++r) {
      l_(k * n_ + r) = -C_d_(r);
      u_(k * n_ + r) = -C_d_(r);
    }
  }

  // x0 fixed
  for (int r = 0; r < n_; ++r) {
    l_(x0_row0 + r) = x_follower(r);
    u_(x0_row0 + r) = x_follower(r);
  }

  // Input bounds
  const double a_lin = p_.max_linear_accel;
  const double a_ang = p_.max_angular_accel;
  for (int k = 0; k < N; ++k) {
    const int r0 = inp_row0 + k * m_;
    l_(r0 + 0) = -a_lin;  u_(r0 + 0) = a_lin;
    l_(r0 + 1) = -a_lin;  u_(r0 + 1) = a_lin;
    l_(r0 + 2) = -a_ang;  u_(r0 + 2) = a_ang;
  }

  // Speed bounds (from x_kSpeed to xN, allow buffer steps for feasibility)
  const double v_lin = p_.max_linear_vel;
  const double v_ang = p_.max_angular_vel;
  for (int k = k_speed_start, idx = 0; k <= N; ++k, ++idx) {
    const int r0 = spd_row0 + idx * 3;
    l_(r0 + 0) = -v_lin;  u_(r0 + 0) = v_lin;
    l_(r0 + 1) = -v_lin;  u_(r0 + 1) = v_lin;
    l_(r0 + 2) = -v_ang;  u_(r0 + 2) = v_ang;
  }
}

// ============================================================================
// Main entry point — single-pass fixed-offset MPC
// ============================================================================
inline Eigen::Vector3d MpcController6D::compute_control(
    const Eigen::VectorXd& x_leader,
    const Eigen::VectorXd& x_follower)
{
  using namespace std::chrono;

  linearize(x_follower);
  build_qp(x_leader, x_follower);
  solver_->setup(P_sparse_, q_, A_sparse_, l_, u_);

  auto t0 = steady_clock::now();
  bool ok = solver_->solve();
  auto t1 = steady_clock::now();

  last_status_ = solver_->status();
  last_time_ms_ = duration<double, std::milli>(t1 - t0).count();

  if (!ok) return Eigen::Vector3d::Zero();

  Eigen::VectorXd z = solver_->solution();
  return z.segment<m_>(n_);
}

}  // namespace formation_control
