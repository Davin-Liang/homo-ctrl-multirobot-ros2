#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

namespace formation_control {

class MapHpcController6DArtsteinDisc {
public:
  MapHpcController6DArtsteinDisc(double radius, int points, double tol,
      double mass, double inertia, double mu, double kp, double kv,
      double c_min, bool use_hpc, double period)
  : radius_(radius), points_(points), tol_(tol), mass_(mass), inertia_(inertia),
    mu_(mu), c_min_(c_min), use_hpc_(use_hpc), period_(period)
  {
    if (radius_ <= 0.0 || points_ <= 0 || mass_ <= 0.0 || inertia_ <= 0.0 ||
        period_ <= 0.0 || c_min_ <= 0.0 || c_min_ > 1.0) {
      throw std::invalid_argument("invalid 6D map HPC parameters");
    }
    k_.setZero();
    k_.block<3, 3>(0, 0) = -kp * Eigen::Vector3d(mass_, mass_, inertia_).asDiagonal();
    k_.block<3, 3>(0, 3) = -kv * Eigen::Vector3d(mass_, mass_, inertia_).asDiagonal();
    Eigen::Matrix<double, 6, 6> a = Eigen::Matrix<double, 6, 6>::Zero();
    a(0, 3) = a(1, 4) = a(2, 5) = 1.0;
    Eigen::Matrix<double, 6, 3> b = Eigen::Matrix<double, 6, 3>::Zero();
    b(3, 0) = b(4, 1) = 1.0 / mass_;
    b(5, 2) = 1.0 / inertia_;
    gd_ = Eigen::Matrix<double, 6, 6>::Identity();
    gd_(0, 0) += -mu_; gd_(1, 1) += -mu_; gd_(2, 2) += -mu_;
    p_ = lyapunov(a + b * k_);
  }

  Eigen::Vector3d command(const Eigen::VectorXd& leader, const Eigen::VectorXd& follower)
  {
    if (leader.size() != 6 || follower.size() != 6) throw std::invalid_argument("state must be 6D");
    Eigen::Matrix<double, 6, 1> error = select_error(leader, follower);
    Eigen::Vector3d force = use_hpc_ ? homogeneous_command(error) : k_ * error;
    Eigen::Vector2d vf_map = rotate(follower(2), follower.segment<2>(3));
    return Eigen::Vector3d(vf_map(0) + period_ * force(0) / mass_,
                           vf_map(1) + period_ * force(1) / mass_,
                           follower(5) + period_ * force(2) / inertia_);
  }

  int target_idx() const { return target_idx_; }

private:
  static double wrap(double value) { return std::atan2(std::sin(value), std::cos(value)); }
  static Eigen::Vector2d rotate(double yaw, const Eigen::Vector2d& value)
  {
    const double c = std::cos(yaw), s = std::sin(yaw);
    return {c * value(0) - s * value(1), s * value(0) + c * value(1)};
  }
  Eigen::Vector2d target(int index) const
  {
    const double phase = 2.0 * M_PI * index / points_;
    return {-radius_ * std::cos(phase), -radius_ * std::sin(phase)};
  }
  Eigen::Matrix<double, 6, 1> error_for(const Eigen::VectorXd& l, const Eigen::VectorXd& f, int index) const
  {
    Eigen::Matrix<double, 6, 1> e;
    e.head<2>() = f.head<2>() - l.head<2>() - rotate(l(2), target(index));
    e(2) = wrap(f(2) - l(2));
    e.segment<2>(3) = rotate(f(2), f.segment<2>(3)) - rotate(l(2), l.segment<2>(3));
    e(5) = f(5) - l(5);
    return e;
  }
  Eigen::Matrix<double, 6, 1> select_error(const Eigen::VectorXd& l, const Eigen::VectorXd& f)
  {
    int best = 0; double best_norm = std::numeric_limits<double>::infinity();
    for (int i = 0; i < points_; ++i) { double n = error_for(l, f, i).norm(); if (n < best_norm) { best = i; best_norm = n; } }
    double current_norm = error_for(l, f, target_idx_).norm();
    if (best != target_idx_ && best_norm + tol_ < current_norm) target_idx_ = best;
    return error_for(l, f, target_idx_);
  }
  Eigen::Matrix<double, 6, 6> lyapunov(const Eigen::Matrix<double, 6, 6>& acl)
  {
    Eigen::Matrix<double, 36, 36> system = Eigen::Matrix<double, 36, 36>::Zero();
    for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) for (int k = 0; k < 6; ++k)
      system(i + 6 * j, k + 6 * j) += acl(k, i), system(i + 6 * j, i + 6 * k) += acl(k, j);
    Eigen::Matrix<double, 36, 1> rhs = Eigen::Matrix<double, 36, 1>::Zero();
    for (int i = 0; i < 6; ++i) rhs(i + 6 * i) = -2.0;
    Eigen::Matrix<double, 36, 1> values = system.fullPivLu().solve(rhs);
    Eigen::Matrix<double, 6, 6> result;
    for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) result(i, j) = values(i + 6 * j);
    return 0.5 * (result + result.transpose());
  }
  Eigen::Vector3d homogeneous_command(const Eigen::Matrix<double, 6, 1>& error) const
  {
    if (error.norm() < 1e-14) return Eigen::Vector3d::Zero();
    double low = -1.0, high = 1.0;
    auto value = [this, &error](double s) { Eigen::VectorXd z = (-gd_ * s).exp() * error; return z.dot(p_ * z); };
    while (value(low) < 1.0) low *= 2.0;
    while (value(high) > 1.0) high *= 2.0;
    for (int i = 0; i < 40; ++i) { double mid = 0.5 * (low + high); if (value(mid) > 1.0) low = mid; else high = mid; }
    double c = std::clamp(std::exp(0.5 * (low + high)), c_min_, 1.0);
    return std::pow(c, 1.0 + mu_) * k_ * (gd_ * (1.0 - std::log(c))).exp() * error;
  }
  double radius_, tol_, mass_, inertia_, mu_, c_min_, period_; int points_, target_idx_ = 0;
  bool use_hpc_; Eigen::Matrix<double, 3, 6> k_; Eigen::Matrix<double, 6, 6> gd_, p_;
};
}  // namespace formation_control
