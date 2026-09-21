#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "homo_multirobot_formation_control/hnorm_nd.hpp"
#include "homo_multirobot_formation_control/lpc2hpc_nd.hpp"

namespace formation_control {

class MapHpcController6DArtstein {
public:
  MapHpcController6DArtstein(int m_p, double radius, double tol,
      double mass, double inertia, double c_min, bool use_hpc, double period,
      double initial_min_lambda, double switch_min_lambda = 4.0)
  : mass_(mass), inertia_(inertia), c_min_(c_min), use_hpc_(use_hpc), period_(period),
    initial_min_lambda_(initial_min_lambda), switch_min_lambda_(switch_min_lambda),
    min_lambda_(initial_min_lambda), tol_(tol)
  {
    if (m_p < 1 || radius <= 0.0 || tol_ < 0.0 || mass_ <= 0.0 || inertia_ <= 0.0 ||
        period_ <= 0.0 || c_min_ <= 0.0 || c_min_ > 1.0) {
      throw std::invalid_argument("invalid 6D map HPC parameters");
    }
    for (int j = 0; j < m_p; ++j) {
      const double angle = 2.0 * M_PI * static_cast<double>(j) / static_cast<double>(m_p);
      offsets_.emplace_back(radius * std::cos(angle), radius * std::sin(angle));
    }
    a_ = Eigen::MatrixXd::Zero(6, 6); a_(0,3)=a_(1,4)=a_(2,5)=1.0;
    b_ = Eigen::MatrixXd::Zero(6, 3); b_(3,0)=b_(4,1)=1.0/mass_; b_(5,2)=1.0/inertia_;
  }

  const std::vector<Eigen::Vector2d>& offsets() const { return offsets_; }
  int target_index() const { return target_index_; }
  bool select_target(const Eigen::VectorXd& leader, const Eigen::VectorXd& follower) {
    int best = 0; double best_distance = std::numeric_limits<double>::max();
    for (int j = 0; j < static_cast<int>(offsets_.size()); ++j) {
      const double distance = (follower.head<2>() - leader.head<2>() - offsets_[j]).norm();
      if (distance < best_distance) { best = j; best_distance = distance; }
    }
    if (target_index_ < 0) { target_index_ = best; return true; }
    const double current_distance = (follower.head<2>() - leader.head<2>() - offsets_[target_index_]).norm();
    if (best != target_index_ && best_distance + tol_ < current_distance) { target_index_ = best; return true; }
    return false;
  }

  void initialize(const Eigen::VectorXd& leader, const Eigen::VectorXd& follower,
                  bool use_switch_bandwidth = false) {
    min_lambda_ = use_switch_bandwidth ? switch_min_lambda_ : initial_min_lambda_;
    k_ = calculate_klin(error_of(leader, follower));
    auto res = lpc2hpc_nd(a_, b_, k_);
    if (res.G0.isZero(1e-12)) throw std::runtime_error("6D map HPC initialization failed");
    p_ = res.P; gd_ = Eigen::MatrixXd::Identity(6,6) + res.nu_min * res.G0; nu_ = res.nu_min; initialized_ = true;
  }

  double min_lambda() const { return min_lambda_; }

  Eigen::Vector3d command(const Eigen::VectorXd& leader, const Eigen::VectorXd& follower)
  {
    if (leader.size() != 6 || follower.size() != 6) throw std::invalid_argument("state must be 6D");
    if (!initialized_) throw std::runtime_error("6D map HPC not initialized");
    Eigen::Matrix<double, 6, 1> error = error_of(leader, follower);
    Eigen::Vector3d force = use_hpc_ ? homogeneous_command(error) : k_ * error;
    Eigen::Vector2d vf_map = rotate(follower(2), follower.segment<2>(3));
    return Eigen::Vector3d(vf_map(0) + period_ * force(0) / mass_,
                           vf_map(1) + period_ * force(1) / mass_,
                           follower(5) + period_ * force(2) / inertia_);
  }

private:
  static double wrap(double value) { return std::atan2(std::sin(value), std::cos(value)); }
  static Eigen::Vector2d rotate(double yaw, const Eigen::Vector2d& value)
  {
    const double c = std::cos(yaw), s = std::sin(yaw);
    return {c * value(0) - s * value(1), s * value(0) + c * value(1)};
  }
  Eigen::Matrix<double, 6, 1> error_of(const Eigen::VectorXd& leader, const Eigen::VectorXd& follower) const {
    Eigen::Matrix<double, 6, 1> error;
    error.head<2>() = follower.head<2>() - leader.head<2>() - offsets_.at(target_index_);
    error(2) = wrap(follower(2) - leader(2));
    error.segment<2>(3) = rotate(follower(2), follower.segment<2>(3)) - rotate(leader(2), leader.segment<2>(3));
    error(5) = follower(5) - leader(5); return error;
  }
  Eigen::Matrix<double, 3, 6> calculate_klin(const Eigen::Matrix<double,6,1>& e) const {
    Eigen::Matrix<double,3,6> K = Eigen::Matrix<double,3,6>::Zero();
    for (int i=0; i<3; ++i) {
      double M = i == 2 ? inertia_ : mass_;
      double ratio = std::abs(e(i)) > 1e-6 ? -M * e(i + 3) / e(i) : 0.0;
      double a = std::max(ratio, min_lambda_);
      K(i, i) = -a * a / M;
      K(i, i + 3) = -2.0 * a;
    }
    return K;
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
    return std::pow(c, 1.0 + nu_) * k_ * (gd_ * (1.0 - std::log(c))).exp() * error;
  }
  std::vector<Eigen::Vector2d> offsets_; int target_index_ = -1;
  double mass_, inertia_, c_min_, period_, initial_min_lambda_, switch_min_lambda_,
      min_lambda_, nu_=0.0, tol_;
  bool use_hpc_, initialized_=false; Eigen::Matrix<double, 3, 6> k_; Eigen::MatrixXd a_, b_, gd_, p_;
};
}  // namespace formation_control
