#include "homo_multirobot_formation_control/hocbf_safety_filter.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace formation_control::hocbf {
namespace {

bool satisfies(const Eigen::Vector2d& command,
               const std::vector<Halfspace>& constraints)
{
  for (const auto& constraint : constraints) {
    if (constraint.a.dot(command) < constraint.b - 1e-10) return false;
  }
  return true;
}

}  // namespace

std::optional<Circle> fit_circle(const std::vector<Eigen::Vector2d>& points,
                                 double max_rms_residual)
{
  if (points.size() < 3 || max_rms_residual < 0.0) return std::nullopt;
  Eigen::MatrixXd A(points.size(), 3);
  Eigen::VectorXd b(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    A(i, 0) = 2.0 * points[i].x();
    A(i, 1) = 2.0 * points[i].y();
    A(i, 2) = 1.0;
    b(i) = points[i].squaredNorm();
  }
  const Eigen::Vector3d solution = A.colPivHouseholderQr().solve(b);
  const Eigen::Vector2d center = solution.head<2>();
  const double radius_sq = solution(2) + center.squaredNorm();
  if (!(radius_sq > 0.0) || !std::isfinite(radius_sq)) return std::nullopt;
  const double radius = std::sqrt(radius_sq);
  double residual_sum = 0.0;
  for (const auto& point : points) {
    const double residual = (point - center).norm() - radius;
    residual_sum += residual * residual;
  }
  const double rms = std::sqrt(residual_sum / static_cast<double>(points.size()));
  if (!std::isfinite(rms) || rms > max_rms_residual) return std::nullopt;
  return Circle{center, radius, rms};
}

Halfspace hocbf_halfspace(const Eigen::Vector4d& state,
                          const Circle& obstacle, double tau,
                          double c1, double c2)
{
  if (obstacle.radius <= 0.0 || tau <= 0.0 || c1 <= 0.0 || c2 <= 0.0) {
    throw std::invalid_argument("HOCBF parameters must be positive");
  }
  const Eigen::Vector2d radial = state.head<2>() - obstacle.center;
  const Eigen::Vector2d velocity = state.tail<2>();
  const double h = radial.squaredNorm() - obstacle.radius * obstacle.radius;
  const double psi1 = 2.0 * radial.dot(velocity) + c1 * h;
  const Eigen::Vector2d a = 2.0 * radial / tau;
  const double b = -2.0 * velocity.squaredNorm()
                 + 2.0 * radial.dot(velocity) / tau
                 - 2.0 * c1 * radial.dot(velocity) - c2 * psi1;
  return Halfspace{a, b};
}

QpResult solve_translation_qp(const Eigen::Vector2d& nominal,
                              const Eigen::Vector2d& previous,
                              const std::vector<Halfspace>& halfspaces,
                              double max_velocity, double max_acceleration,
                              double dt)
{
  if (max_velocity <= 0.0 || max_acceleration <= 0.0 || dt <= 0.0) {
    throw std::invalid_argument("QP limits must be positive");
  }
  const Eigen::Vector2d lower = (-Eigen::Vector2d::Constant(max_velocity))
      .cwiseMax(previous - Eigen::Vector2d::Constant(max_acceleration * dt));
  const Eigen::Vector2d upper = Eigen::Vector2d::Constant(max_velocity)
      .cwiseMin(previous + Eigen::Vector2d::Constant(max_acceleration * dt));
  std::vector<Halfspace> constraints = {
      {Eigen::Vector2d(1.0, 0.0), lower.x()},
      {Eigen::Vector2d(-1.0, 0.0), -upper.x()},
      {Eigen::Vector2d(0.0, 1.0), lower.y()},
      {Eigen::Vector2d(0.0, -1.0), -upper.y()},
  };
  constraints.insert(constraints.end(), halfspaces.begin(), halfspaces.end());
  std::vector<Eigen::Vector2d> candidates;
  if (satisfies(nominal, constraints)) candidates.push_back(nominal);
  for (const auto& constraint : constraints) {
    const double norm_sq = constraint.a.squaredNorm();
    if (norm_sq <= 1e-15) continue;
    const Eigen::Vector2d candidate = nominal
        + (constraint.b - constraint.a.dot(nominal)) / norm_sq * constraint.a;
    if (satisfies(candidate, constraints)) candidates.push_back(candidate);
  }
  for (size_t i = 0; i < constraints.size(); ++i) {
    for (size_t j = i + 1; j < constraints.size(); ++j) {
      Eigen::Matrix2d matrix;
      matrix.row(0) = constraints[i].a.transpose();
      matrix.row(1) = constraints[j].a.transpose();
      if (std::abs(matrix.determinant()) <= 1e-12) continue;
      const Eigen::Vector2d candidate = matrix.fullPivLu().solve(
          Eigen::Vector2d(constraints[i].b, constraints[j].b));
      if (satisfies(candidate, constraints)) candidates.push_back(candidate);
    }
  }
  if (candidates.empty()) return QpResult{};
  auto best = candidates.front();
  double best_cost = (best - nominal).squaredNorm();
  for (const auto& candidate : candidates) {
    const double cost = (candidate - nominal).squaredNorm();
    if (cost < best_cost) { best = candidate; best_cost = cost; }
  }
  int active = 0;
  for (const auto& constraint : constraints) {
    if (std::abs(constraint.a.dot(best) - constraint.b) <= 1e-9) ++active;
  }
  return QpResult{best, true, active};
}

}  // namespace formation_control::hocbf
