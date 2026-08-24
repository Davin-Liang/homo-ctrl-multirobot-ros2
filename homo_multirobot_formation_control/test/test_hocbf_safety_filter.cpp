#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include <Eigen/Dense>

#include "homo_multirobot_formation_control/hocbf_safety_filter.hpp"

int main()
{
  using formation_control::hocbf::Halfspace;
  using formation_control::hocbf::fit_circle;
  using formation_control::hocbf::solve_translation_qp;

  std::vector<Eigen::Vector2d> points;
  const Eigen::Vector2d expected_center(2.0, -1.0);
  for (int i = 0; i < 16; ++i) {
    const double angle = 0.3 + i * 2.0 * M_PI / 16.0;
    points.push_back(expected_center + 0.25 * Eigen::Vector2d(std::cos(angle), std::sin(angle)));
  }
  const auto circle = fit_circle(points, 1e-8);
  assert(circle.has_value());
  assert((circle->center - expected_center).norm() < 1e-8);
  assert(std::abs(circle->radius - 0.25) < 1e-8);

  std::vector<Halfspace> constraints = {
    {Eigen::Vector2d(1.0, 0.0), 0.20},
    {Eigen::Vector2d(0.0, 1.0), -0.10},
  };
  const auto result = solve_translation_qp(
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), constraints, 1.0, 20.0, 0.05);
  assert(result.feasible);
  assert(std::abs(result.command.x() - 0.20) < 1e-10);
  assert(std::abs(result.command.y()) < 1e-10);

  std::cout << "hocbf safety filter test passed\n";
  return 0;
}
