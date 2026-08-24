#pragma once

#include <optional>
#include <vector>

#include <Eigen/Dense>

namespace formation_control::hocbf {

struct Circle
{
  Eigen::Vector2d center;
  double radius;
  double rms_residual;
};

struct Halfspace
{
  Eigen::Vector2d a;
  double b;
};

struct QpResult
{
  Eigen::Vector2d command = Eigen::Vector2d::Zero();
  bool feasible = false;
  int active_constraints = 0;
};

std::optional<Circle> fit_circle(const std::vector<Eigen::Vector2d>& points,
                                 double max_rms_residual);

Halfspace hocbf_halfspace(const Eigen::Vector4d& predicted_state,
                          const Circle& obstacle, double tau,
                          double c1, double c2);

QpResult solve_translation_qp(const Eigen::Vector2d& nominal,
                              const Eigen::Vector2d& previous,
                              const std::vector<Halfspace>& halfspaces,
                              double max_velocity, double max_acceleration,
                              double dt);

}  // namespace formation_control::hocbf
