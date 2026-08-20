#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

namespace formation_control {

inline double max_safe_inward_radial_speed(double clearance,
                                           double max_linear_accel,
                                           double effective_delay_s)
{
  if (clearance <= 0.0 || max_linear_accel <= 0.0) {
    return 0.0;
  }
  const double a = max_linear_accel;
  const double t = std::max(0.0, effective_delay_s);
  return std::max(0.0, -a * t + std::sqrt((a * t) * (a * t) + 2.0 * a * clearance));
}

inline Eigen::Vector2d apply_radial_safety_limit(
    const Eigen::Vector2d& cmd_map,
    const Eigen::Vector2d& leader_vel_map,
    const Eigen::Vector2d& leader_pos,
    const Eigen::Vector2d& follower_pos,
    double formation_radius,
    double max_linear_accel,
    double effective_delay_s,
    double max_linear_vel)
{
  Eigen::Vector2d out = cmd_map;
  const Eigen::Vector2d rel = follower_pos - leader_pos;
  const double dist = rel.norm();
  if (dist < 1e-9) {
    return out;
  }

  const Eigen::Vector2d radial = rel / dist;
  const double inward_speed = std::max(0.0, -(out - leader_vel_map).dot(radial));
  const double clearance = dist - formation_radius;
  const double safe_inward = max_safe_inward_radial_speed(
      clearance, max_linear_accel, effective_delay_s);

  if (inward_speed > safe_inward) {
    out += (inward_speed - safe_inward) * radial;
  }

  const double mag = out.norm();
  if (mag > max_linear_vel && mag > 1e-9) {
    out *= max_linear_vel / mag;
  }
  return out;
}

inline Eigen::Vector2d apply_radial_safety_limit(
    const Eigen::Vector2d& cmd_map,
    const Eigen::Vector2d& follower_vel_map,
    const Eigen::Vector2d& leader_vel_map,
    const Eigen::Vector2d& leader_pos,
    const Eigen::Vector2d& follower_pos,
    double formation_radius,
    double max_linear_accel,
    double effective_delay_s,
    double max_linear_vel)
{
  Eigen::Vector2d out = apply_radial_safety_limit(
      cmd_map, leader_vel_map, leader_pos, follower_pos, formation_radius,
      max_linear_accel, effective_delay_s, max_linear_vel);

  const Eigen::Vector2d rel = follower_pos - leader_pos;
  const double dist = rel.norm();
  if (dist < 1e-9) {
    return out;
  }

  const Eigen::Vector2d radial = rel / dist;
  const double measured_inward_speed =
      std::max(0.0, -(follower_vel_map - leader_vel_map).dot(radial));
  const double safe_inward_speed = max_safe_inward_radial_speed(
      dist - formation_radius, max_linear_accel, effective_delay_s);

  // When the measured plant state is already beyond the braking envelope,
  // do not send another command toward the leader while it slows down.
  if (measured_inward_speed > safe_inward_speed) {
    const double commanded_radial_speed = (out - leader_vel_map).dot(radial);
    if (commanded_radial_speed < 0.0) {
      out -= commanded_radial_speed * radial;
    }
  }

  const double mag = out.norm();
  if (mag > max_linear_vel && mag > 1e-9) {
    out *= max_linear_vel / mag;
  }
  return out;
}

}  // namespace formation_control
