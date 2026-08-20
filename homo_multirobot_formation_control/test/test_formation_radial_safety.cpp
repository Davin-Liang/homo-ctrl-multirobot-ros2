#include <cassert>
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "homo_multirobot_formation_control/formation_safety.hpp"
#include "homo_multirobot_formation_control/kinematic_constraint.hpp"

int main()
{
  using formation_control::apply_radial_safety_limit;
  using formation_control::max_safe_inward_radial_speed;

  {
    const double safe = max_safe_inward_radial_speed(0.0, 0.4, 0.65);
    assert(std::abs(safe) < 1e-12);
  }

  {
    const double safe = max_safe_inward_radial_speed(0.2, 0.4, 0.65);
    assert(std::abs(safe - 0.2170744177) < 1e-6);
  }

  {
    const Eigen::Vector2d cmd(-0.5, 0.0);
    const Eigen::Vector2d leader_vel(0.0, 0.0);
    const Eigen::Vector2d leader(0.0, 0.0);
    const Eigen::Vector2d follower(1.0, 0.0);
    const Eigen::Vector2d out = apply_radial_safety_limit(
        cmd, leader_vel, leader, follower, 2.0, 0.4, 0.65, 0.5);
    assert(out(0) >= -1e-12);
    assert(std::abs(out(1)) < 1e-12);
  }

  {
    const Eigen::Vector2d cmd(-0.5, 0.0);
    const Eigen::Vector2d leader_vel(0.0, 0.0);
    const Eigen::Vector2d leader(0.0, 0.0);
    const Eigen::Vector2d follower(2.2, 0.0);
    const Eigen::Vector2d out = apply_radial_safety_limit(
        cmd, leader_vel, leader, follower, 2.0, 0.4, 0.65, 0.5);
    assert(out(0) < 0.0);
    assert(std::abs(out(0) + 0.2170744177) < 1e-6);
  }

  {
    const Eigen::Vector2d cmd(-0.5, 0.0);
    const Eigen::Vector2d leader_vel(0.0, 0.0);
    const Eigen::Vector2d leader(0.0, 0.0);
    const Eigen::Vector2d follower(3.0, 0.0);
    const Eigen::Vector2d out = apply_radial_safety_limit(
        cmd, leader_vel, leader, follower, 2.0, 0.4, 0.65, 0.5);
    assert(std::abs(out(0) + 0.5) < 1e-12);
  }

  {
    const Eigen::Vector2d cmd(0.0, 0.0);
    const Eigen::Vector2d leader_vel(0.3, 0.0);
    const Eigen::Vector2d leader(0.0, 0.0);
    const Eigen::Vector2d follower(2.2, 0.0);
    const Eigen::Vector2d out = apply_radial_safety_limit(
        cmd, leader_vel, leader, follower, 2.0, 0.4, 0.65, 0.5);
    assert(std::abs(out(0) - 0.0829255823) < 1e-6);
  }

  {
    const Eigen::Vector2d cmd(-0.45, 0.0);
    const Eigen::Vector2d leader_vel(-0.3, 0.0);
    const Eigen::Vector2d leader(0.0, 0.0);
    const Eigen::Vector2d follower(2.2, 0.0);
    const Eigen::Vector2d out = apply_radial_safety_limit(
        cmd, leader_vel, leader, follower, 2.0, 0.4, 0.65, 0.5);
    assert(std::abs(out(0) + 0.45) < 1e-12);
  }

  {
    // Although the requested command is still below the command-only limit,
    // the measured follower velocity is already too inward to stop safely.
    // The state-aware overload must start braking now.
    const Eigen::Vector2d cmd(-0.35, 0.0);
    const Eigen::Vector2d follower_vel(-0.55, 0.0);
    const Eigen::Vector2d leader_vel(0.0, 0.0);
    const Eigen::Vector2d leader(0.0, 0.0);
    const Eigen::Vector2d follower(2.4, 0.0);
    const Eigen::Vector2d out = apply_radial_safety_limit(
        cmd, follower_vel, leader_vel, leader, follower, 2.0, 2.0, 0.65, 1.0);
    assert(std::abs(out(0)) < 1e-12);
    assert(std::abs(out(1)) < 1e-12);
  }

  {
    // Removing an unsafe radial component must preserve the tangential
    // component needed for a moving leader trajectory.
    const Eigen::Vector2d cmd(-0.35, 0.2);
    const Eigen::Vector2d follower_vel(-0.55, 0.2);
    const Eigen::Vector2d leader_vel(0.0, 0.0);
    const Eigen::Vector2d leader(0.0, 0.0);
    const Eigen::Vector2d follower(2.4, 0.0);
    const Eigen::Vector2d out = apply_radial_safety_limit(
        cmd, follower_vel, leader_vel, leader, follower, 2.0, 2.0, 0.65, 1.0);
    assert(std::abs(out(0)) < 1e-12);
    assert(std::abs(out(1) - 0.2) < 1e-12);
  }

  {
    // The final safety projection becomes the command that is actually sent.
    // The next slew-rate step must start from that projected command rather
    // than from the pre-safety command retained by KinematicConstraint.
    formation_control::KinematicConstraint constraint(
        0.03, 0.11, 1000.0, 1.0, 10.0);
    double vx = -0.5;
    double vy = 0.0;
    double omega = 0.0;
    constraint.apply(vx, vy, omega, 0.1);

    constraint.set_last_command(0.0, 0.0, 0.0);
    vx = -0.5;
    vy = 0.0;
    omega = 0.0;
    constraint.apply(vx, vy, omega, 0.1);
    assert(std::abs(vx + 0.1) < 1e-12);
  }

  std::cout << "formation radial safety test passed\n";
  return 0;
}
