#include <cassert>
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "homo_multirobot_formation_control/formation_safety.hpp"

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

  std::cout << "formation radial safety test passed\n";
  return 0;
}
