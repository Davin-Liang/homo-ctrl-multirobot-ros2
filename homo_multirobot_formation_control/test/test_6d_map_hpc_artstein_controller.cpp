#include <cassert>
#include <cmath>

#include <Eigen/Dense>

#include "homo_multirobot_formation_control/homo_controller_6d_map_hpc_artstein.hpp"

int main()
{
  Eigen::VectorXd nu_leader = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd nu_follower = Eigen::VectorXd::Zero(6);
  nu_follower(0) = 1.0;
  formation_control::MapHpcController6DArtstein automatic_nu_controller(
      1, 1.0, 0.0, 2.0, 1.0, 0.5, true, 0.05, 1.0, 4.0);
  assert(automatic_nu_controller.select_target(nu_leader, nu_follower));
  automatic_nu_controller.initialize(nu_leader, nu_follower);
  assert(std::abs(automatic_nu_controller.nu() -
                  automatic_nu_controller.nu_min()) < 1e-12);

  const double nu_override = 0.5 * (automatic_nu_controller.nu_min() +
                                    automatic_nu_controller.nu_max());
  formation_control::MapHpcController6DArtstein overridden_nu_controller(
      1, 1.0, 0.0, 2.0, 1.0, 0.5, true, 0.05, 1.0, 4.0,
      true, nu_override);
  assert(overridden_nu_controller.select_target(nu_leader, nu_follower));
  overridden_nu_controller.initialize(nu_leader, nu_follower);
  assert(std::abs(overridden_nu_controller.nu() - nu_override) < 1e-12);

  formation_control::MapHpcController6DArtstein invalid_nu_controller(
      1, 1.0, 0.0, 2.0, 1.0, 0.5, true, 0.05, 1.0, 4.0,
      true, automatic_nu_controller.nu_min() - 1e-3);
  assert(invalid_nu_controller.select_target(nu_leader, nu_follower));
  bool invalid_nu_rejected = false;
  try {
    invalid_nu_controller.initialize(nu_leader, nu_follower);
  } catch (const std::runtime_error&) {
    invalid_nu_rejected = true;
  }
  assert(invalid_nu_rejected);

  auto rejects_invalid_lambda = [](double initial_min_lambda,
                                   double switch_min_lambda) {
    try {
      formation_control::MapHpcController6DArtstein invalid_controller(
          1, 1.0, 0.0, 2.0, 1.0, 0.5, true, 0.05,
          initial_min_lambda, switch_min_lambda);
    } catch (const std::invalid_argument&) {
      return true;
    }
    return false;
  };
  assert(rejects_invalid_lambda(0.0, 4.0));
  assert(rejects_invalid_lambda(1.0, -1.0));
  assert(rejects_invalid_lambda(1.0, std::numeric_limits<double>::infinity()));

  formation_control::MapHpcController6DArtstein bandwidth_controller(
      4, 1.0, 0.0, 2.0, 1.0, 0.5, true, 0.05, 1.0, 4.0);
  Eigen::VectorXd bandwidth_leader = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd bandwidth_follower = Eigen::VectorXd::Zero(6);
  bandwidth_follower(0) = 1.0;
  assert(bandwidth_controller.select_target(bandwidth_leader, bandwidth_follower));
  bandwidth_controller.initialize(
      bandwidth_leader, bandwidth_follower, false);
  assert(std::abs(bandwidth_controller.min_lambda() - 1.0) < 1e-12);
  bandwidth_follower(0) = 1.5;
  const Eigen::Vector3d initial_bandwidth_command =
      bandwidth_controller.command(bandwidth_leader, bandwidth_follower);
  bandwidth_controller.initialize(
      bandwidth_leader, bandwidth_follower, true);
  assert(std::abs(bandwidth_controller.min_lambda() - 4.0) < 1e-12);
  const Eigen::Vector3d switch_bandwidth_command =
      bandwidth_controller.command(bandwidth_leader, bandwidth_follower);
  assert((initial_bandwidth_command - switch_bandwidth_command).norm() > 1e-6);

  formation_control::MapHpcController6DArtstein controller(
      1, 1.0, 0.0, 2.0, 1.0, 0.5, true, 0.05, 1.0);
  Eigen::VectorXd leader = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd follower = Eigen::VectorXd::Zero(6);
  follower(0) = 1.0;
  assert(controller.select_target(leader, follower));
  controller.initialize(leader, follower);
  assert(controller.command(leader, follower).norm() < 1e-12);
  leader(2) = M_PI_2;
  follower(2) = M_PI_2;
  assert(controller.command(leader, follower).norm() < 1e-12);
  return 0;
}
