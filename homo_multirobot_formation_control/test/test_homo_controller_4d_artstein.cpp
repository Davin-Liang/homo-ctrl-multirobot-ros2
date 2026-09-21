#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

#include <Eigen/Dense>

#include "homo_multirobot_formation_control/homo_controller_4d_artstein.hpp"

int main()
{
  constexpr double mass = 2.0;
  constexpr double control_period = 0.05;
  Eigen::Vector4d leader = Eigen::Vector4d::Zero();
  Eigen::Vector4d follower;
  follower << 2.5, 0.0, 0.0, 0.0;

  formation_control::LpcController4DArtstein automatic_nu_controller(
      4, 2.0, 0.1, mass, 0.43, true, control_period, 0.1,
      0.22, 1.0, 4.0);
  automatic_nu_controller.controller_initial(leader, follower);
  assert(std::abs(automatic_nu_controller.nu() -
                  automatic_nu_controller.nu_min()) < 1e-12);

  const double nu_override = 0.5 * (automatic_nu_controller.nu_min() +
                                    automatic_nu_controller.nu_max());
  formation_control::LpcController4DArtstein overridden_nu_controller(
      4, 2.0, 0.1, mass, 0.43, true, control_period, 0.1,
      0.22, 1.0, 4.0, true, nu_override);
  overridden_nu_controller.controller_initial(leader, follower);
  assert(std::abs(overridden_nu_controller.nu() - nu_override) < 1e-12);

  formation_control::LpcController4DArtstein invalid_nu_controller(
      4, 2.0, 0.1, mass, 0.43, true, control_period, 0.1,
      0.22, 1.0, 4.0, true, automatic_nu_controller.nu_min() - 1e-3);
  bool invalid_nu_rejected = false;
  try {
    invalid_nu_controller.controller_initial(leader, follower);
  } catch (const std::runtime_error&) {
    invalid_nu_rejected = true;
  }
  assert(invalid_nu_rejected);

  formation_control::LpcController4DArtstein nonfinite_nu_controller(
      4, 2.0, 0.1, mass, 0.43, true, control_period, 0.1,
      0.22, 1.0, 4.0, true, std::numeric_limits<double>::quiet_NaN());
  bool nonfinite_nu_rejected = false;
  try {
    nonfinite_nu_controller.controller_initial(leader, follower);
  } catch (const std::runtime_error&) {
    nonfinite_nu_rejected = true;
  }
  assert(nonfinite_nu_rejected);

  formation_control::LpcController4DArtstein controller(
      4, 2.0, 0.1, mass, 0.43, false, control_period, 0.1,
      0.22, 1.0, 4.0);
  formation_control::LpcController baseline(
      4, 2.0, 0.1, mass, false, 0.1, control_period, 1.0, 4.0);

  controller.controller_initial(leader, follower);
  baseline.controller_initial(leader, follower);

  const std::vector<double> command = controller.lpc_calculate(leader, follower);
  const std::vector<double> expected = baseline.lpc_calculate(leader, follower);

  assert(std::abs(command[0] - expected[0]) < 1e-12);
  assert(std::abs(command[1] - expected[1]) < 1e-12);

  std::cout << "4D Artstein HPC wrapper command contract test passed\n";
  return 0;
}
