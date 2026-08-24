#include <cassert>
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "homo_multirobot_formation_control/homo_controller_4d_artstein.hpp"

int main()
{
  constexpr double mass = 2.0;
  constexpr double control_period = 0.05;

  formation_control::LpcController4DArtstein controller(
      4, 2.0, 0.1, mass, 0.43, 0.7, false, control_period, 0.1,
      0.22, 1.0, 4.0);
  formation_control::LpcController baseline(
      4, 2.0, 0.1, mass, 0.7, false, 0.1, control_period, 1.0, 4.0);

  Eigen::Vector4d leader = Eigen::Vector4d::Zero();
  Eigen::Vector4d follower;
  follower << 2.5, 0.0, 0.0, 0.0;
  controller.controller_initial(leader, follower);
  baseline.controller_initial(leader, follower);

  const std::vector<double> command = controller.lpc_calculate(leader, follower);
  const std::vector<double> expected = baseline.lpc_calculate(leader, follower);

  assert(std::abs(command[0] - expected[0]) < 1e-12);
  assert(std::abs(command[1] - expected[1]) < 1e-12);

  std::cout << "4D Artstein HPC wrapper command contract test passed\n";
  return 0;
}
