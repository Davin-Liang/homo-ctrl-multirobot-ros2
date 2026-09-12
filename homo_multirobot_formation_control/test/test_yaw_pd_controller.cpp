#include <cassert>
#include <cmath>

#include "homo_multirobot_formation_control/yaw_pd_controller.hpp"

int main()
{
  assert(formation_control::yaw_pd_command(
      0.5, 0.1, 0.7, 0.2, 4.0, 1.0, 10.0) == 2.1);

  const double wrapped = formation_control::yaw_pd_command(
      -3.10, 3.10, 0.0, 0.0, 1.0, 0.0, 10.0);
  assert(std::abs(wrapped - 0.083185) < 1e-5);

  assert(formation_control::yaw_pd_command(
      2.0, 0.0, 0.0, 0.0, 4.0, 1.0, 0.8) == 0.8);

  assert(formation_control::yaw_pd_command(
      0.0, 0.0, 0.0, 0.3, 4.0, 1.0, 10.0) == -0.3);
  return 0;
}
