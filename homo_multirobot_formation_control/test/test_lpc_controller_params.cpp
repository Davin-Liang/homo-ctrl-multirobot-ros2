#include <cassert>
#include <cmath>

#include "homo_multirobot_formation_control/homo_controller.hpp"

int main()
{
  const formation_control::LpcController controller(
      4, 2.0, 0.1, 8.0, 1.5, true, 0.23, 0.05, 1.7, 3.4);

  assert(std::abs(controller.hpc_c_min() - 0.23) < 1e-12);
  assert(std::abs(controller.initial_min_lambda() - 1.7) < 1e-12);
  assert(std::abs(controller.switch_min_lambda() - 3.4) < 1e-12);
  return 0;
}
