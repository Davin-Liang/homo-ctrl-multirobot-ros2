#include <cassert>
#include <cmath>

#include "homo_multirobot_formation_control/homo_controller_6d_map_hpc_artstein.hpp"

int main()
{
  formation_control::MapHpcController6DArtstein ctrl(
      4, 2.0, 0.10, 2.0, 1.0, 0.5, true, 0.05, 1.0);
  const auto& offsets = ctrl.offsets();
  assert(offsets.size() == 4);
  assert(std::abs(offsets[0].x() - 2.0) < 1e-12);
  assert(std::abs(offsets[1].y() - 2.0) < 1e-12);
  assert(std::abs(offsets[2].x() + 2.0) < 1e-12);
  assert(std::abs(offsets[3].y() + 2.0) < 1e-12);

  Eigen::VectorXd leader = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd follower = Eigen::VectorXd::Zero(6);
  follower(0) = 1.95;
  assert(ctrl.select_target(leader, follower));
  assert(ctrl.target_index() == 0);

  follower(0) = -1.99;
  assert(ctrl.select_target(leader, follower));
  assert(ctrl.target_index() == 2);
  return 0;
}
