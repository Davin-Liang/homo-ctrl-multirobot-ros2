#include <cassert>
#include <cmath>

#include <Eigen/Dense>

#include "homo_multirobot_formation_control/homo_controller_6d_map_hpc_artstein.hpp"

int main()
{
  formation_control::MapHpcController6DArtsteinDisc controller(
      1.0, 4, 0.1, 2.0, 1.0, -0.25, 1.2, 2.0, 0.5, true, 0.05);
  Eigen::VectorXd leader = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd follower = Eigen::VectorXd::Zero(6);
  follower(0) = -1.0;
  assert(controller.command(leader, follower).norm() < 1e-12);
  assert(controller.target_idx() == 0);
  leader(2) = M_PI_2;
  follower.head<3>() << 0.0, -1.0, M_PI_2;
  assert(controller.command(leader, follower).norm() < 1e-12);
  return 0;
}
