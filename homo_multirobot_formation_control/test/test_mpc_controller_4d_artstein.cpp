#include <cassert>
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include "homo_multirobot_formation_control/mpc_controller_4d_artstein.hpp"

int main()
{
  formation_control::MpcController4DArtstein::Params p;
  p.N = 30;
  p.dt = 0.05;
  p.mass = 2.0;
  p.formation_radius = 2.0;
  p.max_linear_vel = 0.6;
  p.max_linear_accel = 0.4;

  formation_control::MpcController4DArtstein ctrl(p);

  Eigen::Matrix4d Ad;
  Eigen::Matrix<double, 4, 2> Bd;
  formation_control::MpcController4DArtstein::zoh_matrices(p.mass, p.dt, Ad, Bd);

  assert(std::abs(Ad(0, 2) - 0.05) < 1e-12);
  assert(std::abs(Ad(1, 3) - 0.05) < 1e-12);
  assert(std::abs(Bd(0, 0) - 0.000625) < 1e-12);
  assert(std::abs(Bd(2, 0) - 0.025) < 1e-12);

  Eigen::Vector4d leader;
  leader << 0.0, 0.0, 0.2, 0.0;
  Eigen::Vector4d follower;
  follower << 2.8, 0.2, 0.0, 0.0;

  ctrl.init(leader, follower);
  const Eigen::Vector2d cmd = ctrl.compute_velocity_command(leader, follower);

  assert(ctrl.last_status() == 1);
  assert((cmd - ctrl.last_x1_pred().tail<2>()).norm() < 1e-9);
  assert((cmd - ctrl.last_u0()).norm() > 1e-3);
  assert(cmd.norm() <= p.max_linear_vel + 1e-9);

  formation_control::MpcController4DArtstein equilibrium_ctrl(p);
  Eigen::Vector4d static_leader;
  static_leader << 0.0, 0.0, 0.0, 0.0;
  Eigen::Vector4d equilibrium_follower;
  equilibrium_follower << 2.0, 0.0, 0.0, 0.0;
  equilibrium_ctrl.init(static_leader, equilibrium_follower);
  const Eigen::Vector2d equilibrium_cmd =
      equilibrium_ctrl.compute_velocity_command(static_leader, equilibrium_follower);

  assert(equilibrium_ctrl.target_index() == 2);
  assert(equilibrium_ctrl.last_status() == 1);
  assert(equilibrium_cmd.norm() < 1e-3);

  formation_control::MpcController4DArtstein far_ctrl(p);
  Eigen::Vector4d far_follower;
  far_follower << 2.5, 0.0, 0.0, 0.0;
  far_ctrl.init(static_leader, far_follower);
  const Eigen::Vector2d far_cmd =
      far_ctrl.compute_velocity_command(static_leader, far_follower);
  assert(far_ctrl.last_status() == 1);
  assert(far_cmd(0) < 0.0);

  formation_control::MpcController4DArtstein safety_ctrl(p);
  Eigen::Vector4d near_follower;
  near_follower << 0.5, 0.0, 0.0, 0.0;
  safety_ctrl.init(static_leader, near_follower);
  const Eigen::Vector2d safety_cmd =
      safety_ctrl.compute_velocity_command(static_leader, near_follower);

  assert(safety_cmd(0) > 0.0);
  assert(safety_ctrl.target_index() == 2);

  formation_control::MpcController4DArtstein::Params low_iter_p = p;
  low_iter_p.osqp_max_iter = 1;
  low_iter_p.osqp_polish = false;
  formation_control::MpcController4DArtstein low_iter_ctrl(low_iter_p);
  Eigen::Vector4d low_iter_follower;
  low_iter_follower << 3.0, 0.4, 0.0, 0.0;
  low_iter_ctrl.init(static_leader, low_iter_follower);
  const Eigen::Vector2d low_iter_cmd =
      low_iter_ctrl.compute_velocity_command(static_leader, low_iter_follower);

  assert(low_iter_ctrl.last_status() == OSQP_MAX_ITER_REACHED);
  assert((low_iter_cmd - low_iter_follower.tail<2>()).norm() > 1e-3);
  assert((low_iter_cmd - low_iter_ctrl.last_x1_pred().tail<2>()).norm() < 1e-9);

  std::cout << "MpcController4DArtstein contract test passed\n";
  return 0;
}
