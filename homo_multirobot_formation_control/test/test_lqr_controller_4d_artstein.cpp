#include <cassert>
#include <cmath>
#include <complex>
#include <iostream>

#include <Eigen/Dense>

#include "homo_multirobot_formation_control/lqr_controller_4d_artstein.hpp"

int main()
{
  formation_control::LqrController4DArtstein::Params p;
  p.dt = 0.05;
  p.mass = 2.0;
  p.formation_radius = 2.0;
  p.q_px = 40.0;
  p.q_py = 40.0;
  p.q_vx = 1.0;
  p.q_vy = 1.0;
  p.r_ux = 0.02;
  p.r_uy = 0.02;

  formation_control::LqrController4DArtstein ctrl(p);

  Eigen::Matrix4d Ad;
  Eigen::Matrix<double, 4, 2> Bd;
  formation_control::LqrController4DArtstein::zoh_matrices(p.mass, p.dt, Ad, Bd);

  assert(std::abs(Ad(0, 2) - 0.05) < 1e-12);
  assert(std::abs(Ad(1, 3) - 0.05) < 1e-12);
  assert(std::abs(Bd(0, 0) - 0.000625) < 1e-12);
  assert(std::abs(Bd(2, 0) - 0.025) < 1e-12);

  const Eigen::Matrix4d Acl = ctrl.Ad() - ctrl.Bd() * ctrl.K();
  const Eigen::EigenSolver<Eigen::Matrix4d> eig(Acl);
  for (int i = 0; i < 4; ++i) {
    assert(std::abs(eig.eigenvalues()(i)) < 1.0);
  }

  Eigen::Vector4d static_leader;
  static_leader << 0.0, 0.0, 0.0, 0.0;
  Eigen::Vector4d equilibrium_follower;
  equilibrium_follower << 2.0, 0.0, 0.0, 0.0;
  ctrl.init(static_leader, equilibrium_follower);
  const Eigen::Vector2d equilibrium_cmd =
      ctrl.compute_velocity_command(static_leader, equilibrium_follower);

  assert(ctrl.target_index() == 2);
  assert(equilibrium_cmd.norm() < 1e-9);
  assert(ctrl.selected_error(static_leader, equilibrium_follower).norm() < 1e-9);

  Eigen::Vector4d far_follower;
  far_follower << 2.5, 0.0, 0.0, 0.0;
  ctrl.init(static_leader, far_follower);
  const Eigen::Vector2d far_cmd =
      ctrl.compute_velocity_command(static_leader, far_follower);

  assert(far_cmd(0) < 0.0);
  assert(ctrl.current_distance(static_leader, far_follower) > 0.0);

  std::cout << "LqrController4DArtstein contract test passed\n";
  return 0;
}
