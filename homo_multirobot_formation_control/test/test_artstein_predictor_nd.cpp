#include <cassert>
#include <cmath>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "homo_multirobot_formation_control/artstein_predictor_nd.hpp"

int main()
{
  Eigen::MatrixXd A(2, 2), B(2, 1);
  A << 0.0, 1.0,
       0.0, -1.0;
  B << 0.0,
       1.0;

  formation_control::ArtsteinPredictorNd predictor;
  predictor.build(A, B, 1.0, 0.5, 0.1);

  Eigen::VectorXd z(2), command(1);
  z << 2.0, -0.5;
  command << 0.8;

  const Eigen::VectorXd delay_only = predictor.predict(z, command, false);
  const Eigen::VectorXd forward = predictor.predict(z, command, true);
  const Eigen::VectorXd expected_delay_only = (A * 0.5).exp() * z;

  const Eigen::VectorXd no_compensation = predictor.predict(
      z, command, false, false);
  const Eigen::VectorXd forward_only = predictor.predict(
      z, command, false, true);

  assert((delay_only - expected_delay_only).norm() < 1e-12);
  assert((forward - delay_only).norm() > 1e-6);
  assert((no_compensation - z).norm() < 1e-12);
  assert((forward_only - no_compensation).norm() > 1e-6);

  const double H = 0.25;
  const Eigen::VectorXd custom_horizon = predictor.predict(
      z, command, false, true, H);
  const double decay = std::exp(-H);
  Eigen::VectorXd expected_custom(2);
  expected_custom(0) = z(0) + command(0) * H
      + (1.0 - decay) * (z(1) - command(0));
  expected_custom(1) = command(0) + decay * (z(1) - command(0));
  assert((custom_horizon - expected_custom).norm() < 1e-12);
  return 0;
}
