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

  assert((delay_only - expected_delay_only).norm() < 1e-12);
  assert((forward - delay_only).norm() > 1e-6);
  return 0;
}
