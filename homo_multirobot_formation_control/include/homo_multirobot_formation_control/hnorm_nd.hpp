#pragma once

#include <cmath>
#include <algorithm>
#include <unsupported/Eigen/MatrixFunctions>
#include "homo_multirobot_formation_control/types_nd.hpp"

namespace formation_control {

/// 二分法计算齐次范数（N-D 泛化版）。
///
/// 寻找 c 使得 expm(-Gd·c) · x 落在由 P 定义的单位椭球 x^T·P·x = 1 上，
/// 返回 q = exp(c) 作为范数值。
///
/// 与原版 hnorm.hpp 算法完全相同，仅类型从 Vec4d/Mat4d 改为 VectorXd/MatrixXd。
inline double hnorm_nd(const Eigen::VectorXd& x,
                       const Eigen::MatrixXd& Gd,
                       const Eigen::MatrixXd& P,
                       double alpha = 0.0, double beta = 0.0,
                       bool use_clip = false, int Nmax = 20)
{
  if (x.norm() < 1e-16) {
    return 0.0;
  }

  const double tol = 1e-6;

  // 扩张下界 a 直至 y'*P*y >= 1
  double a = -1.0;
  Eigen::VectorXd y = ((-Gd * a).exp()) * x;
  while ((y.transpose() * P * y).value() < 1.0 && a > -746.0) {
    a *= 2.0;
    y = ((-Gd * a).exp()) * x;
  }

  // 扩张上界 b 直至 y'*P*y <= 1
  double b = 1.0;
  y = ((-Gd * b).exp()) * x;
  while ((y.transpose() * P * y).value() > 1.0 && b < 710.0) {
    b *= 2.0;
    y = ((-Gd * b).exp()) * x;
  }

  // 二分法收敛到精确值
  double c = (a + b) / 2.0;
  for (int i = 0; i < Nmax; ++i) {
    y = ((-Gd * c).exp()) * x;
    double Qf = (y.transpose() * P * y).value() - 1.0;
    if (std::abs(Qf) < tol) break;
    if (Qf > 0.0) a = c;
    else          b = c;
    c = (a + b) / 2.0;
  }

  double q = std::exp(c);

  if (use_clip) {
    if (beta > 0.0) q = std::min(beta, q);
    if (alpha > 0.0) q = std::max(alpha, q);
  }

  return q;
}

}  // namespace formation_control
