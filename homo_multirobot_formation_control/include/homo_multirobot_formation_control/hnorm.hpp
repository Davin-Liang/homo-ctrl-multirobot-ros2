#pragma once

#include <cmath>
#include <algorithm>
#include <unsupported/Eigen/MatrixFunctions>
#include "homo_multirobot_formation_control/types.hpp"

namespace formation_control {

// 二分法计算齐次范数。
// 寻找 c 使得 expm(-Gd*c) * x 落在由 P 定义的单位椭球 x'*P*x = 1 上，
// 返回 q = exp(c) 作为范数值。
//
// Gd: 膨胀生成元 (4×4)
// P: 定义单位球的 Lyapunov 矩阵
// alpha / beta: 可选输出裁剪（仅 use_clip==true 时生效）
// Nmax: 二分法迭代次数（默认 20）
//
// 等价于 hnorm.py。运行时每次约 35 次 expm(4×4)。
inline double hnorm(const Vec4d& x, const Mat4d& Gd, const Mat4d& P,
                     double alpha = 0.0, double beta = 0.0, bool use_clip = false,
                     int Nmax = 20)
{
  if (x.norm() < 1e-16) {
    return 0.0;
  }

  const double tol = 1e-6;

  // 扩张下界 a 直至 y'*P*y >= 1
  double a = -1.0;
  Vec4d y = ((-Gd * a).exp()) * x;
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
    if (std::abs(Qf) < tol) {
      break;
    }
    if (Qf > 0.0) {
      a = c;
    } else {
      b = c;
    }
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
