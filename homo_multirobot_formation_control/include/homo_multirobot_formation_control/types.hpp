#pragma once

#include <Eigen/Dense>

namespace formation_control {

// 4 阶双重积分器状态：位置 x, y + 速度 vx, vy
using Vec4d  = Eigen::Vector4d;
using Mat4d  = Eigen::Matrix4d;
using Mat24d = Eigen::Matrix<double, 2, 4>;  // 2 输入 × 4 状态
using Mat42d = Eigen::Matrix<double, 4, 2>;  // 4 状态 × 2 输入

// lpc2hpc 升级结果
struct HpcResult {
  Mat24d K0;      // HPC 基础增益 (2×4)
  Mat4d  G0;      // 齐次生成元
  Mat4d  P;       // Lyapunov 矩阵
  double nu_min;  // 齐次度下界
  double nu_max;  // 齐次度上界
};

}  // namespace formation_control
