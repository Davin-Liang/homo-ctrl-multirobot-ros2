#pragma once

#include <Eigen/Dense>

namespace formation_control {

/// N-D 齐次控制器参数集（lpc2hpc 输出）。
/// 与 types.hpp 的 HpcResult 对应，但使用动态尺寸 MatrixXd 替代固定 4D 类型。
struct HpcResultNd {
  Eigen::MatrixXd K0;   // HPC 基础增益 (m × n)，m = 输入数, n = 状态数
  Eigen::MatrixXd G0;   // 齐次生成元 (n × n)
  Eigen::MatrixXd P;    // Lyapunov 矩阵 (n × n), Acl^T·P + P·Acl = -2I
  double nu_min = -1.0; // 齐次度可容许下界
  double nu_max = 1.0;  // 齐次度可容许上界
};

}  // namespace formation_control
