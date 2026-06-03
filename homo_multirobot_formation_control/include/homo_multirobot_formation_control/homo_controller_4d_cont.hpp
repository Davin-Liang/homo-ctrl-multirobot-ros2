#pragma once

/// @file 4D 齐次控制器（连续边界投影变体）。
///
/// 与原 LpcController 的区别：
///   - 编队策略：连续边界投影（径向投影到安全圆），取代离散多边形 + tol 滞后切换
///   - 移除 m_p_、tol_、check_and_switch_target()
///   - omega_d 从硬编码升级为构造参数
///
/// 状态 x = [px, py, vx, vy]^T（map 帧），偏航控制由调用方负责。

#include <cmath>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "homo_multirobot_formation_control/types.hpp"
#include "homo_multirobot_formation_control/lpc2hpc.hpp"
#include "homo_multirobot_formation_control/hnorm.hpp"

namespace formation_control {

class LpcController4DCont {
public:
  /// @param radius  编队安全圆半径 (m)
  /// @param mass    双重积分器模型质量（调参用，非物理质量）
  /// @param omega_d 期望阻尼带宽
  /// @param use_hpc true=齐次控制，false=纯线性比例控制
  LpcController4DCont(double radius = 2.0, double mass = 8.0,
                       double omega_d = 1.5, bool use_hpc = true)
    : radius_(radius), mass_(mass), omega_d_(omega_d), use_hpc_(use_hpc)
  {
    A_ << 0, 0, 1, 0,
          0, 0, 0, 1,
          0, 0, 0, 0,
          0, 0, 0, 0;
    B_ << 0, 0,
          0, 0,
          1.0 / mass_, 0,
          0, 1.0 / mass_;

    k_lin_.setZero();
    P_.setIdentity();
    nu_ = 0.0;
    Gd_.setIdentity();
    G0_.setZero();

    last_cmd_vel_ << 0.0, 0.0;
  }

  // --------------------------------------------------------------------------
  // 一次性初始化：计算初始误差 → k_lin → HPC 升级。
  // --------------------------------------------------------------------------
  void controller_initial(const Vec4d& x1, const Vec4d& x2)
  {
    Vec4d e = compute_error(x1, x2);
    k_lin_ = calculate_klin(e);

    if (use_hpc_) {
      auto res = lpc2hpc(A_, B_, k_lin_);
      if (res.G0.isZero(1e-12)) {
        throw std::runtime_error("控制器初始化失败: lpc2hpc 返回零结果。");
      }
      G0_ = res.G0;
      P_  = res.P;
      nu_ = res.nu_min;
      Gd_ = Mat4d::Identity() + nu_ * G0_;
    }
    initialized_ = true;
  }

  // --------------------------------------------------------------------------
  // 每周期控制量计算。
  //
  // 返回 {vx_cmd, vy_cmd} (m/s, map 帧)。偏航控制由调用方负责。
  //
  // 控制律 (use_hpc_ = true):
  //   e  = compute_error(x1, x2)               // 连续边界投影
  //   c  = clamp(hnorm(e, Gd, P), 0.5, 1)
  //   u2 = c^(1+nu) · K · expm(Gd·(1−ln c)) · e
  //   v  = v_current + h · u2 / mass           // 前向欧拉
  //
  // 控制律 (use_hpc_ = false):
  //   u2 = K · e                               // 纯线性比例控制
  // --------------------------------------------------------------------------
  std::vector<double> lpc_calculate(const Vec4d& x1, const Vec4d& x2)
  {
    if (!initialized_) {
      return {0.0, 0.0};
    }

    Vec4d e = compute_error(x1, x2);
    k_lin_ = calculate_klin(e);

    Eigen::Vector2d u2;
    if (use_hpc_) {
      recompute_hpc();

      double nx = hnorm(e, Gd_, P_);
      double c = std::clamp(nx, 0.5, 1.0);
      double log_c  = std::log(c);
      Mat4d  expm_g = (Gd_ * (1.0 - log_c)).exp();
      Vec4d  warped_e = expm_g * e;
      u2 = std::pow(c, 1.0 + nu_) * (k_lin_ * warped_e);
    } else {
      u2 = k_lin_ * e;
    }

    double h = 0.1;
    Vec4d goal_x2 = x2 + h * (A_ * x2 + B_ * u2);

    double alpha_lpf = 0.3;
    double smooth_vx = (1.0 - alpha_lpf) * last_cmd_vel_(0) + alpha_lpf * goal_x2(2);
    double smooth_vy = (1.0 - alpha_lpf) * last_cmd_vel_(1) + alpha_lpf * goal_x2(3);
    last_cmd_vel_ << smooth_vx, smooth_vy;

    (void)smooth_vx;
    (void)smooth_vy;
    return {goal_x2(2), goal_x2(3)};
  }

  /// 跟随者到安全边界的带符号距离（> 0 在圆外，<= 0 在圆内）。
  double distance_to_boundary(const Vec4d& x1, const Vec4d& x2)
  {
    double dx = x2(0) - x1(0);
    double dy = x2(1) - x1(1);
    return std::sqrt(dx * dx + dy * dy) - radius_;
  }

private:
  // --------------------------------------------------------------------------
  // 连续边界投影误差（map 帧）。
  //
  // 将跟随者相对位置径向投影到安全圆上，投影点作为期望相对位置。
  // 误差 = 实际相对位置 − 投影点，连续变化无需切换。
  // --------------------------------------------------------------------------
  Vec4d compute_error(const Vec4d& x1, const Vec4d& x2)
  {
    double dpx = x2(0) - x1(0);
    double dpy = x2(1) - x1(1);

    double r_dist = std::max(std::sqrt(dpx * dpx + dpy * dpy), 1e-3);
    double dx = radius_ * dpx / r_dist;
    double dy = radius_ * dpy / r_dist;

    Vec4d e;
    e << dpx - dx,
         dpy - dy,
         x2(2) - x1(2),
         x2(3) - x1(3);
    return e;
  }

  // --------------------------------------------------------------------------
  // HPC 参数重算。运行时失败静默保留旧参数（同原版 switch 处理）。
  // --------------------------------------------------------------------------
  void recompute_hpc()
  {
    auto res = lpc2hpc(A_, B_, k_lin_);
    if (!res.G0.isZero(1e-12)) {
      G0_ = res.G0;
      P_  = res.P;
      nu_ = res.nu_min;
      Gd_ = Mat4d::Identity() + nu_ * G0_;
    }
  }

  // --------------------------------------------------------------------------
  // 自适应线性增益（防超调，每通道独立）。
  // --------------------------------------------------------------------------
  Mat24d calculate_klin(const Vec4d& e)
  {
    double val_a = (std::abs(e(0)) > 1e-6) ? -mass_ * e(2) / e(0) : 0.0;
    double val_b = (std::abs(e(1)) > 1e-6) ? -mass_ * e(3) / e(1) : 0.0;

    double max_ratio = omega_d_ * mass_;
    val_a = std::clamp(val_a, -max_ratio, max_ratio);
    val_b = std::clamp(val_b, -max_ratio, max_ratio);

    double a = std::max(val_a, omega_d_ * mass_);
    double b = std::max(val_b, omega_d_ * mass_);

    double k2_00 = -2.0 * a;
    double k2_11 = -2.0 * b;
    double k1_00 = a * (k2_00 + a) / mass_;
    double k1_11 = b * (k2_11 + b) / mass_;

    Mat24d result;
    result << k1_00, 0,      k2_00, 0,
              0,      k1_11, 0,      k2_11;
    return result;
  }

  // ---- 参数 ----------------------------------------------------------------
  double radius_;
  double mass_;
  double omega_d_;

  // ---- 系统模型（双重积分器） ----------------------------------------------
  Mat4d  A_;
  Mat42d B_;

  // ---- 控制器状态 -----------------------------------------------------------
  Mat24d k_lin_;
  Mat4d  P_;
  Mat4d  G0_;
  Mat4d  Gd_;
  double nu_;
  bool   use_hpc_;
  bool   initialized_ = false;

  Eigen::Vector2d last_cmd_vel_;
};

}  // namespace formation_control
