#pragma once

#include <algorithm>
#include <cmath>

namespace formation_control {

/// 三轮全向底盘运动学约束。
///
/// 根据 URDF 真实几何参数（L=0.11m 底盘半径, r=0.03m 轮半径, 120° 均布）
/// 对 cmd_vel 输出做轮速限幅和加速度限幅。

class KinematicConstraint {
public:
  KinematicConstraint(double wheel_radius = 0.03, double base_radius = 0.11,
                      double wheel_max_omega = 20.0,
                      double max_linear_accel = 2.0, double max_angular_accel = 4.0)
    : r_(wheel_radius), L_(base_radius), omega_max_(wheel_max_omega),
      max_lin_accel_(max_linear_accel), max_ang_accel_(max_angular_accel)
  {
    c_ = std::cos(M_PI / 6.0);  // cos(30°) = √3/2
    s_ = std::sin(M_PI / 6.0);  // sin(30°) = 1/2
    first_call_ = true;
  }

  /// 对 (vx, vy, omega) 依次施加轮速约束和加速度约束，返回修正后的三元组。
  /// @return 轮速缩放因子（1.0 = 未触发, < 1.0 = 触发等比缩放）
  double apply(double& vx, double& vy, double& omega, double dt)
  {
    double scale = apply_wheel_speed(vx, vy, omega);
    apply_acceleration(vx, vy, omega, dt);
    return scale;
  }

  void set_wheel_max_omega(double val) { omega_max_ = val; }
  void set_accel_limits(double lin, double ang) { max_lin_accel_ = lin; max_ang_accel_ = ang; }

private:
  // ---- 轮速约束 ---------------------------------------------------------------
  double apply_wheel_speed(double& vx, double& vy, double& omega)
  {
    const double w1 = ( vy + L_ * omega) / r_;
    const double w2 = (-c_ * vx - s_ * vy + L_ * omega) / r_;
    const double w3 = ( c_ * vx - s_ * vy + L_ * omega) / r_;

    double w_max = std::max({std::abs(w1), std::abs(w2), std::abs(w3)});
    if (w_max > omega_max_) {
      double scale = omega_max_ / w_max;
      vx    *= scale;
      vy    *= scale;
      omega *= scale;
      return scale;
    }
    return 1.0;
  }

  // ---- 加速度约束（分量 slew rate limit）--------------------------------------
  void apply_acceleration(double& vx, double& vy, double& omega, double dt)
  {
    if (first_call_) {
      first_call_ = false;
      prev_vx_ = vx;
      prev_vy_ = vy;
      prev_omega_ = omega;
      return;
    }

    double dvx_max = max_lin_accel_ * dt;
    double domg_max = max_ang_accel_ * dt;

    vx    = std::clamp(vx,    prev_vx_    - dvx_max,  prev_vx_    + dvx_max);
    vy    = std::clamp(vy,    prev_vy_    - dvx_max,  prev_vy_    + dvx_max);
    omega = std::clamp(omega, prev_omega_ - domg_max, prev_omega_ + domg_max);

    prev_vx_    = vx;
    prev_vy_    = vy;
    prev_omega_ = omega;
  }

  double r_, L_, c_, s_;
  double omega_max_;
  double max_lin_accel_, max_ang_accel_;
  bool   first_call_;
  double prev_vx_ = 0.0, prev_vy_ = 0.0, prev_omega_ = 0.0;
};

}  // namespace formation_control
