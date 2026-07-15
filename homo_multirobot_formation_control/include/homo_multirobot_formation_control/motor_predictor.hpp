#pragma once

/// @file Smith 预估器：补偿电机响应延迟。
///
/// 电机模型：一阶低通 (时间常数 tau) + 纯传输延迟 (Td)。
/// 两个模型并行运行：
///   - 无延迟模型：cmd_vel → LP(tau) → v_nodelay （理想世界的速度）
///   - 有延迟模型：cmd_vel → delay(Td) → LP(tau) → v_delay （物理世界的速度）
///
/// 补偿量 = v_nodelay - v_delay （"已发出但还没变现"的速度）
/// 控制器看到的 = EKF实测 + 补偿量

#include <deque>
#include <vector>

namespace formation_control {

class MotorPredictor {
public:
  /// @param tau  一阶低通时间常数 (s)，实物 ~0.10–0.15
  /// @param Td   纯传输延迟 (s)，实物串口 ~0.03–0.08
  /// @param dt   控制周期 (s)，= 1/control_rate
  MotorPredictor(double tau = 0.12, double Td = 0.05, double dt = 0.05)
    : tau_(tau), Td_(Td), dt_(dt)
  {
    reset();
  }

  void reset() {
    vx_nodelay_ = 0.0;
    vy_nodelay_ = 0.0;
    vx_delay_   = 0.0;
    vy_delay_   = 0.0;
    history_.clear();
    int n = std::max(1, static_cast<int>(Td_ / dt_));
    for (int i = 0; i < n; ++i)
      history_.push_back({0.0, 0.0});
  }

  /// 每控制周期调用一次，输入本周期实际发给车的 cmd_vel
  void update(double vx_cmd, double vy_cmd) {
    double alpha = dt_ / (tau_ + dt_);  // 一阶 LP 系数

    // 无延迟模型：直接用最新 cmd_vel
    vx_nodelay_ += alpha * (vx_cmd - vx_nodelay_);
    vy_nodelay_ += alpha * (vy_cmd - vy_nodelay_);

    // 有延迟模型：用 Td 秒前的 cmd_vel
    history_.pop_back();
    history_.push_front({vx_cmd, vy_cmd});
    auto old = history_.back();  // Td 秒前的指令
    vx_delay_ += alpha * (old.first - vx_delay_);
    vy_delay_ += alpha * (old.second - vy_delay_);
  }

  /// 预估补偿后的速度 = EKF实测 + 补偿量
  double compensated_vx(double ekf_vx) const {
    return ekf_vx + (vx_nodelay_ - vx_delay_);
  }
  double compensated_vy(double ekf_vy) const {
    return ekf_vy + (vy_nodelay_ - vy_delay_);
  }

  // 诊断：只读
  double comp_vx() const { return vx_nodelay_ - vx_delay_; }
  double comp_vy() const { return vy_nodelay_ - vy_delay_; }
  double nodelay_vx() const { return vx_nodelay_; }
  double delay_vx() const { return vx_delay_; }

  void set_params(double tau, double Td) {
    tau_ = tau; Td_ = Td;
    reset();
  }

private:
  double tau_, Td_, dt_;
  double vx_nodelay_, vy_nodelay_;
  double vx_delay_, vy_delay_;
  std::deque<std::pair<double, double>> history_;
};

}  // namespace formation_control
