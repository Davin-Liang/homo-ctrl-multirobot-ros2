#pragma once

#include <Eigen/Dense>

namespace formation_control
{

class LeaderCommandDeltaFeedforward
{
public:
  LeaderCommandDeltaFeedforward(bool enabled, double lpf_tau, double delta_max)
  : enabled_(enabled), lpf_tau_(lpf_tau), delta_max_(delta_max) {}

  void ingest(const Eigen::Vector2d& command_map, double received_sec)
  {
    last_received_sec_ = received_sec;
    if (!have_previous_command_) {
      previous_command_map_ = command_map;
      previous_received_sec_ = received_sec;
      have_previous_command_ = true;
      return;
    }

    raw_delta_ = command_map - previous_command_map_;
    previous_command_map_ = command_map;
    const double dt = received_sec - previous_received_sec_;
    previous_received_sec_ = received_sec;

    if (!have_filtered_delta_ || lpf_tau_ <= 0.0 || dt <= 0.0) {
      filtered_delta_ = raw_delta_;
      have_filtered_delta_ = true;
    } else {
      const double alpha = dt / (lpf_tau_ + dt);
      filtered_delta_ += alpha * (raw_delta_ - filtered_delta_);
    }

    const double magnitude = filtered_delta_.norm();
    if (delta_max_ > 0.0 && magnitude > delta_max_) {
      filtered_delta_ *= delta_max_ / magnitude;
    }
    pending_ = true;
  }

  Eigen::Vector2d consume(double now_sec, double timeout_sec)
  {
    if (!enabled_ || !pending_ || now_sec - last_received_sec_ > timeout_sec) {
      pending_ = false;
      return Eigen::Vector2d::Zero();
    }
    pending_ = false;
    return filtered_delta_;
  }

  Eigen::Vector2d raw_delta() const { return raw_delta_; }
  Eigen::Vector2d filtered_delta() const { return filtered_delta_; }
  double last_received_sec() const { return last_received_sec_; }

private:
  bool enabled_ = false;
  double lpf_tau_ = 0.0;
  double delta_max_ = 0.0;
  bool have_previous_command_ = false;
  bool have_filtered_delta_ = false;
  bool pending_ = false;
  double previous_received_sec_ = 0.0;
  double last_received_sec_ = 0.0;
  Eigen::Vector2d previous_command_map_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d raw_delta_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d filtered_delta_ = Eigen::Vector2d::Zero();
};

}  // namespace formation_control
