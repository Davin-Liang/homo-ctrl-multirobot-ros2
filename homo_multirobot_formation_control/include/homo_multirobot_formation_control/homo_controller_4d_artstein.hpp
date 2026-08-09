#pragma once

/// @file 4D Artstein + forward-prediction wrapper around the original 4D HPC.
///
/// The HPC core remains the original nilpotent double-integrator controller:
///   x_h = [p_h, v_h],  p_dot = v,  v_dot = a,  A_h^2 = 0.
/// This wrapper only maps measured actuator states to the state seen by that HPC:
///   1. Artstein integral compensates input dead time Td for the actuator model.
///   2. exp(A_a*Td) maps the Artstein state back to delay-compensated actuator state.
///   3. Forward prediction over tau maps the actuator state to x_h.
///   4. LpcController computes the original 4D double-integrator HPC command.

#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <vector>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "homo_multirobot_formation_control/homo_controller.hpp"

namespace formation_control {

class LpcController4DArtstein {
public:
  LpcController4DArtstein(int m_p = 4, double radius = 2.0, double tol = 0.1,
                          double mass = 2.0, double tau_nominal = 0.43,
                          double omega_d = 0.7, bool use_hpc = true,
                          double control_period = 0.05, double hpc_c_min = 0.1,
                          double tau_min = 0.25, double tau_max = 0.55,
                          double v_tau_trans = 0.10, double Td = 0.22,
                          double initial_min_lambda = 1.0,
                          double switch_min_lambda = 4.0)
    : hpc_(m_p, radius, tol, mass, omega_d, use_hpc, hpc_c_min, control_period,
           initial_min_lambda, switch_min_lambda),
      tau_(tau_nominal), h_(control_period), Td_(Td)
  {
    (void)tau_min;
    (void)tau_max;
    (void)v_tau_trans;

    if (tau_ <= 0.0) {
      throw std::invalid_argument("LpcController4DArtstein: tau must be positive");
    }
    if (h_ <= 0.0) {
      throw std::invalid_argument("LpcController4DArtstein: control_period must be positive");
    }
    if (Td_ < 0.0) {
      throw std::invalid_argument("LpcController4DArtstein: Td must be non-negative");
    }

    build_actuator_kernels();
  }

  int artstein_buffer_size() const { return N_; }

  Eigen::Vector4d compute_artstein_integral(
      const std::deque<Eigen::Vector2d>& vcmd_history) const
  {
    Eigen::Vector4d integral = Eigen::Vector4d::Zero();
    if (Td_ <= 0.0) {
      return integral;
    }

    const int len = static_cast<int>(vcmd_history.size());
    for (int k = 0; k < N_ && k < len; ++k) {
      integral += artstein_kernels_[k] * vcmd_history[k] * weights_[k];
    }
    return integral;
  }

  Eigen::Vector4d predict_hpc_state(const Eigen::Vector4d& artstein_state,
                                    const Eigen::Vector2d& current_vcmd) const
  {
    const Eigen::Vector4d delay_free_state = exp_A_Td_ * artstein_state;
    const double decay = std::exp(-1.0);

    Eigen::Vector4d predicted;
    predicted.head<2>() = delay_free_state.head<2>() + current_vcmd * tau_
                        + tau_ * (1.0 - decay) * (delay_free_state.tail<2>() - current_vcmd);
    predicted.tail<2>() = current_vcmd + decay * (delay_free_state.tail<2>() - current_vcmd);
    return predicted;
  }

  Eigen::Vector4d predict_leader_state(const Eigen::Vector4d& measured_state) const
  {
    Eigen::Vector4d predicted = measured_state;
    predicted.head<2>() += measured_state.tail<2>() * (Td_ + tau_);
    return predicted;
  }

  void controller_initial(const Eigen::Vector4d& leader_hpc_state,
                          const Eigen::Vector4d& follower_hpc_state)
  {
    hpc_.controller_initial(leader_hpc_state, follower_hpc_state);
  }

  std::vector<double> lpc_calculate(const Eigen::Vector4d& leader_hpc_state,
                                    const Eigen::Vector4d& follower_hpc_state)
  {
    return hpc_.lpc_calculate(leader_hpc_state, follower_hpc_state);
  }

  Eigen::Vector2d accel_calculate(const Eigen::Vector4d& leader_hpc_state,
                                  const Eigen::Vector4d& follower_hpc_state)
  {
    return hpc_.accel_calculate(leader_hpc_state, follower_hpc_state);
  }

  double calculate_distance(const Eigen::Vector4d& leader_hpc_state,
                            const Eigen::Vector4d& follower_hpc_state)
  {
    return hpc_.calculate_distance(leader_hpc_state, follower_hpc_state);
  }

  int target_index() const
  {
    return hpc_.target_index();
  }

  double current_distance(const Eigen::Vector4d& leader_hpc_state,
                          const Eigen::Vector4d& follower_hpc_state) const
  {
    return hpc_.current_distance(leader_hpc_state, follower_hpc_state);
  }

  double best_distance(const Eigen::Vector4d& leader_hpc_state,
                       const Eigen::Vector4d& follower_hpc_state) const
  {
    return hpc_.best_distance(leader_hpc_state, follower_hpc_state);
  }

  Eigen::Vector4d selected_error(const Eigen::Vector4d& leader_hpc_state,
                                 const Eigen::Vector4d& follower_hpc_state) const
  {
    return hpc_.selected_error(leader_hpc_state, follower_hpc_state);
  }

private:
  void build_actuator_kernels()
  {
    N_ = std::max(1, static_cast<int>(std::ceil(Td_ / h_)));
    weights_.assign(N_, 0.0);
    artstein_kernels_.assign(N_, Eigen::Matrix<double, 4, 2>::Zero());

    Eigen::Matrix4d A;
    A << 0.0, 0.0, 1.0,       0.0,
         0.0, 0.0, 0.0,       1.0,
         0.0, 0.0, -1.0/tau_, 0.0,
         0.0, 0.0, 0.0,      -1.0/tau_;
    exp_A_Td_ = (A * Td_).exp();

    if (Td_ <= 0.0) {
      return;
    }

    Eigen::Matrix<double, 4, 2> B;
    B << 0.0,       0.0,
         0.0,       0.0,
         1.0/tau_,  0.0,
         0.0,       1.0/tau_;

    for (int k = 0; k < N_; ++k) {
      weights_[k] = (k < N_ - 1) ? h_ : Td_ - (N_ - 1) * h_;
      artstein_kernels_[k] = (A * (k * h_ - Td_)).exp() * B;
    }
  }

  LpcController hpc_;
  double tau_;
  double h_;
  double Td_;
  int N_ = 1;
  Eigen::Matrix4d exp_A_Td_ = Eigen::Matrix4d::Identity();
  std::vector<double> weights_;
  std::vector<Eigen::Matrix<double, 4, 2>> artstein_kernels_;
};

}  // namespace formation_control
