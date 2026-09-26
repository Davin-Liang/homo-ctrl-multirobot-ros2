#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

namespace formation_control {

struct ArtsteinPredictorNd {
  void build(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
             double tau, double Td, double h)
  {
    A_ = A;
    B_ = B;
    tau_ = tau;
    Td_ = Td;
    h_ = h;
    if (tau_ <= 0.0) {
      throw std::invalid_argument("6D Artstein Disc predictor: tau must be positive");
    }
    if (Td_ < 0.0) {
      throw std::invalid_argument("6D Artstein Disc predictor: Td must be non-negative");
    }
    if (h_ <= 0.0) {
      throw std::invalid_argument("6D Artstein Disc predictor: sample period must be positive");
    }
    N_ = std::max(1, static_cast<int>(std::ceil(Td_ / h_)));

    weights_.assign(N_, 0.0);
    kernels_.assign(N_, Eigen::MatrixXd::Zero(A_.rows(), B_.cols()));
    if (Td_ <= 0.0) {
      return;
    }

    for (int k = 0; k < N_; ++k) {
      weights_[k] = (k < N_ - 1) ? h_ : Td_ - (N_ - 1) * h_;
      kernels_[k] = (A_ * (k * h_ - Td_)).exp() * B_;
    }
  }

  int buffer_size() const { return N_; }

  Eigen::VectorXd integral(const std::deque<Eigen::VectorXd>& history) const
  {
    Eigen::VectorXd out = Eigen::VectorXd::Zero(A_.rows());
    if (Td_ <= 0.0) {
      return out;
    }
    int len = static_cast<int>(history.size());
    for (int k = 0; k < N_ && k < len; ++k) {
      out += kernels_[k] * history[k] * weights_[k];
    }
    return out;
  }

  Eigen::VectorXd predict(const Eigen::VectorXd& artstein_state,
                          const Eigen::VectorXd& current_cmd,
                          bool enable_forward_prediction = true) const
  {
    Eigen::VectorXd delay_free = (A_ * Td_).exp() * artstein_state;
    if (!enable_forward_prediction) {
      return delay_free;
    }

    int q = static_cast<int>(current_cmd.size());
    double decay = std::exp(-1.0);
    Eigen::VectorXd predicted(2 * q);
    predicted.head(q) = delay_free.head(q) + current_cmd * tau_
                      + tau_ * (1.0 - decay) * (delay_free.tail(q) - current_cmd);
    predicted.tail(q) = current_cmd + decay * (delay_free.tail(q) - current_cmd);
    return predicted;
  }

private:
  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
  double tau_ = 0.43;
  double Td_ = 0.22;
  double h_ = 0.05;
  int N_ = 1;
  std::vector<double> weights_;
  std::vector<Eigen::MatrixXd> kernels_;
};

}  // namespace formation_control
