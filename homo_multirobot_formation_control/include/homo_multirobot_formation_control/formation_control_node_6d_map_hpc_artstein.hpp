#pragma once

/// @file 6D Artstein Disc 编队控制 ROS 2 节点。
///
/// 方向 A 架构：
///   1. EKF/TF 得到 map 位姿 + body 速度；
///   2. follower 平移通道在 map 系做 4D Artstein + tau 前向预测；
///   3. follower 偏航通道做 2D Artstein + tau 前向预测；
///   4. leader 使用常 twist 外推到 Td+tau；
///   5. 预测状态送入 6D Disc HPC 核心；
///   6. 发布 body-frame cmd_vel，并将实际发布命令回写历史。

#include <deque>
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "homo_multirobot_formation_control/homo_controller_6d_map_hpc_artstein.hpp"
#include "homo_multirobot_formation_control/kinematic_constraint.hpp"

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
                          const Eigen::VectorXd& current_cmd) const
  {
    Eigen::VectorXd delay_free = (A_ * Td_).exp() * artstein_state;
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

class FormationController6DMapHpcArtstein : public rclcpp::Node
{
public:
  FormationController6DMapHpcArtstein();

private:
  struct State6D {
    Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
    Eigen::Vector2d v_map = Eigen::Vector2d::Zero();
  };

  void timer_cb();
  void build_predictors(double tau_v, double tau_w, double Td, double dt);
  bool odom_to_state(const std::string& ns,
                     const nav_msgs::msg::Odometry::SharedPtr& odom,
                     State6D& state);
  Eigen::VectorXd predict_leader_state(const Eigen::VectorXd& x,
                                       double horizon) const;
  Eigen::VectorXd predict_follower_state(const State6D& measured);

  static double wrap_angle(double a);
  static Eigen::Vector2d body_to_map(double yaw, const Eigen::Vector2d& v_body);
  static Eigen::Vector2d map_to_body(double yaw, const Eigen::Vector2d& v_map);

  std::string leader_ns_, follower_ns_;
  double control_rate_ = 20.0;
  double tau_v_ = 0.43;
  double tau_w_ = 0.43;
  double Td_ = 0.22;
  double max_linear_vel_ = 1.0;
  double max_angular_vel_ = 0.5;
  double min_cmd_vel_ = 0.0;

  std::unique_ptr<formation_control::MapHpcController6DArtsteinDisc> ctrl_;
  formation_control::KinematicConstraint constraint_;
  formation_control::ArtsteinPredictorNd trans_predictor_;
  formation_control::ArtsteinPredictorNd yaw_predictor_;

  std::deque<Eigen::VectorXd> follower_vcmd_map_hist_;
  std::deque<Eigen::VectorXd> follower_wcmd_hist_;
  Eigen::Vector2d last_vcmd_map_ = Eigen::Vector2d::Zero();
  double last_wcmd_ = 0.0;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr follower_sub_;
  nav_msgs::msg::Odometry::SharedPtr leader_odom_;
  nav_msgs::msg::Odometry::SharedPtr follower_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time leader_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time follower_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_diag_time_{0, 0, RCL_ROS_TIME};
  int diag_tick_ = 0;
  double sum_leader_age_ = 0.0;
  double sum_ekf_age_ = 0.0;

  bool leader_ok_ = false;
  bool follower_ok_ = false;
  bool controller_initialized_ = false;
};
