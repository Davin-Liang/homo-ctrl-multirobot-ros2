#pragma once

/// @file 6D 电机感知模型编队控制 ROS 2 节点 — 4D Artstein 预测 + 6D HPC 级联。
///
/// 架构:
///   4D Artstein 层: v_cmd 历史 → 积分 I(t) → z = x_4d + I（死区补偿）
///   6D Motor HPC 层: 以 Artstein z 替换 [p, v_real] 测量，v_cmd 积分态不变
///
/// 与纯 6D Motor 节点（原版）的差异:
///   - 使用 4D Artstein 预测替代 Smith 预估器做死区补偿
///   - Artstein 预测 z 嵌入 6D 状态: x = [z_p, v_cmd, z_vreal]
///   - v_cmd 积分态 + HPC 完全不变（A_6, B_6, K, G0, P, ν, Gd 同原版）
///   - 自适应 τ 同步更新 4D Artstein 核和 6D A_6 矩阵

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <deque>
#include <memory>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "homo_multirobot_formation_control/homo_controller_6d_motor.hpp"
#include "homo_multirobot_formation_control/kinematic_constraint.hpp"

namespace formation_control {

/// 4D Artstein 预测器 — 将 v_cmd 历史转换为死区补偿的状态预测。
///
/// 模型: x = [p, v_real] (4D), u = v_cmd (2D 速度指令)
///   dp/dt = v_real,  dv_real/dt = (u(t−Td) − v_real)/τ
///
/// 预计算积分核 e^{A(kh−Td)}·B (k=0..N-1) 和截断权重 w_k。
struct ArtsteinPredictor4D {
  double tau_ = 0.43;
  double Td_  = 0.22;
  double h_   = 0.05;
  int    N_   = 5;

  Eigen::MatrixXd A_;  // 4×4
  Eigen::MatrixXd B_;  // 4×2
  std::vector<double> weights_;
  std::vector<Eigen::MatrixXd> kernels_;  // e^{A(kh−Td)}·B, 4×2 each

  void build(double tau, double Td, double h)
  {
    tau_ = tau; Td_ = Td; h_ = h;

    A_.resize(4, 4);
    A_ << 0, 0,  1,          0,
          0, 0,  0,          1,
          0, 0, -1.0 / tau_, 0,
          0, 0,  0,         -1.0 / tau_;

    B_.resize(4, 2);
    B_ << 0, 0,
          0, 0,
          1.0 / tau_, 0,
          0, 1.0 / tau_;

    N_ = std::max(1, static_cast<int>(std::ceil(Td_ / h_)));

    weights_.resize(N_);
    for (int k = 0; k < N_ - 1; ++k) weights_[k] = h_;
    weights_[N_ - 1] = Td_ - (N_ - 1) * h_;

    kernels_.resize(N_);
    for (int k = 0; k < N_; ++k) {
      double arg = k * h_ - Td_;
      kernels_[k] = (A_ * arg).exp() * B_;
    }
  }

  int buffer_size() const { return N_; }

  /// vcmd_hist[0] = v_cmd(t) (最新), vcmd_hist[N-1] = v_cmd(t−(N-1)h)
  Eigen::Vector4d compute(const std::deque<Eigen::Vector2d>& vcmd_hist) const
  {
    Eigen::Vector4d I = Eigen::Vector4d::Zero();
    int len = static_cast<int>(vcmd_hist.size());
    for (int k = 0; k < N_ && k < len; ++k)
      I += kernels_[k] * vcmd_hist[k] * weights_[k];
    return I;
  }
};

}  // namespace formation_control


class FormationController6DMotor : public rclcpp::Node
{
public:
  FormationController6DMotor();

private:
  void timer_cb();

  // ---- 参数 ----------------------------------------------------------------
  std::string leader_ns_, follower_ns_;
  double Kp_yaw_, K_ff_;
  double max_linear_vel_, max_angular_vel_;
  double min_cmd_vel_ = 0.03;
  double control_rate_;

  // ---- 控制器 + 约束 -------------------------------------------------------
  std::unique_ptr<formation_control::LpcController6DMotor> ctrl_;
  formation_control::KinematicConstraint constraint_;

  // ---- 4D Artstein 预测器 --------------------------------------------------
  formation_control::ArtsteinPredictor4D artstein_;
  double Td_ = 0.22;
  std::deque<Eigen::Vector2d> leader_vcmd_hist_;
  std::deque<Eigen::Vector2d> follower_vcmd_hist_;

  // ---- v_cmd 内部状态（map 系）----------------------------------------------
  double vx_cmd_map_ = 0.0;
  double vy_cmd_map_ = 0.0;

  // ---- leader 速度低通滤波 -------------------------------------------------
  double leader_vel_lpf_tau_ = 0.3;
  double lpf_leader_vx_ = 0.0, lpf_leader_vy_ = 0.0;
  bool leader_vel_filtered_ = false;

  // ---- TF ------------------------------------------------------------------
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // ---- EKF 里程计订阅 ------------------------------------------------------
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_sub_, follower_sub_;
  nav_msgs::msg::Odometry::SharedPtr leader_odom_, follower_odom_;

  // ---- 发布 ----------------------------------------------------------------
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  // ---- 定时器 --------------------------------------------------------------
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- 诊断 -----------------------------------------------------------------
  rclcpp::Time leader_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time follower_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_diag_time_{0, 0, RCL_ROS_TIME};
  int diag_tick_ = 0;
  double sum_leader_age_ = 0.0;
  double sum_ekf_age_ = 0.0;

  // ---- 状态标志 ------------------------------------------------------------
  bool leader_ok_ = false, follower_ok_ = false;
  bool controller_initialized_ = false;
};
