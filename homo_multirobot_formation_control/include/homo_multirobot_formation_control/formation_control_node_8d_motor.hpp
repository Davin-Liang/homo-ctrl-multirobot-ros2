#pragma once

/// @file 8D Pade 死区增广编队控制 ROS 2 节点 — TF + EKF 数据管线。
///
/// 与 6D Motor 节点（formation_control_node_6d_motor.hpp）的差异:
///   - 状态 8 维: [px, py, vx_cmd, vy_cmd, ωx, ωy, vx_real, vy_real]（map 系）
///   - ω 是新增内部积分状态（Pade 死区记忆）
///   - 不接 Smith 预估器（死区已内嵌在 A 矩阵中）
///   - 不接 sim_motor_delay 节点（cmd_vel 直接发布）

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include "homo_multirobot_formation_control/homo_controller_8d_motor.hpp"
#include "homo_multirobot_formation_control/kinematic_constraint.hpp"

class FormationController8DMotor : public rclcpp::Node
{
public:
  FormationController8DMotor();

private:
  void timer_cb();

  // ---- 参数 ----------------------------------------------------------------
  std::string leader_ns_, follower_ns_;
  double Kp_yaw_, K_ff_;
  double max_linear_vel_, max_angular_vel_;
  double min_cmd_vel_ = 0.03;
  double control_rate_;
  double Td_;

  // ---- 控制器 + 约束 -------------------------------------------------------
  std::unique_ptr<formation_control::LpcController8DMotor> ctrl_;
  formation_control::KinematicConstraint constraint_;

  // ---- v_cmd 内部状态（map 系）-----------------------------------------------
  double vx_cmd_map_ = 0.0;
  double vy_cmd_map_ = 0.0;

  // ---- ω 内部状态（map 系）—— Pade 死区记忆 --------------------------------
  double omega_x_map_ = 0.0;
  double omega_y_map_ = 0.0;

  // ---- leader 速度低通滤波 --------------------------------------------------
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
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- 诊断 -----------------------------------------------------------------
  rclcpp::Time leader_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time follower_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_diag_time_{0, 0, RCL_ROS_TIME};
  int diag_tick_ = 0;
  double sum_leader_age_ = 0.0;
  double sum_ekf_age_ = 0.0;

  bool leader_ok_ = false, follower_ok_ = false;
  bool controller_initialized_ = false;
};
