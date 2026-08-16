#pragma once

/// @file 4D Artstein-LQR 编队控制 ROS 2 节点 — TF + EKF 数据管线。

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <deque>
#include <memory>
#include <Eigen/Dense>

#include "homo_multirobot_formation_control/homo_controller_4d_artstein.hpp"
#include "homo_multirobot_formation_control/kinematic_constraint.hpp"
#include "homo_multirobot_formation_control/lqr_controller_4d_artstein.hpp"

class FormationController4DArtsteinLqr : public rclcpp::Node
{
public:
  FormationController4DArtsteinLqr();

private:
  void timer_cb();

  std::string leader_ns_, follower_ns_;
  double Kp_yaw_, K_ff_;
  double max_linear_vel_, max_angular_vel_;
  double max_linear_accel_ = 0.4;
  double min_cmd_vel_ = 0.0;
  double formation_radius_ = 2.0;
  double tau_ = 0.43;
  double Td_ = 0.22;
  double control_rate_ = 20.0;
  bool enable_radial_safety_ = true;

  std::unique_ptr<formation_control::LpcController4DArtstein> predictor_;
  std::unique_ptr<formation_control::LqrController4DArtstein> lqr_;
  formation_control::KinematicConstraint constraint_;

  double vx_cmd_map_ = 0.0;
  double vy_cmd_map_ = 0.0;

  std::deque<Eigen::Vector2d> leader_vcmd_history_;
  std::deque<Eigen::Vector2d> follower_vcmd_history_;

  double leader_vel_lpf_tau_ = 0.0;
  double lpf_leader_vx_ = 0.0, lpf_leader_vy_ = 0.0;
  bool leader_vel_filtered_ = false;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_sub_, follower_sub_;
  nav_msgs::msg::Odometry::SharedPtr leader_odom_, follower_odom_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time leader_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time follower_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_diag_time_{0, 0, RCL_ROS_TIME};
  int diag_tick_ = 0;
  double sum_leader_age_ = 0.0;
  double sum_ekf_age_ = 0.0;

  bool leader_ok_ = false, follower_ok_ = false;
  bool controller_initialized_ = false;
};
