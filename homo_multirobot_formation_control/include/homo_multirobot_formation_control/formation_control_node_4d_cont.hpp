#pragma once

/// @file 4D 连续边界投影编队控制 ROS 2 节点。
///
/// 数据线与 FormationController 完全相同：
///   - 位置 + 偏航角来自 TF（map → <prefix>_base_footprint）
///   - 速度来自 EKF odometry/filtered（本体帧旋转到 map 帧）
///   - 角速度来自 EKF（偏航前馈）
///
/// 使用 LpcController4DCont 替代 LpcController，编队策略为连续边界投影。

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include "homo_multirobot_formation_control/homo_controller_4d_cont.hpp"
#include "homo_multirobot_formation_control/kinematic_constraint.hpp"

class FormationController4DCont : public rclcpp::Node
{
public:
  FormationController4DCont();

private:
  void timer_cb();

  // ---- 参数 ----------------------------------------------------------------
  std::string leader_ns_, follower_ns_;
  double Kp_yaw_, K_ff_;
  double control_rate_;

  // ---- 控制器 + 约束 -------------------------------------------------------
  std::unique_ptr<formation_control::LpcController4DCont> ctrl_;
  formation_control::KinematicConstraint constraint_;

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

  // ---- 状态标志 ------------------------------------------------------------
  bool leader_ok_ = false, follower_ok_ = false;
  bool controller_initialized_ = false;
};
