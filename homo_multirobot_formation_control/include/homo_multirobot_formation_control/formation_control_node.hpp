#pragma once

/// @file 编队控制 ROS 2 节点 — TF + EKF 数据管线。
///
/// 位置 + 偏航角来自 TF（map → <prefix>_base_footprint），
/// 速度来自 EKF odometry/filtered（本体帧旋转到 map 帧），
/// 角速度来自 EKF（偏航前馈）。
///
/// 所有数据统一变换到 map 帧后送入齐次控制器，
/// 控制器逻辑与 Python 原版等价。

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include "homo_multirobot_formation_control/homo_controller.hpp"
#include "homo_multirobot_formation_control/kinematic_constraint.hpp"

class FormationController : public rclcpp::Node
{
public:
  FormationController();

private:
  void timer_cb();  // 20 Hz 控制循环

  // ---- 参数 ----------------------------------------------------------------
  std::string leader_ns_, follower_ns_;
  double Kp_yaw_, K_ff_;
  double control_rate_;

  // ---- 控制器 + 约束 -------------------------------------------------------
  std::unique_ptr<formation_control::LpcController> ctrl_;
  formation_control::KinematicConstraint constraint_;

  // ---- TF ------------------------------------------------------------------
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // ---- EKF 里程计订阅 ------------------------------------------------------
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_sub_, follower_sub_;

  // 最新缓冲消息（回调更新，定时器读取）
  nav_msgs::msg::Odometry::SharedPtr leader_odom_, follower_odom_;

  // ---- 发布 ----------------------------------------------------------------
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  // ---- 定时器 --------------------------------------------------------------
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- 状态标志 ------------------------------------------------------------
  bool leader_ok_ = false, follower_ok_ = false;
  bool controller_initialized_ = false;
};
