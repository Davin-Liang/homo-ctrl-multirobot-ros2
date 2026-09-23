#pragma once

/// @file 无 Artstein/前向预测的 6D map-frame HPC 编队控制 ROS 2 节点。

#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "homo_multirobot_formation_control/homo_controller_6d_map_hpc_artstein.hpp"
#include "homo_multirobot_formation_control/kinematic_constraint.hpp"

class FormationController6DMapHpc : public rclcpp::Node
{
public:
  FormationController6DMapHpc();

private:
  struct State6D {
    Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
    Eigen::Vector2d v_map = Eigen::Vector2d::Zero();
  };

  void timer_cb();
  bool odom_to_state(const std::string& ns,
                     const nav_msgs::msg::Odometry::SharedPtr& odom,
                     State6D& state);

  static double wrap_angle(double a);
  static Eigen::Vector2d body_to_map(double yaw, const Eigen::Vector2d& v_body);
  static Eigen::Vector2d map_to_body(double yaw, const Eigen::Vector2d& v_map);

  std::string leader_ns_, follower_ns_;
  std::string state_source_;
  double mocap_state_timeout_ = 0.10;
  double control_rate_ = 20.0;
  double max_linear_vel_ = 1.0;
  double max_angular_vel_ = 0.5;
  double min_cmd_vel_ = 0.0;

  std::unique_ptr<formation_control::MapHpcController6DArtstein> ctrl_;
  formation_control::KinematicConstraint constraint_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr follower_sub_;
  nav_msgs::msg::Odometry::SharedPtr leader_odom_;
  nav_msgs::msg::Odometry::SharedPtr follower_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr leader_mocap_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr follower_mocap_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr leader_mocap_twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr follower_mocap_twist_sub_;
  geometry_msgs::msg::PoseStamped::SharedPtr leader_mocap_pose_, follower_mocap_pose_;
  geometry_msgs::msg::TwistStamped::SharedPtr leader_mocap_twist_, follower_mocap_twist_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time leader_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time follower_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time leader_mocap_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time follower_mocap_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_diag_time_{0, 0, RCL_ROS_TIME};
  int diag_tick_ = 0;
  double sum_leader_age_ = 0.0;
  double sum_ekf_age_ = 0.0;

  bool leader_ok_ = false;
  bool follower_ok_ = false;
  bool controller_initialized_ = false;
};
