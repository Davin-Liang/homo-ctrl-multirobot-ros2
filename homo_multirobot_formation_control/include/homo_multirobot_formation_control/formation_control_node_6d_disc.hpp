#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include "homo_multirobot_formation_control/homo_controller_6d_disc.hpp"
#include "homo_multirobot_formation_control/kinematic_constraint.hpp"

class FormationController6DDisc : public rclcpp::Node
{
public:
  FormationController6DDisc();

private:
  void timer_cb();

  std::string leader_ns_, follower_ns_;
  double control_rate_;
  double max_linear_vel_, max_angular_vel_;

  std::unique_ptr<formation_control::LpcController6DDisc> ctrl_;
  formation_control::KinematicConstraint constraint_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_sub_, follower_sub_;
  nav_msgs::msg::Odometry::SharedPtr leader_odom_, follower_odom_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  bool leader_ok_ = false, follower_ok_ = false;
  bool controller_initialized_ = false;
};
