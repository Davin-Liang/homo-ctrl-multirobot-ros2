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
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "homo_multirobot_formation_control/homo_controller_6d_map_hpc_artstein.hpp"
#include "homo_multirobot_formation_control/artstein_predictor_nd.hpp"
#include "homo_multirobot_formation_control/kinematic_constraint.hpp"
#include "homo_multirobot_formation_control/leader_command_delta_feedforward.hpp"

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
  std::string state_source_;
  double mocap_state_timeout_ = 0.10;
  double control_rate_ = 20.0;
  double tau_v_ = 0.43;
  double tau_w_ = 0.43;
  double Td_ = 0.22;
  bool enable_artstein_compensation_ = true;
  bool enable_forward_prediction_ = true;
  double max_linear_vel_ = 1.0;
  double max_angular_vel_ = 0.5;
  double min_cmd_vel_ = 0.0;
  double leader_cmd_timeout_ = 0.15;
  double leader_cmd_delta_lpf_tau_ = 0.10;
  bool enable_leader_cmd_feedforward_ = false;

  std::unique_ptr<formation_control::MapHpcController6DArtstein> ctrl_;
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
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr leader_cmd_sub_;
  nav_msgs::msg::Odometry::SharedPtr leader_odom_;
  nav_msgs::msg::Odometry::SharedPtr follower_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr leader_mocap_pose_sub_, follower_mocap_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr leader_mocap_twist_sub_, follower_mocap_twist_sub_;
  geometry_msgs::msg::PoseStamped::SharedPtr leader_mocap_pose_, follower_mocap_pose_;
  geometry_msgs::msg::TwistStamped::SharedPtr leader_mocap_twist_, follower_mocap_twist_;
  geometry_msgs::msg::Twist::SharedPtr leader_cmd_;
  rclcpp::Time leader_mocap_received_{0, 0, RCL_ROS_TIME}, follower_mocap_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time leader_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time follower_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time leader_cmd_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_diag_time_{0, 0, RCL_ROS_TIME};
  int diag_tick_ = 0;
  double sum_leader_age_ = 0.0;
  double sum_ekf_age_ = 0.0;

  bool leader_ok_ = false;
  bool follower_ok_ = false;
  bool controller_initialized_ = false;
  uint64_t leader_cmd_sequence_ = 0;
  uint64_t processed_leader_cmd_sequence_ = 0;
  std::unique_ptr<formation_control::LeaderCommandDeltaFeedforward>
      leader_cmd_feedforward_;
};
