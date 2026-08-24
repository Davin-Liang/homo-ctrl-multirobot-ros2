#pragma once

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "homo_multirobot_formation_control/formation_control_node_6d_artstein_disc.hpp"
#include "homo_multirobot_formation_control/hocbf_safety_filter.hpp"
#include "homo_multirobot_formation_control/kinematic_constraint.hpp"

class FormationController6DArtsteinDiscHocbf : public rclcpp::Node
{
public:
  FormationController6DArtsteinDiscHocbf();

private:
  struct State { Eigen::VectorXd x = Eigen::VectorXd::Zero(6); Eigen::Vector2d v_map; };
  void timer_cb();
  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  bool odom_to_state(const std::string& ns, const nav_msgs::msg::Odometry::SharedPtr& odom, State& state);
  Eigen::VectorXd predict_leader(const Eigen::VectorXd& x, double horizon) const;
  Eigen::VectorXd predict_follower(const State& measured);
  static Eigen::Vector2d body_to_map(double yaw, const Eigen::Vector2d& value);
  static Eigen::Vector2d map_to_body(double yaw, const Eigen::Vector2d& value);
  static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q);

  std::string leader_ns_, follower_ns_;
  double rate_, tau_, tau_yaw_, Td_, vmax_, wmax_, amax_;
  double follower_radius_, clearance_, perception_margin_, scan_timeout_;
  double passage_gain_, passage_activation_margin_, passage_release_margin_;
  bool use_latest_tf_fallback_ = true;
  double cluster_tolerance_, max_fit_residual_, min_cylinder_radius_, max_cylinder_radius_;
  int min_cluster_points_, max_obstacles_;
  std::unique_ptr<formation_control::LpcController6DArtsteinDisc> ctrl_;
  formation_control::KinematicConstraint constraint_;
  formation_control::ArtsteinPredictorNd trans_, yaw_;
  std::deque<Eigen::VectorXd> v_history_, w_history_;
  Eigen::Vector2d last_map_cmd_ = Eigen::Vector2d::Zero();
  double last_wcmd_ = 0.0;
  std::vector<formation_control::hocbf::Circle> obstacles_;
  formation_control::hocbf::PassageState passage_state_;
  rclcpp::Time last_scan_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_sub_, follower_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  nav_msgs::msg::Odometry::SharedPtr leader_odom_, follower_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool initialized_ = false;
};
