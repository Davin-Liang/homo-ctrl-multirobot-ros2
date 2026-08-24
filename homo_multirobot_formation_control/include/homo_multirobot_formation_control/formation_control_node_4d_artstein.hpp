#pragma once

/// @file 4D Artstein-HPC 编队控制 ROS 2 节点 — TF + EKF 数据管线。
///
/// 与 6D Motor 节点（formation_control_node_6d_motor.hpp）的差异:
///   - 状态 4 维: [px, py, vx_real, vy_real]（map 系）
///   - v^cmd 是控制输入，不是状态——控制器直接输出速度指令
///   - Artstein 预测: z = x + I(t)，其中 I(t) 是过去 Td 内 v^cmd 的累积贡献
///   - leader 和 follower 各维护独立的 v^cmd 环形缓冲
///   - leader v^cmd 由 EKF 测量速度近似（稳态假设，同 6D Motor）
///   - 加速度限幅在节点侧作为执行器速率约束（同 6D Motor 的 constraint_.apply）
///   - 不使用 Smith 预估器（死区已通过 Artstein 约简内嵌）

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

class FormationController4DArtstein : public rclcpp::Node
{
public:
  FormationController4DArtstein();

private:
  void timer_cb();

  // ---- 参数 ----------------------------------------------------------------
  std::string leader_ns_, follower_ns_;
  double Kp_yaw_, K_ff_;
  double max_linear_vel_, max_angular_vel_;
  double max_linear_accel_ = 2.0;
  double radial_safety_max_decel_ = 0.0;
  double radial_safety_effective_delay_ = -1.0;
  double formation_radius_ = 2.0;
  double tau_ = 0.43;
  double min_cmd_vel_ = 0.03;
  double Td_;       // 死区时延 (s)
  double control_rate_;
  bool enable_radial_safety_ = true;

  // ---- 控制器 + 约束 -------------------------------------------------------
  std::unique_ptr<formation_control::LpcController4DArtstein> ctrl_;
  formation_control::KinematicConstraint constraint_;

  // ---- v^cmd 内部状态（map 系，回写 + 入缓冲用）---------------------------
  double vx_cmd_map_ = 0.0;
  double vy_cmd_map_ = 0.0;

  // ---- Artstein v^cmd 环形缓冲（leader + follower 各一）-------------------
  std::deque<Eigen::Vector2d> leader_vcmd_history_;
  std::deque<Eigen::Vector2d> follower_vcmd_history_;

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
