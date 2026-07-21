#pragma once

/// @file 6D 电机感知模型编队控制 ROS 2 节点 — TF + EKF 数据管线。
///
/// 与 4D 节点（formation_control_node.hpp）的差异:
///   - 状态 6 维: [px, py, vx_cmd, vy_cmd, vx_real, vy_real]（map 系）
///   - v_cmd 是内部积分状态: 初始化对齐 EKF，之后每周期发布 cmd_vel 后
///     把最终发布值（clamp + 轮速约束后）旋转回 map 系回写（抗饱和）
///   - leader 的 v_cmd = v_real = EKF 测量速度（稳态假设）
///   - 不接 Smith 预估器（电机滞后已进模型）

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include "homo_multirobot_formation_control/homo_controller_6d_motor.hpp"
#include "homo_multirobot_formation_control/kinematic_constraint.hpp"
#include "homo_multirobot_formation_control/motor_predictor.hpp"

class FormationController6DMotor : public rclcpp::Node
{
public:
  FormationController6DMotor();

private:
  void timer_cb();  // 20 Hz 控制循环

  // ---- 参数 ----------------------------------------------------------------
  std::string leader_ns_, follower_ns_;
  double Kp_yaw_, K_ff_;
  double max_linear_vel_, max_angular_vel_;
  double min_cmd_vel_ = 0.03;  // 实物 STM32 死区 ~0.03 m/s, 指令低于此值不输出
  double control_rate_;

  // ---- 控制器 + 约束 + 预估器 -----------------------------------------------
  std::unique_ptr<formation_control::LpcController6DMotor> ctrl_;
  formation_control::KinematicConstraint constraint_;
  formation_control::MotorPredictor motor_predictor_;
  bool use_smith_predictor_ = false;

  // ---- v_cmd 内部状态（map 系）----------------------------------------------
  // 初始化时对齐 EKF 速度，之后由发布后的最终 cmd_vel 回写维护。
  double vx_cmd_map_ = 0.0;
  double vy_cmd_map_ = 0.0;

  // ---- leader 速度低通滤波 ----------------------------------------------
  // rf2o vy 噪声 ~0.05m/s 被 k2+k3 放大为虚假控制力，低速时尤其明显。
  double leader_vel_lpf_tau_ = 0.3;     // 滤波器时间常数 (s)
  double lpf_leader_vx_ = 0.0, lpf_leader_vy_ = 0.0;
  bool leader_vel_filtered_ = false;

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
