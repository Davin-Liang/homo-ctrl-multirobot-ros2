/// @file 4D Artstein-LQR 编队控制节点实现。
///
/// 数据流与 4D Artstein-MPC 节点一致，只替换上层平移控制律:
///   EKF/TF -> Artstein Td 补偿 -> 一阶电机 tau 前向预测
///          -> 4D DARE-LQR -> cmd_vel 后处理/发布。

#include "homo_multirobot_formation_control/formation_control_node_4d_artstein_lqr.hpp"

#include <algorithm>
#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

#include "homo_multirobot_formation_control/formation_safety.hpp"

using namespace formation_control;

static double tf2_yaw(const tf2::Quaternion& q)
{
  double r, p, y;
  tf2::Matrix3x3(q).getRPY(r, p, y);
  return y;
}

static double msg_yaw(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

static bool ekf_to_map(tf2_ros::Buffer& tf, const std::string& ns,
                       const nav_msgs::msg::Odometry::SharedPtr& odom,
                       double& px, double& py, double& vx_meas, double& vy_meas,
                       double& map_yaw, double& angular_z)
{
  if (!odom) return false;

  std::string odom_frame = ns;
  if (!odom_frame.empty() && odom_frame[0] == '/') odom_frame = odom_frame.substr(1);
  odom_frame += "_odom";

  try {
    auto t = tf.lookupTransform("map", odom_frame, tf2::TimePoint());
    const double tf_x = t.transform.translation.x;
    const double tf_y = t.transform.translation.y;
    const double tf_yaw_angle = tf2_yaw(tf2::Quaternion(
        t.transform.rotation.x, t.transform.rotation.y,
        t.transform.rotation.z, t.transform.rotation.w));

    const double ekf_px = odom->pose.pose.position.x;
    const double ekf_py = odom->pose.pose.position.y;
    const double ekf_yaw = msg_yaw(odom->pose.pose.orientation);

    px = tf_x + ekf_px * std::cos(tf_yaw_angle) - ekf_py * std::sin(tf_yaw_angle);
    py = tf_y + ekf_px * std::sin(tf_yaw_angle) + ekf_py * std::cos(tf_yaw_angle);

    map_yaw = tf_yaw_angle + ekf_yaw;
    const double vx_body = odom->twist.twist.linear.x;
    const double vy_body = odom->twist.twist.linear.y;
    vx_meas = vx_body * std::cos(map_yaw) - vy_body * std::sin(map_yaw);
    vy_meas = vx_body * std::sin(map_yaw) + vy_body * std::cos(map_yaw);

    angular_z = odom->twist.twist.angular.z;
    return true;
  } catch (const tf2::TransformException&) {
    return false;
  }
}

FormationController4DArtsteinLqr::FormationController4DArtsteinLqr()
: rclcpp::Node("formation_control_node_4d_artstein_lqr")
{
  leader_ns_ = declare_parameter("leader_ns", "/robot1");
  follower_ns_ = declare_parameter("follower_ns", "/robot2");
  const int m_p = declare_parameter("m_p", 4);
  const double radius = declare_parameter("radius", 2.0);
  const double tol = declare_parameter("tol", 0.1);
  const double mass = declare_parameter("mass", 2.0);
  const double tau = declare_parameter("tau", 0.43);
  const double tau_min = declare_parameter("tau_min", 0.25);
  const double tau_max = declare_parameter("tau_max", 0.55);
  const double v_tau_trans = declare_parameter("v_tau_trans", 0.10);

  Kp_yaw_ = declare_parameter("Kp_yaw", 4.0);
  K_ff_ = declare_parameter("K_ff", 1.0);
  control_rate_ = declare_parameter("control_rate", 20.0);
  Td_ = declare_parameter("Td", 0.22);

  const double wheel_radius = declare_parameter("wheel_radius", 0.03);
  const double base_radius = declare_parameter("base_radius", 0.11);
  const double wheel_max_omega = declare_parameter("wheel_max_omega", 20.0);
  const double max_linear_accel = declare_parameter("max_linear_accel", 0.4);
  const double max_angular_accel = declare_parameter("max_angular_accel", 4.0);

  max_linear_vel_ = declare_parameter("max_linear_vel", 0.5);
  max_angular_vel_ = declare_parameter("max_angular_vel", 0.5);
  max_linear_accel_ = max_linear_accel;
  formation_radius_ = radius;
  tau_ = tau;
  leader_vel_lpf_tau_ = declare_parameter("leader_vel_lpf_tau", 0.0);
  min_cmd_vel_ = declare_parameter("min_cmd_vel", 0.0);
  enable_radial_safety_ = declare_parameter("enable_radial_safety", true);

  predictor_ = std::make_unique<LpcController4DArtstein>(
      m_p, radius, tol, mass, tau, 0.7, false, 1.0 / control_rate_, 0.1,
      tau_min, tau_max, v_tau_trans, Td_, 1.0, 4.0);

  LqrController4DArtstein::Params lqr_params;
  lqr_params.dt = 1.0 / control_rate_;
  lqr_params.mass = mass;
  lqr_params.formation_radius = radius;
  lqr_params.m_p = m_p;
  lqr_params.tol = tol;
  lqr_params.q_px = declare_parameter("q_px", 40.0);
  lqr_params.q_py = declare_parameter("q_py", 40.0);
  lqr_params.q_vx = declare_parameter("q_vx", 1.0);
  lqr_params.q_vy = declare_parameter("q_vy", 1.0);
  lqr_params.r_ux = declare_parameter("r_ux", 0.02);
  lqr_params.r_uy = declare_parameter("r_uy", 0.02);
  lqr_params.dare_max_iter = declare_parameter("dare_max_iter", 10000);
  lqr_params.dare_tol = declare_parameter("dare_tol", 1e-12);
  lqr_ = std::make_unique<LqrController4DArtstein>(lqr_params);

  constraint_ = KinematicConstraint(wheel_radius, base_radius,
                                    wheel_max_omega,
                                    max_linear_accel, max_angular_accel);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  auto qos = rclcpp::SensorDataQoS();
  leader_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    leader_ns_ + "/odometry/filtered", qos,
    [this](nav_msgs::msg::Odometry::SharedPtr m) {
      leader_odom_ = m;
      leader_ok_ = true;
      leader_odom_stamp_ = m->header.stamp;
    });
  follower_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    follower_ns_ + "/odometry/filtered", qos,
    [this](nav_msgs::msg::Odometry::SharedPtr m) {
      follower_odom_ = m;
      follower_ok_ = true;
      follower_odom_stamp_ = m->header.stamp;
    });

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  last_diag_time_ = get_clock()->now();
  const int ms = static_cast<int>(1000.0 / control_rate_);
  timer_ = create_wall_timer(std::chrono::milliseconds(ms), [this]() { timer_cb(); });

  RCLCPP_INFO(get_logger(),
              "4D Artstein-LQR node started (Td=%.3fs, tau=%.2fs, Qp=%.1f, R=%.3f).",
              Td_, tau, lqr_params.q_px, lqr_params.r_ux);
  RCLCPP_INFO(get_logger(), "  Leader: %s, Follower: %s",
              leader_ns_.c_str(), follower_ns_.c_str());
}

void FormationController4DArtsteinLqr::timer_cb()
{
  if (!leader_ok_ || !follower_ok_) return;

  double l_px, l_py, l_vx, l_vy, leader_yaw, leader_az;
  double f_px, f_py, f_vx, f_vy, follower_yaw, follower_az;
  if (!ekf_to_map(*tf_buffer_, leader_ns_, leader_odom_,
                  l_px, l_py, l_vx, l_vy, leader_yaw, leader_az)) return;
  if (!ekf_to_map(*tf_buffer_, follower_ns_, follower_odom_,
                  f_px, f_py, f_vx, f_vy, follower_yaw, follower_az)) return;

  double l_vx_f, l_vy_f;
  if (leader_vel_lpf_tau_ <= 0.0) {
    l_vx_f = l_vx;
    l_vy_f = l_vy;
  } else {
    if (!leader_vel_filtered_) {
      lpf_leader_vx_ = l_vx;
      lpf_leader_vy_ = l_vy;
      leader_vel_filtered_ = true;
    } else {
      const double alpha = control_rate_ / (control_rate_ + 1.0 / leader_vel_lpf_tau_);
      lpf_leader_vx_ += alpha * (l_vx - lpf_leader_vx_);
      lpf_leader_vy_ += alpha * (l_vy - lpf_leader_vy_);
    }
    l_vx_f = lpf_leader_vx_;
    l_vy_f = lpf_leader_vy_;
  }

  const int buf_size = predictor_->artstein_buffer_size();

  Eigen::Vector4d x1_meas;
  x1_meas << l_px, l_py, l_vx_f, l_vy_f;
  Eigen::Vector4d x1_h = predictor_->predict_leader_state(x1_meas);

  if (!controller_initialized_) {
    vx_cmd_map_ = f_vx;
    vy_cmd_map_ = f_vy;

    follower_vcmd_history_.clear();
    for (int i = 0; i < buf_size; ++i) {
      follower_vcmd_history_.push_back(Eigen::Vector2d(f_vx, f_vy));
    }

    Eigen::Vector4d x2_meas;
    x2_meas << f_px, f_py, f_vx, f_vy;
    const Eigen::Vector4d I2_init = predictor_->compute_artstein_integral(follower_vcmd_history_);
    const Eigen::Vector4d z2_init = x2_meas + I2_init;
    const Eigen::Vector4d x2_h_init =
        predictor_->predict_hpc_state(z2_init, Eigen::Vector2d(vx_cmd_map_, vy_cmd_map_));

    try {
      lqr_->init(x1_h, x2_h_init);
      controller_initialized_ = true;
      RCLCPP_INFO(get_logger(), "4D Artstein-LQR 控制器初始化完成。");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "初始化失败: %s", e.what());
      return;
    }
  }

  Eigen::Vector4d x2_meas;
  x2_meas << f_px, f_py, f_vx, f_vy;
  const Eigen::Vector4d I2 = predictor_->compute_artstein_integral(follower_vcmd_history_);
  const Eigen::Vector4d z2 = x2_meas + I2;
  Eigen::Vector4d x2_h =
      predictor_->predict_hpc_state(z2, Eigen::Vector2d(vx_cmd_map_, vy_cmd_map_));

  Eigen::Vector4d x1_real;
  x1_real << l_px, l_py, l_vx_f, l_vy_f;
  Eigen::Vector4d x2_real;
  x2_real << f_px, f_py, f_vx, f_vy;

  Eigen::Vector2d out_map = lqr_->compute_velocity_command(x1_h, x2_h);

  if (enable_radial_safety_) {
    const Eigen::Vector2d out_before = out_map;
    out_map = formation_control::apply_radial_safety_limit(
        out_map, x1_real.tail<2>(), x1_real.head<2>(), x2_real.head<2>(),
        formation_radius_, max_linear_accel_, Td_ + tau_, max_linear_vel_);

    const Eigen::Vector2d rel = x2_real.head<2>() - x1_real.head<2>();
    const double dist = rel.norm();
    if ((out_map - out_before).norm() > 1e-6 && dist > 1e-9) {
      const Eigen::Vector2d radial = rel / dist;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "RADIAL_SAFE dist=%.3f radius=%.3f vrel_rad %.3f->%.3f Td_eff=%.3f",
        dist, formation_radius_,
        (out_before - x1_real.tail<2>()).dot(radial),
        (out_map - x1_real.tail<2>()).dot(radial),
        Td_ + tau_);
    }
  }

  const double current_dist = lqr_->current_distance(x1_h, x2_h);
  const double best_dist = lqr_->best_distance(x1_h, x2_h);
  const Eigen::Vector4d selected_err = lqr_->selected_error(x1_h, x2_h);
  const Eigen::Vector2d u_lqr = lqr_->last_u();

  double vx_body = out_map(0) * std::cos(follower_yaw) + out_map(1) * std::sin(follower_yaw);
  double vy_body = -out_map(0) * std::sin(follower_yaw) + out_map(1) * std::cos(follower_yaw);

  double vx_clamped = std::clamp(vx_body, -max_linear_vel_, max_linear_vel_);
  double vy_clamped = std::clamp(vy_body, -max_linear_vel_, max_linear_vel_);

  if (min_cmd_vel_ > 0.0) {
    const double raw_mag = std::hypot(vx_body, vy_body);
    const double cmd_mag = std::hypot(vx_clamped, vy_clamped);
    if (raw_mag > 0.001 && cmd_mag > 0.0 && cmd_mag < min_cmd_vel_) {
      const double scale = min_cmd_vel_ / cmd_mag;
      vx_clamped *= scale;
      vy_clamped *= scale;
    }
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = vx_clamped;
  cmd.linear.y = vy_clamped;

  const double raw_err = leader_yaw - follower_yaw;
  const double norm_err = std::atan2(std::sin(raw_err), std::cos(raw_err));
  cmd.angular.z = std::clamp(norm_err * Kp_yaw_ + leader_az * K_ff_,
                             -max_angular_vel_, max_angular_vel_);

  const double dt = 1.0 / control_rate_;
  const double wheel_scale = constraint_.apply(cmd.linear.x, cmd.linear.y, cmd.angular.z, dt);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "TRACE target=%d best=%.3f current=%.3f sel=(%+6.3f,%+6.3f,%+6.3f,%+6.3f) "
    "raw=(%+6.3f,%+6.3f) final=(%+6.3f,%+6.3f) scale=%.2f "
    "u=(%+.3f,%+.3f) I=(%+6.3f,%+6.3f,%+6.3f,%+6.3f) "
    "xh=(%+6.3f,%+6.3f,%+6.3f,%+6.3f)",
    lqr_->target_index(), best_dist, current_dist,
    selected_err(0), selected_err(1), selected_err(2), selected_err(3),
    vx_body, vy_body, cmd.linear.x, cmd.linear.y, wheel_scale,
    u_lqr(0), u_lqr(1), I2(0), I2(1), I2(2), I2(3),
    x2_h(0), x2_h(1), x2_h(2), x2_h(3));

  ++diag_tick_;
  auto now = get_clock()->now();
  sum_leader_age_ += (now - leader_odom_stamp_).seconds();
  sum_ekf_age_ += (now - follower_odom_stamp_).seconds();
  const double diag_elapsed = (now - last_diag_time_).seconds();
  if (diag_elapsed >= 5.0) {
    const double real_freq = diag_tick_ / diag_elapsed;
    const double avg_leader_age_ms = sum_leader_age_ / diag_tick_ * 1000.0;
    const double avg_ekf_age_ms = sum_ekf_age_ / diag_tick_ * 1000.0;
    RCLCPP_INFO(get_logger(),
      "DIAG: freq=%.1fHz avg_leader_age=%.0fms avg_ekf_age=%.0fms "
      "vcmd=(%+.3f,%+.3f) vreal=(%+.3f,%+.3f) lqr_u=(%+.3f,%+.3f)",
      real_freq, avg_leader_age_ms, avg_ekf_age_ms,
      vx_cmd_map_, vy_cmd_map_, f_vx, f_vy, u_lqr(0), u_lqr(1));
    diag_tick_ = 0;
    sum_leader_age_ = 0.0;
    sum_ekf_age_ = 0.0;
    last_diag_time_ = now;
  }

  if (wheel_scale < 0.99) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "轮速约束触发: scale=%.2f, 限幅后 cmd=(%.2f, %.2f, %.2f)",
      wheel_scale, cmd.linear.x, cmd.linear.y, cmd.angular.z);
  }

  cmd_pub_->publish(cmd);

  vx_cmd_map_ = cmd.linear.x * std::cos(follower_yaw) - cmd.linear.y * std::sin(follower_yaw);
  vy_cmd_map_ = cmd.linear.x * std::sin(follower_yaw) + cmd.linear.y * std::cos(follower_yaw);

  follower_vcmd_history_.push_front(Eigen::Vector2d(vx_cmd_map_, vy_cmd_map_));
  while (static_cast<int>(follower_vcmd_history_.size()) > buf_size) {
    follower_vcmd_history_.pop_back();
  }
}
