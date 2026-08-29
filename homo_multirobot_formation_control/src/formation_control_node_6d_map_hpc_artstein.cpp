#include "homo_multirobot_formation_control/formation_control_node_6d_map_hpc_artstein.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

using namespace formation_control;

namespace {

double tf2_yaw(const tf2::Quaternion& q)
{
  double r, p, y;
  tf2::Matrix3x3(q).getRPY(r, p, y);
  return y;
}

double msg_yaw(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

FormationController6DMapHpcArtstein::FormationController6DMapHpcArtstein()
: rclcpp::Node("formation_control_node_6d_map_hpc_artstein")
{
  leader_ns_ = declare_parameter("leader_ns", "/robot1");
  follower_ns_ = declare_parameter("follower_ns", "/robot2");

  double radius = declare_parameter("radius", 2.0);
  double mass = declare_parameter("mass", 2.0);
  double inertia = declare_parameter("I", 1.0);
  int m_p = declare_parameter("m_p", 4);
  double tol = declare_parameter("tol", 0.1);
  bool use_hpc = declare_parameter("use_hpc", true);
  control_rate_ = declare_parameter("control_rate", 20.0);
  double hpc_c_min = declare_parameter("hpc_c_min", 0.5);
  double mu = declare_parameter("mu", -0.25);
  double kp = declare_parameter("kp", 1.2);
  double kv = declare_parameter("kv", 2.0);

  tau_v_ = declare_parameter("tau", 0.43);
  tau_w_ = declare_parameter("tau_yaw", tau_v_);
  Td_ = declare_parameter("Td", 0.22);

  double wheel_radius = declare_parameter("wheel_radius", 0.03);
  double base_radius = declare_parameter("base_radius", 0.11);
  double wheel_max_omega = declare_parameter("wheel_max_omega", 20.0);
  double max_linear_accel = declare_parameter("max_linear_accel", 2.0);
  double max_angular_accel = declare_parameter("max_angular_accel", 4.0);
  max_linear_vel_ = declare_parameter("max_linear_vel", 1.0);
  max_angular_vel_ = declare_parameter("max_angular_vel", 0.5);
  min_cmd_vel_ = declare_parameter("min_cmd_vel", 0.0);

  if (control_rate_ <= 0.0) {
    throw std::invalid_argument("6D Artstein Disc: control_rate must be positive");
  }
  if (tau_v_ <= 0.0 || tau_w_ <= 0.0) {
    throw std::invalid_argument("6D Artstein Disc: tau and tau_yaw must be positive");
  }
  if (Td_ < 0.0) {
    throw std::invalid_argument("6D Artstein Disc: Td must be non-negative");
  }
  if (hpc_c_min <= 0.0 || hpc_c_min > 1.0) {
    throw std::invalid_argument("6D Artstein Disc: hpc_c_min must be in (0, 1]");
  }

  double dt = 1.0 / control_rate_;
  build_predictors(tau_v_, tau_w_, Td_, dt);

  ctrl_ = std::make_unique<MapHpcController6DArtsteinDisc>(
      radius, m_p, tol, mass, inertia, mu, kp, kv, hpc_c_min, use_hpc, dt);

  constraint_ = KinematicConstraint(wheel_radius, base_radius, wheel_max_omega,
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
  int ms = static_cast<int>(1000.0 / control_rate_);
  timer_ = create_wall_timer(std::chrono::milliseconds(ms),
                             [this]() { timer_cb(); });

  RCLCPP_INFO(get_logger(),
    "6D Artstein Disc node started (Td=%.3fs tau=%.3fs tau_yaw=%.3fs).",
    Td_, tau_v_, tau_w_);
  RCLCPP_INFO(get_logger(), "  Leader: %s, Follower: %s",
              leader_ns_.c_str(), follower_ns_.c_str());
}

void FormationController6DMapHpcArtstein::build_predictors(double tau_v,
                                                         double tau_w,
                                                         double Td,
                                                         double dt)
{
  Eigen::MatrixXd A4(4, 4), B4(4, 2);
  A4 << 0, 0, 1, 0,
        0, 0, 0, 1,
        0, 0, -1.0 / tau_v, 0,
        0, 0, 0, -1.0 / tau_v;
  B4 << 0, 0,
        0, 0,
        1.0 / tau_v, 0,
        0, 1.0 / tau_v;
  trans_predictor_.build(A4, B4, tau_v, Td, dt);

  Eigen::MatrixXd A2(2, 2), B2(2, 1);
  A2 << 0, 1,
        0, -1.0 / tau_w;
  B2 << 0,
        1.0 / tau_w;
  yaw_predictor_.build(A2, B2, tau_w, Td, dt);
}

bool FormationController6DMapHpcArtstein::odom_to_state(
    const std::string& ns,
    const nav_msgs::msg::Odometry::SharedPtr& odom,
    State6D& state)
{
  if (!odom) {
    return false;
  }

  std::string odom_frame = ns;
  if (!odom_frame.empty() && odom_frame[0] == '/') {
    odom_frame = odom_frame.substr(1);
  }
  odom_frame += "_odom";

  try {
    auto t = tf_buffer_->lookupTransform("map", odom_frame, tf2::TimePoint());
    double tf_x = t.transform.translation.x;
    double tf_y = t.transform.translation.y;
    double tf_yaw = tf2_yaw(tf2::Quaternion(
        t.transform.rotation.x, t.transform.rotation.y,
        t.transform.rotation.z, t.transform.rotation.w));

    double ekf_px = odom->pose.pose.position.x;
    double ekf_py = odom->pose.pose.position.y;
    double ekf_yaw = msg_yaw(odom->pose.pose.orientation);
    double yaw = wrap_angle(tf_yaw + ekf_yaw);

    state.x(0) = tf_x + ekf_px * std::cos(tf_yaw) - ekf_py * std::sin(tf_yaw);
    state.x(1) = tf_y + ekf_px * std::sin(tf_yaw) + ekf_py * std::cos(tf_yaw);
    state.x(2) = yaw;
    state.x(3) = odom->twist.twist.linear.x;
    state.x(4) = odom->twist.twist.linear.y;
    state.x(5) = odom->twist.twist.angular.z;
    state.v_map = body_to_map(yaw, state.x.segment<2>(3));
    return true;
  } catch (const tf2::TransformException&) {
    return false;
  }
}

Eigen::VectorXd FormationController6DMapHpcArtstein::predict_leader_state(
    const Eigen::VectorXd& x, double horizon) const
{
  Eigen::VectorXd pred = x;
  double vx = x(3);
  double vy = x(4);
  double omega = x(5);

  Eigen::Vector2d dp_body;
  if (std::abs(omega) < 1e-8) {
    dp_body << vx * horizon, vy * horizon;
  } else {
    double wt = omega * horizon;
    dp_body << vx * std::sin(wt) / omega
             + vy * (std::cos(wt) - 1.0) / omega,
               vx * (1.0 - std::cos(wt)) / omega
             + vy * std::sin(wt) / omega;
  }

  pred.segment<2>(0) += body_to_map(x(2), dp_body);
  pred(2) = wrap_angle(x(2) + omega * horizon);
  return pred;
}

Eigen::VectorXd FormationController6DMapHpcArtstein::predict_follower_state(
    const State6D& measured)
{
  Eigen::VectorXd x4(4);
  x4 << measured.x(0), measured.x(1), measured.v_map(0), measured.v_map(1);
  Eigen::VectorXd z4 = x4 + trans_predictor_.integral(follower_vcmd_map_hist_);
  Eigen::VectorXd pred4 = trans_predictor_.predict(z4, last_vcmd_map_);

  Eigen::VectorXd x2(2);
  x2 << measured.x(2), measured.x(5);
  Eigen::VectorXd z2 = x2 + yaw_predictor_.integral(follower_wcmd_hist_);
  Eigen::VectorXd wcmd(1);
  wcmd << last_wcmd_;
  Eigen::VectorXd pred2 = yaw_predictor_.predict(z2, wcmd);

  double theta_pred = wrap_angle(pred2(0));
  Eigen::Vector2d v_body_pred = map_to_body(theta_pred, pred4.tail<2>());

  Eigen::VectorXd out(6);
  out << pred4(0), pred4(1), theta_pred,
         v_body_pred(0), v_body_pred(1), pred2(1);
  return out;
}

void FormationController6DMapHpcArtstein::timer_cb()
{
  if (!leader_ok_ || !follower_ok_) {
    return;
  }

  State6D leader, follower;
  if (!odom_to_state(leader_ns_, leader_odom_, leader)) {
    return;
  }
  if (!odom_to_state(follower_ns_, follower_odom_, follower)) {
    return;
  }

  int trans_buf_size = trans_predictor_.buffer_size();
  int yaw_buf_size = yaw_predictor_.buffer_size();

  if (!controller_initialized_) {
    last_vcmd_map_ = follower.v_map;
    last_wcmd_ = follower.x(5);
    follower_vcmd_map_hist_.clear();
    follower_wcmd_hist_.clear();
    for (int i = 0; i < trans_buf_size; ++i) {
      Eigen::VectorXd v(2);
      v << last_vcmd_map_(0), last_vcmd_map_(1);
      follower_vcmd_map_hist_.push_back(v);
    }
    for (int i = 0; i < yaw_buf_size; ++i) {
      Eigen::VectorXd w(1);
      w << last_wcmd_;
      follower_wcmd_hist_.push_back(w);
    }

    Eigen::VectorXd x1_h = predict_leader_state(
        leader.x, Td_ + std::max(tau_v_, tau_w_));
    Eigen::VectorXd x2_h = predict_follower_state(follower);

    controller_initialized_ = true;
  }

  Eigen::VectorXd x1_h = predict_leader_state(
      leader.x, Td_ + std::max(tau_v_, tau_w_));
  Eigen::VectorXd x2_h = predict_follower_state(follower);

  Eigen::Vector3d map_out = ctrl_->command(x1_h, x2_h);
  Eigen::Vector2d body_out = map_to_body(follower.x(2), map_out.head<2>());
  const double raw_linear_mag = body_out.norm();

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = std::clamp(body_out(0), -max_linear_vel_, max_linear_vel_);
  cmd.linear.y = std::clamp(body_out(1), -max_linear_vel_, max_linear_vel_);
  cmd.angular.z = std::clamp(map_out(2), -max_angular_vel_, max_angular_vel_);
  const double clamped_linear_mag = std::hypot(cmd.linear.x, cmd.linear.y);
  const double omega_raw = map_out(2);
  const double omega_clamped = cmd.angular.z;
  const double prev_wcmd = last_wcmd_;

  if (min_cmd_vel_ > 0.0) {
    double raw_mag = body_out.norm();
    double cmd_mag = std::hypot(cmd.linear.x, cmd.linear.y);
    if (raw_mag > 1e-3 && cmd_mag > 0.0 && cmd_mag < min_cmd_vel_) {
      double scale = min_cmd_vel_ / cmd_mag;
      cmd.linear.x *= scale;
      cmd.linear.y *= scale;
    }
  }

  double dt = 1.0 / control_rate_;
  double wheel_scale = constraint_.apply(cmd.linear.x, cmd.linear.y,
                                         cmd.angular.z, dt);
  const double final_linear_mag = std::hypot(cmd.linear.x, cmd.linear.y);
  const double omega_final = cmd.angular.z;
  const double domega_cmd = (omega_final - prev_wcmd) / dt;

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "6D_ART pred_shift=(%.3f, %.3f, %.3f) cmd=(%+.3f,%+.3f,%+.3f) "
    "|v_raw|=%.3f |v_clamped|=%.3f |v_final|=%.3f "
    "target=%d scale=%.2f",
    x2_h(0) - follower.x(0),
    x2_h(1) - follower.x(1),
    wrap_angle(x2_h(2) - follower.x(2)),
    cmd.linear.x, cmd.linear.y, cmd.angular.z,
    raw_linear_mag, clamped_linear_mag, final_linear_mag,
    ctrl_->target_idx(), wheel_scale);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "YAW_DIAG yaw_l=%.3f yaw_f=%.3f yaw_lp=%.3f yaw_fp=%.3f "
    "omega_l=%.3f omega_f=%.3f omega_lp=%.3f omega_fp=%.3f "
    "omega_raw=%+.3f omega_clamped=%+.3f omega_final=%+.3f "
    "domega_cmd=%+.3f scale=%.2f",
    leader.x(2), follower.x(2), x1_h(2), x2_h(2),
    leader.x(5), follower.x(5), x1_h(5), x2_h(5),
    omega_raw, omega_clamped, omega_final, domega_cmd, wheel_scale);

  ++diag_tick_;
  auto now = get_clock()->now();
  sum_leader_age_ += (now - leader_odom_stamp_).seconds();
  sum_ekf_age_ += (now - follower_odom_stamp_).seconds();
  double diag_elapsed = (now - last_diag_time_).seconds();
  if (diag_elapsed >= 5.0) {
    double real_freq = diag_tick_ / diag_elapsed;
    double avg_leader_age_ms = sum_leader_age_ / diag_tick_ * 1000.0;
    double avg_ekf_age_ms = sum_ekf_age_ / diag_tick_ * 1000.0;
    RCLCPP_INFO(get_logger(),
      "DIAG: freq=%.1fHz avg_leader_age=%.0fms avg_ekf_age=%.0fms "
      "vcmd_map=(%+.3f,%+.3f) wcmd=%+.3f",
      real_freq, avg_leader_age_ms, avg_ekf_age_ms,
      last_vcmd_map_(0), last_vcmd_map_(1), last_wcmd_);
    diag_tick_ = 0;
    sum_leader_age_ = 0.0;
    sum_ekf_age_ = 0.0;
    last_diag_time_ = now;
  }

  if (wheel_scale < 0.99) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "轮速约束触发: scale=%.2f cmd=(%.2f, %.2f, %.2f)",
      wheel_scale, cmd.linear.x, cmd.linear.y, cmd.angular.z);
  }

  cmd_pub_->publish(cmd);

  last_vcmd_map_ = body_to_map(
      follower.x(2), Eigen::Vector2d(cmd.linear.x, cmd.linear.y));
  last_wcmd_ = cmd.angular.z;

  Eigen::VectorXd vcmd_map_vec(2);
  vcmd_map_vec << last_vcmd_map_(0), last_vcmd_map_(1);
  follower_vcmd_map_hist_.push_front(vcmd_map_vec);
  while (static_cast<int>(follower_vcmd_map_hist_.size()) > trans_buf_size) {
    follower_vcmd_map_hist_.pop_back();
  }

  Eigen::VectorXd w(1);
  w << last_wcmd_;
  follower_wcmd_hist_.push_front(w);
  while (static_cast<int>(follower_wcmd_hist_.size()) > yaw_buf_size) {
    follower_wcmd_hist_.pop_back();
  }
}

double FormationController6DMapHpcArtstein::wrap_angle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

Eigen::Vector2d FormationController6DMapHpcArtstein::body_to_map(
    double yaw, const Eigen::Vector2d& v_body)
{
  double c = std::cos(yaw);
  double s = std::sin(yaw);
  return Eigen::Vector2d(c * v_body(0) - s * v_body(1),
                         s * v_body(0) + c * v_body(1));
}

Eigen::Vector2d FormationController6DMapHpcArtstein::map_to_body(
    double yaw, const Eigen::Vector2d& v_map)
{
  double c = std::cos(yaw);
  double s = std::sin(yaw);
  return Eigen::Vector2d(c * v_map(0) + s * v_map(1),
                        -s * v_map(0) + c * v_map(1));
}
