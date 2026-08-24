#include "homo_multirobot_formation_control/formation_control_node_6d_artstein_disc_hocbf.hpp"

#include <algorithm>
#include <cmath>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using formation_control::hocbf::Circle;

FormationController6DArtsteinDiscHocbf::FormationController6DArtsteinDiscHocbf()
: Node("formation_control_node_6d_artstein_disc_hocbf")
{
  leader_ns_ = declare_parameter("leader_ns", "/robot1");
  follower_ns_ = declare_parameter("follower_ns", "/robot2");
  const double radius = declare_parameter("radius", 2.0);
  const double mass = declare_parameter("mass", 2.0);
  const double inertia = declare_parameter("I", 1.0);
  rate_ = declare_parameter("control_rate", 20.0);
  tau_ = declare_parameter("tau", 0.43);
  tau_yaw_ = declare_parameter("tau_yaw", tau_);
  Td_ = declare_parameter("Td", 0.22);
  vmax_ = declare_parameter("max_linear_vel", 1.0);
  wmax_ = declare_parameter("max_angular_vel", 0.5);
  amax_ = declare_parameter("max_linear_accel", 2.0);
  follower_radius_ = declare_parameter("follower_radius", 0.15);
  clearance_ = declare_parameter("clearance", 0.10);
  perception_margin_ = declare_parameter("perception_margin", 0.15);
  scan_timeout_ = declare_parameter("scan_timeout", 0.30);
  use_latest_tf_fallback_ = declare_parameter("use_latest_tf_fallback", true);
  cluster_tolerance_ = declare_parameter("cluster_tolerance", 0.10);
  min_cluster_points_ = declare_parameter("min_cluster_points", 5);
  max_obstacles_ = declare_parameter("max_obstacles", 10);
  min_cylinder_radius_ = declare_parameter("min_cylinder_radius", 0.03);
  max_cylinder_radius_ = declare_parameter("max_cylinder_radius", 0.60);
  max_fit_residual_ = declare_parameter("max_fit_residual", 0.03);
  const double dt = 1.0 / rate_;
  Eigen::Matrix4d A4 = Eigen::Matrix4d::Zero();
  A4(0, 2) = 1.0;
  A4(1, 3) = 1.0;
  A4(2, 2) = A4(3, 3) = -1.0 / tau_;
  Eigen::Matrix<double, 4, 2> B4 = Eigen::Matrix<double, 4, 2>::Zero();
  B4(2, 0) = B4(3, 1) = 1.0 / tau_;
  trans_.build(A4, B4, tau_, Td_, dt);
  Eigen::Matrix2d A2;
  A2 << 0.0, 1.0, 0.0, -1.0 / tau_yaw_;
  Eigen::Matrix<double, 2, 1> B2;
  B2 << 0.0, 1.0 / tau_yaw_;
  yaw_.build(A2, B2, tau_yaw_, Td_, dt);
  ctrl_ = std::make_unique<formation_control::LpcController6DArtsteinDisc>(
      radius, mass, inertia, declare_parameter("m_p", 4),
      declare_parameter("tol", 0.1), declare_parameter("use_hpc", true), dt,
      declare_parameter("hpc_c_min", 0.5),
      declare_parameter("initial_min_lambda", 1.0),
      declare_parameter("switch_min_lambda", 4.0),
      declare_parameter("hpc_vel_threshold", 0.3),
      declare_parameter("hpc_yaw_threshold", 0.3),
      declare_parameter("stability_margin", 0.01));
  constraint_ = formation_control::KinematicConstraint(
      declare_parameter("wheel_radius", 0.03),
      declare_parameter("base_radius", 0.11),
      declare_parameter("wheel_max_omega", 20.0), amax_,
      declare_parameter("max_angular_accel", 4.0));
  tf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
  const auto qos = rclcpp::SensorDataQoS();
  leader_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      leader_ns_ + "/odometry/filtered", qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { leader_odom_ = msg; });
  follower_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      follower_ns_ + "/odometry/filtered", qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { follower_odom_ = msg; });
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      declare_parameter("scan_topic", "scan"), qos,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { scan_cb(msg); });
  pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / rate_)),
      [this] { timer_cb(); });
}

double FormationController6DArtsteinDiscHocbf::yaw_from_quaternion(
    const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

Eigen::Vector2d FormationController6DArtsteinDiscHocbf::body_to_map(
    double yaw, const Eigen::Vector2d& value)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c * value.x() - s * value.y(), s * value.x() + c * value.y()};
}

Eigen::Vector2d FormationController6DArtsteinDiscHocbf::map_to_body(
    double yaw, const Eigen::Vector2d& value)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c * value.x() + s * value.y(), -s * value.x() + c * value.y()};
}

bool FormationController6DArtsteinDiscHocbf::odom_to_state(
    const std::string& ns, const nav_msgs::msg::Odometry::SharedPtr& odom,
    State& state)
{
  if (!odom) return false;
  std::string frame = ns;
  if (!frame.empty() && frame.front() == '/') frame.erase(0, 1);
  frame += "_odom";
  try {
    const auto tf = tf_->lookupTransform("map", frame, tf2::TimePoint());
    const double tf_yaw = yaw_from_quaternion(tf.transform.rotation);
    const double odom_yaw = yaw_from_quaternion(odom->pose.pose.orientation);
    state.x(0) = tf.transform.translation.x + odom->pose.pose.position.x * std::cos(tf_yaw) - odom->pose.pose.position.y * std::sin(tf_yaw);
    state.x(1) = tf.transform.translation.y + odom->pose.pose.position.x * std::sin(tf_yaw) + odom->pose.pose.position.y * std::cos(tf_yaw);
    state.x(2) = std::atan2(std::sin(tf_yaw + odom_yaw), std::cos(tf_yaw + odom_yaw));
    state.x(3) = odom->twist.twist.linear.x;
    state.x(4) = odom->twist.twist.linear.y;
    state.x(5) = odom->twist.twist.angular.z;
    state.v_map = body_to_map(state.x(2), state.x.segment<2>(3));
    return true;
  } catch (const tf2::TransformException&) {
    return false;
  }
}

Eigen::VectorXd FormationController6DArtsteinDiscHocbf::predict_leader(
    const Eigen::VectorXd& x, double horizon) const
{
  Eigen::VectorXd predicted = x;
  predicted.segment<2>(0) += body_to_map(x(2), x.segment<2>(3)) * horizon;
  predicted(2) = std::atan2(std::sin(x(2) + x(5) * horizon),
                            std::cos(x(2) + x(5) * horizon));
  return predicted;
}
Eigen::VectorXd FormationController6DArtsteinDiscHocbf::predict_follower(
    const State& measured)
{
  Eigen::VectorXd translation_state(4);
  translation_state << measured.x(0), measured.x(1), measured.v_map;
  const auto translation_predicted = trans_.predict(
      translation_state + trans_.integral(v_history_), last_map_cmd_);

  Eigen::VectorXd yaw_state(2);
  yaw_state << measured.x(2), measured.x(5);
  Eigen::VectorXd yaw_command(1);
  yaw_command << last_wcmd_;
  const auto yaw_predicted = yaw_.predict(
      yaw_state + yaw_.integral(w_history_), yaw_command);

  const double predicted_yaw = std::atan2(
      std::sin(yaw_predicted(0)), std::cos(yaw_predicted(0)));
  const auto predicted_body_velocity = map_to_body(
      predicted_yaw, translation_predicted.tail<2>());
  Eigen::VectorXd out(6);
  out << translation_predicted(0), translation_predicted(1), predicted_yaw,
      predicted_body_velocity.x(), predicted_body_velocity.y(), yaw_predicted(1);
  return out;
}

void FormationController6DArtsteinDiscHocbf::scan_cb(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::vector<std::vector<Eigen::Vector2d>> clusters;
  std::vector<Eigen::Vector2d> current_cluster;
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const double range = msg->ranges[i];
    if (!std::isfinite(range) || range < msg->range_min ||
        range > msg->range_max) {
      if (static_cast<int>(current_cluster.size()) >= min_cluster_points_) {
        clusters.push_back(current_cluster);
      }
      current_cluster.clear();
      continue;
    }
    const double angle = msg->angle_min + i * msg->angle_increment;
    const Eigen::Vector2d point(range * std::cos(angle),
                                range * std::sin(angle));
    if (!current_cluster.empty() &&
        (point - current_cluster.back()).norm() > cluster_tolerance_) {
      if (static_cast<int>(current_cluster.size()) >= min_cluster_points_) {
        clusters.push_back(current_cluster);
      }
      current_cluster.clear();
    }
    current_cluster.push_back(point);
  }
  if (static_cast<int>(current_cluster.size()) >= min_cluster_points_) {
    clusters.push_back(current_cluster);
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_->lookupTransform(
        "map", msg->header.frame_id, rclcpp::Time(msg->header.stamp));
  } catch (const tf2::TransformException& error) {
    if (!use_latest_tf_fallback_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "HOCBF scan timestamp TF unavailable: %s", error.what());
      return;
    }
    try {
      transform = tf_->lookupTransform(
          "map", msg->header.frame_id, tf2::TimePointZero);
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "HOCBF scan timestamp TF unavailable; using latest TF: %s",
          error.what());
    } catch (const tf2::TransformException& fallback_error) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "HOCBF scan TF unavailable at timestamp and latest: %s",
          fallback_error.what());
      return;
    }
  }

  try {
    const double yaw = yaw_from_quaternion(transform.transform.rotation);
    std::vector<Circle> detected;
    for (const auto& cluster : clusters) {
      auto fitted = formation_control::hocbf::fit_circle(
          cluster, max_fit_residual_);
      if (!fitted || fitted->radius < min_cylinder_radius_ ||
          fitted->radius > max_cylinder_radius_) {
        continue;
      }
      fitted->center = body_to_map(yaw, fitted->center) + Eigen::Vector2d(
          transform.transform.translation.x, transform.transform.translation.y);
      fitted->radius += follower_radius_ + clearance_ + perception_margin_;
      detected.push_back(*fitted);
    }
    if (static_cast<int>(detected.size()) > max_obstacles_) {
      detected.resize(max_obstacles_);
    }
    obstacles_ = std::move(detected);
    last_scan_ = rclcpp::Time(msg->header.stamp);
  } catch (const std::exception& error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "HOCBF scan processing failed: %s", error.what());
  }
}

void FormationController6DArtsteinDiscHocbf::timer_cb()
{
  State leader;
  State follower;
  if (!odom_to_state(leader_ns_, leader_odom_, leader) ||
      !odom_to_state(follower_ns_, follower_odom_, follower)) {
    return;
  }

  if (!initialized_) {
    last_map_cmd_ = follower.v_map;
    last_wcmd_ = follower.x(5);
    for (int i = 0; i < trans_.buffer_size(); ++i) {
      Eigen::VectorXd command(2);
      command << last_map_cmd_;
      v_history_.push_back(command);
    }
    for (int i = 0; i < yaw_.buffer_size(); ++i) {
      Eigen::VectorXd command(1);
      command << last_wcmd_;
      w_history_.push_back(command);
    }
    ctrl_->controller_initial(
        predict_leader(leader.x, Td_ + std::max(tau_, tau_yaw_)),
        predict_follower(follower));
    initialized_ = true;
  }

  const auto predicted = predict_follower(follower);
  const auto nominal = ctrl_->lpc_calculate(
      predict_leader(leader.x, Td_ + std::max(tau_, tau_yaw_)), predicted);
  const auto nominal_map = body_to_map(
      follower.x(2), Eigen::Vector2d(nominal[0], nominal[1]));
  Eigen::Vector2d safe_map = Eigen::Vector2d::Zero();
  bool safe = false;
  if ((now() - last_scan_).seconds() <= scan_timeout_) {
    std::vector<formation_control::hocbf::Halfspace> constraints;
    Eigen::Vector4d state;
    const auto velocity_map = body_to_map(
        predicted(2), predicted.segment<2>(3));
    state << predicted(0), predicted(1), velocity_map.x(), velocity_map.y();
    for (const auto& obstacle : obstacles_) {
      constraints.push_back(formation_control::hocbf::hocbf_halfspace(
          state, obstacle, tau_, 2.0, 2.0));
    }
    const auto result = formation_control::hocbf::solve_translation_qp(
        nominal_map, last_map_cmd_, constraints, vmax_, amax_, 1.0 / rate_);
    safe_map = result.command;
    safe = result.feasible;
  }
  if (!safe) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "HOCBF stopping: stale scan or infeasible QP");
  }

  const auto body = map_to_body(follower.x(2), safe_map);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = body.x();
  cmd.linear.y = body.y();
  cmd.angular.z = std::clamp(nominal[2], -wmax_, wmax_);
  constraint_.apply(cmd.linear.x, cmd.linear.y, cmd.angular.z, 1.0 / rate_);
  pub_->publish(cmd);

  last_map_cmd_ = body_to_map(
      follower.x(2), Eigen::Vector2d(cmd.linear.x, cmd.linear.y));
  last_wcmd_ = cmd.angular.z;
  Eigen::VectorXd translation_command(2);
  translation_command << last_map_cmd_;
  v_history_.push_front(translation_command);
  while (static_cast<int>(v_history_.size()) > trans_.buffer_size()) {
    v_history_.pop_back();
  }
  Eigen::VectorXd yaw_command(1);
  yaw_command << last_wcmd_;
  w_history_.push_front(yaw_command);
  while (static_cast<int>(w_history_.size()) > yaw_.buffer_size()) {
    w_history_.pop_back();
  }
}
