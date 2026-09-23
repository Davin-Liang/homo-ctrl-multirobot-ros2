#include "homo_multirobot_formation_control/formation_control_node_6d_map_hpc.hpp"

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

FormationController6DMapHpc::FormationController6DMapHpc()
: rclcpp::Node("formation_control_node_6d_map_hpc")
{
  leader_ns_ = declare_parameter("leader_ns", "/robot1");
  follower_ns_ = declare_parameter("follower_ns", "/robot2");
  state_source_ = declare_parameter("state_source", "ekf_tf");
  mocap_state_timeout_ = declare_parameter("mocap_state_timeout", 0.10);
  if (state_source_ != "ekf_tf" && state_source_ != "mocap") {
    throw std::invalid_argument("6D Map HPC: state_source must be ekf_tf or mocap");
  }
  if (mocap_state_timeout_ <= 0.0) {
    throw std::invalid_argument("6D Map HPC: mocap_state_timeout must be positive");
  }

  const int m_p = declare_parameter("m_p", 4);
  const double radius = declare_parameter("radius", 2.0);
  const double tol = declare_parameter("tol", 0.1);
  const double mass = declare_parameter("mass", 2.0);
  const double inertia = declare_parameter("I", 1.0);
  const bool use_hpc = declare_parameter("use_hpc", true);
  control_rate_ = declare_parameter("control_rate", 20.0);
  const double hpc_c_min = declare_parameter("hpc_c_min", 0.5);
  const double initial_min_lambda = declare_parameter("initial_min_lambda", 1.0);
  const double switch_min_lambda = declare_parameter("switch_min_lambda", 4.0);
  const bool use_hpc_nu_override = declare_parameter("use_hpc_nu_override", false);
  const double hpc_nu_override = declare_parameter("hpc_nu_override", -0.30);

  const double wheel_radius = declare_parameter("wheel_radius", 0.03);
  const double base_radius = declare_parameter("base_radius", 0.11);
  const double wheel_max_omega = declare_parameter("wheel_max_omega", 20.0);
  const double max_linear_accel = declare_parameter("max_linear_accel", 2.0);
  const double max_angular_accel = declare_parameter("max_angular_accel", 4.0);
  max_linear_vel_ = declare_parameter("max_linear_vel", 1.0);
  max_angular_vel_ = declare_parameter("max_angular_vel", 0.5);
  min_cmd_vel_ = declare_parameter("min_cmd_vel", 0.0);

  if (control_rate_ <= 0.0) {
    throw std::invalid_argument("6D Map HPC: control_rate must be positive");
  }
  if (hpc_c_min <= 0.0 || hpc_c_min > 1.0) {
    throw std::invalid_argument("6D Map HPC: hpc_c_min must be in (0, 1]");
  }

  const double dt = 1.0 / control_rate_;
  ctrl_ = std::make_unique<MapHpcController6DArtstein>(
      m_p, radius, tol, mass, inertia, hpc_c_min, use_hpc, dt,
      initial_min_lambda, switch_min_lambda,
      use_hpc_nu_override, hpc_nu_override);
  constraint_ = KinematicConstraint(wheel_radius, base_radius, wheel_max_omega,
                                    max_linear_accel, max_angular_accel);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  const auto qos = rclcpp::SensorDataQoS();
  if (state_source_ == "ekf_tf") {
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
  } else {
    leader_mocap_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        leader_ns_ + "/mocap/pose", qos,
        [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
          leader_mocap_pose_ = m;
          leader_mocap_received_ = now();
          leader_ok_ = true;
        });
    follower_mocap_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        follower_ns_ + "/mocap/pose", qos,
        [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
          follower_mocap_pose_ = m;
          follower_mocap_received_ = now();
          follower_ok_ = true;
        });
    leader_mocap_twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        leader_ns_ + "/mocap/twist", qos,
        [this](geometry_msgs::msg::TwistStamped::SharedPtr m) { leader_mocap_twist_ = m; });
    follower_mocap_twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        follower_ns_ + "/mocap/twist", qos,
        [this](geometry_msgs::msg::TwistStamped::SharedPtr m) { follower_mocap_twist_ = m; });
  }

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  last_diag_time_ = get_clock()->now();
  const int ms = static_cast<int>(1000.0 / control_rate_);
  timer_ = create_wall_timer(std::chrono::milliseconds(ms), [this]() { timer_cb(); });

  RCLCPP_INFO(get_logger(), "6D Map HPC baseline node started (no Artstein or prediction).");
  RCLCPP_INFO(get_logger(), "  Leader: %s, Follower: %s", leader_ns_.c_str(), follower_ns_.c_str());
}

bool FormationController6DMapHpc::odom_to_state(
    const std::string& ns, const nav_msgs::msg::Odometry::SharedPtr& odom,
    State6D& state)
{
  if (!odom) return false;
  std::string odom_frame = ns;
  if (!odom_frame.empty() && odom_frame.front() == '/') odom_frame.erase(0, 1);
  odom_frame += "_odom";
  try {
    const auto t = tf_buffer_->lookupTransform("map", odom_frame, tf2::TimePoint());
    const double tf_yaw = tf2_yaw(tf2::Quaternion(
        t.transform.rotation.x, t.transform.rotation.y,
        t.transform.rotation.z, t.transform.rotation.w));
    const double ekf_yaw = msg_yaw(odom->pose.pose.orientation);
    const double yaw = wrap_angle(tf_yaw + ekf_yaw);
    const double px = odom->pose.pose.position.x;
    const double py = odom->pose.pose.position.y;
    state.x(0) = t.transform.translation.x + px * std::cos(tf_yaw) - py * std::sin(tf_yaw);
    state.x(1) = t.transform.translation.y + px * std::sin(tf_yaw) + py * std::cos(tf_yaw);
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

void FormationController6DMapHpc::timer_cb()
{
  if (!leader_ok_ || !follower_ok_) return;

  State6D leader, follower;
  if (state_source_ == "ekf_tf") {
    if (!odom_to_state(leader_ns_, leader_odom_, leader) ||
        !odom_to_state(follower_ns_, follower_odom_, follower)) return;
  } else {
    const auto current_time = now();
    if (!leader_mocap_pose_ || !leader_mocap_twist_ || !follower_mocap_pose_ ||
        !follower_mocap_twist_ ||
        (current_time - leader_mocap_received_).seconds() > mocap_state_timeout_ ||
        (current_time - follower_mocap_received_).seconds() > mocap_state_timeout_) {
      controller_initialized_ = false;
      cmd_pub_->publish(geometry_msgs::msg::Twist{});
      return;
    }
    auto fill_state = [this](const auto& pose, const auto& twist, State6D& state) {
      const double yaw = msg_yaw(pose->pose.orientation);
      const Eigen::Vector2d v_map(twist->twist.linear.x, twist->twist.linear.y);
      const Eigen::Vector2d v_body = map_to_body(yaw, v_map);
      state.x << pose->pose.position.x, pose->pose.position.y, yaw,
          v_body(0), v_body(1), twist->twist.angular.z;
      state.v_map = v_map;
    };
    fill_state(leader_mocap_pose_, leader_mocap_twist_, leader);
    fill_state(follower_mocap_pose_, follower_mocap_twist_, follower);
  }

  const bool target_switched = ctrl_->select_target(leader.x, follower.x);
  try {
    if (!controller_initialized_) {
      ctrl_->initialize(leader.x, follower.x);
      controller_initialized_ = true;
      RCLCPP_INFO(get_logger(), "HPC nu_mode=%s nu_used=%.6f feasible=[%.6f, %.6f]",
                  ctrl_->uses_nu_override() ? "override" : "auto",
                  ctrl_->nu(), ctrl_->nu_min(), ctrl_->nu_max());
    } else if (target_switched) {
      ctrl_->initialize(leader.x, follower.x, true);
      RCLCPP_INFO(get_logger(), "6D Map HPC switched polygon target to %d.",
                  ctrl_->target_index());
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "6D Map HPC initialization failed: %s", e.what());
    return;
  }

  const Eigen::Vector3d map_out = ctrl_->command(leader.x, follower.x);
  const Eigen::Vector2d body_out = map_to_body(follower.x(2), map_out.head<2>());
  const double raw_linear_mag = body_out.norm();
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = std::clamp(body_out(0), -max_linear_vel_, max_linear_vel_);
  cmd.linear.y = std::clamp(body_out(1), -max_linear_vel_, max_linear_vel_);
  cmd.angular.z = std::clamp(map_out(2), -max_angular_vel_, max_angular_vel_);
  const double clamped_linear_mag = std::hypot(cmd.linear.x, cmd.linear.y);

  if (min_cmd_vel_ > 0.0 && raw_linear_mag > 1e-3 && clamped_linear_mag > 0.0 &&
      clamped_linear_mag < min_cmd_vel_) {
    const double scale = min_cmd_vel_ / clamped_linear_mag;
    cmd.linear.x *= scale;
    cmd.linear.y *= scale;
  }

  const double wheel_scale = constraint_.apply(
      cmd.linear.x, cmd.linear.y, cmd.angular.z, 1.0 / control_rate_);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "6D Map HPC cmd=(%+.3f,%+.3f,%+.3f) |v_raw|=%.3f |v_clamped|=%.3f "
      "|v_final|=%.3f scale=%.2f",
      cmd.linear.x, cmd.linear.y, cmd.angular.z, raw_linear_mag,
      clamped_linear_mag, std::hypot(cmd.linear.x, cmd.linear.y), wheel_scale);

  ++diag_tick_;
  const auto current_time = get_clock()->now();
  if (state_source_ == "mocap") {
    sum_leader_age_ += (current_time - leader_mocap_received_).seconds();
    sum_ekf_age_ += (current_time - follower_mocap_received_).seconds();
  } else {
    sum_leader_age_ += (current_time - leader_odom_stamp_).seconds();
    sum_ekf_age_ += (current_time - follower_odom_stamp_).seconds();
  }
  const double elapsed = (current_time - last_diag_time_).seconds();
  if (elapsed >= 5.0) {
    RCLCPP_INFO(get_logger(), "DIAG: freq=%.1fHz avg_leader_age=%.0fms avg_ekf_age=%.0fms",
                diag_tick_ / elapsed, sum_leader_age_ / diag_tick_ * 1000.0,
                sum_ekf_age_ / diag_tick_ * 1000.0);
    diag_tick_ = 0;
    sum_leader_age_ = 0.0;
    sum_ekf_age_ = 0.0;
    last_diag_time_ = current_time;
  }
  if (wheel_scale < 0.99) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "轮速约束触发: scale=%.2f cmd=(%.2f, %.2f, %.2f)",
                         wheel_scale, cmd.linear.x, cmd.linear.y, cmd.angular.z);
  }
  cmd_pub_->publish(cmd);
}

double FormationController6DMapHpc::wrap_angle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

Eigen::Vector2d FormationController6DMapHpc::body_to_map(
    double yaw, const Eigen::Vector2d& v_body)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Eigen::Vector2d(c * v_body(0) - s * v_body(1),
                         s * v_body(0) + c * v_body(1));
}

Eigen::Vector2d FormationController6DMapHpc::map_to_body(
    double yaw, const Eigen::Vector2d& v_map)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Eigen::Vector2d(c * v_map(0) + s * v_map(1),
                         -s * v_map(0) + c * v_map(1));
}
