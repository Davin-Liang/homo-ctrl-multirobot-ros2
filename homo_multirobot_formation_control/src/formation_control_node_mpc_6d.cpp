#include "homo_multirobot_formation_control/formation_control_node_mpc_6d.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace formation_control;

// ============================================================================
// Helpers (same as existing 6D node)
// ============================================================================

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

static bool ekf_to_map_6d(tf2_ros::Buffer& tf, const std::string& ns,
                           const nav_msgs::msg::Odometry::SharedPtr& odom,
                           Eigen::VectorXd& x)
{
  if (!odom) return false;

  std::string odom_frame = ns;
  if (!odom_frame.empty() && odom_frame[0] == '/') odom_frame = odom_frame.substr(1);
  odom_frame += "_odom";

  try {
    auto t = tf.lookupTransform("map", odom_frame, tf2::TimePoint());
    double tf_x   = t.transform.translation.x;
    double tf_y   = t.transform.translation.y;
    double tf_yaw = tf2_yaw(tf2::Quaternion(
        t.transform.rotation.x, t.transform.rotation.y,
        t.transform.rotation.z, t.transform.rotation.w));

    double ekf_px  = odom->pose.pose.position.x;
    double ekf_py  = odom->pose.pose.position.y;
    double ekf_yaw = msg_yaw(odom->pose.pose.orientation);

    x(0) = tf_x + ekf_px * std::cos(tf_yaw) - ekf_py * std::sin(tf_yaw);
    x(1) = tf_y + ekf_px * std::sin(tf_yaw) + ekf_py * std::cos(tf_yaw);
    x(2) = tf_yaw + ekf_yaw;

    x(3) = odom->twist.twist.linear.x;
    x(4) = odom->twist.twist.linear.y;
    x(5) = odom->twist.twist.angular.z;

    return true;
  } catch (const tf2::TransformException&) {
    return false;
  }
}

// ============================================================================
// Constructor
// ============================================================================
FormationControllerMpc6D::FormationControllerMpc6D()
: rclcpp::Node("formation_control_node_mpc_6d")
{
  leader_ns_   = declare_parameter("leader_ns",   "/robot1");
  follower_ns_ = declare_parameter("follower_ns", "/robot2");
  control_rate_ = declare_parameter("control_rate", 20.0);

  // ---- MPC parameters ----------------------------------------------------
  MpcController6D::Params p;
  p.N                  = declare_parameter("mpc_horizon",          40);
  p.dt                 = 1.0 / control_rate_;
  p.formation_radius   = declare_parameter("formation_radius", 2.0);
  p.formation_offset_x = declare_parameter("formation_offset_x", -2.0);
  p.formation_offset_y = declare_parameter("formation_offset_y",  0.0);
  p.q_px     = declare_parameter("mpc_q_px",     5.0);
  p.q_py     = declare_parameter("mpc_q_py",     5.0);
  p.q_theta  = declare_parameter("mpc_q_theta",  20.0);
  p.q_vx     = declare_parameter("mpc_q_vx",     0.5);
  p.q_vy     = declare_parameter("mpc_q_vy",     0.5);
  p.q_omega  = declare_parameter("mpc_q_omega",  2.0);
  p.r_ax     = declare_parameter("mpc_r_ax",     0.01);
  p.r_ay     = declare_parameter("mpc_r_ay",     0.01);
  p.r_alpha  = declare_parameter("mpc_r_alpha",  0.01);
  p.terminal_factor = declare_parameter("mpc_terminal_factor", 10.0);
  p.max_linear_accel  = declare_parameter("max_linear_accel",  2.0);
  p.max_angular_accel = declare_parameter("max_angular_accel", 6.0);
  p.max_linear_vel    = declare_parameter("max_linear_vel",    1.0);
  p.max_angular_vel   = declare_parameter("max_angular_vel",   2.0);

  mpc_ = std::make_unique<MpcController6D>(p);

  // ---- Kinematic constraint -----------------------------------------------
  double wheel_radius    = declare_parameter("wheel_radius",    0.03);
  double base_radius     = declare_parameter("base_radius",     0.11);
  double wheel_max_omega = declare_parameter("wheel_max_omega", 20.0);
  constraint_ = KinematicConstraint(wheel_radius, base_radius,
                                    wheel_max_omega,
                                    p.max_linear_accel, p.max_angular_accel);

  // ---- TF -----------------------------------------------------------------
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // ---- Subscriptions ------------------------------------------------------
  auto qos = rclcpp::SensorDataQoS();
  leader_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    leader_ns_ + "/odometry/filtered", qos,
    [this](nav_msgs::msg::Odometry::SharedPtr m) { leader_odom_ = m; leader_ok_ = true; });
  follower_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    follower_ns_ + "/odometry/filtered", qos,
    [this](nav_msgs::msg::Odometry::SharedPtr m) { follower_odom_ = m; follower_ok_ = true; });

  // ---- Publisher -----------------------------------------------------------
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // ---- Timer ---------------------------------------------------------------
  int ms = static_cast<int>(1000.0 / control_rate_);
  timer_ = create_wall_timer(std::chrono::milliseconds(ms), [this]() { timer_cb(); });

  RCLCPP_INFO(get_logger(), "MPC 6D 编队控制节点已启动 (Single-point Linearized MPC)");
  RCLCPP_INFO(get_logger(), "  领航者: %s, 跟随者: %s, N=%d, dt=%.3f s",
    leader_ns_.c_str(), follower_ns_.c_str(), p.N, p.dt);
}

// ============================================================================
// Timer callback
// ============================================================================
void FormationControllerMpc6D::timer_cb()
{
  using namespace std::chrono;

  if (!leader_ok_ || !follower_ok_) return;

  Eigen::VectorXd x1(6), x2(6);
  if (!ekf_to_map_6d(*tf_buffer_, leader_ns_,   leader_odom_,   x1)) return;
  if (!ekf_to_map_6d(*tf_buffer_, follower_ns_, follower_odom_, x2)) return;

  // ---- MPC compute ---------------------------------------------------------
  auto t0 = steady_clock::now();
  Eigen::Vector3d u = mpc_->compute_control(x1, x2);
  auto t1 = steady_clock::now();
  double const total_ms = duration<double, std::milli>(t1 - t0).count();

  int const osqp_status  = mpc_->last_solve_status();
  double const solve_ms  = mpc_->last_solve_time_ms();
  bool const solved_ok   = (osqp_status == 1);
  double const dt        = 1.0 / control_rate_;

  double v_max_lin = get_parameter("max_linear_vel").as_double();
  double v_max_ang = get_parameter("max_angular_vel").as_double();

  // ---- Failure handling ----------------------------------------------------
  if (!solved_ok) {
    consecutive_failures_++;

    if (consecutive_failures_ >= max_consecutive_failures_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "连续 %d 次求解失败，进入安全停车状态.", consecutive_failures_);
      geometry_msgs::msg::Twist stop;
      cmd_pub_->publish(stop);
      return;
    }

    // Single failure: publish zero velocity, log, skip this cycle
    geometry_msgs::msg::Twist stop;
    cmd_pub_->publish(stop);

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "MPC 求解失败 (status=%d, total=%.1fms solve=%.1fms, fail#%d). 发布零速度.",
      osqp_status, total_ms, solve_ms, consecutive_failures_);
    return;
  }

  consecutive_failures_ = 0;

  // ---- Forward Euler integration -------------------------------------------
  double vx_cmd = x2(3) + dt * u(0);
  double vy_cmd = x2(4) + dt * u(1);
  double w_cmd  = x2(5) + dt * u(2);

  // ---- Logging -------------------------------------------------------------
  double const formation_err = std::hypot(
    x2(0) - x1(0),
    x2(1) - x1(1));

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
    "L[%.2f,%.2f,%.1f°] F[%.2f,%.2f,%.1f°] "
    "u=[%+.3f,%+.3f,%+.3f] cmd_raw=(%.2f,%.2f,%.2f) "
    "osqp=%d solve=%.1fms total=%.1fms err=%.2fm",
    x1(0), x1(1), x1(2)*57.3,
    x2(0), x2(1), x2(2)*57.3,
    u(0), u(1), u(2),
    vx_cmd, vy_cmd, w_cmd,
    osqp_status, solve_ms, total_ms, formation_err);

  // ---- Publish -------------------------------------------------------------
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = std::clamp(vx_cmd, -v_max_lin, v_max_lin);
  cmd.linear.y  = std::clamp(vy_cmd, -v_max_lin, v_max_lin);
  cmd.angular.z = std::clamp(w_cmd,  -v_max_ang, v_max_ang);

  double wheel_scale = constraint_.apply(cmd.linear.x, cmd.linear.y, cmd.angular.z, dt);
  if (wheel_scale < 0.99) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "轮速约束: scale=%.2f limited=(%.2f,%.2f,%.2f)",
      wheel_scale, cmd.linear.x, cmd.linear.y, cmd.angular.z);
  }

  cmd_pub_->publish(cmd);
}
