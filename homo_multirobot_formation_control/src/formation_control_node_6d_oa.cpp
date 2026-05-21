#include "homo_multirobot_formation_control/formation_control_node_6d_oa.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>

using namespace formation_control;

// ============================================================================
// 辅助函数
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

// ============================================================================
// 将单个机器人的 EKF 里程计变换到 map 坐标系，返回 6D 状态。
// ============================================================================
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
// 构造函数
// ============================================================================
FormationController6DOA::FormationController6DOA()
: rclcpp::Node("formation_control_node_6d_oa")
{
  leader_ns_   = declare_parameter("leader_ns",   "/robot1");
  follower_ns_ = declare_parameter("follower_ns", "/robot2");
  double radius  = declare_parameter("radius",  2.0);
  double mass    = declare_parameter("mass",    8.0);
  double I       = declare_parameter("I",       1.0);
  double omega_d        = declare_parameter("omega_d",        1.5);
  double omega_d_theta  = declare_parameter("omega_d_theta",  1.5);
  double hpc_vel_threshold = declare_parameter("hpc_vel_threshold", 0.3);
  control_rate_ = declare_parameter("control_rate", 20.0);

  double wheel_radius    = declare_parameter("wheel_radius",    0.03);
  double base_radius     = declare_parameter("base_radius",     0.11);
  double wheel_max_omega = declare_parameter("wheel_max_omega", 20.0);
  double max_linear_accel  = declare_parameter("max_linear_accel",  2.0);
  double max_angular_accel = declare_parameter("max_angular_accel", 4.0);

  ctrl_ = std::make_unique<LpcController6D>(radius, mass, I,
                                             omega_d, omega_d_theta,
                                             hpc_vel_threshold);

  constraint_ = KinematicConstraint(wheel_radius, base_radius,
                                    wheel_max_omega,
                                    max_linear_accel, max_angular_accel);

  // obstacle avoider (declares its own parameters, subscribes to scan)
  avoider_ = std::make_unique<ObstacleAvoider>(this);

  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  auto qos = rclcpp::SensorDataQoS();
  leader_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    leader_ns_ + "/odometry/filtered", qos,
    [this](nav_msgs::msg::Odometry::SharedPtr m) { leader_odom_ = m; leader_ok_ = true; });
  follower_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    follower_ns_ + "/odometry/filtered", qos,
    [this](nav_msgs::msg::Odometry::SharedPtr m) { follower_odom_ = m; follower_ok_ = true; });

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  int ms = static_cast<int>(1000.0 / control_rate_);
  timer_ = create_wall_timer(std::chrono::milliseconds(ms), [this]() { timer_cb(); });

  RCLCPP_INFO(get_logger(), "6D+OA 编队控制节点已启动。");
  RCLCPP_INFO(get_logger(), "  领航者: %s, 跟随者: %s, 安全半径: %.1f m",
    leader_ns_.c_str(), follower_ns_.c_str(), radius);
}

// ============================================================================
// 定时器回调 — 主控制循环 (~20 Hz)
// ============================================================================
void FormationController6DOA::timer_cb()
{
  if (!leader_ok_ || !follower_ok_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "[OA] timer: waiting for odometry (leader_ok=%d, follower_ok=%d). "
      "Is localization running?",
      leader_ok_ ? 1 : 0, follower_ok_ ? 1 : 0);
    return;
  }

  Eigen::VectorXd x1(6), x2(6);
  if (!ekf_to_map_6d(*tf_buffer_, leader_ns_,   leader_odom_,   x1)) return;
  if (!ekf_to_map_6d(*tf_buffer_, follower_ns_, follower_odom_, x2)) return;

  if (!controller_initialized_) {
    try {
      ctrl_->controller_initial(x1, x2);
      controller_initialized_ = true;
      RCLCPP_INFO(get_logger(), "6D+OA 控制器初始化完成。");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "初始化失败: %s", e.what());
      return;
    }
  }

  // 6D HPC 控制律（输出为车体系速度）
  auto out = ctrl_->lpc_calculate(x1, x2);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
    "x1=[%.2f,%.2f,%.1f°,%.2f,%.2f,%.2f] "
    "x2=[%.2f,%.2f,%.1f°,%.2f,%.2f,%.2f] → hpc=(%.2f,%.2f,%.2f)",
    x1(0), x1(1), x1(2)*57.3, x1(3), x1(4), x1(5),
    x2(0), x2(1), x2(2)*57.3, x2(3), x2(4), x2(5),
    out[0], out[1], out[2]);

  // --------------------------------------------------------------------------
  // 避障融合 — 仅在原 HPC 输出和运动学约束之间插入
  // --------------------------------------------------------------------------
  Eigen::Vector3d v_hpc(out[0], out[1], out[2]);
  Eigen::Vector3d v_current(x2(3), x2(4), x2(5));  // follower body-frame velocity
  double dt = 1.0 / control_rate_;

  Eigen::Vector3d v_opt = avoider_->solve(v_hpc, v_current, dt);

  // --------------------------------------------------------------------------
  // 构建速度指令（车体系，无需坐标系旋转）
  // --------------------------------------------------------------------------
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = std::clamp(v_opt(0), -1.0, 1.0);
  cmd.linear.y  = std::clamp(v_opt(1), -1.0, 1.0);
  cmd.angular.z = std::clamp(v_opt(2), -0.5, 0.5);

  // 全向轮运动学约束
  double wheel_scale = constraint_.apply(cmd.linear.x, cmd.linear.y, cmd.angular.z, dt);
  if (wheel_scale < 0.99) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "[OA] 轮速约束触发: scale=%.2f, 限幅后 cmd=(%.2f, %.2f, %.2f)",
      wheel_scale, cmd.linear.x, cmd.linear.y, cmd.angular.z);
  }

  cmd_pub_->publish(cmd);
}
