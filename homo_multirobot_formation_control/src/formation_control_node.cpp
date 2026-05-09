/// @file 编队控制节点实现。
///
/// 数据流:
///   EKF odometry/filtered ──→ 缓冲区（回调存储最新消息）
///   TF  map→X_base_footprint ──→ 定时器读取当前 TF
///
/// 定时器 (20 Hz):
///   1. 查找两机器人的 map → base_footprint TF → 位置 + 偏航角
///   2. 读取缓冲的 EKF 速度 → 本体帧旋转到 map 帧
///   3. 将 4 维状态向量送入 LpcController::lpc_calculate
///   4. 在 follower 命名空间下发布 cmd_vel

#include "homo_multirobot_formation_control/formation_control_node.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace formation_control;

// ============================================================================
// 构造函数 — 声明参数、创建 TF 缓冲 + 订阅 + 定时器
// ============================================================================
FormationController::FormationController()
: rclcpp::Node("formation_control_node")
{
  leader_ns_   = declare_parameter("leader_ns",   "/robot1");
  follower_ns_ = declare_parameter("follower_ns", "/robot2");
  int m_p      = declare_parameter("m_p",      4);
  double radius = declare_parameter("radius",  2.0);
  double tol    = declare_parameter("tol",     0.1);
  double mass   = declare_parameter("mass",    8.0);
  Kp_yaw_       = declare_parameter("Kp_yaw",  4.0);
  K_ff_         = declare_parameter("K_ff",    1.0);
  double control_rate = declare_parameter("control_rate", 20.0);

  ctrl_ = std::make_unique<LpcController>(m_p, radius, tol, mass);

  // TF 监听 slam_toolbox/AMCL 的 map→odom 变换链
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // EKF 里程计: 自身一致的速度（50 Hz, IMU + rf2o 融合）
  auto qos = rclcpp::SensorDataQoS();
  leader_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    leader_ns_ + "/odometry/filtered", qos,
    [this](nav_msgs::msg::Odometry::SharedPtr m) { leader_odom_ = m; leader_ok_ = true; });
  follower_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    follower_ns_ + "/odometry/filtered", qos,
    [this](nav_msgs::msg::Odometry::SharedPtr m) { follower_odom_ = m; follower_ok_ = true; });

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  int ms = static_cast<int>(1000.0 / control_rate);
  timer_ = create_wall_timer(std::chrono::milliseconds(ms), [this]() { timer_cb(); });

  RCLCPP_INFO(get_logger(), "编队控制节点已启动。");
  RCLCPP_INFO(get_logger(), "  领航者: %s, 跟随者: %s",
    leader_ns_.c_str(), follower_ns_.c_str());
}

// ============================================================================
// 辅助函数
// ============================================================================

// 从 tf2 四元数提取偏航角
static double tf2_yaw(const tf2::Quaternion& q)
{
  double r, p, y;
  tf2::Matrix3x3(q).getRPY(r, p, y);
  return y;
}

// 从 geometry_msgs 四元数提取偏航角
static double msg_yaw(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// ============================================================================
// 将单个机器人的 EKF 里程计变换到 map 坐标系。
//
// 位置: odom 帧 (x, y) → 通过 map→odom TF 旋转平移 → map 帧
// 速度: 本体帧 (vx, vy) → 总偏航角 = tf_yaw + ekf_yaw → 旋转 → map 帧
//
// 如果 map→odom TF 尚不可用则返回 false。
// ============================================================================
static bool ekf_to_map(tf2_ros::Buffer& tf, const std::string& ns,
                       const nav_msgs::msg::Odometry::SharedPtr& odom,
                       Vec4d& x, double& map_yaw, double& angular_z)
{
  if (!odom) return false;

  // 构造 TF 帧名: /robot1 → robot1_odom
  std::string odom_frame = ns;
  if (!odom_frame.empty() && odom_frame[0] == '/') odom_frame = odom_frame.substr(1);
  odom_frame += "_odom";

  try {
    // map → odom 变换（来自 slam_toolbox 或 AMCL）
    auto t = tf.lookupTransform("map", odom_frame, tf2::TimePoint());
    double tf_x   = t.transform.translation.x;
    double tf_y   = t.transform.translation.y;
    double tf_yaw = tf2_yaw(tf2::Quaternion(
        t.transform.rotation.x, t.transform.rotation.y,
        t.transform.rotation.z, t.transform.rotation.w));

    // EKF 在自身 odom 帧中的位姿
    double ekf_px  = odom->pose.pose.position.x;
    double ekf_py  = odom->pose.pose.position.y;
    double ekf_yaw = msg_yaw(odom->pose.pose.orientation);

    // map 帧位置 = T_map_odom · [ekf_px, ekf_py]
    x(0) = tf_x + ekf_px * std::cos(tf_yaw) - ekf_py * std::sin(tf_yaw);
    x(1) = tf_y + ekf_px * std::sin(tf_yaw) + ekf_py * std::cos(tf_yaw);

    // map 帧速度 = 本体速度旋转总偏航角
    double total_yaw = tf_yaw + ekf_yaw;
    double vx_body = odom->twist.twist.linear.x;
    double vy_body = odom->twist.twist.linear.y;
    x(2) = vx_body * std::cos(total_yaw) - vy_body * std::sin(total_yaw);
    x(3) = vx_body * std::sin(total_yaw) + vy_body * std::cos(total_yaw);

    map_yaw   = total_yaw;
    angular_z = odom->twist.twist.angular.z;
    return true;
  } catch (const tf2::TransformException&) {
    return false;  // TF 尚未就绪，跳过本周期
  }
}

// ============================================================================
// 定时器回调 — 主控制循环 (~20 Hz)
// ============================================================================
void FormationController::timer_cb()
{
  if (!leader_ok_ || !follower_ok_) return;

  Vec4d x1, x2;
  double leader_yaw, follower_yaw, leader_az;
  if (!ekf_to_map(*tf_buffer_, leader_ns_,   leader_odom_,   x1, leader_yaw,   leader_az)) return;
  if (!ekf_to_map(*tf_buffer_, follower_ns_, follower_odom_, x2, follower_yaw, leader_az)) return;

  // 延迟初始化: 收到第一帧完整数据后初始化控制器
  if (!controller_initialized_) {
    try {
      ctrl_->controller_initial(x1, x2);
      controller_initialized_ = true;
      RCLCPP_INFO(get_logger(), "控制器初始化完成。");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "初始化失败: %s", e.what());
      return;
    }
  }

  // 齐次控制律（算法本身未修改）
  auto out = ctrl_->lpc_calculate(x1, x2);

  // 构建并发布速度指令
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = std::clamp(out[0], -1.0, 1.0);
  cmd.linear.y = std::clamp(out[1], -1.0, 1.0);

  // 偏航控制: 比例（归一化后）+ 前馈
  double raw_err   = leader_yaw - follower_yaw;
  double norm_err  = std::atan2(std::sin(raw_err), std::cos(raw_err));
  cmd.angular.z = std::clamp(norm_err * Kp_yaw_ + leader_az * K_ff_, -0.5, 0.5);

  cmd_pub_->publish(cmd);
}
