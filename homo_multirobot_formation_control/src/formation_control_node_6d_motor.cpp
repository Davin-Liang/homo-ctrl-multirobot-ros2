/// @file 6D 电机感知模型编队控制节点实现。
///
/// 数据流:
///   EKF odometry/filtered ──→ 缓冲区（回调存储最新消息）
///   TF  map→X_odom        ──→ 定时器读取当前 TF
///
/// 定时器 (20 Hz):
///   1. 查找两机器人的 map → odom TF + EKF 位姿 → map 系位置/偏航/速度
///   2. 组装 6 维状态:
///        leader   x1 = [p, v_meas, v_meas]   (v_cmd 无法获知，稳态假设)
///        follower x2 = [p, v_cmd(内部),  v_meas]
///   3. LpcController6DMotor::lpc_calculate → goal_v_cmd (map 系)
///   4. 旋转到车体系 → clamp → 轮速约束 → 发布 cmd_vel
///   5. 将最终发布值旋转回 map 系回写 v_cmd 内部状态（抗饱和）

#include "homo_multirobot_formation_control/formation_control_node_6d_motor.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace formation_control;

// ============================================================================
// 构造函数 — 声明参数、创建 TF 缓冲 + 订阅 + 定时器
// ============================================================================
FormationController6DMotor::FormationController6DMotor()
: rclcpp::Node("formation_control_node_6d_motor")
{
  leader_ns_   = declare_parameter("leader_ns",   "/robot1");
  follower_ns_ = declare_parameter("follower_ns", "/robot2");
  int m_p      = declare_parameter("m_p",      4);
  double radius = declare_parameter("radius",  2.0);
  double tol    = declare_parameter("tol",     0.1);
  double mass   = declare_parameter("mass",    8.0);
  double tau    = declare_parameter("tau",     0.5);
  double omega_d = declare_parameter("omega_d", 1.5);
  Kp_yaw_       = declare_parameter("Kp_yaw",  4.0);
  K_ff_         = declare_parameter("K_ff",    1.0);
  control_rate_ = declare_parameter("control_rate", 20.0);

  double wheel_radius    = declare_parameter("wheel_radius",    0.03);
  double base_radius     = declare_parameter("base_radius",     0.11);
  double wheel_max_omega = declare_parameter("wheel_max_omega", 20.0);
  double max_linear_accel  = declare_parameter("max_linear_accel",  2.0);
  double max_angular_accel = declare_parameter("max_angular_accel", 4.0);

  max_linear_vel_  = declare_parameter("max_linear_vel",  1.0);
  max_angular_vel_ = declare_parameter("max_angular_vel", 0.5);

  bool use_hpc = declare_parameter("use_hpc", true);
  double hpc_c_min = declare_parameter("hpc_c_min", 0.9);
  leader_vel_lpf_tau_ = declare_parameter("leader_vel_lpf_tau", 0.0);

  ctrl_ = std::make_unique<LpcController6DMotor>(m_p, radius, tol, mass, tau,
                                                 omega_d, use_hpc,
                                                 1.0 / control_rate_,
                                                 hpc_c_min);

  constraint_ = KinematicConstraint(wheel_radius, base_radius,
                                    wheel_max_omega,
                                    max_linear_accel, max_angular_accel);

  // TF 监听 slam_toolbox/AMCL 的 map→odom 变换链
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // EKF 里程计: 自身一致的速度
  auto qos = rclcpp::SensorDataQoS();
  leader_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    leader_ns_ + "/odometry/filtered", qos,
    [this](nav_msgs::msg::Odometry::SharedPtr m) {
      leader_odom_ = m; leader_ok_ = true;
      leader_odom_stamp_ = m->header.stamp;
    });
  follower_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    follower_ns_ + "/odometry/filtered", qos,
    [this](nav_msgs::msg::Odometry::SharedPtr m) {
      follower_odom_ = m; follower_ok_ = true;
      follower_odom_stamp_ = m->header.stamp;
    });

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  last_diag_time_ = get_clock()->now();

  int ms = static_cast<int>(1000.0 / control_rate_);
  timer_ = create_wall_timer(std::chrono::milliseconds(ms), [this]() { timer_cb(); });

  RCLCPP_INFO(get_logger(), "6D 电机感知模型编队控制节点已启动 (tau=%.2fs)。", tau);
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
// 将单个机器人的 EKF 里程计变换到 map 坐标系（位置 + 测量速度）。
//
// 位置: odom 帧 (x, y) → 通过 map→odom TF 旋转平移 → map 帧
// 速度: 本体帧 (vx, vy) → 总偏航角 = tf_yaw + ekf_yaw → 旋转 → map 帧
//
// 输出 px, py, vx_meas, vy_meas；6 维状态的 v_cmd 槽位由调用方填充。
// 如果 map→odom TF 尚不可用则返回 false。
// ============================================================================
static bool ekf_to_map(tf2_ros::Buffer& tf, const std::string& ns,
                       const nav_msgs::msg::Odometry::SharedPtr& odom,
                       double& px, double& py, double& vx_meas, double& vy_meas,
                       double& map_yaw, double& angular_z)
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
    px = tf_x + ekf_px * std::cos(tf_yaw) - ekf_py * std::sin(tf_yaw);
    py = tf_y + ekf_px * std::sin(tf_yaw) + ekf_py * std::cos(tf_yaw);

    // map 帧速度 = 本体速度旋转总偏航角
    double total_yaw = tf_yaw + ekf_yaw;
    double vx_body = odom->twist.twist.linear.x;
    double vy_body = odom->twist.twist.linear.y;
    vx_meas = vx_body * std::cos(total_yaw) - vy_body * std::sin(total_yaw);
    vy_meas = vx_body * std::sin(total_yaw) + vy_body * std::cos(total_yaw);

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
void FormationController6DMotor::timer_cb()
{
  if (!leader_ok_ || !follower_ok_) return;

  double l_px, l_py, l_vx, l_vy, leader_yaw, leader_az;
  double f_px, f_py, f_vx, f_vy, follower_yaw, follower_az;
  if (!ekf_to_map(*tf_buffer_, leader_ns_, leader_odom_,
                  l_px, l_py, l_vx, l_vy, leader_yaw, leader_az)) return;
  if (!ekf_to_map(*tf_buffer_, follower_ns_, follower_odom_,
                  f_px, f_py, f_vx, f_vy, follower_yaw, follower_az)) return;

  // 低速时 leader EKF/rf2o 速度噪声 (~0.05m/s) 被 k2+k3 放大为虚假控制力。
  // 一阶低通切除高频噪声；tau≤0 时直通原始测量（关断低通）。
  double l_vx_f, l_vy_f;
  if (leader_vel_lpf_tau_ <= 0.0) {
    l_vx_f = l_vx; l_vy_f = l_vy;
  } else {
    if (!leader_vel_filtered_) {
      lpf_leader_vx_ = l_vx; lpf_leader_vy_ = l_vy;
      leader_vel_filtered_ = true;
    } else {
      double alpha = control_rate_ / (control_rate_ + 1.0 / leader_vel_lpf_tau_);
      lpf_leader_vx_ += alpha * (l_vx - lpf_leader_vx_);
      lpf_leader_vy_ += alpha * (l_vy - lpf_leader_vy_);
    }
    l_vx_f = lpf_leader_vx_; l_vy_f = lpf_leader_vy_;
  }

  // 组装 6 维状态。leader: v_cmd = v_real = (可能低通后)测量速度 (稳态假设)
  Eigen::VectorXd x1(6), x2(6);
  x1 << l_px, l_py, l_vx_f, l_vy_f, l_vx_f, l_vy_f;

  // 延迟初始化: 收到第一帧完整数据后初始化控制器。
  // v_cmd 仅在此处对齐 EKF 速度，之后由发布回写维护（内部积分状态）。
  if (!controller_initialized_) {
    vx_cmd_map_ = f_vx;
    vy_cmd_map_ = f_vy;
    x2 << f_px, f_py, vx_cmd_map_, vy_cmd_map_, f_vx, f_vy;
    try {
      ctrl_->controller_initial(x1, x2);
      controller_initialized_ = true;
      RCLCPP_INFO(get_logger(), "6D Motor 控制器初始化完成。");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "初始化失败: %s", e.what());
      return;
    }
  }

  // follower: v_cmd 来自内部状态，v_real 来自 EKF（每周期都读）
  x2 << f_px, f_py, vx_cmd_map_, vy_cmd_map_, f_vx, f_vy;

  // 齐次控制律 → map 系指令速度
  auto out = ctrl_->lpc_calculate(x1, x2);

  // 将 map 系速度旋转到车体坐标系（cmd_vel 语义为车体系）
  double vx_body =  out[0] * std::cos(follower_yaw) + out[1] * std::sin(follower_yaw);
  double vy_body = -out[0] * std::sin(follower_yaw) + out[1] * std::cos(follower_yaw);

  // 构建速度指令
  geometry_msgs::msg::Twist cmd;
  double vx_clamped = std::clamp(vx_body, -max_linear_vel_, max_linear_vel_);
  double vy_clamped = std::clamp(vy_body, -max_linear_vel_, max_linear_vel_);
  cmd.linear.x = vx_clamped;
  cmd.linear.y = vy_clamped;

  // 偏航控制: 比例（归一化后）+ 前馈（与 4D 相同，独立于 6D 模型）
  double raw_err   = leader_yaw - follower_yaw;
  double norm_err  = std::atan2(std::sin(raw_err), std::cos(raw_err));
  cmd.angular.z = std::clamp(norm_err * Kp_yaw_ + leader_az * K_ff_,
                              -max_angular_vel_, max_angular_vel_);

  // 全向轮运动学约束（轮速 + 加速度限幅）
  double dt = 1.0 / control_rate_;
  double wheel_scale = constraint_.apply(cmd.linear.x, cmd.linear.y, cmd.angular.z, dt);

  // 调试：raw=算法原始速度  clamped=硬限幅后  final=运动学约束后
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "raw=(%+6.3f,%+6.3f) clamped=(%+6.3f,%+6.3f) final=(%+6.3f,%+6.3f) scale=%.2f "
    "vcmd=(%+6.3f,%+6.3f) vreal=(%+6.3f,%+6.3f)",
    vx_body, vy_body, vx_clamped, vy_clamped, cmd.linear.x, cmd.linear.y, wheel_scale,
    vx_cmd_map_, vy_cmd_map_, f_vx, f_vy);

  // 诊断：实际控制频率 + 平均数据新鲜度（每 5 秒输出平均值）
  ++diag_tick_;
  auto now = get_clock()->now();
  sum_leader_age_ += (now - leader_odom_stamp_).seconds();
  sum_ekf_age_    += (now - follower_odom_stamp_).seconds();
  double diag_elapsed = (now - last_diag_time_).seconds();
  if (diag_elapsed >= 5.0) {
    double real_freq = diag_tick_ / diag_elapsed;
    double avg_leader_age_ms = sum_leader_age_ / diag_tick_ * 1000.0;
    double avg_ekf_age_ms    = sum_ekf_age_    / diag_tick_ * 1000.0;
    RCLCPP_INFO(get_logger(),
      "DIAG: freq=%.1fHz avg_leader_age=%.0fms avg_ekf_age=%.0fms "
      "vcmd_vs_vreal=(%+.3f,%+.3f)",
      real_freq, avg_leader_age_ms, avg_ekf_age_ms,
      vx_cmd_map_ - f_vx, vy_cmd_map_ - f_vy);
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

  // ---- v_cmd 回写（抗饱和）: 用最终发布值更新内部指令状态 ---------------------
  // clamp/轮速约束削掉的部分不计入 v_cmd，避免饱和时内部记账虚高、模型预测失真。
  vx_cmd_map_ = cmd.linear.x * std::cos(follower_yaw) - cmd.linear.y * std::sin(follower_yaw);
  vy_cmd_map_ = cmd.linear.x * std::sin(follower_yaw) + cmd.linear.y * std::cos(follower_yaw);
}
