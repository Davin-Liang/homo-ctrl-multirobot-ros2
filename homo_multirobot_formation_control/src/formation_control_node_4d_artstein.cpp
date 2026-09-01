/// @file 4D Artstein-HPC 编队控制节点实现。
///
/// 数据流:
///   EKF odometry/filtered ──→ 缓冲区（回调存储最新消息）
///   TF  map→X_odom        ──→ 定时器读取当前 TF
///
/// 定时器 (20 Hz):
///   1. 查找两机器人的 map → odom TF + EKF 位姿 → map 系位置/偏航/速度
///   2. 组装 4 维测量状态:
///        leader   x1 = [p, v_real]  (4D, EKF 测量)
///        follower x2 = [p, v_real]  (4D, EKF 测量)
///   3. follower Artstein 积分补偿输入死区: x2 → z2
///   4. 前向预测 Td+tau 得到双积分 HPC 反馈状态 x_h=[p_pred,v_pred]
///   5. 原始 4D 双积分 HPC 计算 v_cmd (map 系)
///   6. 旋转到车体系 → clamp → 加速度限幅 → 轮速约束 → 发布 cmd_vel
///   7. 发布值旋转回 map 系 → 回写 vx_cmd_map_ / vy_cmd_map_ → 存入缓冲

#include "homo_multirobot_formation_control/formation_control_node_4d_artstein.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "homo_multirobot_formation_control/formation_safety.hpp"

using namespace formation_control;

// ============================================================================
// EKF 速度转换辅助函数。
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

// 将 EKF 里程计变换到 map 坐标系（位置 + 测量速度 + 偏航）。
// 返回 4 维相关量（无内部命令速度状态）。
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
    double tf_x   = t.transform.translation.x;
    double tf_y   = t.transform.translation.y;
    double tf_yaw = tf2_yaw(tf2::Quaternion(
        t.transform.rotation.x, t.transform.rotation.y,
        t.transform.rotation.z, t.transform.rotation.w));

    double ekf_px  = odom->pose.pose.position.x;
    double ekf_py  = odom->pose.pose.position.y;
    double ekf_yaw = msg_yaw(odom->pose.pose.orientation);

    px = tf_x + ekf_px * std::cos(tf_yaw) - ekf_py * std::sin(tf_yaw);
    py = tf_y + ekf_px * std::sin(tf_yaw) + ekf_py * std::cos(tf_yaw);

    double total_yaw = tf_yaw + ekf_yaw;
    double vx_body = odom->twist.twist.linear.x;
    double vy_body = odom->twist.twist.linear.y;
    vx_meas = vx_body * std::cos(total_yaw) - vy_body * std::sin(total_yaw);
    vy_meas = vx_body * std::sin(total_yaw) + vy_body * std::cos(total_yaw);

    map_yaw   = total_yaw;
    angular_z = odom->twist.twist.angular.z;
    return true;
  } catch (const tf2::TransformException&) {
    return false;
  }
}

// ============================================================================
// 构造函数
// ============================================================================
FormationController4DArtstein::FormationController4DArtstein()
: rclcpp::Node("formation_control_node_4d_artstein")
{
  // ---- 通用参数 -------------------------------------------------------------
  leader_ns_   = declare_parameter("leader_ns",   "/robot1");
  follower_ns_ = declare_parameter("follower_ns", "/robot2");
  int m_p      = declare_parameter("m_p",      4);
  double radius = declare_parameter("radius",  2.0);
  double tol    = declare_parameter("tol",     0.1);
  double mass   = declare_parameter("mass",    1.0);   // 4D 中为速度通道等效增益
  double tau    = declare_parameter("tau",     0.43);
  double omega_d = declare_parameter("omega_d", 0.7);
  Kp_yaw_       = declare_parameter("Kp_yaw",  4.0);
  K_ff_         = declare_parameter("K_ff",    1.0);
  control_rate_ = declare_parameter("control_rate", 20.0);

  Td_ = declare_parameter("Td", 0.22);

  double wheel_radius    = declare_parameter("wheel_radius",    0.03);
  double base_radius     = declare_parameter("base_radius",     0.11);
  double wheel_max_omega = declare_parameter("wheel_max_omega", 20.0);
  double max_linear_accel  = declare_parameter("max_linear_accel",  2.0);
  double max_angular_accel = declare_parameter("max_angular_accel", 4.0);

  max_linear_vel_  = declare_parameter("max_linear_vel",  1.0);
  max_angular_vel_ = declare_parameter("max_angular_vel", 0.5);
  max_linear_accel_ = max_linear_accel;
  formation_radius_ = radius;
  tau_ = tau;
  radial_safety_max_decel_ = declare_parameter("radial_safety_max_decel", 0.0);
  radial_safety_effective_delay_ =
      declare_parameter("radial_safety_effective_delay", -1.0);

  bool use_hpc = declare_parameter("use_hpc", true);
  double hpc_c_min = declare_parameter("hpc_c_min", 0.1);
  double initial_min_lambda = declare_parameter("initial_min_lambda", 1.0);
  double switch_min_lambda = declare_parameter("switch_min_lambda", 4.0);
  leader_vel_lpf_tau_ = declare_parameter("leader_vel_lpf_tau", 0.0);
  min_cmd_vel_ = declare_parameter("min_cmd_vel", 0.03);
  enable_radial_safety_ = declare_parameter("enable_radial_safety", true);

  // ---- 控制器 ---------------------------------------------------------------
  ctrl_ = std::make_unique<LpcController4DArtstein>(m_p, radius, tol, mass, tau,
                                                    omega_d, use_hpc,
                                                    1.0 / control_rate_,
                                                    hpc_c_min,
                                                    Td_,
                                                    initial_min_lambda,
                                                    switch_min_lambda);

  constraint_ = KinematicConstraint(wheel_radius, base_radius,
                                    wheel_max_omega,
                                    max_linear_accel, max_angular_accel);

  // ---- TF ------------------------------------------------------------------
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // ---- 订阅 EKF 里程计 -----------------------------------------------------
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

  RCLCPP_INFO(get_logger(), "4D Artstein-HPC node started (Td=%.3fs, tau=%.2fs).",
              Td_, tau);
  RCLCPP_INFO(get_logger(), "  Leader: %s, Follower: %s",
              leader_ns_.c_str(), follower_ns_.c_str());
}

// ============================================================================
// 定时器回调 — 主控制循环 (~20 Hz)
// ============================================================================
void FormationController4DArtstein::timer_cb()
{
  if (!leader_ok_ || !follower_ok_) return;

  // ---- 步骤 1: TF + EKF → map 系测量值 ------------------------------------
  double l_px, l_py, l_vx, l_vy, leader_yaw, leader_az;
  double f_px, f_py, f_vx, f_vy, follower_yaw, follower_az;
  if (!ekf_to_map(*tf_buffer_, leader_ns_, leader_odom_,
                  l_px, l_py, l_vx, l_vy, leader_yaw, leader_az)) return;
  if (!ekf_to_map(*tf_buffer_, follower_ns_, follower_odom_,
                  f_px, f_py, f_vx, f_vy, follower_yaw, follower_az)) return;

  // ---- 步骤 2: leader 速度低通滤波 ------------------------------------------
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

  // ---- 步骤 3: Leader 匀速外推到执行器预测时刻 ----------------------------
  int buf_size = ctrl_->artstein_buffer_size();

  Eigen::Vector4d x1_meas;
  x1_meas << l_px, l_py, l_vx_f, l_vy_f;
  Eigen::Vector4d x1_h = ctrl_->predict_leader_state(x1_meas);

  // ---- 步骤 4: Follower Artstein 缓冲 + 积分 --------------------------------
  // 延迟初始化：收到第一帧完整数据后初始化控制器 + v^cmd 内部状态。
  if (!controller_initialized_) {
    vx_cmd_map_ = f_vx;
    vy_cmd_map_ = f_vy;

    // 用当前测量速度初始化 follower 缓冲（假设过去 Td 秒内 v^cmd = 当前测量值）
    follower_vcmd_history_.clear();
    for (int i = 0; i < buf_size; ++i)
      follower_vcmd_history_.push_back(Eigen::Vector2d(f_vx, f_vy));

    Eigen::Vector4d x2_meas;
    x2_meas << f_px, f_py, f_vx, f_vy;
    Eigen::Vector4d I2_init = ctrl_->compute_artstein_integral(follower_vcmd_history_);
    Eigen::Vector4d z2_init = x2_meas + I2_init;
    Eigen::Vector4d x2_h_init = ctrl_->predict_hpc_state(
        z2_init, Eigen::Vector2d(vx_cmd_map_, vy_cmd_map_));

    try {
      ctrl_->controller_initial(x1_h, x2_h_init);
      controller_initialized_ = true;
      RCLCPP_INFO(get_logger(), "4D Artstein-HPC 控制器初始化完成。");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "初始化失败: %s", e.what());
      return;
    }
  }

  // follower Artstein 积分（缓冲存实际发布的 v^cmd）
  Eigen::Vector4d x2_meas;
  x2_meas << f_px, f_py, f_vx, f_vy;
  Eigen::Vector4d I2 = ctrl_->compute_artstein_integral(follower_vcmd_history_);
  Eigen::Vector4d z2 = x2_meas + I2;
  Eigen::Vector4d x2_h = ctrl_->predict_hpc_state(
      z2, Eigen::Vector2d(vx_cmd_map_, vy_cmd_map_));

  Eigen::Vector4d x1_real;
  x1_real << l_px, l_py, l_vx_f, l_vy_f;
  Eigen::Vector4d x2_real;
  x2_real << f_px, f_py, f_vx, f_vy;

  double real_nearest_dist = ctrl_->calculate_distance(x1_real, x2_real);
  double pred_nearest_dist = ctrl_->calculate_distance(x1_h, x2_h);
  double v_real_mag = std::hypot(f_vx, f_vy);
  double v_cmd_mag = std::hypot(vx_cmd_map_, vy_cmd_map_);
  double v_pred_mag = std::hypot(x2_h(2), x2_h(3));
  double pred_pos_shift = (x2_h.head<2>() - x2_real.head<2>()).norm();
  double pred_vel_shift = (x2_h.tail<2>() - Eigen::Vector2d(vx_cmd_map_, vy_cmd_map_)).norm();

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "PRED_DIAG real_dist=%.3f pred_dist=%.3f dist_delta=%+.3f "
    "|v_real|=%.3f |v_cmd|=%.3f |v_pred|=%.3f pos_shift=%.3f vel_shift=%.3f",
    real_nearest_dist, pred_nearest_dist, pred_nearest_dist - real_nearest_dist,
    v_real_mag, v_cmd_mag, v_pred_mag, pred_pos_shift, pred_vel_shift);

  // ---- 步骤 5: 原始 4D 双积分 HPC → map 系速度指令 -------------------------
  // 预测器已经提供 x_h；HPC 直接输出当前周期的 map 系速度命令。
  const auto out = ctrl_->lpc_calculate(x1_h, x2_h);
  Eigen::Vector2d out_map(out[0], out[1]);

  const double safety_max_decel =
      radial_safety_max_decel_ > 0.0
          ? std::min(max_linear_accel_, radial_safety_max_decel_)
          : max_linear_accel_;
  const double safety_effective_delay =
      radial_safety_effective_delay_ >= 0.0
          ? radial_safety_effective_delay_
          : Td_ + tau_;
  if (enable_radial_safety_) {
    out_map = formation_control::apply_radial_safety_limit(
        out_map, x2_real.tail<2>(), x1_real.tail<2>(),
        x1_real.head<2>(), x2_real.head<2>(), formation_radius_,
        safety_max_decel, safety_effective_delay, max_linear_vel_);
  }

  double current_dist = ctrl_->current_distance(x1_h, x2_h);
  double best_dist = ctrl_->best_distance(x1_h, x2_h);
  Eigen::Vector4d selected_err = ctrl_->selected_error(x1_h, x2_h);

  // ---- Step 6: rotate map-frame command into body frame --------------------
  double vx_body =  out_map(0) * std::cos(follower_yaw) + out_map(1) * std::sin(follower_yaw);
  double vy_body = -out_map(0) * std::sin(follower_yaw) + out_map(1) * std::cos(follower_yaw);

  // ---- 步骤 7: 速度 clamp ---------------------------------------------------
  double vx_clamped = std::clamp(vx_body, -max_linear_vel_, max_linear_vel_);
  double vy_clamped = std::clamp(vy_body, -max_linear_vel_, max_linear_vel_);

  // ---- 步骤 8: 最小速度补偿（实物 STM32 死区 ~0.03 m/s）--------------------
  if (min_cmd_vel_ > 0.0) {
    double raw_mag = std::hypot(vx_body, vy_body);
    double cmd_mag = std::hypot(vx_clamped, vy_clamped);
    if (raw_mag > 0.001 && cmd_mag > 0.0 && cmd_mag < min_cmd_vel_) {
      double scale = min_cmd_vel_ / cmd_mag;
      vx_clamped *= scale;
      vy_clamped *= scale;
    }
  }

  // ---- 步骤 9: 偏航控制 -----------------------------------------------------
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = vx_clamped;
  cmd.linear.y = vy_clamped;

  double raw_err   = leader_yaw - follower_yaw;
  double norm_err  = std::atan2(std::sin(raw_err), std::cos(raw_err));
  cmd.angular.z = std::clamp(norm_err * Kp_yaw_ + leader_az * K_ff_,
                              -max_angular_vel_, max_angular_vel_);

  // ---- 步骤 10: 轮速约束 + 加速度限幅 ---------------------------------------
  double dt = 1.0 / control_rate_;
  double wheel_scale = constraint_.apply(cmd.linear.x, cmd.linear.y, cmd.angular.z, dt);

  // The kinematic constraints can retain a previous inward command due to
  // slew limiting. Enforce the braking envelope on the final map-frame
  // command and synchronize that safety-arbitrated command for the next tick.
  if (enable_radial_safety_) {
    const Eigen::Vector2d final_map_before(
        cmd.linear.x * std::cos(follower_yaw) - cmd.linear.y * std::sin(follower_yaw),
        cmd.linear.x * std::sin(follower_yaw) + cmd.linear.y * std::cos(follower_yaw));
    const Eigen::Vector2d final_map = formation_control::apply_radial_safety_limit(
        final_map_before, x2_real.tail<2>(), x1_real.tail<2>(),
        x1_real.head<2>(), x2_real.head<2>(), formation_radius_,
        safety_max_decel, safety_effective_delay, max_linear_vel_);

    if ((final_map - final_map_before).norm() > 1e-6) {
      cmd.linear.x =
          final_map(0) * std::cos(follower_yaw) + final_map(1) * std::sin(follower_yaw);
      cmd.linear.y =
          -final_map(0) * std::sin(follower_yaw) + final_map(1) * std::cos(follower_yaw);
      constraint_.set_last_command(cmd.linear.x, cmd.linear.y, cmd.angular.z);

      const Eigen::Vector2d rel = x2_real.head<2>() - x1_real.head<2>();
      const double dist = rel.norm();
      if (dist > 1e-9) {
        const Eigen::Vector2d radial = rel / dist;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "RADIAL_SAFE dist=%.3f radius=%.3f cmd_rel_rad %.3f->%.3f "
          "a_brake=%.3f Td_eff=%.3f",
          dist, formation_radius_,
          (final_map_before - x1_real.tail<2>()).dot(radial),
          (final_map - x1_real.tail<2>()).dot(radial),
          safety_max_decel, safety_effective_delay);
      }
    }
  }

  // ---- 步骤 11: 调试输出 ----------------------------------------------------
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "TRACE target=%d best=%.3f current=%.3f sel=(%+6.3f,%+6.3f,%+6.3f,%+6.3f) "
    "raw=(%+6.3f,%+6.3f) final=(%+6.3f,%+6.3f) scale=%.2f "
    "I=(%+6.3f,%+6.3f,%+6.3f,%+6.3f) xh=(%+6.3f,%+6.3f,%+6.3f,%+6.3f)",
    ctrl_->target_index(), best_dist, current_dist,
    selected_err(0), selected_err(1), selected_err(2), selected_err(3),
    vx_body, vy_body, cmd.linear.x, cmd.linear.y, wheel_scale,
    I2(0), I2(1), I2(2), I2(3),
    x2_h(0), x2_h(1), x2_h(2), x2_h(3));

  // ---- 诊断：实际控制频率 + 数据新鲜度（每 5 秒）----------------------------
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
      "vcmd=(%+.3f,%+.3f) vreal=(%+.3f,%+.3f)",
      real_freq, avg_leader_age_ms, avg_ekf_age_ms,
      vx_cmd_map_, vy_cmd_map_, f_vx, f_vy);
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

  // ---- 步骤 12: 发布 cmd_vel ------------------------------------------------
  cmd_pub_->publish(cmd);

  // ---- 步骤 13: v^cmd 回写 + 入缓冲 -----------------------------------------
  // 将限幅后实际发布的值旋转回 map 系，回写内部状态，并存入 follower Artstein 缓冲。
  // 缓冲存的是实际发出的值（限幅后），保证 Artstein 积分基于真实执行器历史。
  vx_cmd_map_ = cmd.linear.x * std::cos(follower_yaw) - cmd.linear.y * std::sin(follower_yaw);
  vy_cmd_map_ = cmd.linear.x * std::sin(follower_yaw) + cmd.linear.y * std::cos(follower_yaw);

  follower_vcmd_history_.push_front(Eigen::Vector2d(vx_cmd_map_, vy_cmd_map_));
  while (static_cast<int>(follower_vcmd_history_.size()) > buf_size)
    follower_vcmd_history_.pop_back();
}
