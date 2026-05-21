#include "homo_multirobot_formation_control/obstacle_avoider.hpp"

#include <cmath>
#include <algorithm>
#include <chrono>

namespace formation_control {

// ============================================================================
// 构造函数 — 声明 ROS 参数、创建 scan 订阅
// ============================================================================
ObstacleAvoider::ObstacleAvoider(rclcpp::Node* node)
  : node_(node)
{
  // 障碍物检测参数
  std::string scan_topic = node_->declare_parameter("scan_topic", "scan");
  safety_distance_   = node_->declare_parameter("safety_distance",   0.5);   // d_safe
  obstacle_weight_   = node_->declare_parameter("obstacle_weight",   1.0);   // w_0
  time_horizon_      = node_->declare_parameter("time_horizon",      0.5);   // T
  max_obstacles_     = node_->declare_parameter("max_obstacles",     10);    // M_max
  cluster_tolerance_ = node_->declare_parameter("cluster_tolerance", 0.1);   // d_cluster
  min_cluster_size_  = node_->declare_parameter("min_cluster_size",  5);     // N_min

  // 部分参数可能与节点共用（如 max_linear_accel），声明前检查避免冲突
  auto get_or_declare_double = [&](const std::string& name, double default_val) {
    if (!node_->has_parameter(name))
      node_->declare_parameter(name, default_val);
    return node_->get_parameter(name).as_double();
  };

  max_linear_vel_    = get_or_declare_double("max_linear_vel",    1.0);   // v_max
  max_angular_vel_   = get_or_declare_double("max_angular_vel",   0.5);   // ω_max
  max_linear_accel_  = get_or_declare_double("max_linear_accel",  2.0);   // a_max
  max_angular_accel_ = get_or_declare_double("max_angular_accel", 4.0);   // α_max

  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { scan_callback(msg); });

  RCLCPP_INFO(node_->get_logger(),
    "[OA] ObstacleAvoider initialized. scan_topic=%s, safety=%.2fm, "
    "weight=%.2f, horizon=%.2fs, max_obs=%d",
    scan_topic.c_str(), safety_distance_, obstacle_weight_,
    time_horizon_, max_obstacles_);
}

// ============================================================================
// 激光回调 (~15 Hz) — 每收到一帧 /scan 即执行聚类+关联
// ============================================================================
void ObstacleAvoider::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  double dt = 0.0;
  if (scan_received_) {
    rclcpp::Time now(msg->header.stamp);
    dt = (now - last_scan_stamp_).seconds();
    if (dt <= 0.0) dt = 1.0 / 15.0;    // fallback: 假设 ~15 Hz
  }

  last_scan_stamp_ = rclcpp::Time(msg->header.stamp);
  scan_received_ = true;

  RCLCPP_INFO(node_->get_logger(),
    "[OA] scan rx: seq=%d, ranges=%zu, stamp=%.3f dt=%.3f",
    msg->header.stamp.sec, msg->ranges.size(),
    static_cast<double>(msg->header.stamp.sec) +
    msg->header.stamp.nanosec * 1e-9, dt);

  filter_and_cluster(*msg);
  associate_and_estimate_velocity(dt);
}

// ============================================================================
// 点云滤波 + 欧几里得聚类
//
// 算法：利用扫描点按角度排序的性质，连续点距离 ≤ d_cluster 的归为一簇。
// 特殊处理：绕过 0°/360° 边界的聚类（首尾合并）。
//
// 输出：每个簇取「最近点」作为障碍物位置 o_j，
//       半径 r_j 取点到 o_j 的最大距离（上限 0.5m，下限 0.05m）。
// ============================================================================
void ObstacleAvoider::filter_and_cluster(
    const sensor_msgs::msg::LaserScan& scan)
{
  using Pt = Eigen::Vector2d;
  std::vector<std::vector<Pt>> raw_clusters;
  std::vector<Pt> current;

  int valid_count = 0;

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    double r = scan.ranges[i];

    // 滤除无效点
    if (std::isinf(r) || std::isnan(r) ||
        r < scan.range_min || r > scan.range_max)
    {
      if (current.size() >= static_cast<size_t>(min_cluster_size_))
        raw_clusters.push_back(std::move(current));
      current.clear();
      continue;
    }

    valid_count++;
    double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    Pt pt(r * std::cos(angle), r * std::sin(angle));   // 极坐标 → 笛卡尔（车体系）

    if (current.empty()) {
      current.push_back(pt);
    } else {
      double dist = (pt - current.back()).norm();
      if (dist <= cluster_tolerance_) {
        current.push_back(pt);               // 同簇
      } else {
        if (current.size() >= static_cast<size_t>(min_cluster_size_))
          raw_clusters.push_back(std::move(current));
        current.clear();
        current.push_back(pt);               // 新簇
      }
    }
  }

  // 收尾
  if (current.size() >= static_cast<size_t>(min_cluster_size_))
    raw_clusters.push_back(std::move(current));

  // 环绕边界合并：检查首簇首点与尾簇尾点距离
  if (raw_clusters.size() >= 2) {
    const auto& first = raw_clusters.front();
    const auto& last  = raw_clusters.back();
    double dist = (first.front() - last.back()).norm();
    if (dist <= cluster_tolerance_) {
      auto merged = std::move(raw_clusters.front());
      merged.insert(merged.begin(),
                    raw_clusters.back().begin(), raw_clusters.back().end());
      raw_clusters.front() = std::move(merged);
      raw_clusters.pop_back();
    }
  }

  // 簇 → 障碍物：最近点 + 半径上限
  // 最近点表示使排斥方向 n_j 始终垂直于局部曲面 (§2.2)
  std::vector<Obstacle> new_obstacles;
  for (const auto& cluster : raw_clusters) {
    // 最近点 o_j = argmin ||p||
    Pt closest = cluster[0];
    double min_dist2 = closest.squaredNorm();
    for (size_t i = 1; i < cluster.size(); ++i) {
      double d2 = cluster[i].squaredNorm();
      if (d2 < min_dist2) {
        min_dist2 = d2;
        closest = cluster[i];
      }
    }

    // 半径 r_j = max ||p - o_j||，裁剪到 [0.05, 0.5] m
    double r = 0.0;
    for (const auto& p : cluster) {
      double d = (p - closest).norm();
      if (d > r) r = d;
    }
    if (r > 0.5) r = 0.5;     // 上限：避免狭长墙面产生过大半径
    if (r < 0.05) r = 0.05;   // 下限：保证细小障碍物有最小排斥范围

    Obstacle obs;
    obs.position = closest;
    obs.radius   = r;
    obs.velocity.setZero();
    obs.id        = -1;
    obs.lost_frames = 0;
    new_obstacles.push_back(obs);
  }

  RCLCPP_INFO(node_->get_logger(),
    "[OA] cluster: found %zu obstacles, valid_points=%d",
    new_obstacles.size(), valid_count);

  // 限制障碍物数量（保留最近的 M_max 个）
  if (static_cast<int>(new_obstacles.size()) > max_obstacles_) {
    std::sort(new_obstacles.begin(), new_obstacles.end(),
              [](const Obstacle& a, const Obstacle& b) {
                return a.position.squaredNorm() < b.position.squaredNorm();
              });
    new_obstacles.resize(static_cast<size_t>(max_obstacles_));
  }

  prev_obstacles_ = std::move(obstacles_);
  obstacles_ = std::move(new_obstacles);
}

// ============================================================================
// 多帧关联 + 速度估计
//
// 最近邻匹配（max_match_dist = 0.5m）关联同 ID 障碍物。
// 位置做指数滑动平均：  o_j = (1-α_p)·o_j_prev + α_p·o_j_raw    (§2.3)
// 速度做指数滑动平均：  v_j = (1-α_v)·v_j_prev + α_v·(Δo_j/Δt)
// 未匹配超过 3 帧的追踪目标被丢弃。
// ============================================================================
void ObstacleAvoider::associate_and_estimate_velocity(double dt)
{
  if (dt <= 1e-6) dt = 1.0 / 15.0;

  const double max_match_dist = 0.5;
  const double alpha_vel = 0.3;      // 速度平滑系数 α_v
  const double alpha_pos = 0.4;      // 位置平滑系数 α_p

  std::vector<bool> prev_matched(prev_obstacles_.size(), false);

  for (auto& obs : obstacles_) {
    double best_dist = max_match_dist;
    int    best_idx  = -1;

    for (size_t j = 0; j < prev_obstacles_.size(); ++j) {
      if (prev_matched[j]) continue;
      double d = (obs.position - prev_obstacles_[j].position).norm();
      if (d < best_dist) {
        best_dist = d;
        best_idx  = static_cast<int>(j);
      }
    }

    if (best_idx >= 0) {
      prev_matched[best_idx] = true;
      const auto& prev = prev_obstacles_[best_idx];

      obs.id = prev.id;                                            // 继承 ID
      obs.position = (1.0 - alpha_pos) * prev.position + alpha_pos * obs.position;
      Eigen::Vector2d raw_vel = (obs.position - prev.position) / dt;
      obs.velocity = (1.0 - alpha_vel) * prev.velocity + alpha_vel * raw_vel;
      obs.lost_frames = 0;
    } else {
      obs.id = next_obstacle_id_++;                                // 新障碍物，分配新 ID
      obs.velocity.setZero();
      obs.lost_frames = 0;
    }
  }

  // 未匹配的旧障碍物保留最多 3 帧（处理短暂遮挡）
  for (size_t j = 0; j < prev_obstacles_.size(); ++j) {
    if (prev_matched[j]) continue;
    auto& prev = prev_obstacles_[j];
    prev.lost_frames++;
    if (prev.lost_frames <= 3) {
      obstacles_.push_back(prev);
    }
  }

  for (size_t i = 0; i < obstacles_.size(); ++i) {
    const auto& o = obstacles_[i];
    double d_surface = o.position.norm() - o.radius;   // d_surf = d - r
    RCLCPP_INFO(node_->get_logger(),
      "[OA] obs[%d]: id=%d pos=(%.2f,%.2f) r=%.2f d_center=%.2f d_surf=%.2f v=(%.2f,%.2f)",
      static_cast<int>(i), o.id,
      o.position.x(), o.position.y(), o.radius, o.position.norm(), d_surface,
      o.velocity.x(), o.velocity.y());
  }
}

// ============================================================================
// 光滑 max(0, x) 函数  φ(x) = 0.5·(x + √(x²+ε²)),  ε = 1e-4     (§3.2)
//
// 性质：处处 C² 连续。
//   x ≫ 0 → φ(x) ≈ x,   φ'(x) ≈ 1
//   x ≪ 0 → φ(x) ≈ 0,   φ'(x) ≈ 0
//   x = 0 → φ(0) ≈ ε/2 ≈ 0
// ============================================================================
double ObstacleAvoider::penalty(double x)
{
  return 0.5 * (x + std::sqrt(x * x + kSmoothEps * kSmoothEps));
}

double ObstacleAvoider::penalty_deriv(double x)
{
  return 0.5 * (1.0 + x / std::sqrt(x * x + kSmoothEps * kSmoothEps));
}

// ============================================================================
// 计算单障碍物的安全速度 v_safe 与方向 n                          (§4)
//
// 参数：
//   n     — (输出) 机器人→障碍物的单位方向向量（车体系）
//   v_safe — (输出) 安全接近/后退速度
//
// 逻辑：
//   clearance ≥ 0（安全距离外）→ v_safe ≥ 0，限制靠近速度
//   clearance < 0（安全距离内）→ v_safe < 0，要求主动后退
//
// 动态障碍物速度仅在 |v_obs| > 0.2 m/s 时参与计算（过滤最近点滑动
// 引起的虚假速度）。
// ============================================================================
void ObstacleAvoider::compute_safe_velocity(
    const Obstacle& obs, double& v_safe, Eigen::Vector2d& n) const
{
  double d = obs.position.norm();                // 障碍物表面距离 d = ||o_j||
  if (d < 1e-9) {
    n = Eigen::Vector2d(1.0, 0.0);               // 退化情况：障碍物在原点
    v_safe = 0.0;
    return;
  }
  n = obs.position / d;                          // 单位方向向量

  double clearance = d - obs.radius - safety_distance_;   // δ = d - r - d_safe (§4)

  // 障碍物在 n 方向的速度分量。仅信任真正运动的障碍物（>0.2 m/s）
  double obs_approach = 0.0;
  if (obs.velocity.squaredNorm() > 0.2 * 0.2) {
    obs_approach = obs.velocity.dot(n);          // v_obs · n
  }

  if (clearance >= 0.0) {
    // 安全距离外：限制靠近速度  v_safe = max(0, δ/T + v_obs·n)   (§4.1)
    double geo_safe = clearance / time_horizon_;
    v_safe = geo_safe + obs_approach;
    v_safe = std::max(0.0, v_safe);
  } else {
    // 安全距离内：要求主动后退  v_safe = δ/T (< 0)               (§4.2)
    // 后退速度幅值：最大 v_max，最小 0.15 m/s
    double retreat_speed = clearance / time_horizon_;    // 负值
    retreat_speed = std::max(retreat_speed, -max_linear_vel_);   // 上限
    retreat_speed = std::min(retreat_speed, -0.15);             // 下限
    v_safe = retreat_speed + obs_approach;                       // 负值 = 要求后退
  }
}

// ============================================================================
// 障碍物有效权重  w_j = w_0 · η(d_surf)                         (§5)
//
// 双曲线严重度函数：
//   d_surf ≥ d_safe   → η = 1.0
//   d_surf <  d_safe   → η = clamp(d_safe / d_surf, 1.5, 8.0)
//
// 近表面时 η ∝ 1/d_surf（双曲线增长），上限 8.0 防止数值刚度。
// ============================================================================
double ObstacleAvoider::obstacle_effective_weight(const Obstacle& obs) const
{
  double d = obs.position.norm();
  double d_surface = std::max(0.01, d - obs.radius);   // 下限 0.01 防止除零
  double severity = 1.0;
  if (d_surface < safety_distance_) {
    severity = safety_distance_ / d_surface;            // η = d_safe / d_surf
    severity = std::max(severity, 1.5);                 // 安全区内最低 1.5x
    severity = std::min(severity, 8.0);                 // 最高 8x
  }
  return obstacle_weight_ * severity;                   // w_j = w_0 · η
}

// ============================================================================
// 目标函数  J(v) = ||v - v_hpc||² + Σ_j w_j · φ²(v·n_j - v_safe_j)   (§3.1)
// ============================================================================
double ObstacleAvoider::objective(const Eigen::Vector3d& v,
                                   const Eigen::Vector3d& v_hpc) const
{
  double J = (v - v_hpc).squaredNorm();                // 编队跟踪项

  for (const auto& obs : obstacles_) {
    double v_safe;
    Eigen::Vector2d n;
    compute_safe_velocity(obs, v_safe, n);

    double d_radial = v.head<2>().dot(n) - v_safe;     // 违反量 d_j = v·n - v_safe
    double p = penalty(d_radial);                      // φ(d_j)
    double w = obstacle_effective_weight(obs);         // w_j
    J += w * p * p;                                    // w_j · φ²(d_j)
  }

  return J;
}

// ============================================================================
// 单步投影梯度迭代                                             (§6)
//
// 计算梯度 ∇J(v) = 2(v - v_hpc) + Σ_j g_j^obs
// 其中 g_j^obs = 2 w_j φ(d_j) φ'(d_j) [n_x, n_y, 0]^T  (仅约束激活时)
//
// 若无障碍物约束激活 → 直接返回 v = Π(v_hpc)（已收敛）
// 否则执行 Armijo 回溯线搜索 (§6.3)：
//   α ← 1.0, 循环: 检查 J(v - α∇J) ≤ J(v) + c·∇J^T·Δv
//   不满足则 α = α/2，最多回溯 12 次。
// ============================================================================
bool ObstacleAvoider::gradient_step(
    Eigen::Vector3d& v, const Eigen::Vector3d& v_hpc,
    const Eigen::Vector3d& lb, const Eigen::Vector3d& ub)
{
  // 编队跟踪梯度：∇_form = 2(v - v_hpc)
  Eigen::Vector3d grad = 2.0 * (v - v_hpc);

  int n_active = 0;   // 激活的约束数

  for (const auto& obs : obstacles_) {
    double v_safe;
    Eigen::Vector2d n;
    compute_safe_velocity(obs, v_safe, n);

    double d_radial = v.head<2>().dot(n) - v_safe;     // d_j = v·n - v_safe
    if (d_radial > 0.0) {                              // 约束激活（违反安全条件）
      n_active++;
      double phi   = penalty(d_radial);                // φ(d_j)
      double phi_d = penalty_deriv(d_radial);          // φ'(d_j)
      double w_eff = obstacle_effective_weight(obs);   // w_j
      // g_j^obs = 2·w_j·φ(d_j)·φ'(d_j)·[n_x, n_y, 0]^T
      double coeff = 2.0 * w_eff * phi * phi_d;
      grad(0) += coeff * n.x();
      grad(1) += coeff * n.y();
      // grad(2) 保持 0：角速度不影响朝向障碍物的平动分量（短时域假设）
    }
  }

  // 无约束激活 → 已收敛到 v_hpc（投影后）
  if (n_active == 0) {
    v = v_hpc;
    v(0) = std::clamp(v(0), lb(0), ub(0));
    v(1) = std::clamp(v(1), lb(1), ub(1));
    v(2) = std::clamp(v(2), lb(2), ub(2));
    return true;
  }

  // Armijo 回溯线搜索（§6.3）
  double alpha = 1.0;
  double f_cur = objective(v, v_hpc);

  Eigen::Vector3d v_new;
  for (int bt = 0; bt < 12; ++bt) {
    v_new = v - alpha * grad;                          // 非投影更新
    v_new(0) = std::clamp(v_new(0), lb(0), ub(0));     // 投影到盒 B
    v_new(1) = std::clamp(v_new(1), lb(1), ub(1));
    v_new(2) = std::clamp(v_new(2), lb(2), ub(2));

    double f_new = objective(v_new, v_hpc);
    double f_armijo = f_cur + 1e-4 * grad.dot(v_new - v);   // Armijo 条件 (§6.3)

    if (f_new <= f_armijo) break;                      // 条件满足，接受步长

    alpha *= 0.5;                                      // 否则步长减半
  }

  double step_norm = (v_new - v).norm();
  v = v_new;

  return step_norm < kGradTol;                          // 收敛判定 (§6.4)
}

// ============================================================================
// 核心求解接口 — 每控制周期被 timer_cb() 调用一次 (20 Hz)      (§7)
//
// 输入：v_hpc（HPC 期望速度）、v_current（当前速度）、dt（控制周期）
// 输出：v*（最优速度指令，车体系）
//
// 管线：
//   1. 检查 scan 新鲜度，超时则清空障碍物列表
//   2. 无障碍物 → 直接返回 v_hpc
//   3. 构建加速度/速度 box 约束 B = [lb, ub]
//   4. 初始化 v₀ = Π_B(v_hpc)
//   5. 投影梯度下降（最多 K_max=20 次迭代）
//   6. 返回 v*
// ============================================================================
Eigen::Vector3d ObstacleAvoider::solve(
    const Eigen::Vector3d& v_hpc, const Eigen::Vector3d& v_current, double dt)
{
  RCLCPP_INFO(node_->get_logger(),
    "[OA] solve: v_hpc=[%.3f,%.3f,%.3f] v_cur=[%.3f,%.3f,%.3f] dt=%.3f",
    v_hpc(0), v_hpc(1), v_hpc(2),
    v_current(0), v_current(1), v_current(2), dt);

  // 检查 scan 新鲜度（超过 0.5s 清空障碍物）
  auto now = node_->now();
  double scan_age = (now - last_scan_stamp_).seconds();
  if (!scan_received_ || scan_age > kScanTimeout) {
    if (scan_received_ && scan_age > kScanTimeout) {
      RCLCPP_WARN(node_->get_logger(),
        "[OA] scan stale (age=%.2fs > %.2fs), clearing obstacles",
        scan_age, kScanTimeout);
    }
    obstacles_.clear();
  }

  // 无障碍物：直接返回 HPC 期望速度
  if (obstacles_.empty()) {
    RCLCPP_INFO(node_->get_logger(),
      "[OA] no obstacles, v_opt = v_hpc = [%.3f,%.3f,%.3f]",
      v_hpc(0), v_hpc(1), v_hpc(2));
    return v_hpc;
  }

  // 构建盒约束 B：速度上限 + 加速度上限
  //   lb_i = max(-vel_max,  v_cur_i - accel_max * dt)
  //   ub_i = min( vel_max,  v_cur_i + accel_max * dt)
  if (dt <= 1e-6) dt = 1.0 / 20.0;

  Eigen::Vector3d lb, ub;
  lb(0) = std::max(-max_linear_vel_,  v_current(0) - max_linear_accel_  * dt);
  ub(0) = std::min( max_linear_vel_,  v_current(0) + max_linear_accel_  * dt);
  lb(1) = std::max(-max_linear_vel_,  v_current(1) - max_linear_accel_  * dt);
  ub(1) = std::min( max_linear_vel_,  v_current(1) + max_linear_accel_  * dt);
  lb(2) = std::max(-max_angular_vel_, v_current(2) - max_angular_accel_ * dt);
  ub(2) = std::min( max_angular_vel_, v_current(2) + max_angular_accel_ * dt);

  // 投影梯度下降
  auto t0 = std::chrono::steady_clock::now();

  // 初始化：v₀ = Π_B(v_hpc)                                          (§6.5)
  Eigen::Vector3d v = v_hpc;
  v(0) = std::clamp(v(0), lb(0), ub(0));
  v(1) = std::clamp(v(1), lb(1), ub(1));
  v(2) = std::clamp(v(2), lb(2), ub(2));

  int iters = 0;
  bool converged = false;

  for (int i = 0; i < kMaxGradIters; ++i) {       // K_max = 20
    iters = i + 1;
    if (gradient_step(v, v_hpc, lb, ub)) {
      converged = true;
      break;
    }
  }

  auto t1 = std::chrono::steady_clock::now();
  double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  if (!converged) {
    RCLCPP_WARN(node_->get_logger(),
      "[OA] gradient descent did not converge in %d iters (%.2f ms)",
      iters, elapsed_ms);
  }

  RCLCPP_INFO(node_->get_logger(),
    "[OA] qp solved: v_opt=[%.3f,%.3f,%.3f] dt=%.2fms iters=%d converged=%d "
    "|v_opt-v_hpc|=[%.3f,%.3f,%.3f]",
    v(0), v(1), v(2), elapsed_ms, iters, converged ? 1 : 0,
    v(0)-v_hpc(0), v(1)-v_hpc(1), v(2)-v_hpc(2));

  double dv_norm = (v.head<2>() - v_hpc.head<2>()).norm();
  double dw_diff = std::abs(v(2) - v_hpc(2));
  RCLCPP_INFO(node_->get_logger(),
    "[OA] final cmd_vel: vx=%.3f vy=%.3f w=%.3f (delta from hpc: dv=%.4f dw=%.4f)",
    v(0), v(1), v(2), dv_norm, dw_diff);

  // 日志：记录哪些障碍物约束被激活
  for (size_t i = 0; i < obstacles_.size(); ++i) {
    const auto& obs = obstacles_[i];
    double v_safe;
    Eigen::Vector2d n;
    compute_safe_velocity(obs, v_safe, n);
    double v_dot_n = v.head<2>().dot(n);

    if (v_dot_n > v_safe) {
      double p = penalty(v_dot_n - v_safe);
      RCLCPP_INFO(node_->get_logger(),
        "[OA] constraint active: obs[%zu] id=%d v·n=%.3f > v_safe=%.3f penalty=%.4f",
        i, obs.id, v_dot_n, v_safe, p);
    }
  }

  return v;
}

}  // namespace formation_control
