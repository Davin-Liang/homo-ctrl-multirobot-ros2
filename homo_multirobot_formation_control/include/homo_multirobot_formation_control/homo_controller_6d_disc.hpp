#pragma once

#include <cmath>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "homo_multirobot_formation_control/types_nd.hpp"
#include "homo_multirobot_formation_control/lpc2hpc_nd.hpp"
#include "homo_multirobot_formation_control/hnorm_nd.hpp"

namespace formation_control {

/// 6D 运动学模型齐次编队控制器（离散多边形编队策略）。
///
/// 状态 x = [px, py, θ, vx_body, vy_body, ω]^T
///   - 位置/朝向在 map 系
///   - 速度在车体系
///
/// 编队点：m_p 个均匀分布在安全圆上的离散点，tol 迟滞切换。
class LpcController6DDisc {
public:
  /// @param radius  编队安全圆半径 (m)
  /// @param mass    质量调谐参数（位置通道）
  /// @param I       转动惯量调谐参数（偏航通道）
  /// @param omega_d 位置通道期望阻尼带宽
  /// @param omega_d_theta 偏航通道期望阻尼带宽
  /// @param hpc_vel_threshold  leader 速度变化触发 HPC 重算的阈值
  /// @param use_hpc 启用齐次升级
  /// @param m_p     离散编队点数量
  /// @param tol     编队点切换迟滞容差 (m)
  LpcController6DDisc(double radius = 2.0, double mass = 8.0, double I = 1.0,
                      double omega_d = 1.5, double omega_d_theta = 1.5,
                      double hpc_vel_threshold = 0.3, bool use_hpc = true,
                      int m_p = 4, double tol = 0.1)
    : radius_(radius), mass_(mass), I_(I),
      omega_d_(omega_d), omega_d_theta_(omega_d_theta),
      hpc_vel_threshold_(hpc_vel_threshold), use_hpc_(use_hpc),
      m_p_(m_p), tol_(tol)
  {
    const int n = 6, m = 3;

    A_.resize(n, n);
    build_A(0.0, 0.0, 0.0);

    B_.resize(n, m);
    B_ << 0, 0, 0,
          0, 0, 0,
          0, 0, 0,
          1.0 / mass_, 0,       0,
          0,           1.0 / mass_, 0,
          0,           0,           1.0 / I_;

    k_lin_.resize(m, n);
    k_lin_.setZero();
    P_.resize(n, n);
    P_.setIdentity();
    G0_.resize(n, n);
    G0_.setZero();
    Gd_.resize(n, n);
    Gd_.setIdentity();
    nu_ = 0.0;

    // 离散编队点
    dl_.resize(6, m_p_);
    d_.resize(6);
    d_.setZero();
    last_best_idx_ = 0;

    last_hpc_leader_vel_.setZero(3);
    initialized_ = false;
  }

  /// 一次性初始化：构建离散编队点 + 计算第一次 HPC 参数。
  void controller_initial(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2)
  {
    // 构建 m_p 个编队偏移点（leader 车体系坐标）
    for (int i = 0; i < m_p_; ++i) {
      double angle = 2.0 * M_PI * i / m_p_;
      dl_(0, i) = -radius_ * std::cos(angle);
      dl_(1, i) = -radius_ * std::sin(angle);
      dl_(2, i) = 0.0;
      dl_(3, i) = 0.0;
      dl_(4, i) = 0.0;
      dl_(5, i) = 0.0;
    }

    // 选择离当前位置最近的编队点
    select_nearest(x1, x2);

    update_A(x1);
    last_hpc_leader_vel_ << x1(3), x1(4), x1(5);

    double dtheta, cos_dt, sin_dt;
    Eigen::VectorXd e = compute_error(x1, x2, dtheta, cos_dt, sin_dt);
    last_dtheta_ = dtheta;
    k_lin_ = calculate_klin(e);

    if (use_hpc_) {
      auto res = lpc2hpc_nd(A_, B_, k_lin_);
      if (res.G0.isZero(1e-12)) {
        throw std::runtime_error("6D Disc 控制器初始化失败: lpc2hpc 返回零结果。");
      }
      G0_ = res.G0;
      P_  = res.P;
      nu_ = res.nu_min;
      Gd_ = Eigen::MatrixXd::Identity(6, 6) + nu_ * G0_;
    }
    initialized_ = true;
  }

  /// 每周期控制量计算。
  /// @return {vx_body_cmd, vy_body_cmd, omega_cmd}
  std::vector<double> lpc_calculate(const Eigen::VectorXd& x1,
                                     const Eigen::VectorXd& x2)
  {
    if (!initialized_) {
      return {0.0, 0.0, 0.0};
    }

    // 1. 更新 A 矩阵
    update_A(x1);

    // 2. 检查并切换编队点
    check_and_switch_target(x1, x2);

    // 3. 计算误差（follower 速度已旋转到 leader 车体系）
    double dtheta, cos_dt, sin_dt;
    Eigen::VectorXd e = compute_error(x1, x2, dtheta, cos_dt, sin_dt);

    // 4. 自适应线性增益
    k_lin_ = calculate_klin(e);

    // 5. HPC 参数时变重算（仅 HPC 模式）
    Eigen::VectorXd u_L;
    if (use_hpc_) {
      Eigen::Vector3d leader_vel(x1(3), x1(4), x1(5));
      bool vel_changed = (leader_vel - last_hpc_leader_vel_).norm() > hpc_vel_threshold_;
      bool yaw_changed = std::abs(dtheta - last_dtheta_) > 0.3;
      if (vel_changed || yaw_changed) {
        auto res = lpc2hpc_nd(A_, B_, k_lin_);
        if (!res.G0.isZero(1e-12)) {
          G0_ = res.G0;
          P_  = res.P;
          nu_ = res.nu_min;
          Gd_ = Eigen::MatrixXd::Identity(6, 6) + nu_ * G0_;
          last_hpc_leader_vel_ = leader_vel;
          last_dtheta_ = dtheta;
        }
      }

      double nx = hnorm_nd(e, Gd_, P_);
      double c = std::clamp(nx, 0.5, 1.0);
      double log_c = std::log(c);
      Eigen::MatrixXd expm_g = (Gd_ * (1.0 - log_c)).exp();
      Eigen::VectorXd warped_e = expm_g * e;
      u_L = std::pow(c, 1.0 + nu_) * (k_lin_ * warped_e);
    } else {
      u_L = k_lin_ * e;
    }

    // 6. 控制力从 leader 车体系旋转到 follower 车体系
    double ux_f =  u_L(0) * cos_dt + u_L(1) * sin_dt;
    double uy_f = -u_L(0) * sin_dt + u_L(1) * cos_dt;

    // 7. 前向欧拉（follower 车体系）
    double h = 0.1;
    double goal_vx = x2(3) + h * ux_f / mass_;
    double goal_vy = x2(4) + h * uy_f / mass_;
    double goal_omega = x2(5) + h * u_L(2) / I_;

    return {goal_vx, goal_vy, goal_omega};
  }

  /// 跟随者到安全边界的距离（> 0 在圆外, <=0 在圆内）。
  double distance_to_boundary(const Eigen::VectorXd& x1,
                               const Eigen::VectorXd& x2)
  {
    double dx = x2(0) - x1(0);
    double dy = x2(1) - x1(1);
    return std::sqrt(dx * dx + dy * dy) - radius_;
  }

private:
  // ---- 构建 A 矩阵（含 leader 速度耦合） --------------------------------------
  void build_A(double vx_l, double vy_l, double omega_l)
  {
    A_ << 0,       omega_l,  -vy_l,  1, 0, 0,
          -omega_l, 0,        vx_l,  0, 1, 0,
          0,        0,        0,     0, 0, 1,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0;
  }

  void update_A(const Eigen::VectorXd& x1)
  {
    build_A(x1(3), x1(4), x1(5));
  }

  // ---- 选择最近的编队点 --------------------------------------------------------
  void select_nearest(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2)
  {
    // 相对位置（leader 车体系）
    double dpx = x2(0) - x1(0);
    double dpy = x2(1) - x1(1);
    double cos_tl = std::cos(x1(2));
    double sin_tl = std::sin(x1(2));
    double dex =  dpx * cos_tl + dpy * sin_tl;
    double dey = -dpx * sin_tl + dpy * cos_tl;

    // 相对速度（leader 车体系）
    double dtheta = x2(2) - x1(2);
    double cos_dt = std::cos(dtheta);
    double sin_dt = std::sin(dtheta);
    double vx_f_in_L = x2(3) * cos_dt - x2(4) * sin_dt;
    double vy_f_in_L = x2(3) * sin_dt + x2(4) * cos_dt;

    Eigen::VectorXd rel(6);
    rel << dex, dey, dtheta, vx_f_in_L - x1(3), vy_f_in_L - x1(4), x2(5) - x1(5);

    int best = 0;
    double best_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < m_p_; ++i) {
      double dist = (rel - dl_.col(i)).norm();
      if (dist < best_dist) {
        best_dist = dist;
        best = i;
      }
    }
    d_ = dl_.col(best);
    last_best_idx_ = best;
  }

  // ---- 检查并切换编队点（迟滞）------------------------------------------------
  void check_and_switch_target(const Eigen::VectorXd& x1,
                                const Eigen::VectorXd& x2)
  {
    // 相对位置（leader 车体系）
    double dpx = x2(0) - x1(0);
    double dpy = x2(1) - x1(1);
    double cos_tl = std::cos(x1(2));
    double sin_tl = std::sin(x1(2));
    double dex =  dpx * cos_tl + dpy * sin_tl;
    double dey = -dpx * sin_tl + dpy * cos_tl;

    double dtheta = x2(2) - x1(2);
    double cos_dt = std::cos(dtheta);
    double sin_dt = std::sin(dtheta);
    double vx_f_in_L = x2(3) * cos_dt - x2(4) * sin_dt;
    double vy_f_in_L = x2(3) * sin_dt + x2(4) * cos_dt;

    Eigen::VectorXd rel(6);
    rel << dex, dey, dtheta, vx_f_in_L - x1(3), vy_f_in_L - x1(4), x2(5) - x1(5);

    // 找最近的编队点
    int best = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < m_p_; ++i) {
      double dist = (rel - dl_.col(i)).norm();
      if (dist < min_dist) {
        min_dist = dist;
        best = i;
      }
    }

    // 迟滞切换：新点距离 + tol 仍小于当前点距离才切换
    if (best != last_best_idx_) {
      double current_dist = (rel - d_).norm();
      if (min_dist + tol_ < current_dist) {
        d_ = dl_.col(best);
        last_best_idx_ = best;

        // 切换时重算 HPC 参数
        if (use_hpc_) {
          auto res = lpc2hpc_nd(A_, B_, k_lin_);
          if (!res.G0.isZero(1e-12)) {
            G0_ = res.G0;
            P_  = res.P;
            nu_ = res.nu_min;
            Gd_ = Eigen::MatrixXd::Identity(6, 6) + nu_ * G0_;
          }
        }
      }
    }
  }

  // ---- 误差计算（leader 车体系 + 离散编队点）-----------------------------------
  Eigen::VectorXd compute_error(const Eigen::VectorXd& x1,
                                 const Eigen::VectorXd& x2,
                                 double& dtheta, double& cos_dt, double& sin_dt)
  {
    dtheta = x2(2) - x1(2);
    cos_dt = std::cos(dtheta);
    sin_dt = std::sin(dtheta);

    // 位置误差（map 系 → leader 车体系）
    double dpx = x2(0) - x1(0);
    double dpy = x2(1) - x1(1);
    double cos_tl = std::cos(x1(2));
    double sin_tl = std::sin(x1(2));

    double dex =  dpx * cos_tl + dpy * sin_tl;
    double dey = -dpx * sin_tl + dpy * cos_tl;

    // follower 速度旋转到 leader 车体系
    double vx_f_in_L = x2(3) * cos_dt - x2(4) * sin_dt;
    double vy_f_in_L = x2(3) * sin_dt + x2(4) * cos_dt;

    // 误差 = 当前相对状态 - 目标编队偏移
    Eigen::VectorXd e(6);
    e << dex - d_(0),              // 位置 X 误差（leader 车体系）
         dey - d_(1),              // 位置 Y 误差（leader 车体系）
         dtheta - d_(2),           // 偏航误差
         vx_f_in_L - x1(3) - d_(3),// 车体系 vx 误差
         vy_f_in_L - x1(4) - d_(4),// 车体系 vy 误差
         x2(5) - x1(5) - d_(5);    // ω 误差
    return e;
  }

  // ---- 自适应线性增益 (6D 分块解耦) -------------------------------------------
  Eigen::MatrixXd calculate_klin(const Eigen::VectorXd& e)
  {
    const int m = 3;

    auto compute_channel = [this](double e_p, double e_v, double M, double wd) {
      double val = (std::abs(e_p) > 1e-6) ? -M * e_v / e_p : 0.0;
      double max_ratio = wd * M;
      val = std::clamp(val, -max_ratio, max_ratio);
      double a = std::max(val, wd * M);
      double k2 = -2.0 * a;
      double k1 = a * (k2 + a) / M;
      return std::make_pair(k1, k2);
    };

    auto [k1_x, k2_x] = compute_channel(e(0), e(3), mass_, omega_d_);
    auto [k1_y, k2_y] = compute_channel(e(1), e(4), mass_, omega_d_);
    auto [k1_t, k2_t] = compute_channel(e(2), e(5), I_, omega_d_theta_);

    Eigen::MatrixXd K(m, 6);
    K << k1_x, 0,     0,     k2_x, 0,     0,
         0,     k1_y, 0,     0,     k2_y, 0,
         0,     0,     k1_t, 0,     0,     k2_t;
    return K;
  }

  // ---- 参数 ------------------------------------------------------------------
  double radius_, mass_, I_;
  double omega_d_, omega_d_theta_;
  double hpc_vel_threshold_;
  int    m_p_;
  double tol_;

  // ---- 系统模型 --------------------------------------------------------------
  Eigen::MatrixXd A_, B_;

  // ---- 控制器状态 ------------------------------------------------------------
  Eigen::MatrixXd k_lin_, P_, G0_, Gd_;
  Eigen::MatrixXd dl_;       // 6 × m_p 离散编队点矩阵
  Eigen::VectorXd d_;        // 当前目标编队偏移
  int    last_best_idx_;
  double nu_;
  bool   use_hpc_;
  Eigen::Vector3d last_hpc_leader_vel_;
  double last_dtheta_ = 0.0;
  bool initialized_;
};

}  // namespace formation_control
