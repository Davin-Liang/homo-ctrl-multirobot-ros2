#pragma once

/// @file 6D 电机感知模型齐次编队控制器。
///
/// 状态 x = [px, py, vx_cmd, vy_cmd, vx_real, vy_real]^T（全部 map 系）
///   - px, py:            位置（TF + EKF）
///   - vx_cmd, vy_cmd:    指令速度——控制器内部积分状态，不可测量。
///                        初始化对齐 EKF，之后由 sync_cmd_vel() 用实际发布的
///                        cmd_vel 回写（抗饱和）
///   - vx_real, vy_real:  电机实际速度（EKF 测量）
///
/// 系统方程（执行器一阶滞后显式增广）:
///   dp/dt      = v_real                       位置由实际速度积分
///   dv_cmd/dt  = u / mass                     控制力作用于指令（保留 HPC 语义）
///   dv_real/dt = (v_cmd - v_real) / tau       一阶 LP 模拟电机响应
///
/// 与 4D 的关系: tau→0+ 时 v_real 瞬时跟上 v_cmd，退化为 4D 双积分器
/// （数值上 1/tau 发散不可行，需要 4D 行为请直接用 formation_control_node）。
///
/// 编队点逻辑与 4D LpcController 相同（离散多边形 + tol 滞后切换），
/// A 为常值 → HPC 仅在初始化和编队点切换时重算。
///
/// 齐次性说明: A 含特征值 -1/tau（v_real 自阻尼），非幂零，闭环齐次性近似
/// 成立——与 LpcController6D（时变 A 含 ω 耦合）的近似同性质，耗散项偏安全侧。

#include <cmath>
#include <tuple>
#include <vector>
#include <limits>
#include <algorithm>
#include <stdexcept>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "homo_multirobot_formation_control/types_nd.hpp"
#include "homo_multirobot_formation_control/lpc2hpc_nd.hpp"
#include "homo_multirobot_formation_control/hnorm_nd.hpp"

namespace formation_control {

class LpcController6DMotor {
public:
  /// @param m_p     安全编队点数量
  /// @param radius  编队圆半径 (m)
  /// @param tol     编队点切换容差 (m)
  /// @param mass    控制力→加速度增益（调参用，非物理质量）
  /// @param tau     电机一阶时间常数 (s)，实测约 0.43，不建议 < 0.1
  /// @param omega_d 期望阻尼带宽
  /// @param control_period 控制周期 (s) = 1/control_rate。
  ///   注意: 与 4D 不同（4D 的 h 只是输出整形系数，v 每周期重新测量），
  ///   本控制器的 v_cmd 是跨周期积分状态，积分步长必须等于真实控制周期，
  ///   否则等效 B 矩阵被缩放、极点配置失真（h=0.1@20Hz 曾导致欠阻尼慢震荡）。
  /// @param hpc_c_min HPC warp 的 c 下界 (hnorm clamp 下限)。
  ///   6D Motor 的齐次链 (权重 [2,1,0]) 比 4D ([1,0]) 深——c_min=0.5 时
  ///   expm(Gd·1.69) 对位置权重方向放大约 30×（vs 4D 的 5×），与慢执行器
  ///   (accel≤0.25) 和 EKF/激光噪声组合产生弛豫振荡。0.7 离线够、实物不够；
  ///   经扫参实测稳定值约 0.9（~1.17× 放大），将此设为默认。
  ///   纯 LPC 模式 (use_hpc=false) 不受此参数影响。
  LpcController6DMotor(int m_p = 4, double radius = 2.0, double tol = 0.1,
                       double mass = 2.0, double tau = 0.43,
                       double omega_d = 0.7, bool use_hpc = true,
                       double control_period = 0.05, double hpc_c_min = 0.9)
    : m_p_(m_p), radius_(radius), tol_(tol), mass_(mass), tau_(tau),
      omega_d_(omega_d), h_(control_period), use_hpc_(use_hpc),
      hpc_c_min_(hpc_c_min)
  {
    if (tau_ < 0.1) {
      // k2 = m(1/tau - 3λ) 随 tau→0 发散，禁止极小 tau
      throw std::invalid_argument("LpcController6DMotor: tau 不得小于 0.1 s");
    }

    const int n = 6, m = 2;

    // 电机感知模型（常值 A/B，见文件头部方程）
    A_.resize(n, n);
    A_ << 0, 0,  0,        0,        1,         0,
          0, 0,  0,        0,        0,         1,
          0, 0,  0,        0,        0,         0,
          0, 0,  0,        0,        0,         0,
          0, 0,  1.0/tau_, 0,       -1.0/tau_,  0,
          0, 0,  0,        1.0/tau_, 0,        -1.0/tau_;

    B_.resize(n, m);
    B_ << 0, 0,
          0, 0,
          1.0 / mass_, 0,
          0, 1.0 / mass_,
          0, 0,
          0, 0;

    d_.resize(n);
    d_.setZero();
    k_lin_.resize(m, n);
    k_lin_.setZero();
    P_.resize(n, n);
    P_.setIdentity();
    G0_.resize(n, n);
    G0_.setZero();
    Gd_.resize(n, n);
    Gd_.setIdentity();
    nu_ = 0.0;
  }

  // --------------------------------------------------------------------------
  // 一次性初始化: 创建编队点集 → 选最近编队点 → 计算 k_lin → 升级到 HPC。
  // 必须在 lpc_calculate 之前调用。x2 的 v_cmd 分量此时应等于 EKF 速度。
  // --------------------------------------------------------------------------
  void controller_initial(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2)
  {
    // 领航者周围半径 radius_ 的圆上均匀分布 m_p_ 个编队偏移向量（速度分量全零）
    dl_.resize(6, m_p_);
    dl_.setZero();
    for (int i = 0; i < m_p_; ++i) {
      double angle = 2.0 * M_PI * i / m_p_;
      dl_(0, i) = -radius_ * std::cos(angle);
      dl_(1, i) = -radius_ * std::sin(angle);
    }

    // 选择距离当前跟随者位置最近的编队点
    int best_idx = 0;
    double best_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < m_p_; ++i) {
      double dist = (x2 - x1 - dl_.col(i)).norm();
      if (dist < best_dist) {
        best_dist = dist;
        best_idx = i;
      }
    }
    d_ = dl_.col(best_idx);

    Eigen::VectorXd e = x2 - x1 - d_;
    k_lin_ = calculate_klin(e);

    if (use_hpc_) {
      auto res = lpc2hpc_nd(A_, B_, k_lin_);
      if (res.G0.isZero(1e-12)) {
        throw std::runtime_error("6D Motor 控制器初始化失败: lpc2hpc 返回零结果。");
      }
      G0_ = res.G0;
      P_  = res.P;
      nu_ = res.nu_min;
      Gd_ = Eigen::MatrixXd::Identity(6, 6) + nu_ * G0_;
    }
  }

  // --------------------------------------------------------------------------
  // 每周期控制量计算（20 Hz）。
  //
  // 返回 map 系 {goal_vx_cmd, goal_vy_cmd} (m/s)。偏航控制由调用方负责。
  //
  // 控制律（同 4D，前向欧拉改为基于 v_cmd 内部状态而非测量速度）:
  //   e  = x2 − x1 − d
  //   c  = clamp(hnorm(e, Gd, P), 0.5, 1)
  //   u  = c^(1+nu) · K · expm(Gd·(1−ln c)) · e
  //   goal_v_cmd = v_cmd + h · u / mass
  // --------------------------------------------------------------------------
  std::vector<double> lpc_calculate(const Eigen::VectorXd& x1,
                                    const Eigen::VectorXd& x2)
  {
    check_and_switch_target(x1, x2);

    Eigen::VectorXd e = x2 - x1 - d_;

    Eigen::Vector2d u2;
    if (use_hpc_) {
      double nx = hnorm_nd(e, Gd_, P_);
      double c = std::clamp(nx, hpc_c_min_, 1.0);
      double log_c = std::log(c);
      Eigen::MatrixXd expm_g = (Gd_ * (1.0 - log_c)).exp();
      Eigen::VectorXd warped_e = expm_g * e;
      u2 = std::pow(c, 1.0 + nu_) * (k_lin_ * warped_e);
    } else {
      u2 = k_lin_ * e;  // 纯线性比例控制
    }

    // 前向欧拉：指令速度自演化（不读测量速度——这是与 4D 的本质差异）。
    // 步长必须为真实控制周期（v_cmd 是跨周期积分状态）。
    double goal_vx_cmd = x2(2) + h_ * u2(0) / mass_;
    double goal_vy_cmd = x2(3) + h_ * u2(1) / mass_;

    return {goal_vx_cmd, goal_vy_cmd};
  }

  // 跟随者到最近编队点的距离（调试/度量用）
  double calculate_distance(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2)
  {
    double min_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < m_p_; ++i) {
      double dist = (x2 - x1 - dl_.col(i)).norm();
      min_dist = std::min(min_dist, dist);
    }
    return min_dist;
  }

private:
  // --------------------------------------------------------------------------
  // 编队点切换（带 tol_ 滞后避免频繁跳动）。切换后重算 k_lin + HPC（A 常值，
  // 这是唯一需要重算 HPC 的时机）。
  // --------------------------------------------------------------------------
  void check_and_switch_target(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2)
  {
    double min_dist = std::numeric_limits<double>::max();
    int best_idx = 0;
    for (int i = 0; i < m_p_; ++i) {
      double dist = (x2 - x1 - dl_.col(i)).norm();
      if (dist < min_dist) {
        min_dist = dist;
        best_idx = i;
      }
    }

    double current_dist = (x2 - x1 - d_).norm();
    if (min_dist + tol_ < current_dist) {
      // 诊断: 切换会使目标瞬间跳变 ~2.8m (m_p=4)，输出随之突增
      std::cout << "[LpcController6DMotor] 编队点切换 -> idx " << best_idx
                << " (err " << current_dist << " -> " << min_dist << " m)" << std::endl;
      d_ = dl_.col(best_idx);

      Eigen::VectorXd e = x2 - x1 - d_;
      k_lin_ = calculate_klin(e);

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

  // --------------------------------------------------------------------------
  // 单轴三阶极点配置（p → v_real → v_cmd 链 + 电机滞后）。
  //
  // 每轴闭环: dp/dt = vr, dvc/dt = (k1·p + k2·vc + k3·vr)/M, dvr/dt = (vc−vr)/τ
  // 特征多项式 s³ + (1/τ − k2/M)s² − (k2+k3)/(Mτ)·s − k1/(Mτ) 对 (s+λ)³ 配置。
  //
  // 注意: 4D 的 a 不是极点（4D 闭环极点 = a/M），须换算 λ = a/M（λ ≥ wd）。
  // a 的自适应逻辑沿用 4D compute_channel（e_v 取 v_real 误差分量）。
  //
  // 独立成静态函数以便后续 8D（6d_disc + 电机模型）x/y 通道复用。
  // --------------------------------------------------------------------------
  static std::tuple<double, double, double> compute_channel_3rd(
      double e_p, double e_v, double M, double tau, double wd)
  {
    // 防超调比值（同 4D）: a = −M · e_v / e_p，clamp 防止位置误差极小时增益爆炸
    double val = (std::abs(e_p) > 1e-6) ? -M * e_v / e_p : 0.0;
    double max_ratio = wd * M;
    val = std::clamp(val, -max_ratio, max_ratio);
    double a = std::max(val, wd * M);

    // 三重极点 (s+λ)³, λ = a/M ≥ wd
    double lambda = a / M;
    double k1 = -lambda * lambda * lambda * M * tau;
    double k2 = M * (1.0 / tau - 3.0 * lambda);
    double k3 = -3.0 * lambda * lambda * M * tau - k2;
    return {k1, k2, k3};
  }

  // 自适应线性增益: K 为 2×6，x/y 两轴解耦，每轴三阶极点配置。
  Eigen::MatrixXd calculate_klin(const Eigen::VectorXd& e)
  {
    auto [k1_x, k2_x, k3_x] = compute_channel_3rd(e(0), e(4), mass_, tau_, omega_d_);
    auto [k1_y, k2_y, k3_y] = compute_channel_3rd(e(1), e(5), mass_, tau_, omega_d_);

    Eigen::MatrixXd K(2, 6);
    K << k1_x, 0,    k2_x, 0,    k3_x, 0,
         0,    k1_y, 0,    k2_y, 0,    k3_y;
    return K;
  }

  // ---- 参数 ----------------------------------------------------------------
  int    m_p_;       // 编队点数量
  double radius_;    // 编队圆半径 (m)
  double tol_;       // 切换滞后容差 (m)
  double mass_;      // 模型质量（调参）
  double tau_;       // 电机一阶时间常数 (s)
  double omega_d_;   // 期望阻尼带宽
  double h_;         // 控制周期 (s)，v_cmd 积分步长

  // ---- 系统模型（双积分器 + 执行器一阶滞后） ---------------------------------
  Eigen::MatrixXd A_;   // 6×6 常值
  Eigen::MatrixXd B_;   // 6×2

  // ---- 控制器状态 -----------------------------------------------------------
  Eigen::Matrix<double, 6, Eigen::Dynamic> dl_;  // 编队偏移向量集 (6 × m_p)
  Eigen::VectorXd d_;        // 当前目标偏移向量
  Eigen::MatrixXd k_lin_;    // 线性反馈增益 (2×6)
  Eigen::MatrixXd P_;        // Lyapunov 矩阵
  Eigen::MatrixXd G0_;       // 齐次生成元
  Eigen::MatrixXd Gd_;       // 膨胀生成元 (I + nu * G0)
  double nu_;                // 齐次度
  bool   use_hpc_;           // false 时退化为纯 LPC
  double hpc_c_min_ = 0.9;    // hnorm clamp 下界（6D 三阶链比 4D 深，默认 0.9 替代 0.5）
};

}  // namespace formation_control
