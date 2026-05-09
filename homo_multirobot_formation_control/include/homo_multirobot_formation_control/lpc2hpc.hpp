#pragma once

/// @file LPC → HPC 升级算法（移植自 lpc2hpc.py）
///
/// 包含：
///   1. solve_lyapunov_4x4 — 4×4 Lyapunov 方程求解器
///   2. trans_con          — SVD 正交分解到块形式
///   3. block_con          — 块可控标准型
///   4. lpc2hpc            — 线性→齐次控制器参数升级

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>
#include <unsupported/Eigen/KroneckerProduct>
#include "homo_multirobot_formation_control/types.hpp"

namespace formation_control {

// ============================================================================
// 4×4 Lyapunov 方程求解器。
//
// 求解  A^T * P + P * A = -Q，其中 P 和 Q 为对称矩阵。
//
// 通过 Kronecker 积将方程转为 16×16 的线性系统：
//   (I ⊗ A^T + A^T ⊗ I) · vec(P) = −vec(Q)
//
// Eigen 默认列主序，vec 操作和 Map 重塑直接对应。
// ============================================================================
inline Mat4d solve_lyapunov_4x4(const Mat4d& A, const Mat4d& Q)
{
  using Mat16d = Eigen::Matrix<double, 16, 16>;
  using Vec16d = Eigen::Matrix<double, 16, 1>;

  Mat4d AT = A.transpose();
  Mat4d I4 = Mat4d::Identity();

  Mat16d K = Eigen::kroneckerProduct(I4, AT) + Eigen::kroneckerProduct(AT, I4);

  Vec16d b;
  Eigen::Map<Vec16d> q_map(const_cast<double*>(Q.data()));
  b = -q_map;

  Vec16d vec_P = K.colPivHouseholderQr().solve(b);

  Mat4d P = Eigen::Map<Mat4d>(vec_P.data());
  return P;
}

// ============================================================================
// trans_con — 基于 SVD 正交分解将可控系统 (A,B) 转换为块形式。
//
// 每轮迭代通过 SVD 分离 Bk 的列空间和正交补，用正交补相似变换收缩系统维度，
// 并累积整体变换矩阵 T。
//
// 返回 {T, nt}：
//   T  — 总体变换矩阵 (4×4)
//   nt — 块尺寸序列（4 阶 2D 积分器时为 [2, 2]）
//
// 工作矩阵使用动态尺寸 MatrixXd，因为每轮迭代后 Ak、Bk 的有效维度会缩小。
// 如果用固定尺寸 Mat4d，Ak.rows() 始终返回 4，循环终止条件永远为假。
// ============================================================================
inline std::pair<Mat4d, std::vector<int>> trans_con(const Mat4d& A, const Mat42d& B)
{
  const int n = 4;

  // Kalman 可控性秩检验
  Eigen::Matrix<double, 4, 8> U_ctrl;
  U_ctrl << B, A * B, A * A * B, A * A * A * B;
  Eigen::JacobiSVD<Eigen::Matrix<double, 4, 8>> svd_ctrl(U_ctrl);
  svd_ctrl.setThreshold(1e-10);
  if (static_cast<int>(svd_ctrl.rank()) < n) {
    std::cerr << "错误: 系统 {A,B} 不可控。" << std::endl;
    return {Mat4d::Identity(), {}};
  }

  Mat4d T = Mat4d::Identity();
  Eigen::MatrixXd Ak = A;
  Eigen::MatrixXd Bk = B;
  std::vector<int> nt;
  int l = 0;

  while (true) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Bk, Eigen::ComputeFullU);
    svd.setThreshold(1e-10);
    int rank_Bk = static_cast<int>(svd.rank());

    if (rank_Bk >= static_cast<int>(Ak.rows())) {
      nt.insert(nt.begin(), rank_Bk);
      break;
    }

    nt.insert(nt.begin(), rank_Bk);

    int rows_Ak = static_cast<int>(Ak.rows());
    auto U_full = svd.matrixU();

    // 将 U 划分为列空间 B_p_cols 和正交补 B_ort
    Eigen::MatrixXd B_p_cols = U_full.leftCols(rank_Bk);
    Eigen::MatrixXd B_ort   = U_full.rightCols(rows_Ak - rank_Bk).transpose();

    // T_block 垂直拼接: [B_ort; B_p_cols^T]
    Eigen::MatrixXd T_block(rows_Ak, rows_Ak);
    T_block.topRows(rows_Ak - rank_Bk) = B_ort;
    T_block.bottomRows(rank_Bk) = B_p_cols.transpose();

    if (rows_Ak < n) {
      Eigen::MatrixXd T_temp = Eigen::MatrixXd::Identity(n, n);
      T_temp.topLeftCorner(rows_Ak, rows_Ak) = T_block;
      T = Mat4d(T_temp * T);
    } else {
      T = T_block;
    }

    l += rank_Bk;

    // 通过正交补收缩 Ak 和 Bk
    Eigen::MatrixXd Ak_old = Ak;
    Ak = B_ort * Ak_old * B_ort.transpose();
    Bk = B_ort * Ak_old * B_p_cols;
  }

  return {T, nt};
}

// ============================================================================
// block_con — 块可控标准型。
// 在 trans_con 分解的基础上对非对角块做三角化。
// ============================================================================
inline std::pair<Mat4d, std::vector<int>> block_con(const Mat4d& A, const Mat42d& B)
{
  auto [T1, nt] = trans_con(A, B);
  if (nt.empty()) {
    return {Mat4d::Identity(), {}};
  }

  int k = static_cast<int>(nt.size());
  int n = 4;

  std::vector<int> n_ind = {0};
  for (int s : nt) n_ind.push_back(n_ind.back() + s);

  Mat4d A1  = T1 * A * T1.inverse();
  Mat4d Phi = Mat4d::Identity();

  for (int i = 0; i < k - 1; ++i) {
    Mat4d A_prime = Phi * A1 * Phi.inverse();

    int r0 = n_ind[i],     r1 = n_ind[i + 1];
    int c0 = n_ind[i + 1], c1 = n_ind[i + 2];

    auto A_pivot = A_prime.block(r0, c0, r1 - r0, c1 - c0);
    auto A_lower = A_prime.block(r1, r0, c1 - r1, r1 - r0);
    Eigen::MatrixXd L = A_lower * A_pivot.completeOrthogonalDecomposition().pseudoInverse();

    Mat4d T_step = Mat4d::Identity();
    T_step.block(r1, r0, c1 - r1, r1 - r0) = -L;
    Phi = T_step * Phi;
  }

  Mat4d T_final = Phi * T1;
  return {T_final, nt};
}

// ============================================================================
// lpc2hpc — 从线性比例控制器 (LPC) 计算齐次比例控制器 (HPC) 参数。
//
// 通过块可控分解 + 齐次化权重 + Lyapunov 方程求解，将线性增益 K 升级为
// 齐次控制参数。返回的 HpcResult 包含：
//   K0      — HPC 基础增益
//   G0      — 齐次生成元（编码各块的齐次度权重）
//   P       — Lyapunov 矩阵（Acl^T·P + P·Acl = −2I 的解）
//   nu_min, nu_max — 齐次度 ν 的可容许范围
//
// 运行时膨胀生成元为  Gd = I + nu * G0（通常取 nu = nu_min）。
//
// 完全等价于 lpc2hpc.py。
// ============================================================================
inline HpcResult lpc2hpc(const Mat4d& A, const Mat42d& B, const Mat24d& K)
{
  HpcResult res;
  int n = 4;

  // 验证 A+BK 是 Hurwitz 稳定的
  Eigen::EigenSolver<Mat4d> es(A + B * K);
  double max_real = -std::numeric_limits<double>::max();
  for (int i = 0; i < n; ++i) {
    max_real = std::max(max_real, std::real(es.eigenvalues()(i)));
  }
  if (-max_real * 0.001 < 1e-5) {
    std::cerr << "错误: 闭环系统稳定性裕度不足。" << std::endl;
    res.K0.setZero();
    return res;
  }

  auto [T, nt] = block_con(A, B);
  if (nt.empty()) {
    std::cerr << "错误: 块分解失败。" << std::endl;
    res.K0.setZero();
    return res;
  }

  Mat4d Anew = T * A * T.inverse();
  int k = static_cast<int>(nt.size());
  std::vector<int> n_ind = {0};
  for (int s : nt) n_ind.push_back(n_ind.back() + s);

  Mat42d Bnew = T * B;

  // 取最后 nt[k-1] 行——"被驱动"子系统
  int b0_rows = n - n_ind[k - 1];
  auto B0 = Bnew.bottomRows(b0_rows);
  auto A0 = Anew.bottomRows(b0_rows);

  // K0_new = −pinv(B0) · A0，再变换回去: K0 = K0_new · T
  Mat24d K0_new = Mat24d::Zero();
  Eigen::MatrixXd pinv_B0 = B0.completeOrthogonalDecomposition().pseudoInverse();
  K0_new.topRows(b0_rows) = -pinv_B0 * A0.topRows(b0_rows);
  res.K0 = K0_new * T;

  // vG0 编码各块的齐次度权重：k-1, k-2, ..., 0
  std::vector<double> vG0;
  for (int i = 0; i < k; ++i) {
    for (int j = 0; j < nt[i]; ++j) {
      vG0.push_back(static_cast<double>(k - 1 - i));
    }
  }
  res.G0 = -T.inverse() * Eigen::DiagonalMatrix<double, 4>(
      vG0[0], vG0[1], vG0[2], vG0[3]) * T;

  // Lyapunov: (A+BK)^T · P + P · (A+BK) = −2I
  Mat4d Acl = A + B * K;
  res.P = solve_lyapunov_4x4(Acl, 2.0 * Mat4d::Identity());

  // 从 M = sqrt(P)·G0·inv(sqrt(P)) + (sqrt(P)·G0·inv(sqrt(P)))^T 的特征值
  // 计算齐次度 ν 的可容许范围
  try {
    Mat4d sqrt_P     = res.P.sqrt();
    Mat4d inv_sqrt_P = sqrt_P.inverse();
    Mat4d M = sqrt_P * res.G0 * inv_sqrt_P;
    // 必须用临时变量避免 Eigen aliasing 错误
    Mat4d M_sym = M + M.transpose();
    M = M_sym;

    Eigen::EigenSolver<Mat4d> es_m(M);
    double lambda_min = 0.0, lambda_max = 0.0;
    for (int i = 0; i < 4; ++i) {
      double eig_real = std::real(es_m.eigenvalues()(i));
      if (i == 0) {
        lambda_min = lambda_max = eig_real;
      } else {
        lambda_min = std::min(lambda_min, eig_real);
        lambda_max = std::max(lambda_max, eig_real);
      }
    }

    res.nu_min = -1.0;
    if (lambda_max > 1e-6) {
      res.nu_min = std::max(-1.0, -1.0 / lambda_max + 1e-6);
    }

    res.nu_max = 1.0 / k;
    if (lambda_min < -1e-6) {
      res.nu_max = std::min(1.0 / k, -1.0 / lambda_min);
    }
  } catch (...) {
    std::cerr << "警告: 矩阵 P 奇异或接近奇异，无法计算 mu。" << std::endl;
    res.nu_min = -1.0;
    res.nu_max = 1.0 / k;
  }

  return res;
}

}  // namespace formation_control
