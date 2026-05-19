#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>
#include <unsupported/Eigen/KroneckerProduct>
#include "homo_multirobot_formation_control/types_nd.hpp"

namespace formation_control {

// ============================================================================
// N×N Lyapunov 方程求解器（泛化版）。
//
// 求解  A^T·P + P·A = -Q，P 与 Q 为对称矩阵。
//
// Kronecker 积: (I ⊗ A^T + A^T ⊗ I)·vec(P) = -vec(Q)
// 线性系统规模: n² × n²（4D → 16×16, 6D → 36×36）
// ============================================================================
inline Eigen::MatrixXd solve_lyapunov_nd(const Eigen::MatrixXd& A,
                                          const Eigen::MatrixXd& Q)
{
  int n = static_cast<int>(A.rows());
  Eigen::MatrixXd AT = A.transpose();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);

  Eigen::MatrixXd K = Eigen::kroneckerProduct(I, AT).eval()
                    + Eigen::kroneckerProduct(AT, I).eval();

  int n2 = n * n;
  Eigen::VectorXd b(n2);
  // Q 按列主序 (Eigen default) 展平为 vec(Q)
  for (int col = 0; col < n; ++col)
    for (int row = 0; row < n; ++row)
      b(col * n + row) = -Q(row, col);

  Eigen::VectorXd vec_P = K.colPivHouseholderQr().solve(b);

  Eigen::MatrixXd P(n, n);
  for (int col = 0; col < n; ++col)
    for (int row = 0; row < n; ++row)
      P(row, col) = vec_P(col * n + row);

  return P;
}

// ============================================================================
// trans_con — 基于 SVD 正交分解将可控系统 (A,B) 转换为块形式（N-D 泛化版）。
//
// 算法与原版 4D trans_con 相同，使用动态尺寸 MatrixXd。
//
// 返回 {T, nt}:
//   T  — 总体变换矩阵 (n×n)
//   nt — 块尺寸序列（6D 三通道双积分器: [3, 3]）
// ============================================================================
inline std::pair<Eigen::MatrixXd, std::vector<int>>
trans_con_nd(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
{
  int n = static_cast<int>(A.rows());
  int m = static_cast<int>(B.cols());

  // Kalman 可控性检验
  Eigen::MatrixXd U_ctrl(n, n * m);
  Eigen::MatrixXd Ak_pow = Eigen::MatrixXd::Identity(n, n);
  for (int k = 0; k < n; ++k) {
    U_ctrl.middleCols(k * m, m) = Ak_pow * B;
    Ak_pow = A * Ak_pow;
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_ctrl(U_ctrl);
  svd_ctrl.setThreshold(1e-10);
  if (static_cast<int>(svd_ctrl.rank()) < n) {
    std::cerr << "错误: 系统 {A,B} 不可控。" << std::endl;
    return {Eigen::MatrixXd::Identity(n, n), {}};
  }

  Eigen::MatrixXd T = Eigen::MatrixXd::Identity(n, n);
  Eigen::MatrixXd Ak = A;
  Eigen::MatrixXd Bk = B;
  std::vector<int> nt;

  while (true) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Bk, Eigen::ComputeFullU);
    svd.setThreshold(1e-10);
    int rank_Bk = static_cast<int>(svd.rank());

    int rows_Ak = static_cast<int>(Ak.rows());
    if (rank_Bk >= rows_Ak) {
      nt.insert(nt.begin(), rank_Bk);
      break;
    }

    nt.insert(nt.begin(), rank_Bk);

    auto U_full = svd.matrixU();
    Eigen::MatrixXd B_p_cols = U_full.leftCols(rank_Bk);
    Eigen::MatrixXd B_ort   = U_full.rightCols(rows_Ak - rank_Bk).transpose();

    Eigen::MatrixXd T_block(rows_Ak, rows_Ak);
    T_block.topRows(rows_Ak - rank_Bk) = B_ort;
    T_block.bottomRows(rank_Bk) = B_p_cols.transpose();

    if (rows_Ak < n) {
      Eigen::MatrixXd T_temp = Eigen::MatrixXd::Identity(n, n);
      T_temp.topLeftCorner(rows_Ak, rows_Ak) = T_block;
      T = T_temp * T;
    } else {
      T = T_block;
    }

    Eigen::MatrixXd Ak_old = Ak;
    Ak = B_ort * Ak_old * B_ort.transpose();
    Bk = B_ort * Ak_old * B_p_cols;
  }

  return {T, nt};
}

// ============================================================================
// block_con — 块可控标准型（N-D 泛化版）。
// ============================================================================
inline std::pair<Eigen::MatrixXd, std::vector<int>>
block_con_nd(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
{
  auto [T1, nt] = trans_con_nd(A, B);
  if (nt.empty()) {
    return {Eigen::MatrixXd::Identity(A.rows(), A.rows()), {}};
  }

  int k = static_cast<int>(nt.size());
  int n = static_cast<int>(A.rows());

  std::vector<int> n_ind = {0};
  for (int s : nt) n_ind.push_back(n_ind.back() + s);

  Eigen::MatrixXd A1 = T1 * A * T1.inverse();
  Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(n, n);

  for (int i = 0; i < k - 1; ++i) {
    Eigen::MatrixXd A_prime = Phi * A1 * Phi.inverse();

    int r0 = n_ind[i],     r1 = n_ind[i + 1];
    int c0 = n_ind[i + 1], c1 = n_ind[i + 2];

    auto A_pivot = A_prime.block(r0, c0, r1 - r0, c1 - c0);
    auto A_lower = A_prime.block(r1, r0, c1 - r1, r1 - r0);
    Eigen::MatrixXd L = A_lower * A_pivot.completeOrthogonalDecomposition().pseudoInverse();

    Eigen::MatrixXd T_step = Eigen::MatrixXd::Identity(n, n);
    T_step.block(r1, r0, c1 - r1, r1 - r0) = -L;
    Phi = T_step * Phi;
  }

  Eigen::MatrixXd T_final = Phi * T1;
  return {T_final, nt};
}

// ============================================================================
// lpc2hpc_nd — 从线性比例控制器 (LPC) 计算齐次比例控制器 (HPC) 参数（N-D 版）。
//
// 算法与原版 4D lpc2hpc 完全相同，使用动态尺寸 MatrixXd。
// ============================================================================
inline HpcResultNd lpc2hpc_nd(const Eigen::MatrixXd& A,
                               const Eigen::MatrixXd& B,
                               const Eigen::MatrixXd& K)
{
  HpcResultNd res;
  int n = static_cast<int>(A.rows());

  // 验证 A+BK 是 Hurwitz 稳定的
  Eigen::EigenSolver<Eigen::MatrixXd> es(A + B * K);
  double max_real = -std::numeric_limits<double>::max();
  for (int i = 0; i < n; ++i) {
    max_real = std::max(max_real, std::real(es.eigenvalues()(i)));
  }
  if (-max_real * 0.001 < 1e-5) {
    std::cerr << "错误: 闭环系统稳定性裕度不足。" << std::endl;
    res.K0 = Eigen::MatrixXd::Zero(B.cols(), n);
    return res;
  }

  auto [T, nt] = block_con_nd(A, B);
  if (nt.empty()) {
    std::cerr << "错误: 块分解失败。" << std::endl;
    res.K0 = Eigen::MatrixXd::Zero(B.cols(), n);
    return res;
  }

  Eigen::MatrixXd Anew = T * A * T.inverse();
  int k = static_cast<int>(nt.size());
  std::vector<int> n_ind = {0};
  for (int s : nt) n_ind.push_back(n_ind.back() + s);

  Eigen::MatrixXd Bnew = T * B;
  int m = static_cast<int>(B.cols());

  // 取最后 nt[k-1] 行——"被驱动"子系统
  int b0_rows = n - n_ind[k - 1];
  auto B0 = Bnew.bottomRows(b0_rows);
  auto A0 = Anew.bottomRows(b0_rows);

  // K0_new = -pinv(B0)·A0, 再变换回去: K0 = K0_new·T
  Eigen::MatrixXd K0_new = Eigen::MatrixXd::Zero(m, n);
  Eigen::MatrixXd pinv_B0 = B0.completeOrthogonalDecomposition().pseudoInverse();
  K0_new.topRows(b0_rows) = -pinv_B0 * A0.topRows(b0_rows);
  res.K0 = K0_new * T;

  // vG0: 各块齐次度权重 k-1, k-2, ..., 0
  Eigen::VectorXd vG0(n);
  int idx = 0;
  for (int i = 0; i < k; ++i) {
    double weight = static_cast<double>(k - 1 - i);
    for (int j = 0; j < nt[i]; ++j) {
      vG0(idx++) = weight;
    }
  }
  res.G0 = -T.inverse() * vG0.asDiagonal() * T;

  // Lyapunov: (A+BK)^T·P + P·(A+BK) = -2I
  Eigen::MatrixXd Acl = A + B * K;
  res.P = solve_lyapunov_nd(Acl, 2.0 * Eigen::MatrixXd::Identity(n, n));

  // 从 M = sqrt(P)·G0·inv(sqrt(P)) + (...) 计算 nu 可容许范围
  try {
    Eigen::MatrixXd sqrt_P     = res.P.sqrt();
    Eigen::MatrixXd inv_sqrt_P = sqrt_P.inverse();
    Eigen::MatrixXd M = sqrt_P * res.G0 * inv_sqrt_P;
    Eigen::MatrixXd M_sym = M + M.transpose();  // 避免 aliasing
    M = M_sym;

    Eigen::EigenSolver<Eigen::MatrixXd> es_m(M);
    double lambda_min = 0.0, lambda_max = 0.0;
    for (int i = 0; i < n; ++i) {
      double eig_real = std::real(es_m.eigenvalues()(i));
      if (i == 0) {
        lambda_min = lambda_max = eig_real;
      } else {
        lambda_min = std::min(lambda_min, eig_real);
        lambda_max = std::max(lambda_max, eig_real);
      }
    }

    res.nu_min = -1.0;
    if (lambda_max > 1e-6)
      res.nu_min = std::max(-1.0, -1.0 / lambda_max + 1e-6);

    res.nu_max = 1.0 / k;
    if (lambda_min < -1e-6)
      res.nu_max = std::min(1.0 / k, -1.0 / lambda_min);
  } catch (...) {
    std::cerr << "警告: 矩阵 P 奇异或接近奇异，无法计算 nu。" << std::endl;
    res.nu_min = -1.0;
    res.nu_max = 1.0 / k;
  }

  return res;
}

}  // namespace formation_control
