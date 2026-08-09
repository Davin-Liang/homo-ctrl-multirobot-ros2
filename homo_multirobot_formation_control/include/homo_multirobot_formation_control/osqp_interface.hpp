#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#if __has_include(<osqp/osqp.h>)
#include <osqp/osqp.h>
#else
#include <osqp.h>
#endif
#include <memory>
#include <vector>

namespace formation_control {

/// Minimal RAII wrapper around the OSQP C API.
///
/// Usage:
///   OsqpSolver solver;
///   solver.setup(P, q, A, l, u);
///   if (solver.solve()) {
///     Eigen::VectorXd x = solver.solution();
///   }
///
/// Note: no update() in this version — each control cycle calls setup()+solve().
/// Optimisation (workspace reuse, warm-start) deferred to stage 2.
class OsqpSolver {
public:
  OsqpSolver();
  ~OsqpSolver();

  OsqpSolver(const OsqpSolver&) = delete;
  OsqpSolver& operator=(const OsqpSolver&) = delete;

  /// Configure the QP:  min 0.5*x'*P*x + q'*x  s.t.  l <= A*x <= u
  /// Destroys any previous workspace; caller owns the Eigen matrices.
  void setup(const Eigen::SparseMatrix<double>& P,
             const Eigen::VectorXd& q,
             const Eigen::SparseMatrix<double>& A,
             const Eigen::VectorXd& l,
             const Eigen::VectorXd& u);

  void configure(int max_iter, double eps_abs, double eps_rel, bool polish);

  /// Solve the QP. Returns true iff OSQP_SOLVED.
  bool solve();
  bool has_usable_solution() const;

  /// Primal solution (size n).
  Eigen::VectorXd solution() const;

  /// OSQP status code (1 = solved, -2 = primal infeasible, etc.).
  int status() const;
  const char* status_string() const;

private:
  /// Convert Eigen::SparseMatrix<double> to OSQP csc struct (caller frees with csc_spfree).
  static csc* eigen_to_csc(const Eigen::SparseMatrix<double>& M);
  static c_float* vec_to_raw(const Eigen::VectorXd& v);

  void cleanup();

  OSQPWorkspace* work_ = nullptr;
  OSQPSettings*  settings_ = nullptr;
  OSQPData*      data_ = nullptr;
  int n_vars_ = 0;
};

// ============================================================================
// Inline implementations
// ============================================================================

inline OsqpSolver::OsqpSolver()
{
  settings_ = reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(settings_);
  settings_->warm_start = 0;   // no warm-start for rebuild strategy
  settings_->polish     = 0;
  settings_->verbose       = 0;
  settings_->max_iter      = 2000;
  settings_->eps_abs       = 1e-3;
  settings_->eps_rel       = 1e-3;
}

inline OsqpSolver::~OsqpSolver()
{
  cleanup();
  if (settings_) { c_free(settings_); settings_ = nullptr; }
}

inline void OsqpSolver::cleanup()
{
  if (work_) { osqp_cleanup(work_); work_ = nullptr; }
  if (data_) {
    // data_->P and data_->A were allocated by eigen_to_csc — free them
    if (data_->P) { csc_spfree(data_->P); data_->P = nullptr; }
    if (data_->A) { csc_spfree(data_->A); data_->A = nullptr; }
    if (data_->q) { c_free(data_->q); data_->q = nullptr; }
    if (data_->l) { c_free(data_->l); data_->l = nullptr; }
    if (data_->u) { c_free(data_->u); data_->u = nullptr; }
    c_free(data_);
    data_ = nullptr;
  }
}

inline void OsqpSolver::setup(const Eigen::SparseMatrix<double>& P,
                               const Eigen::VectorXd& q,
                               const Eigen::SparseMatrix<double>& A,
                               const Eigen::VectorXd& l,
                               const Eigen::VectorXd& u)
{
  cleanup();

  n_vars_ = P.rows();

  data_ = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  data_->n = n_vars_;
  data_->m = A.rows();
  data_->P = eigen_to_csc(P);
  data_->q = vec_to_raw(q);
  data_->A = eigen_to_csc(A);
  data_->l = vec_to_raw(l);
  data_->u = vec_to_raw(u);

  osqp_setup(&work_, data_, settings_);
}

inline void OsqpSolver::configure(int max_iter, double eps_abs, double eps_rel, bool polish)
{
  if (!settings_) return;
  settings_->max_iter = max_iter;
  settings_->eps_abs = eps_abs;
  settings_->eps_rel = eps_rel;
  settings_->polish = polish ? 1 : 0;
}

inline bool OsqpSolver::solve()
{
  if (!work_) return false;
  osqp_solve(work_);
  return work_->info->status_val == OSQP_SOLVED;
}

inline bool OsqpSolver::has_usable_solution() const
{
  if (!work_ || !work_->solution || !work_->solution->x || !work_->info) {
    return false;
  }
  const int status = work_->info->status_val;
  return status == OSQP_SOLVED ||
         status == OSQP_SOLVED_INACCURATE ||
         status == OSQP_MAX_ITER_REACHED;
}

inline Eigen::VectorXd OsqpSolver::solution() const
{
  Eigen::VectorXd x(n_vars_);
  if (work_ && work_->solution) {
    for (int i = 0; i < n_vars_; ++i)
      x(i) = work_->solution->x[i];
  } else {
    x.setZero();
  }
  return x;
}

inline int OsqpSolver::status() const
{
  return work_ ? work_->info->status_val : -1;
}

inline const char* OsqpSolver::status_string() const
{
  return work_ && work_->info ? work_->info->status : "unavailable";
}

// ---- Private helpers -------------------------------------------------------

inline csc* OsqpSolver::eigen_to_csc(const Eigen::SparseMatrix<double>& M)
{
  Eigen::SparseMatrix<double, Eigen::ColMajor> Mc = M;
  Mc.makeCompressed();

  c_int nz = Mc.nonZeros();
  csc* C = csc_spalloc(Mc.rows(), Mc.cols(), nz, /*values=*/1, /*indexed=*/0);
  if (!C) return nullptr;

  for (c_int j = 0; j <= Mc.cols(); ++j)
    C->p[j] = Mc.outerIndexPtr()[j];
  for (c_int k = 0; k < nz; ++k)
    C->i[k] = Mc.innerIndexPtr()[k];
  for (c_int k = 0; k < nz; ++k)
    C->x[k] = Mc.valuePtr()[k];

  C->nzmax = nz;
  return C;
}

inline c_float* OsqpSolver::vec_to_raw(const Eigen::VectorXd& v)
{
  c_int n = v.size();
  c_float* raw = reinterpret_cast<c_float*>(c_malloc(n * sizeof(c_float)));
  for (c_int i = 0; i < n; ++i)
    raw[i] = v(i);
  return raw;
}

}  // namespace formation_control
