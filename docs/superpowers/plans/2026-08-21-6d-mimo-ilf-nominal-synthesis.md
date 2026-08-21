# 6D MIMO-ILF Nominal Synthesis Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reproduce a continuous-time, zero-delay, frozen-workpoint 6D MIMO ILF controller and its finite-time Lyapunov identity numerically, before making any delayed-system or ROS claim.

**Architecture:** At the sole nominal point `rho=0`, use an exact static input transformation to convert the actuator-aware model into three parallel double integrators. Solve the finite-time MIMO ILF matrix equality of Polyakov--Efimov--Perruquetti (2016) with a normalized CVXPY feasibility problem, find the positive implicit-Lyapunov root numerically, and simulate only the resulting continuous canonical system. Keep the existing controllability scanner as the common model source and keep all results offline.

**Tech Stack:** Python 3, NumPy, SciPy (`brentq`, `solve_ivp`), CVXPY with installed Clarabel solver, pytest. No ROS node, C++ target, launch file, or `CMakeLists.txt` modification.

## Global Constraints

- This plan applies only to the frozen, zero-delay, zero-residual nominal model: `rho=0`, `r=0`, `d=0`, fixed Disc target, and no saturation.
- The actual published command remains `u_f = u_star + delta_u`; at this nominal point `u_star=0`.
- Use only the exact no-delay input change `delta_u=e_v+Lambda^{-1}nu`; do not apply it as an equivalence transformation to `delta_u(t-d(t))`.
- Use the MIMO finite-time ILF theorem for a block-controllability form (Polyakov, Efimov, Perruquetti, 2016, Theorem 10), not the SISO delayed-ILF theorem.
- `mu` must be strictly between `0` and `1`, so the nominal ILF feedback is continuous at the origin; use `mu=0.5` for the first reproducible run.
- Normalize the homogeneous LMI by `trace(X)=1` only to select a numerical representative; this is not an extra theorem assumption.
- Every code change follows a red-green-refactor test cycle.
- Do not modify existing 4D/6D Artstein controllers or launch files.

---

### Task 1: State the exact nominal reduction and its delay boundary

**Files:**
- Modify: `homo_multirobot_formation_control/doc/6d_ilf_delay_robust_control_proposal.md`

**Interfaces:**
- Documents the nominal state `s=xi=[e_p^T,e_v^T]^T`, the canonical matrices `A_tilde=[[0,I_3],[0,0]]`, `B_tilde=[[0],[I_3]]`, and the command restoration `delta_u=e_v+Lambda^{-1}nu`.
- Documents explicitly that with delayed input the transformed velocity equation contains `-Lambda(e_v(t)-e_v(t-d)) + nu(t-d)` and is therefore not the delay-free canonical system.

- [x] **Step 1: Add the derivation before implementation**

Insert a subsection after the candidate-A description containing:

```math
\dot\xi=\begin{bmatrix}0&I_3\\0&-\Lambda\end{bmatrix}\xi+
\begin{bmatrix}0\\\Lambda\end{bmatrix}\delta u,
\qquad
\delta u=e_v+\Lambda^{-1}\nu
\Longrightarrow
\dot s=\widetilde A s+\widetilde B\nu.
```

Then state the delayed counter-calculation:

```math
\dot e_v(t)=-\Lambda[e_v(t)-e_v(t-d(t))]+\nu(t-d(t)).
```

and prohibit using the nominal finite-time theorem as a delay proof.

- [x] **Step 2: Verify the equations against the implementation interfaces**

Run:

```bash
rg -n "canonical|delta_u|e_v\\(t-d|Theorem 10" \
  homo_multirobot_formation_control/doc/6d_ilf_delay_robust_control_proposal.md
```

Expected: all three boundaries (nominal equivalence, command restoration, non-equivalence with delay) appear in the document.

### Task 2: Add and test canonical-model and command-map helpers

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/ilf_6d_feasibility.py`
- Modify: `homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py`

**Interfaces:**
- Produces `build_nominal_canonical_model() -> tuple[np.ndarray, np.ndarray]`, returning `(A_tilde, B_tilde)` with shapes `(6,6)` and `(6,3)`.
- Produces `canonical_to_deviation_input(xi: np.ndarray, nu: np.ndarray, tau: np.ndarray) -> np.ndarray`, returning `delta_u=e_v+diag(tau)nu`.

- [x] **Step 1: Write the failing tests**

```python
def test_nominal_input_transformation_gives_three_double_integrators():
    A, B = feasibility.build_local_model(np.zeros(3), np.array([0.43] * 3))
    A_tilde, B_tilde = feasibility.build_nominal_canonical_model()
    state_feedback = np.hstack((np.zeros((3, 3)), np.eye(3)))

    np.testing.assert_allclose(A + B @ state_feedback, A_tilde)
    np.testing.assert_allclose(B @ np.diag([0.43] * 3), B_tilde)


def test_canonical_control_maps_back_to_deviation_cmd_vel():
    delta_u = feasibility.canonical_to_deviation_input(
        xi=np.array([1.0, 2.0, 3.0, 0.1, -0.2, 0.3]),
        nu=np.array([2.0, -1.0, 0.5]),
        tau=np.array([0.5, 0.4, 0.2]),
    )

    np.testing.assert_allclose(delta_u, [1.1, -0.6, 0.4])
```

- [x] **Step 2: Run the focused tests and confirm they fail for missing helpers**

Run:

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py \
  -k 'nominal_input_transformation or canonical_control_maps_back' -q
```

Expected: import/attribute failure mentioning the absent helper names.

- [x] **Step 3: Implement the two minimal helpers**

```python
def build_nominal_canonical_model() -> tuple[np.ndarray, np.ndarray]:
    a_tilde = np.zeros((6, 6))
    a_tilde[:3, 3:] = np.eye(3)
    b_tilde = np.zeros((6, 3))
    b_tilde[3:, :] = np.eye(3)
    return a_tilde, b_tilde


def canonical_to_deviation_input(
    xi: np.ndarray, nu: np.ndarray, tau: np.ndarray
) -> np.ndarray:
    xi = np.asarray(xi, dtype=float)
    nu = np.asarray(nu, dtype=float)
    tau = np.asarray(tau, dtype=float)
    if xi.shape != (6,) or nu.shape != (3,) or tau.shape != (3,) or np.any(tau <= 0.0):
        raise ValueError("xi must have 6 entries; nu and positive tau must have 3")
    return xi[3:] + tau * nu
```

- [x] **Step 4: Run focused and package tests**

Run:

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py -q
python3 -m pytest homo_multirobot_formation_control/test -q
```

Expected: all tests pass.

### Task 3: Solve and verify the normalized MIMO ILF matrix condition

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/ilf_6d_feasibility.py`
- Modify: `homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py`

**Interfaces:**
- Produces immutable `IlfDesign(mu: float, X: np.ndarray, P: np.ndarray, Y: np.ndarray, K: np.ndarray, H: np.ndarray)`.
- Produces `synthesize_nominal_mimo_ilf(mu: float, solver: str = "CLARABEL") -> IlfDesign`.
- The solver enforces `A_tilde X+X A_tilde^T+B_tilde Y+Y^T B_tilde^T+HX+XH=0`, `X>0`, `XH+HX>0`, and `trace(X)=1`, then returns `P=X^{-1}`, `K=YX^{-1}`.

- [x] **Step 1: Write the failing synthesis test**

```python
def test_nominal_mimo_ilf_synthesis_satisfies_theorem_10_matrix_identity():
    design = feasibility.synthesize_nominal_mimo_ilf(mu=0.5)
    A_tilde, B_tilde = feasibility.build_nominal_canonical_model()
    residual = (
        A_tilde @ design.X + design.X @ A_tilde.T
        + B_tilde @ design.Y + design.Y.T @ B_tilde.T
        + design.H @ design.X + design.X @ design.H
    )

    assert np.linalg.eigvalsh(design.X).min() > 1e-6
    assert np.linalg.eigvalsh(design.X @ design.H + design.H @ design.X).min() > 1e-6
    np.testing.assert_allclose(residual, np.zeros((6, 6)), atol=1e-7)
    assert np.linalg.eigvals(A_tilde + B_tilde @ design.K).real.max() < 0.0
```

- [x] **Step 2: Run it and confirm failure because the synthesizer is absent**

Run:

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py \
  -k theorem_10_matrix_identity -q
```

Expected: import/attribute failure for `synthesize_nominal_mimo_ilf`.

- [x] **Step 3: Implement the normalized CVXPY synthesis**

Implement a frozen `@dataclass` and the function below. Reject `mu <= 0` or `mu >= 1`; import CVXPY only inside the synthesis function and raise an actionable `RuntimeError` if unavailable. Use `epsilon=1e-5`, solver status `optimal` or `optimal_inaccurate`, and a deterministic objective `Minimize(sum_squares(Y))`.

```python
H = np.diag([1.0 + mu] * 3 + [1.0] * 3)
X = cp.Variable((6, 6), symmetric=True)
Y = cp.Variable((3, 6))
identity = A_tilde @ X + X @ A_tilde.T + B_tilde @ Y + Y.T @ B_tilde.T + H @ X + X @ H
constraints = [
    identity == 0,
    X >> epsilon * np.eye(6),
    X @ H + H @ X >> epsilon * np.eye(6),
    cp.trace(X) == 1.0,
]
```

After solve, symmetrize `X`, calculate `P=np.linalg.inv(X)`, `K=Y@P`, and reject a residual greater than `1e-7` or a nonpositive eigenvalue before returning.

- [x] **Step 4: Run focused and package tests**

Run:

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py \
  -k theorem_10_matrix_identity -q
python3 -m pytest homo_multirobot_formation_control/test -q
```

Expected: all tests pass.

### Task 4: Add the implicit root, ILF feedback, and continuous nominal simulation

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/ilf_6d_feasibility.py`
- Modify: `homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py`

**Interfaces:**
- Produces `implicit_lyapunov_value(xi: np.ndarray, design: IlfDesign) -> float`, with `V(0)=0` and positive `V` found from `Q(V,xi)=0` via a bracketed scalar root on `log(V)`.
- Produces `nominal_ilf_control(xi: np.ndarray, design: IlfDesign) -> np.ndarray`, implementing `nu=V^(1-mu) K D(V^-1) xi` and returning zero at the origin.
- Produces `simulate_nominal_ilf(x0, design, duration, max_step) -> dict[str,np.ndarray]`; its output keys are exactly `time`, `state`, `lyapunov`, `control`.

- [x] **Step 1: Write the failing behavioral tests**

```python
def test_implicit_lyapunov_root_and_controller_satisfy_the_nominal_identity():
    design = feasibility.synthesize_nominal_mimo_ilf(mu=0.5)
    xi = np.array([1.0, -0.7, 0.3, 0.0, 0.0, 0.0])
    value = feasibility.implicit_lyapunov_value(xi, design)
    nu = feasibility.nominal_ilf_control(xi, design)
    dilation = np.diag([value ** -(1.0 + design.mu)] * 3 + [value ** -1.0] * 3)
    q = xi @ dilation @ design.P @ dilation @ xi - 1.0
    A_tilde, B_tilde = feasibility.build_nominal_canonical_model()
    expected_v_dot = -value ** (1.0 - design.mu)
    finite_difference = (feasibility.implicit_lyapunov_value(
        xi + 1e-6 * (A_tilde @ xi + B_tilde @ nu), design
    ) - value) / 1e-6

    assert value > 0.0
    np.testing.assert_allclose(q, 0.0, atol=1e-8)
    np.testing.assert_allclose(finite_difference, expected_v_dot, rtol=2e-3, atol=2e-4)


def test_continuous_nominal_simulation_decreases_implicit_lyapunov_value():
    design = feasibility.synthesize_nominal_mimo_ilf(mu=0.5)
    result = feasibility.simulate_nominal_ilf(
        x0=np.array([1.0, -0.7, 0.3, 0.0, 0.0, 0.0]),
        design=design,
        duration=4.0,
        max_step=1e-3,
    )

    assert result["lyapunov"][-1] < result["lyapunov"][0] * 1e-3
    assert np.linalg.norm(result["state"][-1]) < 2e-3
```

- [x] **Step 2: Run focused tests and confirm they fail because the functions are absent**

Run:

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py \
  -k 'implicit_lyapunov_root or continuous_nominal_simulation' -q
```

Expected: import/attribute failures naming the absent functions.

- [x] **Step 3: Implement the root, feedback, and solver**

Use `scipy.optimize.brentq` on `log_value` with initial bracket `[-30,30]`, expanding by 10 until signs differ; evaluate `Q(exp(log_value),xi)`. Use `scipy.integrate.solve_ivp` with `rtol=1e-8`, `atol=1e-10`, and `max_step` supplied by the caller. Evaluate `V` and `nu` for every returned integration state; do not add a stop event, saturation, sampling, delay, leader motion, or residual disturbance.

- [x] **Step 4: Run focused and package tests**

Run:

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py -q
python3 -m pytest homo_multirobot_formation_control/test -q
```

Expected: all tests pass.

### Task 5: Generate a reproducible nominal-result record and update the theory gate

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/ilf_6d_feasibility.py`
- Create: `homo_multirobot_formation_control/analysis/results/6d_ilf_feasibility/nominal_ilf_run.csv`
- Modify: `homo_multirobot_formation_control/analysis/results/6d_ilf_feasibility/README.md`
- Modify: `homo_multirobot_formation_control/doc/6d_ilf_delay_robust_control_proposal.md`

**Interfaces:**
- CLI gains `--run-nominal-ilf` and `--nominal-csv`; the latter defaults to `analysis/results/6d_ilf_feasibility/nominal_ilf_run.csv`.
- The CSV columns are exactly `time,e_x,e_y,e_theta,e_vx,e_vy,e_omega,V,nu_x,nu_y,nu_omega` using LF endings.

- [x] **Step 1: Write the failing CSV test**

```python
def test_write_nominal_ilf_csv_has_expected_header_and_lf_endings(tmp_path):
    design = feasibility.synthesize_nominal_mimo_ilf(mu=0.5)
    result = feasibility.simulate_nominal_ilf(
        np.array([0.2, 0.0, 0.0, 0.0, 0.0, 0.0]), design, duration=0.1, max_step=1e-3
    )
    output = tmp_path / "nominal.csv"

    feasibility.write_nominal_ilf_csv(result, output)

    assert output.read_text(encoding="utf-8").splitlines()[0] == (
        "time,e_x,e_y,e_theta,e_vx,e_vy,e_omega,V,nu_x,nu_y,nu_omega"
    )
    assert b"\r\n" not in output.read_bytes()
```

- [x] **Step 2: Run it and confirm failure for the absent writer**

Run:

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py \
  -k write_nominal_ilf_csv -q
```

Expected: import/attribute failure for `write_nominal_ilf_csv`.

- [x] **Step 3: Implement the CSV writer and CLI path**

Add `write_nominal_ilf_csv(result, csv_path)` using `csv.writer(..., lineterminator="\n")`. For `--run-nominal-ilf`, synthesize with `mu=0.5`, simulate `x0=[1,-0.7,0.3,0,0,0]` for 4.0 seconds with `max_step=1e-3`, write the result, and print initial/final `V` and final state norm.

- [x] **Step 4: Run the reproducible numerical gate**

Run:

```bash
python3 homo_multirobot_formation_control/scripts/ilf_6d_feasibility.py \
  --run-nominal-ilf \
  --nominal-csv homo_multirobot_formation_control/analysis/results/6d_ilf_feasibility/nominal_ilf_run.csv
```

Expected: a non-empty CSV; `V_final < 10^-3 V_initial` and final state norm `< 2e-3` for this numerical run.

- [x] **Step 5: Record bounded conclusions only**

In both result documents state: this gate confirms a numerical reproduction of the *zero-delay frozen canonical model* and MIMO-ILF matrix identity. It neither proves a delay margin nor validates a time-varying Leader, Disc switching, saturation, sampling, measurements, or a ROS controller. Name the next mandatory gate: construct an ILKF/Razumikhin delay bound and then perform DDE simulation.

- [x] **Step 6: Final verification and commit**

Run:

```bash
git diff --check
python3 -m pytest homo_multirobot_formation_control/test -q
git status --short
```

Expected: no whitespace errors; tests pass; only intended tracked changes are staged. Commit only the plan’s tracked files with:

```bash
git add docs/superpowers/plans/2026-08-21-6d-mimo-ilf-nominal-synthesis.md \
  homo_multirobot_formation_control/doc/6d_ilf_delay_robust_control_proposal.md \
  homo_multirobot_formation_control/scripts/ilf_6d_feasibility.py \
  homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py \
  homo_multirobot_formation_control/analysis/results/6d_ilf_feasibility/
git commit -m "实现6D MIMO ILF名义综合验证"
```

## Plan Self-Review

- Spec coverage: Task 1 prevents an invalid delayed-system inference; Tasks 2--4 reproduce the source theorem only for the frozen no-delay model; Task 5 writes a repeatable numerical record and sets the next gate.
- Placeholder scan: every code task identifies exact files, function signatures, test code, execution command, and acceptance criterion.
- Type consistency: the canonical helper returns `(6,6)` and `(6,3)` matrices; `IlfDesign` feeds implicit-root, feedback, simulation, and CSV functions; all simulation outputs use the same four named arrays.
