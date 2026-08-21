# 6D MIMO-ILF Delay-Gap Audit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the frozen 6D input-delay model into an exact nominal MIMO-ILF system plus a matched history disturbance, then audit the sufficient robust-ILF condition on reproducible DDE trajectories without claiming an unproved delay margin.

**Architecture:** For `rho=0`, rewrite the physical delayed actuator model as `dot xi=A_tilde xi+B_tilde(nu(xi)+w_d)`, where `w_d` is calculated exactly from present and delayed state. Solve the robust MIMO ILF inequality of Polyakov--Efimov--Perruquetti (2016, Theorem 15) for a fixed positive disturbance matrix, then use a method-of-steps Euler DDE simulator to record the theorem's sufficient matched-disturbance ratio. The ratio is a numerical audit only: a value above one rejects that sufficient condition for that trace; a value below one at sampled instants is not a continuous-time proof.

**Tech Stack:** Python 3, NumPy, SciPy, CVXPY/Clarabel, pytest. Existing `ilf_6d_feasibility.py` only; no ROS node, C++ target, launch file, or `CMakeLists.txt` change.

## Global Constraints

- Scope is the frozen local model with `rho=0`, fixed target, `r=0`, no saturation, and positive diagonal `tau`; it excludes Leader variation, Disc switching, sampling at 20 Hz, measurement errors, and physical experiments.
- The actual delayed plant is `dot e_v=-Lambda e_v+Lambda delta_u(t-d)`, while the controller command is `delta_u(t)=e_v(t)+Lambda^{-1}nu(xi(t))`.
- Define `w_d(t)=-Lambda(e_v(t)-e_v(t-d))+nu(xi(t-d))-nu(xi(t))`; do not replace it by `nu(t-d)-nu(t)` alone.
- The robust MIMO ILF theorem applies conditionally to an additive matched disturbance. It does not by itself establish that `w_d` obeys its condition for every delay and history.
- Use `mu=0.5`, `tau=[0.43,0.43,0.43] s`, `dt=0.001 s`, and a constant prehistory `xi(s)=xi(0)` for the first numerical audit.
- The delay must be a nonnegative integer multiple of `dt`; reject non-grid values instead of silently rounding them.
- Treat a finite-step Euler DDE result as a numerical N1 gate only, not as a continuous-time proof or a 20 Hz implementation result.
- Every code change follows a red-green-refactor test cycle.

---

### Task 1: Record the exact matched-history decomposition and certificate boundary

**Files:**
- Modify: `homo_multirobot_formation_control/doc/6d_ilf_delay_robust_control_proposal.md`

**Interfaces:**
- Documents `w_d`, `d_tilde=[0_3^T,w_d^T]^T`, and the exact equality `dot xi=A_tilde xi+B_tilde(nu+w_d)`.
- States the Theorem-15 sufficient condition in terms of `D(V^-1)`, `R>0`, `P`, and `HP+PH` and labels it conditional.

- [x] **Step 1: Add the derivation before writing DDE code**

Insert after the zero-delay reduction:

```math
\widetilde d(t)=\begin{bmatrix}0_3\\w_d(t)\end{bmatrix},\qquad
w_d(t)=-\Lambda[e_v(t)-e_v(t-d)]+\nu(\xi(t-d))-\nu(\xi(t)),
```

```math
\dot\xi=\widetilde A\xi+\widetilde B\nu(\xi)+\widetilde d.
```

For a robust design with `R>0`, write the conditional audit ratio

```math
\mathcal R_d(t)=\frac{\widetilde d^\mathsf TD(V^{-1})R^{-1}D(V^{-1})\widetilde d}
{V^{-2\mu}z^\mathsf T(HP+PH)z},\qquad z=D(V^{-1})\xi.
```

Then state exactly: `sup_t R_d(t)<1` is the source theorem's sufficient disturbance condition only if it holds for the continuous DDE trajectory; sampled values cannot prove it.

- [x] **Step 2: Check required symbols are defined**

Run:

```bash
rg -n "w_d|widetilde d|mathcal R_d|Theorem 15|充分条件" \
  homo_multirobot_formation_control/doc/6d_ilf_delay_robust_control_proposal.md
```

Expected: every symbol in the ratio is defined and the conclusion is explicitly conditional.

### Task 2: Add a robust MIMO-ILF synthesis certificate

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/ilf_6d_feasibility.py`
- Modify: `homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py`

**Interfaces:**
- Extend `IlfDesign` with `R: np.ndarray | None`.
- Produce `synthesize_robust_nominal_mimo_ilf(mu, disturbance_weight, solver="CLARABEL") -> IlfDesign`.
- It uses `R=disturbance_weight*I_6` and solves
  `A_tilde X+X A_tilde^T+B_tilde Y+Y^T B_tilde^T+HX+XH+R <= 0`,
  with `X>0`, `XH+HX>0`, `trace(X)=1`, and objective `min ||Y||_F^2`.
- Produce `matched_disturbance_ratio(xi, w_d, design) -> float`; require `design.R is not None`.

- [x] **Step 1: Write failing certificate tests**

```python
def test_robust_mimo_ilf_design_satisfies_theorem_15_matrix_inequality():
    design = feasibility.synthesize_robust_nominal_mimo_ilf(0.5, 1e-3)
    a_tilde, b_tilde = feasibility.build_nominal_canonical_model()
    lmi_left = (
        a_tilde @ design.X + design.X @ a_tilde.T
        + b_tilde @ design.Y + design.Y.T @ b_tilde.T
        + design.H @ design.X + design.X @ design.H + design.R
    )

    assert design.R.shape == (6, 6)
    assert np.linalg.eigvalsh(lmi_left).max() <= 1e-7
    assert np.linalg.eigvalsh(design.X).min() > 1e-6


def test_matched_disturbance_ratio_is_zero_without_disturbance():
    design = feasibility.synthesize_robust_nominal_mimo_ilf(0.5, 1e-3)

    ratio = feasibility.matched_disturbance_ratio(
        np.array([1.0, -0.7, 0.3, 0.0, 0.0, 0.0]), np.zeros(3), design
    )

    assert ratio == 0.0
```

- [x] **Step 2: Run the tests and confirm missing-interface failures**

Run:

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py \
  -k 'robust_mimo_ilf or matched_disturbance_ratio' -q
```

Expected: attribute failures for the new robust synthesizer and ratio function.

- [x] **Step 3: Implement the LMI and ratio**

Use the same `A_tilde`, `B_tilde`, `H`, `epsilon=1e-5`, and `trace(X)=1` convention as the nominal synthesizer. Reject nonpositive `disturbance_weight`, nonoptimal solver status, nonpositive matrices, or a positive LMI residual above `1e-7`.

For the ratio, set `d_tilde=np.r_[np.zeros(3), w_d]`, `z=D(V^-1)xi`, and return the quotient shown in Task 1. Return `0.0` only when both `xi` and `w_d` are zero; otherwise reject a nonpositive denominator.

- [x] **Step 4: Run focused and package tests**

Run:

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py -q
python3 -m pytest homo_multirobot_formation_control/test -q
```

Expected: all tests pass.

### Task 3: Add a deterministic delayed-plant N1 simulator

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/ilf_6d_feasibility.py`
- Modify: `homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py`

**Interfaces:**
- Produce `simulate_delayed_ilf(x0, design, tau, delay, duration, dt) -> dict[str,np.ndarray]`.
- Output keys are exactly `time`, `state`, `lyapunov`, `nu`, `delta_u_delayed`, `w_d`, `ratio`.
- It uses forward Euler and constant prehistory `xi(s)=x0` for `s in [-delay,0]`.

- [x] **Step 1: Write failing DDE behavior tests**

```python
def test_delayed_ilf_zero_delay_has_zero_history_disturbance():
    design = feasibility.synthesize_robust_nominal_mimo_ilf(0.5, 1e-3)
    result = feasibility.simulate_delayed_ilf(
        x0=np.array([0.2, 0.0, 0.0, 0.0, 0.0, 0.0]), design=design,
        tau=np.array([0.43, 0.43, 0.43]), delay=0.0, duration=0.1, dt=0.001,
    )

    np.testing.assert_allclose(result["w_d"], 0.0, atol=1e-12)
    np.testing.assert_allclose(result["ratio"], 0.0, atol=1e-12)


def test_delayed_ilf_rejects_delay_not_on_simulation_grid():
    design = feasibility.synthesize_robust_nominal_mimo_ilf(0.5, 1e-3)

    with pytest.raises(ValueError, match="integer multiple"):
        feasibility.simulate_delayed_ilf(
            np.zeros(6), design, np.array([0.43, 0.43, 0.43]),
            delay=0.0005, duration=0.1, dt=0.001,
        )
```

Add `import pytest` at the top of the test file.

- [x] **Step 2: Run focused tests and confirm missing-interface failures**

Run:

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py \
  -k 'zero_delay_has_zero_history or rejects_delay_not_on' -q
```

Expected: attribute failure for `simulate_delayed_ilf`.

- [x] **Step 3: Implement the method-of-steps update**

For each grid time `t_k`, obtain `xi_d=history[0]`, calculate `nu_d=nominal_ilf_control(xi_d, design)`, and calculate the actually delayed command by `delta_u_d=xi_d[3:]+tau*nu_d`. Update only the physical actuator model:

```python
derivative = np.concatenate((xi[3:], (-xi[3:] + delta_u_d) / tau))
xi_next = xi + dt * derivative
```

At the same `xi`, calculate `nu_now`, then calculate

```python
w_d = -(xi[3:] - xi_d[3:]) / tau + nu_d - nu_now
```

and record the matched-disturbance ratio. Do not use `solve_ivp`, interpolate history, round a delay, insert saturation, or reuse an Artstein predictor.

- [x] **Step 4: Run focused and package tests**

Run:

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py -q
python3 -m pytest homo_multirobot_formation_control/test -q
```

Expected: all tests pass.

### Task 4: Run the delay audit and record only numerical findings

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/ilf_6d_feasibility.py`
- Create: `homo_multirobot_formation_control/analysis/results/6d_ilf_feasibility/delayed_ilf_audit.csv`
- Modify: `homo_multirobot_formation_control/analysis/results/6d_ilf_feasibility/README.md`
- Modify: `homo_multirobot_formation_control/doc/6d_ilf_delay_robust_control_proposal.md`

**Interfaces:**
- CLI gains `--run-delayed-ilf-audit` and `--delayed-audit-csv`.
- Audit CSV columns are exactly `delay,final_state_norm,final_V,max_state_norm,max_ratio,ratio_samples_below_one`.

- [x] **Step 1: Write the failing audit-CSV test**

```python
def test_write_delayed_ilf_audit_csv_has_expected_header_and_lf_endings(tmp_path):
    output = tmp_path / "audit.csv"
    feasibility.write_delayed_ilf_audit_csv([
        {"delay": 0.0, "final_state_norm": 0.0, "final_V": 0.0,
         "max_state_norm": 1.0, "max_ratio": 0.0, "ratio_samples_below_one": True}
    ], output)

    assert output.read_text(encoding="utf-8").splitlines()[0] == (
        "delay,final_state_norm,final_V,max_state_norm,max_ratio,ratio_samples_below_one"
    )
    assert b"\r\n" not in output.read_bytes()
```

- [x] **Step 2: Run it and confirm failure for the absent writer**

Run:

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py \
  -k write_delayed_ilf_audit_csv -q
```

Expected: attribute failure for `write_delayed_ilf_audit_csv`.

- [x] **Step 3: Implement the audit CLI and writer**

Use `delay_values=[0.0,0.05,0.10,0.15,0.22,0.30]`, `duration=8.0`, `dt=0.001`, `x0=[1,-0.7,0.3,0,0,0]`, `tau=[0.43]*3`, `mu=0.5`, and `disturbance_weight=1e-3`. For each row, omit `ratio` values at exactly zero Lyapunov value; report the maximum remaining finite ratio, or `0.0` if none. `ratio_samples_below_one` is true only if every recorded finite ratio is `<1`.

- [x] **Step 4: Run the reproducible audit**

Run:

```bash
python3 homo_multirobot_formation_control/scripts/ilf_6d_feasibility.py \
  --run-delayed-ilf-audit \
  --delayed-audit-csv homo_multirobot_formation_control/analysis/results/6d_ilf_feasibility/delayed_ilf_audit.csv
```

Expected: six rows, each with a grid-aligned delay, and the zero-delay row has `max_ratio=0` and `ratio_samples_below_one=True`.

- [x] **Step 5: Update theory/result documents without overclaiming**

Record the source theorem and the robust-LMI parameter `R=1e-3 I_6`; distinguish the LMI's conditional matched-disturbance certificate from the sampled DDE audit. If any delayed row has `max_ratio>=1`, state that it fails this sufficient certificate at sampled points, not that the controller is proven unstable. State that no numerical row is an estimated `d_bar^*`.

- [x] **Step 6: Final verification and commit**

Run:

```bash
git diff --check
python3 -m pytest homo_multirobot_formation_control/test -q
git status --short
```

Expected: no whitespace errors; tests pass; the unrelated `.vscode/` and two user PDF files remain unstaged. Commit only this plan's files:

```bash
git add docs/superpowers/plans/2026-08-21-6d-ilf-delay-gap-audit.md \
  homo_multirobot_formation_control/doc/6d_ilf_delay_robust_control_proposal.md \
  homo_multirobot_formation_control/scripts/ilf_6d_feasibility.py \
  homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py \
  homo_multirobot_formation_control/analysis/results/6d_ilf_feasibility/README.md \
  homo_multirobot_formation_control/analysis/results/6d_ilf_feasibility/delayed_ilf_audit.csv
git commit -m "增加6D ILF时滞缺口数值审计"
```

## Plan Self-Review

- Spec coverage: Task 1 establishes the only valid delayed decomposition; Task 2 gives the conditional MIMO robust certificate; Task 3 simulates the exact delayed actuator plant; Task 4 records a reproducible audit while preserving the theory boundary.
- Placeholder scan: every implementation step names exact files, APIs, test code, equations, commands, grids, and acceptance criteria.
- Type consistency: `IlfDesign` holds optional `R`; robust synthesis produces it; the DDE simulator consumes it and returns the ratio; the audit writer receives scalar row dictionaries from that simulator.
