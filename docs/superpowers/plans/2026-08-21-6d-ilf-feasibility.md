# 6D ILF Feasibility Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Establish, by reproducible offline calculations and numerical simulation, whether a local 6D MIMO ILF/ILKF controller is theoretically and numerically viable for the measured actuator-delay envelope before any ROS implementation is attempted.

**Architecture:** Add a standalone Python feasibility module that builds the frozen 6D actuator-aware error model, computes controllability and conditioning over a leader-twist grid, and serializes the results. Keep controller synthesis separate: it consumes only feasible operating points and must not be implemented until a MIMO ILF/ILKF construction has been selected. Test the mathematical model with `pytest` before adding the scanner.

**Tech Stack:** Python 3, NumPy, SciPy, pytest; existing ROS package scripts only as offline inputs. No ROS node, Gazebo launch file, C++ target, or `CMakeLists.txt` change in this plan.

## Global Constraints

- Work on the local model in `homo_multirobot_formation_control/doc/6d_ilf_delay_robust_control_proposal.md`, equations (1)–(2).
- Use real velocity states and actual post-constraint `cmd_vel` semantics; do not use the controller's internal command state as a plant state.
- Do not claim a MIMO-ILF theorem from the SISO delayed-ILF literature.
- Numerical DDE simulation and a passed feasibility gate are mandatory before ROS implementation.
- The physical control period is `0.05 s` (20 Hz); include it in later sampled-data tests.
- Do not modify existing 4D/6D Artstein controllers or launch files.

---

### Task 1: Build and test the frozen actuator-aware 6D model

**Files:**
- Create: `homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py`
- Create: `homo_multirobot_formation_control/scripts/ilf_6d_feasibility.py`

**Interfaces:**
- Produces `build_local_model(rho, tau) -> tuple[np.ndarray, np.ndarray]` for the
  **deviation-input** model `delta_u = u_cmd - u_star`.
- `rho` is a length-3 vector `[vx_leader, vy_leader, omega_leader]` in leader body coordinates.
- `tau` is a length-3 positive vector `[tau_x, tau_y, tau_omega]` in seconds.
- Returns `A` with shape `(6, 6)` and `B` with shape `(6, 3)` implementing equation (2) in the proposal.
- The absolute equilibrium command is `u_star = rho`; it is restored only when a
  later controller is converted to actual `cmd_vel`.

- [x] **Step 1: Write the failing test**

```python
import numpy as np

from ilf_6d_feasibility import build_local_model


def test_build_local_model_includes_leader_coupling_and_actuator_poles():
    A, B = build_local_model(
        rho=np.array([0.4, -0.2, 0.3]),
        tau=np.array([0.5, 0.4, 0.25]),
    )

    assert A.shape == (6, 6)
    assert B.shape == (6, 3)
    np.testing.assert_allclose(A[0, :], [0.0, 0.3, 0.2, 1.0, 0.0, 0.0])
    np.testing.assert_allclose(A[1, :], [-0.3, 0.0, 0.4, 0.0, 1.0, 0.0])
    np.testing.assert_allclose(np.diag(A)[3:], [-2.0, -2.5, -4.0])
    np.testing.assert_allclose(B[3:, :], np.diag([2.0, 2.5, 4.0]))
    np.testing.assert_allclose(B[:3, :], np.zeros((3, 3)))
```

- [x] **Step 2: Run the test to verify it fails because the module is absent**

Run:

```bash
PYTHONPATH=homo_multirobot_formation_control/scripts \
python3 -m pytest homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py -q
```

Expected: `ModuleNotFoundError: No module named 'ilf_6d_feasibility'`.

- [x] **Step 3: Write the minimal model implementation**

```python
def build_local_model(rho: np.ndarray, tau: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    rho = np.asarray(rho, dtype=float)
    tau = np.asarray(tau, dtype=float)
    if rho.shape != (3,) or tau.shape != (3,) or np.any(tau <= 0.0):
        raise ValueError("rho and tau must be length-3 arrays; tau must be positive")
    vx_l, vy_l, omega_l = rho
    inv_tau = 1.0 / tau
    A = np.zeros((6, 6))
    A[0] = [0.0, omega_l, -vy_l, 1.0, 0.0, 0.0]
    A[1] = [-omega_l, 0.0, vx_l, 0.0, 1.0, 0.0]
    A[2, 5] = 1.0
    A[3:, 3:] = -np.diag(inv_tau)
    B = np.zeros((6, 3))
    B[3:, :] = np.diag(inv_tau)
    return A, B
```

- [x] **Step 4: Run the focused test and package tests**

Run:

```bash
PYTHONPATH=homo_multirobot_formation_control/scripts \
python3 -m pytest homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py -q
python3 -m pytest homo_multirobot_formation_control/test -q
```

Expected: all tests pass.

### Task 2: Add controllability and conditioning diagnostics

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/ilf_6d_feasibility.py`
- Modify: `homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py`

**Interfaces:**
- Produces `controllability_diagnostics(A, B, rank_tol=1e-9) -> dict[str, float | int]`.
- Returned keys are exactly `rank`, `sigma_min`, `sigma_max`, `condition_number`.
- `condition_number` is `inf` when `sigma_min <= rank_tol`.

- [x] **Step 1: Write the failing test**

```python
from ilf_6d_feasibility import build_local_model, controllability_diagnostics


def test_nominal_actuator_aware_6d_model_is_full_rank_controllable():
    A, B = build_local_model(np.zeros(3), np.array([0.43, 0.43, 0.43]))
    result = controllability_diagnostics(A, B)

    assert result["rank"] == 6
    assert result["sigma_min"] > 0.0
    assert np.isfinite(result["condition_number"])
```

- [x] **Step 2: Run the test to verify it fails because the diagnostic is absent**

Run:

```bash
PYTHONPATH=homo_multirobot_formation_control/scripts \
python3 -m pytest homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py::test_nominal_actuator_aware_6d_model_is_full_rank_controllable -q
```

Expected: import failure for `controllability_diagnostics`.

- [x] **Step 3: Implement the diagnostic**

```python
def controllability_diagnostics(A: np.ndarray, B: np.ndarray, rank_tol: float = 1e-9) -> dict:
    n = A.shape[0]
    blocks = []
    power = np.eye(n)
    for _ in range(n):
        blocks.append(power @ B)
        power = A @ power
    controllability = np.hstack(blocks)
    singular_values = np.linalg.svd(controllability, compute_uv=False)
    sigma_max = float(singular_values[0])
    sigma_min = float(singular_values[-1])
    rank = int(np.linalg.matrix_rank(controllability, tol=rank_tol))
    return {
        "rank": rank,
        "sigma_min": sigma_min,
        "sigma_max": sigma_max,
        "condition_number": float("inf") if sigma_min <= rank_tol else sigma_max / sigma_min,
    }
```

- [x] **Step 4: Run the focused and package tests**

Run:

```bash
PYTHONPATH=homo_multirobot_formation_control/scripts \
python3 -m pytest homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py -q
python3 -m pytest homo_multirobot_formation_control/test -q
```

Expected: all tests pass.

### Task 3: Add a reproducible operating-envelope scanner

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/ilf_6d_feasibility.py`
- Modify: `homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py`
- Create: `homo_multirobot_formation_control/analysis/results/6d_ilf_feasibility/controllability_scan.csv`

**Interfaces:**
- Produces `scan_envelope(vx_values, vy_values, omega_values, tau_values) -> list[dict]`.
- Every row contains `vx_leader`, `vy_leader`, `omega_leader`, `tau_x`, `tau_y`, `tau_omega`, and the Task 2 diagnostic keys.
- CLI writes a CSV selected by `--csv`; its default must be under `analysis/results/6d_ilf_feasibility/`.

- [x] **Step 1: Write the failing test**

```python
from ilf_6d_feasibility import scan_envelope


def test_scan_envelope_returns_one_full_rank_row_for_single_operating_point():
    rows = scan_envelope(
        vx_values=[0.0], vy_values=[0.0], omega_values=[0.0],
        tau_values=[(0.43, 0.43, 0.43)],
    )

    assert len(rows) == 1
    assert rows[0]["rank"] == 6
    assert rows[0]["tau_x"] == 0.43
```

- [x] **Step 2: Run the test to verify it fails because the scanner is absent**

Run:

```bash
PYTHONPATH=homo_multirobot_formation_control/scripts \
python3 -m pytest homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py::test_scan_envelope_returns_one_full_rank_row_for_single_operating_point -q
```

Expected: import failure for `scan_envelope`.

- [x] **Step 3: Implement the scanner and CSV CLI**

```python
def scan_envelope(vx_values, vy_values, omega_values, tau_values):
    rows = []
    for vx_l, vy_l, omega_l, tau in itertools.product(vx_values, vy_values, omega_values, tau_values):
        tau_array = np.asarray(tau, dtype=float)
        A, B = build_local_model(np.array([vx_l, vy_l, omega_l]), tau_array)
        row = controllability_diagnostics(A, B)
        row.update({
            "vx_leader": float(vx_l), "vy_leader": float(vy_l), "omega_leader": float(omega_l),
            "tau_x": float(tau_array[0]), "tau_y": float(tau_array[1]), "tau_omega": float(tau_array[2]),
        })
        rows.append(row)
    return rows
```

Use `argparse`, `csv.DictWriter`, and explicit `Path.parent.mkdir(parents=True, exist_ok=True)` for the CSV output. Default grid: `vx, vy in {-0.5, 0.0, 0.5}`, `omega in {-0.5, 0.0, 0.5}`, and `tau in {(0.25,0.25,0.25), (0.43,0.43,0.43), (0.55,0.55,0.55)}`.

- [x] **Step 4: Run tests and a scanner smoke test**

Run:

```bash
PYTHONPATH=homo_multirobot_formation_control/scripts \
python3 -m pytest homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py -q
python3 homo_multirobot_formation_control/scripts/ilf_6d_feasibility.py \
  --csv homo_multirobot_formation_control/analysis/results/6d_ilf_feasibility/controllability_scan.csv
```

Expected: tests pass; CSV contains 81 data rows plus header and every row has `rank=6` for the stated grid.

### Task 4: Record the numerical gate result and decide whether MIMO-ILF synthesis can begin

**Files:**
- Modify: `homo_multirobot_formation_control/doc/6d_ilf_delay_robust_control_proposal.md`
- Create: `homo_multirobot_formation_control/analysis/results/6d_ilf_feasibility/README.md`

**Interfaces:**
- Consumes `controllability_scan.csv` from Task 3.
- Produces a dated result table with exact grid, rank range, singular-value/condition-number range, Python command, and decision.

- [x] **Step 1: Add the result-table schema before running the scan**

```markdown
| run date | rho grid | tau grid | rank range | sigma_min range | condition range | decision |
|----------|----------|----------|------------|-----------------|-----------------|----------|
```

- [x] **Step 2: Run the scan and compute summary statistics from its CSV**

Run:

```bash
python3 homo_multirobot_formation_control/scripts/ilf_6d_feasibility.py \
  --csv homo_multirobot_formation_control/analysis/results/6d_ilf_feasibility/controllability_scan.csv
```

Expected: a non-empty CSV with the exact grid recorded in Task 3.

- [x] **Step 3: Fill the result table with observed values only**

Decision rule:

```text
all sampled points rank 6 and condition numbers remain finite/non-pathological
    -> proceed to select/derive a MIMO ILF/ILKF synthesis condition.
otherwise
    -> shrink the stated operating domain or stop the direct 6D route.
```

- [x] **Step 4: Verify documentation and tests**

Run:

```bash
git diff --check
PYTHONPATH=homo_multirobot_formation_control/scripts \
python3 -m pytest homo_multirobot_formation_control/test -q
```

Expected: no whitespace errors; all Python tests pass.

## Plan Self-Review

- Spec coverage: Tasks 1–3 implement T2 in the proposal and Task 4 records its gate. T1 identification, T3 MIMO-ILF/ILKF LMI synthesis, and N1–N5 DDE simulation are deliberately outside this first plan because they depend on the T2 result.
- Placeholder scan: no unfinished placeholder items; each code task has a test, command, expected condition, and exact paths.
- Type consistency: `build_local_model` returns NumPy `(6,6)`/`(6,3)` matrices consumed by `controllability_diagnostics`; the scanner consumes the same function and serializes its result.
