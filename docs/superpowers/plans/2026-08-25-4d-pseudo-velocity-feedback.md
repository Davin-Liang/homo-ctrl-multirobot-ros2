# 4D Pseudo Velocity Feedback Ablation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add and run a numerical ablation in which the 4D Artstein controller receives its existing predicted position and the previous final velocity command as a deliberately pseudo velocity feedback.

**Architecture:** Extend `simulate_delay_case()` in the existing 4D comparison script with one explicit `pseudo_velocity_feedback` mode. The delayed first-order plant remains unchanged; only the state supplied to `Hpc4D.accel()` changes from `[p_pred, v_pred]` to `[p_pred, last_cmd]`. Add the mode to the existing default comparison runs and report its metrics with an unambiguous label.

**Tech Stack:** Python 3, NumPy, SciPy, Matplotlib, existing `Hpc4D` numerical simulator.

## Global Constraints

- Modify only `homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py` and its directly related documentation/output.
- The plant must keep its current delay queue and first-order update based on the true state `x2[2:4]`.
- `last_cmd` must be the previous final clipped map-frame velocity command.
- Do not describe the new group as ideal execution or true velocity feedback.
- Existing `original` and `compensated` modes must retain their current behavior.

---

### Task 1: Specify and test the pseudo-feedback state selection

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py:332-376`
- Test: `homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py`

**Interfaces:**
- Consumes: `predict_follower_state_from_artstein(z, last_cmd, tau, Td) -> np.ndarray`.
- Produces: `simulate_delay_case("pseudo_velocity_feedback", ...) -> rows` with the same eight-field row layout used by delayed cases.

- [ ] **Step 1: Add a failing assertion for the requested feedback state**

Add a local helper:

```python
def pseudo_velocity_feedback_state(predicted: np.ndarray, last_cmd: np.ndarray) -> np.ndarray:
    state = predicted.copy()
    state[2:4] = last_cmd
    return state
```

Before adding the helper, run:

```bash
python3 - <<'PY'
import runpy
module = runpy.run_path('homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py')
assert 'pseudo_velocity_feedback_state' in module
PY
```

Expected: `AssertionError`.

- [ ] **Step 2: Implement the minimal helper and mode branch**

Add the helper above. In `simulate_delay_case()`, make `kind == "pseudo_velocity_feedback"` calculate the same Artstein prediction as `"compensated"`, then replace only `x2_ctrl[2:4]` with `last_cmd`. Keep the existing delayed plant update unchanged.

- [ ] **Step 3: Verify state selection and unchanged plant behavior**

Run:

```bash
python3 - <<'PY'
import runpy
import numpy as np
module = runpy.run_path('homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py')
state = module['pseudo_velocity_feedback_state'](
    np.array([1.0, 2.0, 3.0, 4.0]), np.array([5.0, 6.0]))
assert np.allclose(state, [1.0, 2.0, 5.0, 6.0])
PY
```

Expected: exit code 0.

### Task 2: Run the ablation and expose comparable output

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py:580-610`
- Modify: `homo_multirobot_formation_control/doc/4d_artstein_prediction_simulation.md`
- Create: `homo_multirobot_formation_control/analysis/results/4d_artstein_pseudo_velocity_feedback/*`

**Interfaces:**
- Consumes: delayed rows returned by all three modes.
- Produces: saved figures/CSV or JSON data and a concise console metric comparison.

- [ ] **Step 1: Add pseudo-feedback runs for existing default step and circle cases**

Call `simulate_delay_case("pseudo_velocity_feedback", ...)` and `simulate_circle_case("pseudo_velocity_feedback", ...)` with exactly the same `Tmax`, `h`, `tau`, and `Td` passed to the compensated cases. Add the group to output data using the key `pseudo_velocity_feedback`.

- [ ] **Step 2: Generate and inspect results**

Run:

```bash
MPLBACKEND=Agg python3 homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py
```

Expected: exit code 0; existing result files plus clearly labelled pseudo-feedback results are generated.

- [ ] **Step 3: Document the group honestly**

Add a short subsection to `homo_multirobot_formation_control/doc/4d_artstein_prediction_simulation.md` stating that this is an ablation with predicted position and previous velocity command supplied only to the HPC feedback state while the plant retains true delayed dynamics. Do not characterize its result as physical delay compensation.

### Task 3: Regression verification and handoff

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py`
- Modify: `homo_multirobot_formation_control/doc/4d_artstein_prediction_simulation.md`

**Interfaces:**
- Consumes: all three simulation modes and generated output.
- Produces: evidence that original and compensated baseline output remain valid and a reported metric table.

- [ ] **Step 1: Check all mode row layouts**

```bash
python3 - <<'PY'
import runpy
module = runpy.run_path('homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py')
for kind in ('original', 'compensated', 'pseudo_velocity_feedback'):
    rows = module['simulate_delay_case'](kind, 1.0, 0.01, 0.43, 0.22)
    assert rows and all(len(row) == 8 for row in rows), kind
PY
```

Expected: exit code 0.

- [ ] **Step 2: Run syntax and whitespace validation**

```bash
python3 -m py_compile homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py
git diff --check
```

Expected: both commands exit 0.

- [ ] **Step 3: Commit the scoped implementation**

```bash
git add homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py \
  homo_multirobot_formation_control/doc/4d_artstein_prediction_simulation.md
git commit -m "新增4D伪速度反馈消融"
```
