# MPC Circular 3 m Offset Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add and run isolated ordinary/delay-aware circular Leader MPC experiments with a fixed map-frame 3 m Follower offset.

**Architecture:** Add one scenario factory with `offset_map=[-3,0,0]`, reuse the existing circle command and all MPC/plant parameters, and write two results to an independent ignored output directory. Existing 1 m circle scenarios and their result directory remain unchanged.

**Tech Stack:** Python 3, NumPy, CVXPY/OSQP, Matplotlib, pytest.

## Global Constraints

- The 3 m offset is fixed in map coordinates, not rotated by Leader yaw.
- Use Leader `R=2.0 m`, `Omega=0.25 rad/s`, actual plant delay 6 steps, existing weights and input constraints.
- Add ordinary model delay 0 and delay-aware model delay 6 only as new far-3 cases.
- Follow test-first development and save generated files in ignored `mpc_sim/results/circle_far3_20261021/`.

---

### Task 1: Add the far-3 circle scenario with a contract test

**Files:**
- Modify: `homo_multirobot_formation_control/mpc_sim/scenarios.py`
- Modify: `homo_multirobot_formation_control/mpc_sim/tests/test_simulation.py`

**Interfaces:**
- Produces `make_circle_far3_scenario(duration: float = 32.0) -> Scenario`.
- Returns `name="circle_far3"`, `offset_map=np.array([-3.0,0.0,0.0])`, and the unchanged circular Leader input.

- [ ] **Step 1: Write the failing offset and command-equivalence test**

```python
from mpc_sim.scenarios import make_circle_far3_scenario, make_circle_scenario

def test_far3_circle_uses_three_meter_map_offset_and_same_leader_circle():
    far = make_circle_far3_scenario()
    base = make_circle_scenario()
    assert far.name == "circle_far3"
    assert_allclose(far.offset_map, [-3.0, 0.0, 0.0])
    assert_allclose(far.leader_command_at(4.0), base.leader_command_at(4.0))
```

- [ ] **Step 2: Verify RED**

Run: `python3 -m pytest homo_multirobot_formation_control/mpc_sim/tests/test_simulation.py::test_far3_circle_uses_three_meter_map_offset_and_same_leader_circle -v`

Expected: import failure for `make_circle_far3_scenario`.

- [ ] **Step 3: Implement the minimal factory**

```python
def make_circle_far3_scenario(duration: float = 32.0) -> Scenario:
    base = make_circle_scenario(duration)
    return Scenario("circle_far3", duration, base.initial_error.copy(), np.array([-3.0, 0.0, 0.0]), base.leader_command_at)
```

- [ ] **Step 4: Verify GREEN**

Run: `python3 -m pytest homo_multirobot_formation_control/mpc_sim/tests/test_simulation.py -v`

Expected: all scenario tests pass.

### Task 2: Produce isolated far-3 results

**Files:**
- Generated, ignored: `homo_multirobot_formation_control/mpc_sim/results/circle_far3_20261021/`

**Interfaces:**
- Creates `circle_far3_ordinary_mpc/` using controller/plant delay `0/6`.
- Creates `circle_far3_delay_aware/` using controller/plant delay `6/6`.

- [ ] **Step 1: Execute two cases using existing `run_case`, `compute_metrics`, and `write_case_results`**

```python
for name, controller_delay in (("circle_far3_ordinary_mpc", 0), ("circle_far3_delay_aware", 6)):
    log = run_case(make_circle_far3_scenario(), controller_delay, 6, default_config())
    write_case_results(output / name, log, compute_metrics(log))
```

- [ ] **Step 2: Verify generated files and full regression suite**

Run:

```bash
python3 -m pytest homo_multirobot_formation_control/mpc_sim/tests -v
find homo_multirobot_formation_control/mpc_sim/results/circle_far3_20261021 -maxdepth 2 -type f | sort
```

Expected: tests pass; both directories contain CSV and three PNGs.

- [ ] **Step 3: Commit source and report actual metrics**

Run:

```bash
git diff --check
git add homo_multirobot_formation_control/mpc_sim
git commit -m "新增MPC圆周三米偏移实验"
```

Report RMSE, tail error, constraint violations, non-optimal solves, and the comparison to the 1 m baseline without altering either result set.
