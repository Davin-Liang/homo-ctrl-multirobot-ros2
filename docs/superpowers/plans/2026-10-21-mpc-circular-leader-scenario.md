# MPC Circular Leader Scenario Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a 2 m-radius, 0.25 rad/s map-frame circular Leader trajectory and run ordinary versus six-step delay-aware MPC comparisons.

**Architecture:** Extend only the scenario factory and the experiment case list. Reuse the existing delayed plant, QP, simulation runner, CSV writer, metric calculator and plotting functions unchanged; test the command analytically and verify generated artifacts using a short experiment run.

**Tech Stack:** Python 3, NumPy, CVXPY/OSQP, Matplotlib, pytest.

## Global Constraints

- Modify only `homo_multirobot_formation_control/mpc_sim/` and its tests; do not change ROS 2, Gazebo, existing controllers, or paper files.
- Set `R=2.0 m`, `Omega=0.25 rad/s`; use map-frame Leader input `[-R*Omega*sin(Omega*t), R*Omega*cos(Omega*t), Omega]`.
- Preserve map-frame follower offset `[-1, 0, 0]`, one-order actuator lag, and the actual input delay object.
- Add `circle_ordinary_mpc` with controller delay 0 and plant delay 6; add `circle_delay_aware` with both delays 6.
- Use test-first development and retain all generated experiment outputs under ignored `mpc_sim/results/`.

---

### Task 1: Add and test the circular scenario factory

**Files:**
- Modify: `homo_multirobot_formation_control/mpc_sim/scenarios.py`
- Modify: `homo_multirobot_formation_control/mpc_sim/tests/test_simulation.py`

**Interfaces:**
- Produces `make_circle_scenario(duration: float = 32.0) -> Scenario`.
- The returned scenario has `name == "circle"`, map-frame offset `[-1,0,0]`, and callable `leader_command_at(time: float) -> np.ndarray`.

- [ ] **Step 1: Write failing command-direction tests**

```python
from mpc_sim.scenarios import make_circle_scenario

def test_circle_scenario_uses_configured_map_frame_tangential_command():
    scenario = make_circle_scenario()
    assert scenario.name == "circle"
    assert_allclose(scenario.leader_command_at(0.0), [0.0, 0.5, 0.25])
    assert_allclose(
        scenario.leader_command_at(2.0 * np.pi), [-0.5, 0.0, 0.25], atol=1e-12
    )
```

- [ ] **Step 2: Verify the test fails because `make_circle_scenario` is absent**

Run: `python3 -m pytest homo_multirobot_formation_control/mpc_sim/tests/test_simulation.py::test_circle_scenario_uses_configured_map_frame_tangential_command -v`

Expected: import failure naming `make_circle_scenario`.

- [ ] **Step 3: Implement the factory**

```python
def make_circle_scenario(duration: float = 32.0) -> Scenario:
    radius, omega = 2.0, 0.25
    def command(time: float) -> np.ndarray:
        phase = omega * time
        return np.array([-radius * omega * np.sin(phase), radius * omega * np.cos(phase), omega])
    return Scenario("circle", duration, np.array([1.0, -0.6, 0.45, 0.0, 0.0, 0.0]), np.array([-1.0, 0.0, 0.0]), command)
```

- [ ] **Step 4: Run the focused scenario tests**

Run: `python3 -m pytest homo_multirobot_formation_control/mpc_sim/tests/test_simulation.py -v`

Expected: all scenario and closed-loop tests pass.

### Task 2: Include ordinary and delay-aware circle cases in experiment output

**Files:**
- Modify: `homo_multirobot_formation_control/mpc_sim/run_experiments.py`
- Modify: `homo_multirobot_formation_control/mpc_sim/tests/test_reporting.py`
- Modify: `homo_multirobot_formation_control/mpc_sim/README.md`

**Interfaces:**
- `run_all(output_dir, duration_scale)` writes `circle_ordinary_mpc/` and `circle_delay_aware/`, each containing `timeseries.csv`, `metrics.csv`, and three PNGs.

- [ ] **Step 1: Write a failing artifact test**

```python
from mpc_sim.run_experiments import run_all

def test_short_full_run_writes_both_circle_comparison_cases(tmp_path):
    run_all(tmp_path, duration_scale=0.02)
    for name in ("circle_ordinary_mpc", "circle_delay_aware"):
        assert (tmp_path / name / "trajectory.png").is_file()
        assert (tmp_path / name / "metrics.csv").is_file()
```

- [ ] **Step 2: Verify the test fails because the two directories are absent**

Run: `python3 -m pytest homo_multirobot_formation_control/mpc_sim/tests/test_reporting.py::test_short_full_run_writes_both_circle_comparison_cases -v`

Expected: assertion failure for `circle_ordinary_mpc`.

- [ ] **Step 3: Add two case tuples to `run_all` and document them**

```python
("circle_ordinary_mpc", make_circle_scenario(duration=32.0 * duration_scale), 0, 6, base),
("circle_delay_aware", make_circle_scenario(duration=32.0 * duration_scale), 6, 6, base),
```

Add “圆周 Leader（普通/延迟感知 MPC 对照）” to the README scenario list.

- [ ] **Step 4: Run the focused reporting test**

Run: `python3 -m pytest homo_multirobot_formation_control/mpc_sim/tests/test_reporting.py -v`

Expected: all reporting tests pass and create both circle case directories.

### Task 3: Run the circle comparison and hand off results

**Files:**
- Generated, ignored: `homo_multirobot_formation_control/mpc_sim/results/circle_<timestamp>/`

- [ ] **Step 1: Run package tests**

Run: `python3 -m pytest homo_multirobot_formation_control/mpc_sim/tests -v`

Expected: all tests pass.

- [ ] **Step 2: Generate the full circle comparison**

Run: `python3 homo_multirobot_formation_control/mpc_sim/run_experiments.py --output-dir homo_multirobot_formation_control/mpc_sim/results/circle_20261021`

Expected: `circle_ordinary_mpc/`, `circle_delay_aware/`, and `experiment_summary.md` exist.

- [ ] **Step 3: Inspect actual circle metrics and commit source changes**

Run:

```bash
sed -n '1,200p' homo_multirobot_formation_control/mpc_sim/results/circle_20261021/experiment_summary.md
git diff --check
git add homo_multirobot_formation_control/mpc_sim
git commit -m "新增MPC圆周领航数值场景"
```

Report actual ordinary/delay-aware circle metrics without inferring a conclusion before inspection.
