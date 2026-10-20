# Delay-Aware Constrained MPC Simulation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and run an isolated Python numerical experiment that compares a six-step delay-aware constrained Leader--Follower MPC against a delay-ignorant MPC on the same delayed, first-order velocity plant.

**Architecture:** `mpc_sim` is a standalone Python package under the formation-control package. Its model module owns exact ZOH and delay-line semantics; controller owns a parameterized CVXPY QP; scenarios produce known Leader inputs; simulation owns the closed-loop timeline and data; reporting writes all persistent results. Tests import only this package and do not require ROS 2.

**Tech Stack:** Python 3, NumPy, SciPy, CVXPY, OSQP, Matplotlib, pytest, CSV, Markdown.

## Global Constraints

- Create only `homo_multirobot_formation_control/mpc_sim/` and its tests/results-ignore entry; do not modify ROS 2 controllers, Gazebo, HPC, Artstein, or paper files.
- Use map-frame state `e=[ex, ey, e_theta, evx, evy, e_omega]` and fixed map-frame formation offsets.
- Use `h=0.05 s`, `Td_nominal=0.30 s`, therefore a nominal integer delay of exactly `d=6`.
- Use exact ZOH matrices `Ad=[[I,H],[0,E]]`, `Bd=[[hI-H],[I-E]]`, with `E=exp(-h T^-1)` and `H=T(I-E)`.
- Delay queue ordering is oldest-to-newest: `[delta_u[k-d], ..., delta_u[k-1]]`; the head applies during the next plant interval.
- Optimize `delta_u`; enforce amplitude and slew constraints on `u_f=u_l+delta_u`.
- The QP must stay convex; default solver is OSQP via CVXPY.
- Write a failing test and observe its intended failure before production code for every behavior.
- Outputs go to ignored `mpc_sim/results/`; failed or non-optimal scenarios remain recorded and reported.

---

### Task 1: Package configuration and exact discretization

**Files:**
- Create: `homo_multirobot_formation_control/mpc_sim/__init__.py`
- Create: `homo_multirobot_formation_control/mpc_sim/config.py`
- Create: `homo_multirobot_formation_control/mpc_sim/model.py`
- Create: `homo_multirobot_formation_control/mpc_sim/tests/test_model.py`
- Modify: `.gitignore`

**Interfaces:**
- Produces `PlantConfig(sample_time: float, tau: np.ndarray, delay_steps: int)` with validation.
- Produces `exact_zoh(config: PlantConfig) -> tuple[np.ndarray, np.ndarray]` returning `(Ad, Bd)` of shape `(6,6)` and `(6,3)`.
- Produces `DelayedPlant(config: PlantConfig, initial_error: np.ndarray, initial_input: np.ndarray)` with `step(command: np.ndarray) -> tuple[np.ndarray, np.ndarray]`, returning `(new_error, applied_command)`.

- [ ] **Step 1: Write the failing exact-ZOH test**

```python
def test_exact_zoh_matches_scalar_analytic_blocks():
    config = PlantConfig(0.05, np.array([0.4, 0.4, 0.2]), 6)
    ad, bd = exact_zoh(config)
    e = np.exp(-0.05 / 0.4)
    h = 0.4 * (1.0 - e)
    assert ad[0, 3] == pytest.approx(h)
    assert bd[0, 0] == pytest.approx(0.05 - h)
    assert ad[3, 3] == pytest.approx(e)
    assert bd[3, 0] == pytest.approx(1.0 - e)
```

- [ ] **Step 2: Run the test and verify it fails because the module does not exist**

Run: `pytest homo_multirobot_formation_control/mpc_sim/tests/test_model.py::test_exact_zoh_matches_scalar_analytic_blocks -v`

Expected: `ModuleNotFoundError: No module named 'mpc_sim'`.

- [ ] **Step 3: Add minimal configuration and exact-ZOH implementation**

```python
@dataclass(frozen=True)
class PlantConfig:
    sample_time: float
    tau: np.ndarray
    delay_steps: int

    def __post_init__(self):
        if self.sample_time <= 0 or np.any(np.asarray(self.tau) <= 0) or self.delay_steps < 0:
            raise ValueError("sample_time/tau must be positive and delay_steps non-negative")

def exact_zoh(config: PlantConfig) -> tuple[np.ndarray, np.ndarray]:
    h, tau = config.sample_time, np.asarray(config.tau, dtype=float)
    e = np.exp(-h / tau)
    H = tau * (1.0 - e)
    ad = np.block([[np.eye(3), np.diag(H)], [np.zeros((3, 3)), np.diag(e)]])
    bd = np.vstack((np.diag(h - H), np.diag(1.0 - e)))
    return ad, bd
```

- [ ] **Step 4: Run the test and verify it passes**

Run: `pytest homo_multirobot_formation_control/mpc_sim/tests/test_model.py::test_exact_zoh_matches_scalar_analytic_blocks -v`

Expected: `1 passed`.

- [ ] **Step 5: Write the failing continuous-reference test and queue-index test**

```python
def test_zoh_step_matches_scipy_augmented_exponential():
    config = PlantConfig(0.05, np.array([0.43, 0.43, 0.30]), 0)
    x0 = np.array([.3, -.1, .2, .4, -.2, .1])
    u0 = np.array([.1, .2, -.05])
    ad, bd = exact_zoh(config)
    assert_allclose(ad @ x0 + bd @ u0, continuous_zoh_reference(x0, u0, config))

def test_six_step_delay_applies_a_new_command_on_seventh_step_call():
    plant = DelayedPlant(PlantConfig(.05, np.ones(3), 6), np.zeros(6), np.zeros(3))
    for _ in range(6):
        _, applied = plant.step(np.array([1., 0., 0.]))
        assert_allclose(applied, np.zeros(3))
    _, applied = plant.step(np.array([1., 0., 0.]))
    assert_allclose(applied, np.array([1., 0., 0.]))
```

- [ ] **Step 6: Implement `continuous_zoh_reference` only in the test and the FIFO `DelayedPlant`**

```python
class DelayedPlant:
    def __init__(self, config, initial_error, initial_input):
        self.ad, self.bd = exact_zoh(config)
        self.error = np.asarray(initial_error, dtype=float).copy()
        self.queue = deque([np.asarray(initial_input, dtype=float).copy() for _ in range(config.delay_steps)])

    def step(self, command):
        command = np.asarray(command, dtype=float)
        if not self.queue:
            applied = command.copy()
        else:
            applied = self.queue.popleft()
            self.queue.append(command.copy())
        self.error = self.ad @ self.error + self.bd @ applied
        return self.error.copy(), applied
```

- [ ] **Step 7: Run the complete model tests**

Run: `pytest homo_multirobot_formation_control/mpc_sim/tests/test_model.py -v`

Expected: all tests pass; the six-step test demonstrates zero commands for calls 1--6 and first command application on call 7.

- [ ] **Step 8: Ignore generated experiment output and commit**

Add `homo_multirobot_formation_control/mpc_sim/results/` to `.gitignore`.

Run: `git add .gitignore homo_multirobot_formation_control/mpc_sim && git commit -m "新增MPC仿真离散延迟模型"`

### Task 2: Augmented prediction model and parameterized QP controller

**Files:**
- Create: `homo_multirobot_formation_control/mpc_sim/controller.py`
- Create: `homo_multirobot_formation_control/mpc_sim/tests/test_controller.py`
- Modify: `homo_multirobot_formation_control/mpc_sim/config.py`

**Interfaces:**
- Produces `MpcConfig(horizon, q, q_terminal, r_delta, r_slew, input_min, input_max, delta_min, delta_max, model_delay_steps)`.
- Produces `augment_delay_model(ad, bd, delay_steps) -> tuple[np.ndarray, np.ndarray]` with `z=[e; queue]` and dimension `6+3*delay_steps`.
- Produces `DelayAwareMpc(plant_config, mpc_config).solve(error, queued_relative_inputs, leader_commands, previous_follower_command) -> MpcResult`.
- `MpcResult` has `relative_command`, `follower_command`, `status`, `solve_seconds`, and `predicted_errors`.

- [ ] **Step 1: Write the failing augmented-row test**

```python
def test_augmented_model_uses_oldest_queued_input_in_first_state_update():
    ad, bd = exact_zoh(PlantConfig(.05, np.ones(3), 2))
    az, bz = augment_delay_model(ad, bd, 2)
    error = np.zeros(6)
    old, newest = np.array([1., 2., 3.]), np.array([4., 5., 6.])
    z = np.r_[error, old, newest]
    z_next = az @ z + bz @ np.array([7., 8., 9.])
    assert_allclose(z_next[:6], bd @ old)
    assert_allclose(z_next[6:9], newest)
    assert_allclose(z_next[9:12], [7., 8., 9.])
```

- [ ] **Step 2: Verify the test fails**

Run: `pytest homo_multirobot_formation_control/mpc_sim/tests/test_controller.py::test_augmented_model_uses_oldest_queued_input_in_first_state_update -v`

Expected: import failure for `augment_delay_model`.

- [ ] **Step 3: Implement the explicit augmented transition**

```python
def augment_delay_model(ad, bd, delay_steps):
    n = 6 + 3 * delay_steps
    az, bz = np.zeros((n, n)), np.zeros((n, 3))
    az[:6, :6] = ad
    if delay_steps == 0:
        bz[:6] = bd
        return az, bz
    az[:6, 6:9] = bd
    for i in range(delay_steps - 1):
        az[6 + 3*i:9 + 3*i, 9 + 3*i:12 + 3*i] = np.eye(3)
    bz[-3:] = np.eye(3)
    return az, bz
```

- [ ] **Step 4: Run the augmented-row test**

Run: `pytest homo_multirobot_formation_control/mpc_sim/tests/test_controller.py::test_augmented_model_uses_oldest_queued_input_in_first_state_update -v`

Expected: `1 passed`.

- [ ] **Step 5: Write the failing QP zero-equilibrium and follower-constraint tests**

```python
def test_zero_error_and_zero_leader_returns_zero_follower_command():
    controller = make_controller(delay_steps=6)
    result = controller.solve(np.zeros(6), np.zeros((6, 3)), np.zeros((20, 3)), np.zeros(3))
    assert result.status in {"optimal", "optimal_inaccurate"}
    assert_allclose(result.follower_command, np.zeros(3), atol=1e-7)

def test_constraints_apply_to_follower_not_relative_command():
    controller = make_controller(delay_steps=0, input_min=[-.2]*3, input_max=[.2]*3)
    leader = np.tile([.5, 0., 0.], (20, 1))
    result = controller.solve(np.zeros(6), np.empty((0, 3)), leader, np.zeros(3))
    assert result.follower_command[0] == pytest.approx(.2, abs=1e-5)
```

- [ ] **Step 6: Implement the CVXPY QP**

Use decision variables `relative = cp.Variable((N,3))`, `z = cp.Variable((N+1,nz))`; constrain `z[0]` to the supplied augmented state and `z[j+1] == Az @ z[j] + Bz @ relative[j]`. Define `follower = leader_commands + relative`; constrain every row to `input_min/input_max`; constrain `follower[0]-previous_follower_command` and all successive row differences to `delta_min/delta_max`. Sum state quadratic costs over `z[j,:6]`, terminal cost over `z[N,:6]`, and command/slew costs; solve with `solver=cp.OSQP`, `warm_start=True`.

- [ ] **Step 7: Run all controller tests**

Run: `pytest homo_multirobot_formation_control/mpc_sim/tests/test_controller.py -v`

Expected: all tests pass and every returned command satisfies the follower constraints.

- [ ] **Step 8: Commit**

Run: `git add homo_multirobot_formation_control/mpc_sim && git commit -m "新增延迟感知MPC二次规划控制器"`

### Task 3: Scenarios and closed-loop simulator

**Files:**
- Create: `homo_multirobot_formation_control/mpc_sim/scenarios.py`
- Create: `homo_multirobot_formation_control/mpc_sim/simulation.py`
- Create: `homo_multirobot_formation_control/mpc_sim/tests/test_simulation.py`

**Interfaces:**
- Produces `Scenario(name, duration, initial_error, leader_command_at, offset_map)`.
- Produces `run_case(scenario, controller_delay_steps, plant_delay_steps, config) -> SimulationLog`.
- `SimulationLog` records time, leader/follower states, errors, sent/applied inputs, QP status and solve time.

- [ ] **Step 1: Write failing scenarios and zero-invariance integration test**

```python
def test_scenarios_expose_static_straight_and_curved_leader_inputs():
    assert make_static_scenario().leader_command_at(0.0).shape == (3,)
    assert make_straight_scenario().leader_command_at(1.0)[0] > 0.0
    assert abs(make_curve_scenario().leader_command_at(1.0)[2]) > 0.0

def test_zero_state_is_invariant_for_delay_aware_closed_loop():
    scenario = make_static_scenario(initial_error=np.zeros(6))
    log = run_case(scenario, controller_delay_steps=6, plant_delay_steps=6, config=default_config())
    assert_allclose(log.error, 0.0, atol=1e-7)
    assert set(log.statuses) <= {"optimal", "optimal_inaccurate"}
```

- [ ] **Step 2: Verify tests fail**

Run: `pytest homo_multirobot_formation_control/mpc_sim/tests/test_simulation.py -v`

Expected: imports fail because scenarios/simulation are absent.

- [ ] **Step 3: Implement scenario generators and the timeline exactly once per sample**

At each index `k`: obtain `u_l[k:k+N]`; solve using current error and relative queue; compute and record `u_f[k]`; step Leader and Follower plants with their respective commands; derive map-frame formation error from their independent states; update the controller queue with `u_f[k]-u_l[k]`. Do not regenerate error through a second dynamics path.

- [ ] **Step 4: Run simulation tests**

Run: `pytest homo_multirobot_formation_control/mpc_sim/tests/test_simulation.py -v`

Expected: all tests pass, including zero equilibrium for a six-step delayed object.

- [ ] **Step 5: Add constraint postcondition test and commit**

```python
def test_recorded_sent_follower_commands_obey_amplitude_and_slew_limits():
    log = run_case(make_straight_scenario(), 6, 6, strict_default_config())
    assert np.all(log.sent_follower <= log.input_max + 1e-8)
    assert np.all(log.sent_follower >= log.input_min - 1e-8)
    assert np.all(np.abs(np.diff(log.sent_follower, axis=0)) <= log.delta_max + 1e-8)
```

Run: `pytest homo_multirobot_formation_control/mpc_sim/tests/test_simulation.py -v && git add homo_multirobot_formation_control/mpc_sim && git commit -m "新增MPC编队闭环数值场景"`

### Task 4: Result writers, plots, metrics, and CLI

**Files:**
- Create: `homo_multirobot_formation_control/mpc_sim/reporting.py`
- Create: `homo_multirobot_formation_control/mpc_sim/run_experiments.py`
- Create: `homo_multirobot_formation_control/mpc_sim/tests/test_reporting.py`

**Interfaces:**
- Produces `compute_metrics(log) -> dict[str, float]` including position RMSE, maximum error, tail mean, violations and solve-time statistics.
- Produces `write_case_results(output_dir, log) -> None` with `timeseries.csv`, `metrics.csv`, and images.
- Produces `run_all(output_dir) -> Path` with all requested comparisons and `experiment_summary.md`.

- [ ] **Step 1: Write the failing metric and CSV schema test**

```python
def test_metrics_and_csv_include_required_delay_experiment_fields(tmp_path):
    log = synthetic_log_for_reporting()
    metrics = compute_metrics(log)
    assert {"position_rmse", "position_max", "position_tail_mean", "constraint_violations", "solve_ms_mean"} <= metrics.keys()
    write_case_results(tmp_path, log)
    columns = set(pd.read_csv(tmp_path / "timeseries.csv").columns)
    assert {"time", "ex", "ey", "e_theta", "uf_x", "uf_y", "uf_w", "solve_ms", "qp_status"} <= columns
```

- [ ] **Step 2: Verify the test fails**

Run: `pytest homo_multirobot_formation_control/mpc_sim/tests/test_reporting.py -v`

Expected: import failure for reporting APIs.

- [ ] **Step 3: Implement writers without pandas runtime dependency**

Use `csv.DictWriter` for files, Matplotlib Agg backend for trajectory/error/input/solve-time figures, and `numpy` for metrics. `experiment_summary.md` must render a scenario-by-controller table including non-optimal status count and mismatch setting.

- [ ] **Step 4: Run the reporting tests**

Run: `pytest homo_multirobot_formation_control/mpc_sim/tests/test_reporting.py -v`

Expected: `1 passed` and files exist under the temporary directory.

- [ ] **Step 5: Implement the CLI and run its smoke test**

```bash
python3 homo_multirobot_formation_control/mpc_sim/run_experiments.py \
  --duration 1.0 --output-dir /tmp/mpc_smoke
test -f /tmp/mpc_smoke/experiment_summary.md
```

Expected: process exits zero, all cases write an explicit status, including any failed QP.

- [ ] **Step 6: Commit**

Run: `git add homo_multirobot_formation_control/mpc_sim && git commit -m "新增MPC仿真结果与报告输出"`

### Task 5: Full experiment, README, and verification handoff

**Files:**
- Create: `homo_multirobot_formation_control/mpc_sim/README.md`
- Modify: `homo_multirobot_formation_control/mpc_sim/run_experiments.py` only if full-run diagnostics expose a real defect.

**Interfaces:**
- README specifies dependency check, exact command, output tree, scenario meanings and limits of the nominal model.

- [ ] **Step 1: Write the failing README contract test**

```python
def test_readme_documents_reproducible_full_experiment_command():
    text = (Path(__file__).parents[1] / "README.md").read_text()
    assert "run_experiments.py" in text
    assert "0.30" in text
    assert "6" in text
    assert "CVXPY" in text
```

- [ ] **Step 2: Verify it fails, then write the README**

Run: `pytest homo_multirobot_formation_control/mpc_sim/tests/test_readme.py -v`

Expected before documentation: missing file; afterward: `1 passed`.

The README must include:

```bash
python3 -c 'import numpy, scipy, cvxpy, osqp, matplotlib; print("dependencies ready")'
python3 homo_multirobot_formation_control/mpc_sim/run_experiments.py
pytest homo_multirobot_formation_control/mpc_sim/tests -v
```

- [ ] **Step 3: Run all package tests**

Run: `pytest homo_multirobot_formation_control/mpc_sim/tests -v`

Expected: all pass.

- [ ] **Step 4: Execute every requested numerical scenario**

Run: `python3 homo_multirobot_formation_control/mpc_sim/run_experiments.py --output-dir /tmp/delay_aware_mpc_full`

Expected: outputs for static, straight, curve, strict-constraints, ordinary-vs-delay-aware, and actual delay `4/6/8` are written, with no hidden failures.

- [ ] **Step 5: Inspect results and preserve evidence**

Run:

```bash
find /tmp/delay_aware_mpc_full -maxdepth 2 -type f | sort
sed -n '1,240p' /tmp/delay_aware_mpc_full/experiment_summary.md
```

Report actual metrics only. If delay-aware MPC is not better for a scenario, retain that row and state why it may occur.

- [ ] **Step 6: Final checks and commit**

Run:

```bash
git diff --check
pytest homo_multirobot_formation_control/mpc_sim/tests -v
git status --short
git add homo_multirobot_formation_control/mpc_sim .gitignore
git commit -m "完成延迟感知MPC数值仿真验证"
```

Expected: no whitespace errors, tests pass, generated results remain untracked/ignored, and only simulation implementation/docs are committed.
