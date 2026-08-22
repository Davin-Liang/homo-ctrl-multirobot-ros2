# 6D Artstein HOCBF Numerical Feasibility Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Build a deterministic offline oracle simulation that tests whether a predictor-state, hard-constraint HOCBF filter keeps a delayed first-order omnidirectional plant outside static circular obstacles.

**Architecture:** Add a standalone Python module, independent of ROS and the current soft-penalty ObstacleAvoider. It uses exact ZOH matrices for the two-dimensional actuator model, a finite-history predictor, and finite active-set enumeration to exactly solve the 2D strictly convex HOCBF-QP without a new dependency. Commands update at 20 Hz while the plant and delay queue integrate at 10 ms, so the model represents Td=0.22 s exactly. The scenario runner reports safety and infeasibility metrics.

**Tech Stack:** Python 3, NumPy, SciPy scipy.linalg.expm, pytest, standard-library CSV.

## Global Constraints

- Scope is static circular map-frame obstacles, known constant tau, and known constant Td only.
- Do not modify C++, launch, CMake, Gazebo, or the current ObstacleAvoider.
- Store final published map-frame commands in predictor history, never nominal commands.
- Do not add a safety slack variable. Report QP infeasibility and use deterministic braking.
- Results are model-level feasibility evidence, not a 20 Hz formal safety proof.
- Use control_dt=0.05 s and integration_dt=0.01 s for the primary scan; the delay must be an integer multiple of integration_dt.
- CSV output must use LF line endings.

---

### Task 1: Exact actuator model, predictor, and HOCBF half-space

**Files:**
- Create: homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py
- Create: homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py

**Interfaces:**
- PlantParams(tau: float, delay: float, integration_dt: float, control_dt: float)
- zoh_matrices(params: PlantParams) -> tuple[np.ndarray, np.ndarray]
- predict_delayed_state(x, queued_commands, ad, bd) -> np.ndarray
- hocbf_halfspace(x_pred, obstacle, safe_radius, tau, c1, c2) -> tuple[np.ndarray, float, float, float]

- [ ] **Step 1: Write the failing tests**

    def test_exact_predictor_matches_future_delayed_zoh_state():
        params = module.PlantParams(tau=0.4, delay=0.1, dt=0.05)
        ad, bd = module.zoh_matrices(params)
        x0 = np.array([0.0, 0.0, 0.2, -0.1])
        queued = [np.array([0.3, 0.0]), np.array([0.1, -0.2])]
        actual = module.predict_delayed_state(x0, queued, ad, bd)
        expected = ad @ (ad @ x0 + bd @ queued[0]) + bd @ queued[1]
        np.testing.assert_allclose(actual, expected, atol=1e-12)

    def test_hocbf_halfspace_tightens_head_on_approach():
        a, b, h, psi1 = module.hocbf_halfspace(
            np.array([2.0, 0.0, -0.5, 0.0]), np.zeros(2),
            safe_radius=1.0, tau=0.5, c1=1.0, c2=1.0)
        np.testing.assert_allclose(a, [8.0, 0.0])
        assert h == pytest.approx(3.0)
        assert psi1 == pytest.approx(1.0)
        assert a @ np.array([-1.0, 0.0]) < b
        assert a @ np.zeros(2) >= b

- [ ] **Step 2: Run the tests to verify they fail**

Run: pytest -q homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py -k 'predictor or halfspace'

Expected: collection fails because the module does not exist.

- [ ] **Step 3: Write minimal implementation**

Use the augmented matrix exponential to calculate Ad and Bd at integration_dt. Require delay / integration_dt and control_dt / integration_dt to be integers. Starting from x, apply each queued command in time order as x <- Ad x + Bd u. For predicted position p, velocity v, and r=p-obstacle, return:

    h = r @ r - safe_radius**2
    psi1 = 2 * r @ v + c1 * h
    a = 2 * r / tau
    b = -2 * v @ v + 2 * r @ v / tau - 2 * c1 * r @ v - c2 * psi1

- [ ] **Step 4: Run tests to verify they pass**

Run: pytest -q homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py -k 'predictor or halfspace'

Expected: 2 passed.

- [ ] **Step 5: Commit**

Run:

    git add homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py
    git commit -m "新增HOCBF数值模型与约束"

### Task 2: Exact 2D hard-QP solver

**Files:**
- Modify: homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py
- Modify: homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py

**Interfaces:**
- SafetyFilterResult(command: np.ndarray, feasible: bool, active_constraints: int)
- solve_hocbf_qp(u_nom, u_prev, halfspaces, vmax, amax, dt) -> SafetyFilterResult

- [ ] **Step 1: Write the failing tests**

    def test_hard_qp_returns_nominal_command_when_feasible():
        result = module.solve_hocbf_qp(
            np.array([0.3, -0.2]), np.zeros(2),
            [(np.array([1.0, 0.0]), -1.0)], 1.0, 20.0, 0.05)
        assert result.feasible
        np.testing.assert_allclose(result.command, [0.3, -0.2])

    def test_hard_qp_projects_nominal_command_onto_barrier():
        result = module.solve_hocbf_qp(
            np.array([-1.0, 0.0]), np.zeros(2),
            [(np.array([1.0, 0.0]), 0.2)], 1.0, 20.0, 0.05)
        assert result.feasible
        np.testing.assert_allclose(result.command, [0.2, 0.0], atol=1e-12)

    def test_hard_qp_reports_conflicting_barriers_as_infeasible():
        result = module.solve_hocbf_qp(
            np.zeros(2), np.zeros(2),
            [(np.array([1.0, 0.0]), 0.8), (np.array([-1.0, 0.0]), 0.8)],
            1.0, 20.0, 0.05)
        assert not result.feasible

- [ ] **Step 2: Run failing tests**

Run: pytest -q homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py -k hard_qp

Expected: AttributeError because solve_hocbf_qp is absent.

- [ ] **Step 3: Write minimal implementation**

Convert speed and rate bounds to four half-spaces. Generate all valid candidates: u_nom, its orthogonal projection to each half-space boundary, and intersections of each nonparallel boundary pair. Retain candidates satisfying every inequality a @ u >= b within 1e-10. Return the candidate minimizing 0.5 * norm(u-u_nom)**2. Return feasible=False only if none remain; count rows with residual below 1e-9 as active.

- [ ] **Step 4: Run tests to verify pass**

Run: pytest -q homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py

Expected: all tests pass.

- [ ] **Step 5: Commit**

Run:

    git add homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py
    git commit -m "实现HOCBF硬约束二维求解"

### Task 3: Delayed scenario runner and braking contract

**Files:**
- Modify: homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py
- Modify: homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py

**Interfaces:**
- ScenarioConfig(plant, obstacle, safe_radius, initial_state, nominal_command, vmax, amax, c1, c2, duration, predictor_delay: float | None = None)
- simulate_scenario(config: ScenarioConfig) -> dict[str, np.ndarray]

- [ ] **Step 1: Write the failing tests**

    def test_no_obstacle_scenario_preserves_nominal_command():
        result = module.simulate_scenario(module.ScenarioConfig(
            plant=module.PlantParams(0.43, 0.2, 0.05),
            obstacle=np.array([100.0, 0.0]), safe_radius=0.5,
            initial_state=np.zeros(4), nominal_command=np.array([0.2, 0.0]),
            vmax=1.0, amax=20.0, c1=1.0, c2=1.0, duration=0.5))
        assert not result["braking"].any()
        np.testing.assert_allclose(result["command"][1:], [0.2, 0.0])

    def test_head_on_feasible_case_keeps_h_nonnegative():
        result = module.simulate_scenario(module.ScenarioConfig(
            plant=module.PlantParams(0.43, 0.2, 0.05),
            obstacle=np.zeros(2), safe_radius=0.8,
            initial_state=np.array([2.0, 0.0, -0.1, 0.0]),
            nominal_command=np.array([-0.8, 0.0]),
            vmax=1.0, amax=20.0, c1=2.0, c2=2.0, duration=4.0))
        assert result["feasible"].all()
        assert result["h"].min() >= -1e-9

- [ ] **Step 2: Run failing tests**

Run: pytest -q homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py -k scenario

Expected: AttributeError because ScenarioConfig and simulate_scenario are absent.

- [ ] **Step 3: Write minimal implementation**

At every 50 ms sample, first predict through the complete high-rate queue, including the command that acts during the current interval; this produces the state at which the new command first acts. Use predictor_delay when it is provided, otherwise plant.delay. Construct the one-obstacle HOCBF row and solve Task 2 with control_dt. Then apply the final command over control_dt by repeatedly popping the oldest 10 ms queued command, appending the final command, and advancing the plant at integration_dt. On infeasibility, issue the componentwise rate-limited command toward zero, mark braking=True, and still append that final command to history. Return control-rate arrays time, state, command, h, psi1, psi2, feasible, braking and high-rate arrays time_internal, state_internal, h_internal.

- [ ] **Step 4: Run tests to verify pass**

Run: pytest -q homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py

Expected: all tests pass.

- [ ] **Step 5: Commit**

Run:

    git add homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py
    git commit -m "新增延迟HOCBF场景仿真"

### Task 4: Envelope scan, CSV, and result guide

**Files:**
- Modify: homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py
- Modify: homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py
- Create: homo_multirobot_formation_control/analysis/results/6d_artstein_hocbf_feasibility/README.md

**Interfaces:**
- scan_envelope(tau_values, delay_values, clearances, delay_mismatches) -> list[dict[str, float | int]]
- write_metrics_csv(rows, output) -> None

- [ ] **Step 1: Write the failing tests**

    def test_scan_envelope_has_one_row_per_parameter_combination():
        rows = module.scan_envelope(
            tau_values=[0.3, 0.5], delay_values=[0.0],
            clearances=[1.0], delay_mismatches=[0.0, 0.05])
        assert len(rows) == 4
        assert set(rows[0]) >= {
            "tau", "delay_model", "delay_actual", "initial_clearance",
            "min_h", "min_distance", "min_psi2",
            "infeasible_steps", "braking_steps"}

    def test_metrics_csv_has_lf_and_expected_header(tmp_path):
        output = tmp_path / "scan.csv"
        module.write_metrics_csv([{
            "tau": 0.43, "delay_model": 0.2, "delay_actual": 0.2,
            "initial_clearance": 1.0, "min_h": 0.1, "min_distance": 0.6,
            "min_psi2": 0.0, "max_command_norm": 0.5,
            "infeasible_steps": 0, "braking_steps": 0}], output)
        assert output.read_text().splitlines()[0].startswith(
            "tau,delay_model,delay_actual,initial_clearance")
        assert b"\r\n" not in output.read_bytes()

- [ ] **Step 2: Run failing tests**

Run: pytest -q homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py -k 'scan_envelope or metrics_csv'

Expected: AttributeError because scan and CSV functions are absent.

- [ ] **Step 3: Write minimal implementation**

Scan the Cartesian product with the Task 3 head-on scenario. Set plant.delay=delay_actual=delay_model+delay_mismatch and predictor_delay=delay_model, so mismatch is an explicit plant-versus-predictor model violation. Emit tau, both delays, initial clearance, min_h, min_distance, min_psi2, max_command_norm, infeasible_steps, and braking_steps. Use csv.DictWriter with lineterminator="\n". The README must state that only rows with zero infeasible and braking steps and min_h >= 0 support the exact-model feasible-case claim.

- [ ] **Step 4: Run tests and generate evidence**

Run:

    pytest -q homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py
    python3 homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py --output homo_multirobot_formation_control/analysis/results/6d_artstein_hocbf_feasibility/scan.csv

Expected: all tests pass and the CLI writes a nonempty CSV with the stated header.

- [ ] **Step 5: Commit**

Run:

    git add homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py homo_multirobot_formation_control/analysis/results/6d_artstein_hocbf_feasibility
    git commit -m "增加HOCBF数值可行性扫描"
