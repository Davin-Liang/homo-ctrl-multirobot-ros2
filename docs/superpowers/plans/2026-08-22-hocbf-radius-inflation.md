# HOCBF Radius Inflation Search Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Determine the smallest scanned internal HOCBF radius inflation that restores base-radius safety in selected actuator and delay mismatch scenarios.

**Architecture:** Reuse ScenarioConfig and compare_sampling_rates. For each original scenario, hold the physical base radius fixed at 0.8 m, replace only the filter radius by base plus each grid candidate, and measure both 20 Hz and 1 kHz physical minimum distances against the unchanged base radius. Return every candidate evaluation and a per-scenario selected minimum; do not infer monotonicity.

**Tech Stack:** Python 3, NumPy, SciPy, pytest, standard-library CSV.

## Global Constraints

- The output is an empirical result for static circular Oracle scenarios, not a robust CBF proof.
- Candidates are 0 through 0.03 m in 0.001 m increments.
- A candidate is safe only if its 20 Hz and 1 kHz minimum distances meet the base radius and the 20 Hz QP has zero infeasible steps.
- Preserve scenarios without a qualifying candidate; never replace them with an averaged value.
- Do not modify ROS, Gazebo, C++, launch files, or perception code.

---

### Task 1: Candidate evaluation and per-scenario radius search

**Files:**
- Modify: homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py
- Modify: homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py

**Interfaces:**
- evaluate_radius_inflation(config: ScenarioConfig, base_radius: float, inflation: float) -> dict[str, float | int | bool]
- find_required_radius_inflation(config: ScenarioConfig, base_radius: float, candidates: list[float]) -> dict[str, float | int | bool]

- [ ] **Step 1: Write the failing tests**

    def make_known_failed_mismatch(module):
        return module.ScenarioConfig(
            plant=module.PlantParams(0.55, 0.22, 0.01, 0.05),
            predictor_tau=0.66, predictor_delay=0.22,
            obstacle=np.zeros(2), safe_radius=0.8,
            initial_state=np.array([1.2, 0.0, -0.5, 0.2]),
            nominal_command=np.array([-0.8, 0.0]),
            vmax=1.0, amax=20.0, c1=2.0, c2=2.0, duration=4.0)

    def make_exact_safe_case(module):
        return module.ScenarioConfig(
            plant=module.PlantParams(0.55, 0.22, 0.01, 0.05),
            predictor_tau=0.55, predictor_delay=0.22,
            obstacle=np.zeros(2), safe_radius=0.8,
            initial_state=np.array([1.2, 0.0, -0.5, 0.0]),
            nominal_command=np.array([-0.8, 0.0]),
            vmax=1.0, amax=20.0, c1=2.0, c2=2.0, duration=4.0)

    def test_known_tau_mismatch_requires_positive_radius_inflation():
        config = make_known_failed_mismatch(module)
        result = module.find_required_radius_inflation(
            config, base_radius=0.8, candidates=[0.0, 0.005, 0.010])
        assert result["baseline_safe"] is False
        assert result["found"] is True
        assert result["required_inflation"] > 0.0

    def test_exact_model_safe_case_allows_zero_radius_inflation():
        config = make_exact_safe_case(module)
        result = module.find_required_radius_inflation(
            config, base_radius=0.8, candidates=[0.0, 0.005])
        assert result["baseline_safe"] is True
        assert result["found"] is True
        assert result["required_inflation"] == pytest.approx(0.0)

- [ ] **Step 2: Run the tests to verify they fail**

Run: python3 -m pytest -q homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py -k radius_inflation

Expected: AttributeError because the two search functions are absent.

- [ ] **Step 3: Write minimal implementation**

For each candidate in increasing order, create a copy of config with safe_radius=base_radius+inflation and run simulate_scenario. Record its 20 Hz distance, h, infeasibility and braking. Run compare_sampling_rates for zero inflation and every candidate whose 20 Hz distance is at least base_radius minus 1e-9 with zero infeasible steps; a candidate that already fails this necessary 20 Hz condition cannot satisfy the final dual-rate criterion. Set safe only when both available distances are at least base_radius minus 1e-9 and infeasible_steps is zero. The first safe candidate is the smallest on the ordered discrete grid, so stop after recording it. Return baseline metrics from the zero-inflation candidate even if zero was not supplied.

- [ ] **Step 4: Run focused tests**

Run: python3 -m pytest -q homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py -k radius_inflation

Expected: 2 passed.

- [ ] **Step 5: Commit**

Run:

    git add homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py
    git commit -m "增加HOCBF半径膨胀搜索"

### Task 2: Selected scenario set, CSV, summary, and documentation

**Files:**
- Modify: homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py
- Modify: homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py
- Modify: homo_multirobot_formation_control/analysis/results/6d_artstein_hocbf_feasibility/README.md
- Create: homo_multirobot_formation_control/analysis/results/6d_artstein_hocbf_feasibility/radius_inflation.csv
- Create: homo_multirobot_formation_control/analysis/results/6d_artstein_hocbf_feasibility/radius_inflation_summary.csv

**Interfaces:**
- run_radius_inflation_cases(cases: list[ScenarioConfig], base_radius: float, candidates: list[float]) -> list[dict[str, float | int | bool]]
- summarize_radius_inflation(rows) -> list[dict[str, float | int | str]]

- [ ] **Step 1: Write the failing tests**

    def test_radius_inflation_summary_keeps_unresolved_case():
        rows = [
            {"exact_model": 1, "found": True, "required_inflation": 0.0},
            {"exact_model": 0, "found": True, "required_inflation": 0.006},
            {"exact_model": 0, "found": False, "required_inflation": float("nan")},
        ]
        summary = module.summarize_radius_inflation(rows)
        mismatch = next(row for row in summary if row["group"] == "mismatch")
        assert mismatch["scenario_count"] == 2
        assert mismatch["unresolved_count"] == 1
        assert mismatch["max_required_inflation"] == pytest.approx(0.006)

- [ ] **Step 2: Run the test to verify it fails**

Run: python3 -m pytest -q homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py -k radius_inflation_summary

Expected: AttributeError because summarize_radius_inflation is absent.

- [ ] **Step 3: Write minimal implementation**

Build the default case set from the known tau-mismatch failure and the four exact-model representative rows in robustness_envelope.csv. Run candidates from 0.000 through 0.030 in 0.001 increments. Write radius_inflation.csv with model values, base radius, found, required inflation, baseline distances, selected distances, and selected infeasibility count. Summary groups exact and mismatch rows, reports scenario_count, resolved_count, unresolved_count, and maximum required inflation among resolved rows. Extend README with the command and the restriction that these values are empirical model-scan margins only.

- [ ] **Step 4: Run full verification and generate evidence**

Run:

    python3 -m pytest -q homo_multirobot_formation_control/test
    python3 homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py --output homo_multirobot_formation_control/analysis/results/6d_artstein_hocbf_feasibility/scan.csv

Expected: all tests pass; radius_inflation.csv and radius_inflation_summary.csv are nonempty, and unresolved rows remain present.

- [ ] **Step 5: Commit**

Run:

    git add homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py homo_multirobot_formation_control/analysis/results/6d_artstein_hocbf_feasibility
    git commit -m "输出HOCBF半径膨胀结果"
