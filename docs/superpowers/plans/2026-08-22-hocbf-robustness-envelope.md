# HOCBF Robustness Envelope Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Extend the offline HOCBF simulator with an auditable robustness-envelope scan and empirical sampled-control margin summary.

**Architecture:** Keep the plant parameters as the real system and add separate predictor tau to the existing ScenarioConfig. Construct each scenario from radial and tangential initial velocities, run it at 20 Hz with a 10 ms plant, and compare it with the same scenario at 1 kHz control. Write every row, including unsafe or infeasible rows, to CSV; summarize exact-model feasible rows separately from all rows.

**Tech Stack:** Python 3, NumPy, SciPy, pytest, standard-library CSV.

## Global Constraints

- Static circular obstacles and Oracle map-frame geometry only.
- Primary controller is 20 Hz and plant integration is 10 ms; reference controller is 1 kHz.
- No ROS, Gazebo, scan, TF, dynamic obstacle, or perception-error implementation.
- Predictor history always records the final HOCBF-filtered command.
- No slack variable; retain all QP-infeasible and braking scenarios in output.
- epsilon_numeric_obs is an empirical scan statistic, never a robust safety guarantee.

---

### Task 1: Separate predicted and actual actuator time constants

**Files:**
- Modify: homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py
- Modify: homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py

**Interfaces:**
- ScenarioConfig gains predictor_tau: float | None = None.
- simulate_scenario uses plant.tau for real ZOH propagation and predictor_tau, or plant.tau when omitted, for predictor ZOH and the HOCBF constraint.

- [ ] **Step 1: Write the failing tests**

    def test_predictor_tau_changes_the_predicted_actuator_state_only():
        config = module.ScenarioConfig(
            plant=module.PlantParams(0.5, 0.22, 0.01, 0.05),
            predictor_tau=0.3, obstacle=np.zeros(2), safe_radius=0.8,
            initial_state=np.array([2.0, 0.0, -0.2, 0.0]),
            nominal_command=np.array([-0.6, 0.0]),
            vmax=1.0, amax=20.0, c1=2.0, c2=2.0, duration=0.5)
        result = module.simulate_scenario(config)
        assert result["predictor_tau"] == pytest.approx(0.3)
        assert result["plant_tau"] == pytest.approx(0.5)

    def test_predictor_tau_must_be_positive():
        with pytest.raises(ValueError, match="predictor_tau"):
            module.ScenarioConfig(
                plant=module.PlantParams(0.5, 0.22, 0.01, 0.05),
                predictor_tau=0.0, obstacle=np.zeros(2), safe_radius=0.8,
                initial_state=np.zeros(4), nominal_command=np.zeros(2),
                vmax=1.0, amax=20.0, c1=2.0, c2=2.0, duration=0.5)

- [ ] **Step 2: Run the tests to verify they fail**

Run: python3 -m pytest -q homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py -k predictor_tau

Expected: FAIL because ScenarioConfig does not accept predictor_tau.

- [ ] **Step 3: Write minimal implementation**

Add predictor_tau to ScenarioConfig, validate it is positive when supplied, and set model_tau = predictor_tau or plant.tau. Build predictor PlantParams with model_tau; retain plant.tau for actual ZOH. Pass model_tau, not plant.tau, to hocbf_halfspace. Add scalar arrays or constant fields plant_tau and predictor_tau to the simulation result.

- [ ] **Step 4: Run focused tests**

Run: python3 -m pytest -q homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py -k predictor_tau

Expected: 2 passed.

- [ ] **Step 5: Commit**

Run:

    git add homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py
    git commit -m "支持HOCBF执行器模型失配"

### Task 2: Envelope scan and empirical sampled-control margin

**Files:**
- Modify: homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py
- Modify: homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py

**Interfaces:**
- run_robustness_envelope(tau_actual_values, tau_ratios, delay_model_values, delay_mismatches, clearances, radial_speeds, lateral_speeds, nominal_speeds) -> list[dict[str, float | int]]
- Each row contains tau_actual, tau_model, delay_model, delay_actual, initial_clearance, initial_radial_speed, initial_lateral_speed, nominal_radial_speed, min_h, min_distance, min_psi2, infeasible_steps, braking_steps, min_h_1khz, min_distance_1khz, sample_distance_gap, and exact_model.

- [ ] **Step 1: Write the failing tests**

    def test_robustness_envelope_has_every_parameter_combination_and_margin():
        rows = module.run_robustness_envelope(
            tau_actual_values=[0.43], tau_ratios=[1.0, 1.2],
            delay_model_values=[0.22], delay_mismatches=[0.0],
            clearances=[0.4], radial_speeds=[0.1],
            lateral_speeds=[0.0], nominal_speeds=[0.4])
        assert len(rows) == 2
        assert rows[0]["sample_distance_gap"] >= 0.0
        assert rows[0]["exact_model"] == 1
        assert rows[1]["exact_model"] == 0

- [ ] **Step 2: Run the test to verify it fails**

Run: python3 -m pytest -q homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py -k robustness_envelope

Expected: AttributeError because run_robustness_envelope is absent.

- [ ] **Step 3: Write minimal implementation**

For each Cartesian-product point, set tau_model=tau_actual*tau_ratio and delay_actual=delay_model+delay_mismatch. Construct initial state [0.8+clearance, 0, -radial_speed, lateral_speed] and nominal command [-nominal_speed, 0]. Run the 20 Hz scenario and compare it to 1 kHz using compare_sampling_rates. Set sample_distance_gap=max(0, min_distance_1khz-min_distance_20hz). Set exact_model=1 only when tau_ratio is 1 and delay_mismatch is 0. Retain every row.

- [ ] **Step 4: Run focused tests**

Run: python3 -m pytest -q homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py -k robustness_envelope

Expected: 1 passed.

- [ ] **Step 5: Commit**

Run:

    git add homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py
    git commit -m "增加HOCBF鲁棒性包络扫描"

### Task 3: CSV summary, CLI evidence, and documentation

**Files:**
- Modify: homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py
- Modify: homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py
- Modify: homo_multirobot_formation_control/analysis/results/6d_artstein_hocbf_feasibility/README.md
- Create: homo_multirobot_formation_control/analysis/results/6d_artstein_hocbf_feasibility/robustness_envelope.csv
- Create: homo_multirobot_formation_control/analysis/results/6d_artstein_hocbf_feasibility/robustness_summary.csv

**Interfaces:**
- summarize_robustness_rows(rows) -> list[dict[str, float | int]]
- CLI writes robustness_envelope.csv and robustness_summary.csv next to the requested scan CSV.

- [ ] **Step 1: Write the failing tests**

    def test_robustness_summary_reports_all_and_exact_feasible_groups():
        rows = [
            {"exact_model": 1, "infeasible_steps": 0, "min_h": 0.1,
             "min_distance": 0.9, "sample_distance_gap": 0.02},
            {"exact_model": 1, "infeasible_steps": 1, "min_h": -0.1,
             "min_distance": 0.7, "sample_distance_gap": 0.03},
            {"exact_model": 0, "infeasible_steps": 0, "min_h": 0.01,
             "min_distance": 0.81, "sample_distance_gap": 0.04},
        ]
        summary = module.summarize_robustness_rows(rows)
        exact = next(row for row in summary if row["group"] == "exact_feasible")
        assert exact["scenario_count"] == 1
        assert exact["max_sample_distance_gap"] == pytest.approx(0.02)

- [ ] **Step 2: Run the test to verify it fails**

Run: python3 -m pytest -q homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py -k robustness_summary

Expected: AttributeError because summarize_robustness_rows is absent.

- [ ] **Step 3: Write minimal implementation**

Return two summary rows: all and exact_feasible, where exact_feasible means exact_model=1 and infeasible_steps=0. For each nonempty group calculate scenario_count, unsafe_count using min_h<0, min_h, min_distance, infeasible_scenarios, and max_sample_distance_gap. Write both CSV files with LF endings. Extend the README with the exact command and the restriction that the maximum observed gap is not a perception or model-error bound.

- [ ] **Step 4: Run full verification and generate evidence**

Run:

    python3 -m pytest -q homo_multirobot_formation_control/test
    python3 homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py --output homo_multirobot_formation_control/analysis/results/6d_artstein_hocbf_feasibility/scan.csv

Expected: all tests pass; scan.csv, sampling_rate_compare.csv, robustness_envelope.csv, and robustness_summary.csv exist and are nonempty.

- [ ] **Step 5: Commit**

Run:

    git add homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py homo_multirobot_formation_control/analysis/results/6d_artstein_hocbf_feasibility
    git commit -m "输出HOCBF鲁棒性包络结果"
