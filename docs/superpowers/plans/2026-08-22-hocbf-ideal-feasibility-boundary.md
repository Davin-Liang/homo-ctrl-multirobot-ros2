# HOCBF Ideal Feasibility Boundary Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Map the numerical ideal-model safety and hard-QP feasibility boundary of the predictor-state HOCBF implementation before coupling it to the 6D Artstein nominal controller.

**Architecture:** Add an exact-parameter scenario builder that rotates radial geometry, velocity, and nominal command together. Run the full variable grid at 20 Hz and retain every result. A 1 kHz reference is run for all 20 Hz violations, QP-infeasible rows, and rows whose 20 Hz distance is within a configured 20 mm boundary band; other rows are explicitly marked as not reference-checked, rather than silently treated as dual-rate safe.

**Tech Stack:** Python 3, NumPy, SciPy, pytest, CSV.

## Global Constraints

- Fixed exact model: tau=0.43 s, Td=0.22 s, radius=0.8 m.
- No parameter mismatch, scan, localization, TF, external geometry, or perception error.
- Full grid variables are the values in the approved ideal-feasibility specification.
- A dual-rate-safe conclusion requires reference_checked=true, no 20 Hz QP infeasibility, and both minimum distances at least 0.8 m.
- All unreferenced rows are classified as unverified, not safe.

---

### Task 1: Exact ideal scenario construction and 20 Hz grid

**Files:**
- Modify: homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py
- Modify: homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py

**Interfaces:**
- build_ideal_scenario(clearance, radial_speed, lateral_speed, nominal_speed, bearing_rad) -> ScenarioConfig
- run_ideal_feasibility_grid(clearances, radial_speeds, lateral_speeds, nominal_speeds, bearings_rad, reference_band) -> list[dict[str, float | int | bool | str]]

- [ ] **Step 1: Write failing tests**

    def test_ideal_scenario_rotates_position_velocity_and_command():
        config = module.build_ideal_scenario(
            clearance=0.4, radial_speed=0.5, lateral_speed=0.2,
            nominal_speed=0.8, bearing_rad=np.pi / 2)
        np.testing.assert_allclose(config.initial_state[:2], [0.0, 1.2])
        np.testing.assert_allclose(config.initial_state[2:], [-0.2, -0.5])
        np.testing.assert_allclose(config.nominal_command, [0.0, -0.8])
        assert config.plant.tau == config.predictor_tau == pytest.approx(0.43)
        assert config.plant.delay == config.predictor_delay == pytest.approx(0.22)

    def test_ideal_grid_returns_one_row_per_20hz_scenario():
        rows = module.run_ideal_feasibility_grid(
            clearances=[0.4], radial_speeds=[0.1],
            lateral_speeds=[0.0], nominal_speeds=[0.2],
            bearings_rad=[0.0, np.pi / 2], reference_band=0.02)
        assert len(rows) == 2
        assert set(rows[0]) >= {"reference_checked", "classification", "ideal_safe"}

- [ ] **Step 2: Verify RED**

Run: python3 -m pytest -q homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py -k ideal

Expected: AttributeError because the scenario builder and grid runner are absent.

- [ ] **Step 3: Write minimal implementation**

For bearing beta, use radial unit vector n=[cos(beta),sin(beta)] and tangential unit vector t=[-sin(beta),cos(beta)]. Set position=(0.8+clearance)n, velocity=-radial_speed n+lateral_speed t, and nominal command=-nominal_speed n. Use exact identical plant and predictor parameters. Run simulate_scenario at 20 Hz. Mark every row reference_checked=false initially. Set preliminary classification to qp_infeasible, physical_violation, or pending_reference from 20 Hz information.

- [ ] **Step 4: Verify GREEN**

Run: python3 -m pytest -q homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py -k ideal

Expected: 2 passed.

- [ ] **Step 5: Commit**

Run:

    git add homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py
    git commit -m "增加HOCBF理想模型网格"

### Task 2: Boundary reference classification, CSV summary, and evidence

**Files:**
- Modify: homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py
- Modify: homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py
- Modify: homo_multirobot_formation_control/analysis/results/6d_artstein_hocbf_feasibility/README.md
- Create: homo_multirobot_formation_control/analysis/results/6d_artstein_hocbf_feasibility/ideal_feasibility_boundary.csv
- Create: homo_multirobot_formation_control/analysis/results/6d_artstein_hocbf_feasibility/ideal_feasibility_summary.csv

**Interfaces:**
- summarize_ideal_feasibility(rows) -> list[dict[str, float | int | str]]

- [ ] **Step 1: Write failing tests**

    def test_ideal_summary_separates_safe_unsafe_and_unverified():
        rows = [
            {"classification": "ideal_safe", "min_distance_20hz": 0.81},
            {"classification": "physical_violation", "min_distance_20hz": 0.79},
            {"classification": "unverified", "min_distance_20hz": 0.90},
        ]
        summary = module.summarize_ideal_feasibility(rows)
        assert summary["ideal_safe_count"] == 1
        assert summary["physical_violation_count"] == 1
        assert summary["unverified_count"] == 1

- [ ] **Step 2: Verify RED**

Run: python3 -m pytest -q homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py -k ideal_summary

Expected: AttributeError because summarize_ideal_feasibility is absent.

- [ ] **Step 3: Write minimal implementation**

Run 1 kHz reference whenever min_distance_20hz is below 0.82 m, h is negative, or 20 Hz has QP infeasibility. For reference-checked rows classify as ideal_safe only when both distances meet 0.8 m and 20 Hz infeasibility is zero; classify violations and infeasibility explicitly. For rows outside the boundary band, leave classification=unverified. Write all rows to CSV with LF endings. Summarize counts, the 20 Hz minimum distance, and the dual-rate minimum distance over checked rows.

- [ ] **Step 4: Verify and generate evidence**

Run:

    python3 -m pytest -q homo_multirobot_formation_control/test
    python3 homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py --ideal-feasibility --output homo_multirobot_formation_control/analysis/results/6d_artstein_hocbf_feasibility/scan.csv

Expected: all tests pass; both ideal CSV files are nonempty and report the number of unverified rows.

- [ ] **Step 5: Commit**

Run:

    git add homo_multirobot_formation_control/scripts/hocbf_6d_feasibility.py homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py homo_multirobot_formation_control/analysis/results/6d_artstein_hocbf_feasibility
    git commit -m "输出HOCBF理想模型可行域"
