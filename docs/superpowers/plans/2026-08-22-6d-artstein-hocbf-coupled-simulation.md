# 6D Artstein HOCBF Coupled Simulation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement task-by-task.

**Goal:** Produce a Python-only, ideal static-obstacle comparison between the existing 6D Artstein Disc simulator and the same simulator filtered by HOCBF.

**Architecture:** Create a new script that imports the existing sim_6d_disc_artstein_compare module and hocbf_6d_feasibility module. Reuse simulate-case state prediction, controller and plant semantics; duplicate only its loop to place HOCBF after nominal body command generation. The final safe map command is converted back to body frame and written into the existing command histories.

**Tech Stack:** Python 3, NumPy, Matplotlib, SciPy, pytest.

## Global Constraints

- No reimplementation of Hpc6DDisc or Artstein predictors.
- Static map-frame circular obstacle; exact tau, Td, and state.
- HOCBF filters translation only; preserve nominal yaw command.
- Final safe command, not nominal command, is used by the plant and map-frame Artstein history.
- No slack; record infeasibility.

---

### Task 1: HOCBF command adapter and history contract

**Files:**
- Create: homo_multirobot_formation_control/scripts/sim_6d_disc_artstein_hocbf_compare.py
- Create: homo_multirobot_formation_control/test/test_sim_6d_disc_artstein_hocbf_compare.py

**Interfaces:**
- filter_translation_command(x_pred, yaw_meas, cmd_nom_body, obstacle, safe_radius, tau, c1, c2, previous_map_command, vmax, amax, dt) -> tuple[np.ndarray, np.ndarray, bool, float]

- [ ] Write failing tests: no active barrier returns the nominal body command and zero correction; an approaching command is modified; returned angular command equals cmd_nom_body[2].
- [ ] Run: python3 -m pytest -q homo_multirobot_formation_control/test/test_sim_6d_disc_artstein_hocbf_compare.py
- [ ] Implement: use imported body_to_map, map_to_body, hocbf_halfspace, and solve_hocbf_qp. Compute predicted map velocity from x_pred. Return final body command, final map command, QP feasibility, and h.
- [ ] Re-run focused tests.
- [ ] Commit: git commit -m "增加6D Artstein HOCBF命令适配"

### Task 2: Coupled simulation, CSV, and plot

**Files:**
- Modify: homo_multirobot_formation_control/scripts/sim_6d_disc_artstein_hocbf_compare.py
- Modify: homo_multirobot_formation_control/test/test_sim_6d_disc_artstein_hocbf_compare.py
- Create: homo_multirobot_formation_control/analysis/results/6d_artstein_disc_hocbf/README.md

**Interfaces:**
- simulate_compensated_hocbf(Tmax, h, tau_v, tau_w, Td, obstacle, safe_radius) -> rows
- CLI writes coupled_summary.csv, coupled_timeseries.csv, and coupled_compare.png.

- [ ] Write failing test: HOCBF rows retain final safe map command in their next predictor-history sample and include h, distance, correction norm, and feasibility fields.
- [ ] Implement by copying only simulate_case control-loop scaffolding; use imported Hpc6DDisc, predictor, leader, and plant functions. Run baseline compensated and HOCBF compensated from identical initial state. Plot both trajectories plus obstacle/safety circle, h and distance, formation/yaw errors, nominal/safe map commands, and correction norm.
- [ ] Run: python3 -m pytest -q homo_multirobot_formation_control/test and python3 homo_multirobot_formation_control/scripts/sim_6d_disc_artstein_hocbf_compare.py --tmax 30 --out-dir homo_multirobot_formation_control/analysis/results/6d_artstein_disc_hocbf
- [ ] Commit: git commit -m "输出6D Artstein HOCBF耦合仿真"
