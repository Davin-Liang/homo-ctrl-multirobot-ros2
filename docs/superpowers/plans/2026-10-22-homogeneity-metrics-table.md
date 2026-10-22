# Homogeneity Metrics Table Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Create a PNG/PDF table comparing six group-level position, yaw, and velocity metrics across the existing homogeneity-degree experiments.

**Architecture:** One MATLAB script owns the explicit nu/trial mapping, reads raw.csv, aggregates three trials as mean ± sample standard deviation, prints a MATLAB table, and renders it through uitable. A Python unittest verifies the required mapping, metrics, and exports.

**Tech Stack:** MATLAB readtable/table/uitable/exportgraphics; Python unittest.

## Global Constraints

- Modify only the new MATLAB script and its focused test.
- Read raw.csv from robot_traj/real and do not change existing data or plotting scripts.
- Set ideal_radius_m=1.0 and use a trailing 10-second window, falling back to every sample for shorter trials.
- Require three trial directories per nu and all required position, velocity, yaw, and time columns.
- Output homogeneity_metrics_table.png at 300 dpi and homogeneity_metrics_table.pdf beside the script.

---

### Task 1: Add the MATLAB script contract and experiment mapping

**Files:**
- Create: homo_multirobot_formation_control/robot_traj/real/plot_homogeneity_metrics_table.m
- Create: homo_multirobot_formation_control/test/test_plot_homogeneity_metrics_table.py

**Interfaces:**
- Produces functions read_trial_metrics(script_dir, trial_dir, ideal_radius_m, steady_window_s) and wrap_to_pi(angle).
- Produces homogeneity_metrics_table.png and homogeneity_metrics_table.pdf.

- [ ] **Step 1: Write the failing static test**

Create test_plot_homogeneity_metrics_table.py with this test:

    import unittest
    from pathlib import Path

    SCRIPT = Path(__file__).parents[1] / 'robot_traj' / 'real' / 'plot_homogeneity_metrics_table.m'

    class PlotHomogeneityMetricsTableTest(unittest.TestCase):
        def test_contract(self):
            source = SCRIPT.read_text(encoding='utf-8')
            required = ('homogeneity_groups', '-1.0', '-0.9', '-0.8', '-0.7',
                        '-0.6', '-0.5', '-0.4', '-0.3', '-0.2', '-0.1', '0.0',
                        'full_distance_mae', 'steady_distance_mae',
                        'full_yaw_rms', 'steady_yaw_rms',
                        'full_velocity_mae', 'steady_velocity_mae',
                        'read_trial_metrics', 'wrap_to_pi', 'uitable',
                        'exportgraphics', 'homogeneity_metrics_table.png',
                        'homogeneity_metrics_table.pdf')
            for token in required:
                self.assertIn(token, source)

- [ ] **Step 2: Verify red**

Run: python3 -m unittest homo_multirobot_formation_control/test/test_plot_homogeneity_metrics_table.py -v

Expected: FAIL because the script does not exist.

- [ ] **Step 3: Implement the explicit mapping**

Create plot_homogeneity_metrics_table.m. Set ideal_radius_m=1.0, steady_window_s=10.0, and script_dir=fileparts(mfilename('fullpath')). Configure nu=-1.0 through 0.0, using the three trial directories listed under “不同齐次度对比（无前馈）” in homo_multirobot_formation_control/doc/experimental_statistics.md. Copy the complete directory names into a struct array rather than globbing directories.

- [ ] **Step 4: Verify green and commit**

Run the Step 2 command. Expected: one test passes.

    git add homo_multirobot_formation_control/robot_traj/real/plot_homogeneity_metrics_table.m homo_multirobot_formation_control/test/test_plot_homogeneity_metrics_table.py
    git commit -m "新增齐次度指标表脚本"

### Task 2: Compute and aggregate six metrics

**Files:**
- Modify: homo_multirobot_formation_control/robot_traj/real/plot_homogeneity_metrics_table.m
- Modify: homo_multirobot_formation_control/test/test_plot_homogeneity_metrics_table.py

**Interfaces:**
- read_trial_metrics returns full/steady distance MAE, full/steady yaw RMS, and full/steady velocity MAE, in that order.
- summary_table contains group mean ± sample standard deviation results.

- [ ] **Step 1: Extend the failing test**

Add these tokens to required: time_s, distance_m, leader_vx_map_ms, leader_vy_map_ms, follower_vx_map_ms, follower_vy_map_ms, leader_yaw_rad, follower_yaw_rad, steady_window_s, std(, and summary_table.

- [ ] **Step 2: Verify red**

Run the Task 1 unittest command. Expected: FAIL because metric calculation is absent.

- [ ] **Step 3: Implement validation, metrics, and aggregation**

For each group, require numel(group.trials)==3. The reader must reject a missing CSV or required column with nu and file path in the error. Use these equations:

    distance_error = abs(trial.distance_m - ideal_radius_m);
    yaw_error = wrap_to_pi(trial.follower_yaw_rad - trial.leader_yaw_rad);
    velocity_error = hypot(trial.follower_vx_map_ms - trial.leader_vx_map_ms, ...
        trial.follower_vy_map_ms - trial.leader_vy_map_ms);
    steady_mask = trial.time_s >= trial.time_s(end) - steady_window_s;
    if ~any(steady_mask), steady_mask = true(height(trial), 1); end
    metrics = [mean(distance_error), mean(distance_error(steady_mask)), ...
        sqrt(mean(yaw_error.^2)), sqrt(mean(yaw_error(steady_mask).^2)), ...
        mean(velocity_error), mean(velocity_error(steady_mask))];

Aggregate the 3-by-6 matrix using mean(values,1) and std(values,0,1), format each cell as %.4f ± %.4f, and implement wrap_to_pi as atan2(sin(angle),cos(angle)).

- [ ] **Step 4: Verify green and commit**

Run the unittest; expected: one test passes. Commit with:

    git add homo_multirobot_formation_control/robot_traj/real/plot_homogeneity_metrics_table.m homo_multirobot_formation_control/test/test_plot_homogeneity_metrics_table.py
    git commit -m "统计齐次度编队六项指标"

### Task 3: Render and export the publication table

**Files:**
- Modify: homo_multirobot_formation_control/robot_traj/real/plot_homogeneity_metrics_table.m
- Modify: homo_multirobot_formation_control/test/test_plot_homogeneity_metrics_table.py

**Interfaces:**
- Displays summary_table and writes both output images.

- [ ] **Step 1: Extend the failing test**

Require the strings Full Distance MAE (m), Steady Distance MAE (m), Full Yaw RMS (rad), Steady Yaw RMS (rad), Full Velocity MAE (m/s), Steady Velocity MAE (m/s), and Resolution', 300.

- [ ] **Step 2: Verify red**

Run the Task 1 unittest command. Expected: FAIL because labels and 300-dpi export are absent.

- [ ] **Step 3: Render the table**

Create a white 1600-by-540 figure and a full-figure uitable. Use rows nu = %.1f and the six labels from Step 1. Display the formatted mean/std cells, print summary_table, and export with:

    exportgraphics(fig, fullfile(script_dir, 'homogeneity_metrics_table.png'), 'Resolution', 300);
    exportgraphics(fig, fullfile(script_dir, 'homogeneity_metrics_table.pdf'), 'ContentType', 'vector');

- [ ] **Step 4: Verify and commit**

Run:

    python3 -m unittest homo_multirobot_formation_control/test/test_plot_homogeneity_metrics_table.py -v
    octave --quiet --eval "run('homo_multirobot_formation_control/robot_traj/real/plot_homogeneity_metrics_table.m');"

Expected: unittest passes. If Octave lacks uitable or exportgraphics, report that MATLAB Desktop is required for rendering. Run git diff --check and commit:

    git add homo_multirobot_formation_control/robot_traj/real/plot_homogeneity_metrics_table.m homo_multirobot_formation_control/test/test_plot_homogeneity_metrics_table.py
    git commit -m "导出齐次度指标表图片"

