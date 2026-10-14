# Homogeneous vs Linear XY Trajectory Plot Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a MATLAB script that creates one publication-ready XY trajectory figure comparing selected homogeneous and linear controller trials.

**Architecture:** A single MATLAB script owns its user-editable configuration, validates two selected trial directories, reads each `raw.csv` with `readtable`, and plots the Leader and Follower XY series without coordinate manipulation. A lightweight Python test validates the expected CSV schema and the script's required configuration/output contract because this workspace has neither MATLAB nor GNU Octave installed.

**Tech Stack:** MATLAB (`readtable`, `plot`, `exportgraphics`), Python standard library (`csv`, `pathlib`, `unittest`).

## Global Constraints

- Create the MATLAB script in `homo_multirobot_formation_control/robot_traj/real/`.
- Expose exactly two editable trial-directory variables at the beginning of the script.
- Default to `6d_artstein_no_forw_vs_forw_20260922_112130` and `6d_artstein_lpc_vs_hpc(no_fora)_20260922_150051`.
- Plot the raw map-frame XY data only; do not align, shift, resample, or compute statistics.
- Output `homogeneous_vs_linear_xy.png` and `homogeneous_vs_linear_xy.pdf` beside the script.
- Validate `raw.csv` existence and all four required coordinate columns with clear MATLAB errors.

---

### Task 1: Define and verify the plotting-script contract

**Files:**
- Create: `homo_multirobot_formation_control/test/test_plot_homogeneous_linear_xy.py`

**Interfaces:**
- Consumes: `homo_multirobot_formation_control/robot_traj/real/plot_homogeneous_linear_xy.m` as UTF-8 text.
- Produces: an executable Python `unittest` regression check for the required defaults, coordinate columns, and two export names.

- [ ] **Step 1: Write the failing test**

```python
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "homo_multirobot_formation_control" / "robot_traj" / "real" / "plot_homogeneous_linear_xy.m"

class PlotHomogeneousLinearXYTest(unittest.TestCase):
    def test_script_declares_selected_trials_and_outputs(self):
        source = SCRIPT.read_text(encoding="utf-8")
        for required in (
            "homogeneous_trial_dir",
            "linear_trial_dir",
            "6d_artstein_no_forw_vs_forw_20260922_112130",
            "6d_artstein_lpc_vs_hpc(no_fora)_20260922_150051",
            "leader_x_m", "leader_y_m", "follower_x_m", "follower_y_m",
            "homogeneous_vs_linear_xy.png", "homogeneous_vs_linear_xy.pdf",
        ):
            self.assertIn(required, source)

if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `python3 -m unittest homo_multirobot_formation_control/test/test_plot_homogeneous_linear_xy.py -v`

Expected: FAIL with `FileNotFoundError` because the MATLAB script does not exist yet.

- [ ] **Step 3: Commit the failing test**

```bash
git add homo_multirobot_formation_control/test/test_plot_homogeneous_linear_xy.py
git commit -m "新增轨迹绘图脚本契约测试"
```

### Task 2: Implement the MATLAB XY plotting script

**Files:**
- Create: `homo_multirobot_formation_control/robot_traj/real/plot_homogeneous_linear_xy.m`

**Interfaces:**
- Consumes: two user-edited trial-directory names and each selected directory's `raw.csv`.
- Produces: a figure with four plotted series and `homogeneous_vs_linear_xy.png` / `homogeneous_vs_linear_xy.pdf` in the script directory.

- [ ] **Step 1: Add the minimal script implementation**

```matlab
% Compare raw map-frame XY trajectories from one homogeneous and one linear trial.
% Edit only these two directory names to select another experiment pair.
homogeneous_trial_dir = '6d_artstein_no_forw_vs_forw_20260922_112130';
linear_trial_dir = '6d_artstein_lpc_vs_hpc(no_fora)_20260922_150051';

script_dir = fileparts(mfilename('fullpath'));
homogeneous = read_trial(script_dir, homogeneous_trial_dir, '齐次控制');
linear = read_trial(script_dir, linear_trial_dir, '线性控制');

fig = figure('Color', 'w', 'Position', [100, 100, 760, 560]);
hold on;
plot(homogeneous.leader_x_m, homogeneous.leader_y_m, '-', 'Color', [0.00, 0.45, 0.74], 'LineWidth', 1.5);
plot(homogeneous.follower_x_m, homogeneous.follower_y_m, '--', 'Color', [0.00, 0.45, 0.74], 'LineWidth', 1.5);
plot(linear.leader_x_m, linear.leader_y_m, '-', 'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.5);
plot(linear.follower_x_m, linear.follower_y_m, '--', 'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.5);
axis equal;
grid on;
xlabel('x (m)');
ylabel('y (m)');
legend({'齐次 Leader', '齐次 Follower', '线性 Leader', '线性 Follower'}, 'Location', 'best');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 10, 'LineWidth', 1);

exportgraphics(fig, fullfile(script_dir, 'homogeneous_vs_linear_xy.png'), 'Resolution', 300);
exportgraphics(fig, fullfile(script_dir, 'homogeneous_vs_linear_xy.pdf'), 'ContentType', 'vector');

function trial = read_trial(script_dir, trial_dir, group_name)
csv_path = fullfile(script_dir, trial_dir, 'raw.csv');
if ~isfile(csv_path)
    error('plot_homogeneous_linear_xy:MissingCsv', '%s组找不到文件: %s', group_name, csv_path);
end
trial = readtable(csv_path);
required = {'leader_x_m', 'leader_y_m', 'follower_x_m', 'follower_y_m'};
missing = required(~ismember(required, trial.Properties.VariableNames));
if ~isempty(missing)
    error('plot_homogeneous_linear_xy:MissingColumn', '%s组的 raw.csv 缺少列: %s', group_name, strjoin(missing, ', '));
end
end
```

- [ ] **Step 2: Run the contract test to verify it passes**

Run: `python3 -m unittest homo_multirobot_formation_control/test/test_plot_homogeneous_linear_xy.py -v`

Expected: PASS.

- [ ] **Step 3: Perform available input validation**

Run:

```bash
python3 - <<'PY'
import csv
from pathlib import Path
root = Path('homo_multirobot_formation_control/robot_traj/real')
for name in ('6d_artstein_no_forw_vs_forw_20260922_112130', '6d_artstein_lpc_vs_hpc(no_fora)_20260922_150051'):
    with (root / name / 'raw.csv').open(newline='', encoding='utf-8') as handle:
        fields = set(csv.DictReader(handle).fieldnames or [])
    assert {'leader_x_m', 'leader_y_m', 'follower_x_m', 'follower_y_m'} <= fields, name
print('Default trial CSV schemas are valid.')
PY
```

Expected: `Default trial CSV schemas are valid.`

- [ ] **Step 4: Run the MATLAB script manually in MATLAB**

Run in MATLAB:

```matlab
cd('homo_multirobot_formation_control/robot_traj/real')
plot_homogeneous_linear_xy
```

Expected: one equal-aspect XY figure with four labelled trajectories, plus the PNG and PDF output files. MATLAB is not installed in this workspace, so this is a user-side validation step.

- [ ] **Step 5: Commit the implementation**

```bash
git add homo_multirobot_formation_control/robot_traj/real/plot_homogeneous_linear_xy.m homo_multirobot_formation_control/test/test_plot_homogeneous_linear_xy.py
git commit -m "新增齐次线性XY轨迹绘图脚本"
```

