# Experiment Chapter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Write a standalone Chinese academic experiment chapter covering homogeneity-degree selection, homogeneous-versus-linear control, and delay-compensation ablation from the repository's real-robot data.

**Architecture:** The chapter is a new Markdown document independent of the theory draft. It will cite existing table and figure filenames with manuscript-number placeholders, report only reproducible numerical evidence, and separate nominal finite-time theory from physical-system engineering validation.

**Tech Stack:** Markdown, repository experiment statistics, CSV/XLSX summaries, existing MATLAB-generated figures.

## Global Constraints

- Create `homo_multirobot_formation_control/doc/experiment_chapter_draft.md`; do not modify `paper_draft.md`.
- Cover only the three approved experiments: homogeneity degree, homogeneous/linear comparison, and Artstein/forward-prediction ablation.
- Cite `homogeneity_metrics_table.xlsx`/`.csv` and existing PNG filenames as `表 X` and `图 X` placeholders.
- Give units for numerical results and never state strict finite-time convergence for the delayed, sampled, saturated physical system.
- Preserve all user-owned experiment artifacts and statistics files.

---

### Task 1: Establish a checked evidence set for the chapter

**Files:**
- Create: `homo_multirobot_formation_control/test/test_experiment_chapter_draft.py`
- Read: `homo_multirobot_formation_control/doc/experimental_statistics.md`
- Read: `homo_multirobot_formation_control/robot_traj/real/homogeneity_metrics_table.csv`

**Interfaces:**
- Produces a static chapter contract listing the three required experiment sections, six evaluation concepts, figure filenames, and theoretical-boundary wording.

- [ ] **Step 1: Write the failing document contract test**

Create `test_experiment_chapter_draft.py`:

```python
import unittest
from pathlib import Path


CHAPTER = Path(__file__).parents[1] / 'doc' / 'experiment_chapter_draft.md'


class ExperimentChapterDraftTest(unittest.TestCase):
    def test_chapter_covers_only_approved_experiments(self):
        source = CHAPTER.read_text(encoding='utf-8')
        for required in (
            '不同齐次度下的编队性能', '齐次控制与线性控制对比',
            '延迟补偿方案消融实验', '距离误差', '相对航向误差',
            '速度差', 'homogeneity_metrics_table.xlsx',
            'homogeneous_vs_linear_xy.png', 'four_compensation_xy.png',
            '不宜据此宣称严格有限时间收敛'):
            self.assertIn(required, source)


if __name__ == '__main__':
    unittest.main()
```

- [ ] **Step 2: Verify red**

Run:

```bash
python3 -m unittest homo_multirobot_formation_control/test/test_experiment_chapter_draft.py -v
```

Expected: FAIL because the new chapter document does not yet exist.

- [ ] **Step 3: Verify the numerical inputs before writing prose**

Run:

```bash
test -f homo_multirobot_formation_control/robot_traj/real/homogeneity_metrics_table.xlsx
test -f homo_multirobot_formation_control/robot_traj/real/homogeneity_metrics_table.csv
rg -n "不同齐次度对比|齐次/线性对比|原始 6D map hpc|仅有 Artstein|仅有前向预测" homo_multirobot_formation_control/doc/experimental_statistics.md
```

Expected: all evidence files and group definitions are present before numerical claims are written.

### Task 2: Write and validate the standalone experiment chapter

**Files:**
- Create: `homo_multirobot_formation_control/doc/experiment_chapter_draft.md`
- Test: `homo_multirobot_formation_control/test/test_experiment_chapter_draft.py`

**Interfaces:**
- Produces sections `X.1` through `X.6`: platform/task, metrics, homogeneity degree, homogeneous/linear comparison, delay-compensation ablation, and summary.

- [ ] **Step 1: Write the chapter with this section skeleton**

```markdown
# X 实验与结果分析

## X.1 实验平台与任务说明
## X.2 评价指标与数据处理
## X.3 不同齐次度下的编队性能
## X.4 齐次控制与线性控制对比
## X.5 延迟补偿方案消融实验
## X.6 本章小结
```

Use three-repeat `均值 ± 样本标准差`, define all three error channels, insert figure/table placeholders and filenames, cite exact homogeneity statistics where available, and limit ablation/comparison prose to evidence shown by their plots and raw data.

- [ ] **Step 2: Run the document contract test**

Run the Task 1 unittest command. Expected: 1 test passes.

- [ ] **Step 3: Conduct a factual self-review**

Run:

```bash
rg -n "有限时间收敛|图 X|表 X|m/s|rad|m" homo_multirobot_formation_control/doc/experiment_chapter_draft.md
git diff --check -- homo_multirobot_formation_control/doc/experiment_chapter_draft.md homo_multirobot_formation_control/test/test_experiment_chapter_draft.py
```

Confirm every numerical claim is present in `experimental_statistics.md` or the generated metrics table, and retain the physical-system conclusion boundary.

- [ ] **Step 4: Commit the chapter and test**

```bash
git add homo_multirobot_formation_control/doc/experiment_chapter_draft.md homo_multirobot_formation_control/test/test_experiment_chapter_draft.py
git commit -m "新增小论文实验章节草稿"
```
