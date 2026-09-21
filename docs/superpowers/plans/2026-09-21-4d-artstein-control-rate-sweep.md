# 4D Artstein 控制频率数值扫描 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 用同一 200 Hz 真实 plant 比较 4D Artstein + prediction 在 10、25、40 Hz 下的数值编队跟踪效果。

**Architecture:** 不改动仿真代码；调用既有脚本三次，以独立目录保存完整输出。然后从每组 `summary_metrics.csv` 中提取 clean/noise 的 Artstein + prediction 行，以同一指标横向报告。

**Tech Stack:** Python 3、NumPy、SciPy、Matplotlib。

## Global Constraints

- 共同参数：`tau=0.43`、`predict_tau=0.43`、`Td=0.22`、`plant_dt=0.005`、`circle_tmax=60`、`leader_speed=0.5`、`hpc_c_min=0.1`、`max_cmd=1.5`。
- 仅改变 `dt`：10 Hz=`0.100`、25 Hz=`0.040`、40 Hz=`0.025`。
- 不修改源码，不覆盖既有结果目录。

---

### Task 1: 运行三组可复现实验

**Files:**
- Create: `homo_multirobot_formation_control/analysis/results/4d_artstein_rate_{10,25,40}hz_20260921/*`

- [ ] **Step 1: 运行 10 Hz**

Run: `python3 homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py --out-dir homo_multirobot_formation_control/analysis/results/4d_artstein_rate_10hz_20260921 --dt 0.100 --plant-dt 0.005 --tau 0.43 --predict-tau 0.43 --Td 0.22 --circle-tmax 60 --leader-speed 0.5 --hpc-c-min 0.1 --max-cmd 1.5`

Expected: 输出 `summary_metrics.csv`、clean/noise 圆轨迹图与延迟对比图。

- [ ] **Step 2: 运行 25 Hz**

Run: 将 Step 1 的 `--out-dir` 改为 `4d_artstein_rate_25hz_20260921`，`--dt` 改为 `0.040`。

Expected: 输出独立结果目录。

- [ ] **Step 3: 运行 40 Hz**

Run: 将 Step 1 的 `--out-dir` 改为 `4d_artstein_rate_40hz_20260921`，`--dt` 改为 `0.025`。

Expected: 输出独立结果目录。

### Task 2: 验证输出并汇总

**Files:**
- Read: 三组 `summary_metrics.csv`
- Read: 三组 `circle_original_vs_artstein_clean.png` 与 `circle_original_vs_artstein_noise.png`

- [ ] **Step 1: 验证每组输出完整**

Run: `for rate in 10 25 40; do test -s homo_multirobot_formation_control/analysis/results/4d_artstein_rate_${rate}hz_20260921/summary_metrics.csv; done`

Expected: 所有命令返回 0。

- [ ] **Step 2: 提取目标行**

Run: `rg 'circle_artstein_prediction_(clean|noise)' homo_multirobot_formation_control/analysis/results/4d_artstein_rate_{10,25,40}hz_20260921/summary_metrics.csv`

Expected: 每个目录各有 clean/noise 两行，字段为最大误差、尾段均值、尾段标准差、最终误差。

- [ ] **Step 3: 报告结果**

逐项比较三组 clean/noise 指标，并明确结论限于固定 plant 的数值仿真。
