# 双移动机器人实物跟踪实验汇报制作 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 基于 2026-09-14 至 2026-09-15 的实物轨迹记录，生成一份可独立阅读的 Word 总结和一份配套 PPT，呈现 Leader--Follower 跟踪过程、两项对比和当前阶段结果。

**Architecture:** 先用不可修改的原始 `raw.csv` 与 `metadata.yaml` 建立实验清单，再由单一 Python 分析脚本导出统一指标和出版级图表。Word 与 PPT 只消费清单、指标和图表；PPT 保留视频占位页而不嵌入视频，Word 不出现视频章节、链接或附件说明。

**Tech Stack:** Python 3、标准库 `csv`/`statistics`、PyYAML、Matplotlib、`python-docx`、`python-pptx`；可选 LibreOffice 用于人工打开和导出 PDF 验收。

## Global Constraints

- 结果主体只使用 `homo_multirobot_formation_control/robot_traj/real/` 下的实物数据。
- 不纳入仿真、6D 方案或 10/25/40 Hz 控制频率对比。
- “有/无前馈”严格指 `enable_leader_cmd_feedforward` 的 Leader 命令速度前馈开关；Follower 的 `tau` 前向预测是共同基础链路，不是该对比变量。
- Follower 链路表述固定为：Artstein 积分补偿纯滞后 → 按 `tau` 前向预测 → HPC/LPC 计算 → 速度/轮速约束 → `cmd_vel`。
- 实验标签和参数以 `metadata.yaml` 为准；目录名与元数据不一致时，报告不得使用目录名推断前馈开关。
- HPC/LPC 结果属于不同实际工况下的阶段性对照，不得宣称严格的单变量算法优劣：LPC `initial_min_lambda=1.5`、Leader 速度 0.25 m/s 时发生碰撞；LPC `lambda=2.0` 使用 0.20 m/s，`lambda=2.5` 使用 0.25 m/s。
- Word 不提及视频；PPT 只含用户后续手动填入视频的占位页。
- 所有新生成物写入 `docs/reports/2026-09-15-real-robot-tracking/`，不修改原始实验目录。

---

## File Structure

- Create: `homo_multirobot_formation_control/analysis/real_tracking_report/experiment_manifest.yaml` — 报告唯一实验清单、展示标签、对比归属和已知工况限制。
- Create: `homo_multirobot_formation_control/scripts/analyze_real_tracking_report.py` — 从清单读取原始 CSV/元数据，计算指标并生成图表与表格数据。
- Create: `homo_multirobot_formation_control/test/test_analyze_real_tracking_report.py` — 清单校验、距离误差和指标计算的单元测试。
- Create: `homo_multirobot_formation_control/scripts/build_real_tracking_deliverables.py` — 根据图表、指标和固定中文文案生成 `.docx` 与 `.pptx`。
- Create: `homo_multirobot_formation_control/requirements-report.txt` — `python-docx>=1.1,<2`、`python-pptx>=1.0,<2` 的可复现依赖声明。
- Create: `docs/reports/2026-09-15-real-robot-tracking/assets/` — 分图 PNG 与方法流程图。
- Create: `docs/reports/2026-09-15-real-robot-tracking/metrics.csv` — 每个展示试验的可审计量化指标。
- Create: `docs/reports/2026-09-15-real-robot-tracking/experiment-conditions.csv` — 报告中使用的实验标签、原始目录及关键条件。
- Create: `docs/reports/2026-09-15-real-robot-tracking/双移动机器人实物跟踪实验总结.docx` — 最终 Word。
- Create: `docs/reports/2026-09-15-real-robot-tracking/双移动机器人实物跟踪实验汇报.pptx` — 最终 PPT。

### Task 1: 建立可追溯实验清单

**Files:**
- Create: `homo_multirobot_formation_control/analysis/real_tracking_report/experiment_manifest.yaml`
- Create: `docs/reports/2026-09-15-real-robot-tracking/experiment-conditions.csv`

**Consumes:** 四组原始实验目录的 `metadata.yaml` 和 `raw.csv`。

**Produces:** 后续分析和排版共享的实验身份、比较组别与文字边界。

- [ ] **Step 1: 创建输出目录，不写入任何原始实验目录**

Run:

```bash
mkdir -p docs/reports/2026-09-15-real-robot-tracking/assets
```

Expected: 只出现报告输出目录；`robot_traj/real/` 中没有新增或改写文件。

- [ ] **Step 2: 写入四组展示记录的清单**

清单必须包含以下记录，并以 `metadata.yaml` 的参数作为 `leader_command_feedforward` 与 `controller_family` 的来源：

```yaml
experiments:
  - id: hpc_feedforward_on
    source_dir: robot_traj/real/4d_artstein_forward_8traj_20260914_204226
    display_label: Artstein-HPC（Leader 命令前馈：开）
    comparison: leader_command_feedforward
    controller_family: artstein_hpc
    leader_command_feedforward: true
  - id: hpc_feedforward_off
    source_dir: robot_traj/real/4d_artstein_no_forward_8traj_20260914_204546
    display_label: Artstein-HPC（Leader 命令前馈：关）
    comparison: leader_command_feedforward
    controller_family: artstein_hpc
    leader_command_feedforward: false
  - id: lpc_lambda_20_v020
    source_dir: robot_traj/real/4d_artstein_lpc_20260915_104805
    display_label: Artstein-LPC（λ初值=2.0，Leader 速度=0.20 m/s）
    comparison: hpc_lpc_stage_result
    controller_family: artstein_lpc
    initial_min_lambda: 2.0
    leader_speed_mps: 0.20
  - id: lpc_lambda_25_v025
    source_dir: robot_traj/real/4d_artstein_lpc_20260915_105440
    display_label: Artstein-LPC（λ初值=2.5，Leader 速度=0.25 m/s）
    comparison: hpc_lpc_stage_result
    controller_family: artstein_lpc
    initial_min_lambda: 2.5
    leader_speed_mps: 0.25
```

- [ ] **Step 3: 为不成功的 LPC 条件建立文字记录，而非伪造轨迹数据**

在清单中增加：

```yaml
non_recorded_conditions:
  - controller_family: artstein_lpc
    initial_min_lambda: 1.5
    leader_speed_mps: 0.25
    outcome: collision
    report_statement: 该工况发生碰撞，未形成可用于轨迹统计的有效记录。
```

- [ ] **Step 4: 导出实验条件表并人工核对**

表至少包含 `实验标签、原始目录、控制器、use_hpc、Leader 命令前馈、initial_min_lambda、Leader 速度、录制时长、状态源、备注`。逐行对照对应 `metadata.yaml`，确认前馈开关不会因目录名相反而写反。

- [ ] **Step 5: 验收清单**

Run:

```bash
python3 - <<'PY'
import yaml
with open('homo_multirobot_formation_control/analysis/real_tracking_report/experiment_manifest.yaml', encoding='utf-8') as f:
    data = yaml.safe_load(f)
assert len(data['experiments']) == 4
assert data['experiments'][0]['leader_command_feedforward'] is True
assert data['experiments'][1]['leader_command_feedforward'] is False
assert data['non_recorded_conditions'][0]['outcome'] == 'collision'
print('manifest OK')
PY
```

Expected: `manifest OK`。

### Task 2: 实现统一指标计算与分析校验

**Files:**
- Create: `homo_multirobot_formation_control/scripts/analyze_real_tracking_report.py`
- Create: `homo_multirobot_formation_control/test/test_analyze_real_tracking_report.py`
- Create: `docs/reports/2026-09-15-real-robot-tracking/metrics.csv`

**Consumes:** `experiment_manifest.yaml`、各实验的 `raw.csv`、`metadata.yaml`。

**Produces:** 同一口径的距离误差、轨迹和速度统计；后续图表和 Word/PPT 仅使用这些产物。

- [ ] **Step 1: 写失败测试，固定指标定义**

测试使用如下最小数据，确认误差相对于理想编队半径计算而非直接把车间距离当作误差：

```python
def test_distance_error_metrics_use_ideal_radius():
    distances = [0.8, 1.0, 1.2]
    metrics = compute_distance_metrics(distances, ideal_radius_m=1.0)
    assert metrics['mean_abs_distance_error_m'] == 0.13333333333333333
    assert metrics['rms_distance_error_m'] == 0.16329931618554522
    assert metrics['final_distance_error_m'] == 0.2
```

- [ ] **Step 2: 运行测试确认失败**

Run:

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_analyze_real_tracking_report.py -q
```

Expected: FAIL，原因是 `compute_distance_metrics` 尚未定义。

- [ ] **Step 3: 实现指标函数和 CSV 校验**

`compute_distance_metrics(distances, ideal_radius_m)` 输出以下字段：

```python
{
    'sample_count': len(distances),
    'mean_distance_m': mean(distances),
    'std_distance_m': pstdev(distances),
    'mean_abs_distance_error_m': mean(abs(d - ideal_radius_m) for d in distances),
    'rms_distance_error_m': sqrt(mean((d - ideal_radius_m) ** 2 for d in distances)),
    'final_distance_error_m': abs(distances[-1] - ideal_radius_m),
}
```

同时要求每个 CSV 恰有以下列：

```text
time_s, leader_x_m, leader_y_m, leader_vx_ms, leader_vy_ms, leader_v_ms,
follower_x_m, follower_y_m, follower_vx_ms, follower_vy_ms, follower_v_ms, distance_m
```

数据为空、`distance_m` 非有限数、理想半径非正数或 CSV 列缺失时，脚本必须以清晰的文件路径和原因退出，不能生成部分报告。

- [ ] **Step 4: 运行单元测试和真实数据分析**

Run:

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_analyze_real_tracking_report.py -q
python3 homo_multirobot_formation_control/scripts/analyze_real_tracking_report.py \
  --manifest homo_multirobot_formation_control/analysis/real_tracking_report/experiment_manifest.yaml \
  --output docs/reports/2026-09-15-real-robot-tracking
```

Expected: 测试通过，生成 4 行 `metrics.csv`；每行样本数大于 0，所有数值有限。

- [ ] **Step 5: 人工审查指标适用边界**

确认报告对 9 月 14 日的前馈开关使用相同的“理想半径 1.0 m”误差口径。HPC/LPC 图表可并列展示，但表格标题写为“不同实物工况下的阶段性结果”，不写“公平单变量对比”。

### Task 3: 生成报告图表和控制链路图

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/analyze_real_tracking_report.py`
- Create: `docs/reports/2026-09-15-real-robot-tracking/assets/feedforward_comparison.png`
- Create: `docs/reports/2026-09-15-real-robot-tracking/assets/feedforward_distance_error.png`
- Create: `docs/reports/2026-09-15-real-robot-tracking/assets/hpc_lpc_trajectory.png`
- Create: `docs/reports/2026-09-15-real-robot-tracking/assets/hpc_lpc_distance_error.png`
- Create: `docs/reports/2026-09-15-real-robot-tracking/assets/control_pipeline.png`

**Consumes:** Task 2 生成的清洗后序列和指标。

**Produces:** Word/PPT 共用的高分辨率中文图表；不直接复制 `check.png` 作为最终排版图。

- [ ] **Step 1: 定义图表统一规则**

使用中文字体可用时的无衬线字体；若本机无中文字体，脚本应输出英文坐标轴并打印字体告警，而不是生成乱码。图中统一：Leader 蓝色、Follower 橙色、理想半径灰色虚线；坐标单位 m、s、m/s；导出 300 dpi PNG。

- [ ] **Step 2: 生成前馈开关对比图**

`feedforward_comparison.png` 使用两列子图，分别展示前馈开/关时的 Leader/Follower 平面轨迹和起止点；`feedforward_distance_error.png` 将两组 `distance_m - 1.0` 画在同一时间轴，并标出零误差线。图例必须以清单的 `display_label` 生成。

- [ ] **Step 3: 生成 HPC/LPC 阶段性结果图**

`hpc_lpc_trajectory.png` 与 `hpc_lpc_distance_error.png` 至少展示一个 HPC 代表组及两组 LPC 安全运行记录。图注或图例必须写出 LPC `lambda=2.0/0.20 m/s` 与 `lambda=2.5/0.25 m/s`，不能使读者误以为 Leader 速度相同。

- [ ] **Step 4: 绘制方法流程图**

`control_pipeline.png` 的节点顺序必须为：

```text
动捕状态 → Follower Artstein 积分补偿 Td → Follower 前向预测 tau
→ HPC 或 LPC 控制律 → 速度/轮速约束 → Follower cmd_vel
```

在 Artstein-HPC 实验分支旁附加“可选：Leader cmd_vel 速度增量前馈（实验一开/关）”，用虚线指向控制输出相加点；不要把该支路画成 Follower 前向预测的开关。

- [ ] **Step 5: 验收所有图表**

Run:

```bash
file docs/reports/2026-09-15-real-robot-tracking/assets/*.png
```

Expected: 恰有上述 5 个 PNG，均识别为 PNG image data；人工检查无中文乱码、图例遮挡或单位缺失。

### Task 4: 生成 Word 实验总结

**Files:**
- Create: `homo_multirobot_formation_control/requirements-report.txt`
- Create: `homo_multirobot_formation_control/scripts/build_real_tracking_deliverables.py`
- Create: `docs/reports/2026-09-15-real-robot-tracking/双移动机器人实物跟踪实验总结.docx`

**Consumes:** Task 1 的条件表、Task 2 的指标表、Task 3 的五张图。

**Produces:** 可独立阅读的中文 Word，不含视频内容。

- [ ] **Step 1: 安装文档生成依赖至项目专用虚拟环境**

Run:

```bash
python3 -m venv /tmp/homo-report-venv
/tmp/homo-report-venv/bin/pip install -r homo_multirobot_formation_control/requirements-report.txt
```

`requirements-report.txt` 内容为：

```text
python-docx>=1.1,<2
python-pptx>=1.0,<2
```

Expected: `/tmp/homo-report-venv/bin/python -c 'import docx, pptx'` 成功；不修改系统 Python 包。

- [ ] **Step 2: 固定 Word 章节与内容边界**

按以下顺序生成正文：

```text
标题：双移动机器人 Leader–Follower 实物跟踪实验总结
1. 实验任务说明
2. 实物实验平台、数据流与控制方法
3. 实验过程与统一条件
4. 实验一：Artstein-HPC 的 Leader 命令速度前馈开/关对比
5. 实验二：Artstein-HPC 与 Artstein-LPC 的阶段性结果
6. 阶段性成果、当前问题与下一步工作
```

第 2 节插入 `control_pipeline.png`；第 3 节插入条件表；第 4、5 节分别插入对应轨迹图、误差图和指标表。禁止生成“视频”“附件”“二维码”“链接”等标题、段落或页脚。

- [ ] **Step 3: 写入必须如实保留的结论边界**

第 4 节仅按指标描述前馈开关组的实测差异；第 5 节必须逐字保留下列事实：

```text
LPC 在 initial_min_lambda=1.5、Leader 速度 0.25 m/s 的实物工况下发生碰撞，
未形成可用于轨迹统计的有效记录。LPC 的 λ初值=2.0 记录同时将 Leader 速度降至
0.20 m/s；λ初值=2.5 记录使用 0.25 m/s。因此现有 HPC/LPC 结果用于说明阶段性
可运行性和现象，不用于宣称严格单变量条件下的性能优劣。
```

- [ ] **Step 4: 生成并人工打开 Word**

Run:

```bash
/tmp/homo-report-venv/bin/python homo_multirobot_formation_control/scripts/build_real_tracking_deliverables.py \
  --input docs/reports/2026-09-15-real-robot-tracking \
  --word-out docs/reports/2026-09-15-real-robot-tracking/双移动机器人实物跟踪实验总结.docx \
  --skip-ppt
```

Expected: 文件存在，能由 LibreOffice 或 Word 打开；章节、表格、图片不溢出页面；全文不含“视频”“二维码”“附件”。

### Task 5: 生成展示 PPT 与视频占位页

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/build_real_tracking_deliverables.py`
- Create: `docs/reports/2026-09-15-real-robot-tracking/双移动机器人实物跟踪实验汇报.pptx`

**Consumes:** 与 Word 相同的图、表和结论，另加一个空白视频区域。

**Produces:** 8--10 页 PPT；用户可手动将视频粘贴到占位页。

- [ ] **Step 1: 固定幻灯片大纲**

按 9 页生成：

```text
1. 标题
2. 实验任务与阶段目标
3. 实物平台与数据流
4. 控制方法：Artstein + Follower 前向预测 + HPC/LPC
5. 实验流程与统一条件
6. 实验一：Leader 命令速度前馈开/关
7. 实验二：HPC/LPC 阶段性结果与安全边界
8. 最新实物实验视频（占位页）
9. 阶段结论与下一步工作
```

- [ ] **Step 2: 制作视频占位页而非嵌入媒体**

第 8 页放置 16:9 边框矩形，标题为“请在此处手动插入最新实物实验视频”，副标题为“建议对应：Artstein-LPC，λ初值=2.5，Leader 速度=0.25 m/s”。该页不得嵌入、链接、复制或生成任何视频文件。

- [ ] **Step 3: 生成 PPT 并检查可编辑性**

Run:

```bash
/tmp/homo-report-venv/bin/python homo_multirobot_formation_control/scripts/build_real_tracking_deliverables.py \
  --input docs/reports/2026-09-15-real-robot-tracking \
  --ppt-out docs/reports/2026-09-15-real-robot-tracking/双移动机器人实物跟踪实验汇报.pptx \
  --skip-word
```

Expected: 生成 9 页可编辑 PPT；第 8 页仅有文字和形状占位框；全部结论与 Word 一致。

### Task 6: 交叉验收、可追溯性检查与提交

**Files:**
- Modify: `docs/reports/2026-09-15-real-robot-tracking/metrics.csv`
- Modify: `docs/reports/2026-09-15-real-robot-tracking/experiment-conditions.csv`
- Modify: `docs/reports/2026-09-15-real-robot-tracking/双移动机器人实物跟踪实验总结.docx`
- Modify: `docs/reports/2026-09-15-real-robot-tracking/双移动机器人实物跟踪实验汇报.pptx`

**Consumes:** 所有前序产物。

**Produces:** 可交付、可复查的一致性材料。

- [ ] **Step 1: 进行数据可追溯性检查**

逐项核查报告中每个实验标签均能回到 `experiment-conditions.csv` 中的原始目录，且前馈开/关与对应 `metadata.yaml` 一致。检查清单要求：HPC 前馈开组为 `true`、关组为 `false`；两组 LPC 的 `lambda` 和 Leader 速度分别为 `2.0/0.20` 与 `2.5/0.25`。

- [ ] **Step 2: 进行 Word/PPT 文本一致性检查**

确认两份材料均清晰区分 Follower 前向预测与 Leader 命令速度前馈；两份材料都不作严格 HPC/LPC 单变量性能结论；仅 PPT 存在视频占位页，Word 中没有任何视频提及。

- [ ] **Step 3: 进行文件完整性检查**

Run:

```bash
test -s docs/reports/2026-09-15-real-robot-tracking/metrics.csv
test -s docs/reports/2026-09-15-real-robot-tracking/experiment-conditions.csv
test -s docs/reports/2026-09-15-real-robot-tracking/双移动机器人实物跟踪实验总结.docx
test -s docs/reports/2026-09-15-real-robot-tracking/双移动机器人实物跟踪实验汇报.pptx
python3 -m pytest homo_multirobot_formation_control/test/test_analyze_real_tracking_report.py -q
```

Expected: 每条 `test -s` 成功，单元测试全部通过。

- [ ] **Step 4: 提交报告源文件与交付物**

Run:

```bash
git add homo_multirobot_formation_control/analysis/real_tracking_report \
        homo_multirobot_formation_control/scripts/analyze_real_tracking_report.py \
        homo_multirobot_formation_control/scripts/build_real_tracking_deliverables.py \
        homo_multirobot_formation_control/test/test_analyze_real_tracking_report.py \
        homo_multirobot_formation_control/requirements-report.txt \
        docs/reports/2026-09-15-real-robot-tracking
git commit -m '生成实物双机器人跟踪汇报材料'
```

Expected: 提交仅包含报告生成源、报告资产、Word 和 PPT，不包含视频文件或原始 `robot_traj` 数据修改。

## Plan Self-Review

- 设计说明中的两项对比、Follower 前向预测、LPC 碰撞及速度工况边界，均分别由 Task 1、Task 3、Task 4 和 Task 6 覆盖。
- 输出目录、源文件、交付物与测试路径均为确切路径；没有未决占位描述。
- Word 与 PPT 的分工、视频占位规则和可追溯性要求已在 Task 4--6 明确，且不会改写原始实验数据。
