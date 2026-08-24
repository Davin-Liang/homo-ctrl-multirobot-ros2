# 多半径圆柱 HOCBF 数值仿真实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 6D Artstein predictor-HOCBF 数值仿真增加逐圆柱真实半径、派生安全半径与逐圆柱安全指标。

**Architecture:** 以 `ObstacleSpec` 保存中心、物理安全半径和滤波半径。每个圆柱产生一条 HOCBF 半空间；CLI 从真实圆柱半径派生这些数据。绘图和 CSV 读取同一对象列表。

**Tech Stack:** Python 3、NumPy、Matplotlib、pytest。

## Global Constraints

- 只在 `main` 分支开发，用户已明确授权。
- 不修改 ROS 控制器或 Gazebo；障碍物几何仍是数值仿真 Oracle 输入。
- `R_physical,j = follower_radius + cylinder_radius,j + clearance`。
- `R_filter,j = R_physical,j + filter_margin`。
- 每个圆柱都有一条 QP HOCBF 约束。
- 所有行为修改均先写 pytest 并确认其失败。

---

### Task 1: 建立逐圆柱半径数据接口

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/sim_6d_disc_artstein_hocbf_compare.py:1-150`
- Test: `homo_multirobot_formation_control/test/test_sim_6d_disc_artstein_hocbf_compare.py`

**Interfaces:**
- Produces: `ObstacleSpec(center: np.ndarray, physical_radius: float, filter_radius: float)`.
- Produces: `make_obstacle_specs(centers, cylinder_radii, follower_radius, clearance, filter_margin) -> list[ObstacleSpec]`.

- [x] **Step 1: Write the failing test**

```python
def test_obstacle_specs_derive_distinct_physical_and_filter_radii():
    module = load_module()
    specs = module.make_obstacle_specs([np.array([1., 2.]), np.array([3., 4.])],
        [0.20, 0.35], follower_radius=.15, clearance=.10, filter_margin=.12)
    assert [s.physical_radius for s in specs] == [.45, .60]
    assert [s.filter_radius for s in specs] == [.57, .72]
```

- [x] **Step 2: Run test to verify it fails**

Run: `pytest -q homo_multirobot_formation_control/test/test_sim_6d_disc_artstein_hocbf_compare.py::test_obstacle_specs_derive_distinct_physical_and_filter_radii`

Expected: FAIL because `make_obstacle_specs` is absent.

- [x] **Step 3: Write minimal implementation**

```python
@dataclass(frozen=True)
class ObstacleSpec:
    center: np.ndarray
    physical_radius: float
    filter_radius: float

def make_obstacle_specs(centers, cylinder_radii, follower_radius, clearance, filter_margin):
    if len(centers) != len(cylinder_radii):
        raise ValueError("centers and cylinder_radii must have equal length")
    return [ObstacleSpec(np.asarray(center, dtype=float), follower_radius + radius + clearance,
             follower_radius + radius + clearance + filter_margin)
            for center, radius in zip(centers, cylinder_radii)]
```

- [x] **Step 4: Run test to verify it passes**

Run: `pytest -q homo_multirobot_formation_control/test/test_sim_6d_disc_artstein_hocbf_compare.py::test_obstacle_specs_derive_distinct_physical_and_filter_radii`

Expected: PASS.

### Task 2: 接入逐圆柱滤波半径

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/sim_6d_disc_artstein_hocbf_compare.py:70-165`
- Test: `homo_multirobot_formation_control/test/test_sim_6d_disc_artstein_hocbf_compare.py`

**Interfaces:**
- Consumes: `list[ObstacleSpec]`.
- Produces: `filter_translation_command(..., obstacles: list[ObstacleSpec], ...)`.
- Produces: `obstacle_distances` and `h_values` on each simulation row.

- [x] **Step 1: Write the failing test**

```python
def test_filter_uses_each_obstacles_individual_filter_radius(monkeypatch):
    module = load_module(); calls = []; real = module.hocbf_halfspace
    def capture(state, center, radius, *args):
        calls.append(radius); return real(state, center, radius, *args)
    monkeypatch.setattr(module, "hocbf_halfspace", capture)
    specs = module.make_obstacle_specs([np.array([10., 0.]), np.array([0., 10.])],
                                       [.20, .35], .15, .10, .12)
    module.filter_translation_command(np.array([0., 0., 0., .1, 0., .2]), 0.,
        np.array([.2, 0., .3]), specs, .43, 2., 2., np.zeros(2), 1., 20., .05)
    assert calls == [.57, .72]
```

- [x] **Step 2: Run test to verify it fails**

Run: `pytest -q homo_multirobot_formation_control/test/test_sim_6d_disc_artstein_hocbf_compare.py::test_filter_uses_each_obstacles_individual_filter_radius`

Expected: FAIL because the existing signature accepts one shared radius.

- [x] **Step 3: Write minimal implementation**

```python
for obstacle in obstacles:
    a, b, h, _ = hocbf_halfspace(np.r_[x_pred[:2], v_pred_map], obstacle.center,
                                  obstacle.filter_radius, tau, c1, c2)
    halfspaces.append((a, b)); h_values.append(h)
```

Add `obstacle_distances` and `h_values` to the row after each plant advance.

- [x] **Step 4: Run focused tests to verify they pass**

Run: `pytest -q homo_multirobot_formation_control/test/test_sim_6d_disc_artstein_hocbf_compare.py`

Expected: PASS.

### Task 3: CLI、图和 CSV 输出

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/sim_6d_disc_artstein_hocbf_compare.py:165-245`
- Test: `homo_multirobot_formation_control/test/test_sim_6d_disc_artstein_hocbf_compare.py`

**Interfaces:**
- Produces: `parse_radius_list(value: str) -> list[float]`.
- Produces: per-obstacle CSV columns `distance_obsN`, `physical_margin_obsN`, `filter_margin_obsN`.

- [x] **Step 1: Write the failing test**

```python
def test_parse_radius_list_rejects_non_positive_radius():
    module = load_module()
    assert module.parse_radius_list("0.20,0.35") == [.20, .35]
    with pytest.raises(ValueError, match="positive"):
        module.parse_radius_list("0.20,0")
```

- [x] **Step 2: Run test to verify it fails**

Run: `pytest -q homo_multirobot_formation_control/test/test_sim_6d_disc_artstein_hocbf_compare.py::test_parse_radius_list_rejects_non_positive_radius`

Expected: FAIL because `parse_radius_list` is absent.

- [x] **Step 3: Write minimal implementation**

```python
def parse_radius_list(value):
    radii = [float(part) for part in value.split(",")]
    if not radii or any(radius <= 0.0 for radius in radii):
        raise ValueError("all cylinder radii must be positive")
    return radii
```

Add `--cylinder-radii`, `--follower-radius`, `--clearance`, and `--filter-margin`. Derive specs; draw each pair of circles; add one distance trace and CSV triplet per cylinder; print its minima and physical-violation state.

- [x] **Step 4: Run a real scenario**

Run: `python3 homo_multirobot_formation_control/scripts/sim_6d_disc_artstein_hocbf_compare.py --auto-two-cylinders --auto-two-offset 0.30 --cylinder-radii 0.20,0.35 --follower-radius 0.15 --clearance 0.10 --filter-margin 0.15 --leader-speed 0.25 --out-dir homo_multirobot_formation_control/analysis/results/6d_artstein_disc_hocbf_multi_radius`

Expected: PNG, CSV and two distinct summaries.

### Task 4: 回归验证与提交

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/sim_6d_disc_artstein_hocbf_compare.py`
- Modify: `homo_multirobot_formation_control/test/test_sim_6d_disc_artstein_hocbf_compare.py`
- Create: `docs/superpowers/plans/2026-08-24-multi-radius-cylinder-hocbf.md`

- [x] **Step 1: Run all related tests**

Run: `pytest -q homo_multirobot_formation_control/test/test_hocbf_6d_feasibility.py homo_multirobot_formation_control/test/test_sim_6d_disc_artstein_compare.py homo_multirobot_formation_control/test/test_sim_6d_disc_artstein_hocbf_compare.py homo_multirobot_formation_control/test/test_local_reference_governor.py`

Expected: PASS.

- [x] **Step 2: Review and commit**

Run: `git diff --check && git add homo_multirobot_formation_control/scripts/sim_6d_disc_artstein_hocbf_compare.py homo_multirobot_formation_control/test/test_sim_6d_disc_artstein_hocbf_compare.py docs/superpowers/plans/2026-08-24-multi-radius-cylinder-hocbf.md && git commit -m "支持多半径圆柱HOCBF数值仿真"`

Expected: source, test and plan are committed; result files stay untracked.
