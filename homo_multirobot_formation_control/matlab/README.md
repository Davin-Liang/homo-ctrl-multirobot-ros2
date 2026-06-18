# MATLAB 仿真与算法验证

本目录包含齐次编队控制（HPC）的 MATLAB 仿真代码，分为两个子目录：

| 目录 | 内容 |
|------|------|
| `source/` | HPC 工具箱 + 4D 粒子模型编队控制仿真 |
| `6d_analysis/` | 6D 运动学模型稳定性分析与仿真 |

## 依赖

`6d_analysis/` 依赖 `source/` 中的 HPC 工具箱函数（`lpc2hpc.m`、`hnorm.m` 等），MATLAB 路径需包含 `source/`。

## 运行

所有脚本从所在目录运行即可：

```matlab
cd matlab/source
demo_4d_cont           % 4D 连续边界投影 (纯径向)
lpc_hpc_distance_square % 4D 离散多边形 (原版论文算法)
```

## source/ — HPC 工具箱

| 文件 | 作用 |
|------|------|
| `hnorm.m` | 齐次范数：二分法求解 `nx` 使 `x'·d'(-ln nx)·P·d(-ln nx)·x = 1` |
| `lpc2hpc.m` | LPC→HPC 升级：从线性增益 K 计算 HPC 参数 (K0, G0, P, ν) |
| `e_hpc.m` | 显式 HPC 控制律：`u = K0·x + nx^(1+ν)·K·d(-ln nx)·x` |
| `block_con.m` | 块可控标准型变换 |
| `trans_con.m` | 基于 SVD 的正交块分解 |
| `ZOH.m` | 零阶保持器离散化 |
| `curveintersect.m` | 曲线求交（工具函数） |
| `demo_lpc2hpc.m` | LPC→HPC 升级示例 |

## source/ — 4D 编队控制仿真

4D 状态：`x = [px, py, vx, vy]^T`（双重积分器质点模型，map 坐标系）。

领航者轨迹：PD 控制器跟踪 `[sin(t), cos(t)]`（单位圆，~1 rad/s）。

| 文件 | 编队策略 | 增益机制 | 噪声 | 说明 |
|------|---------|---------|------|------|
| `lpc_hpc_distance_square.m` | 离散多边形 (m_p=4, tol=0.1 切换) | 自适应 (min=1 或 4) | 无 | **原版论文算法**，对照基准 |
| `demo_4d_cont.m` | 连续边界投影 (纯径向) | 平滑混合 (min=4, max=40) | 无 | 对照 C++ `homo_controller_4d_cont.hpp` |
| `demo_4d_cont_tangent.m` | 纯径向投影 | 平滑混合 (min=4, max=40) | **有** | 噪声影响分析 |
| `demo_4d_cont_tangent_noise.m` | 固定角度切向修正 | 平滑混合 (min=4, max=40) | **有** | 径向+切向 = 等价 m_p=1 |
| `demo_4d_cont_omegad.m` | 纯径向投影 | **C++ ω_d 机制** (恒定增益) | **有** | 对照 C++ `calculate_klin` |

### 编队策略对比

```
离散多边形 (原版):          连续边界投影 (C++ 移植):
  e = x2 - x1 - d_i           e_pos = dpos - r · dpos/|dpos|
  d_i ∈ {r·[cosθ_i, sinθ_i]}  (纯径向, 无角度约束)
  最近点 + tol 滞后切换        连续, 无需切换

固定角度切向修正 (改进):
  e_pos = dpos - r · [cos θ_d, sin θ_d]
  θ_d 固定在 world frame (不跟踪速度方向)
  等价于 m_p=1, 连续无切换
```

### 增益机制对比

| 机制 | 公式 | 增益行为 |
|------|------|---------|
| MATLAB 原版 | `a = max(-m·ev/ep, 1)` | 自适应, 无上界 → 近圆时爆炸 |
| MATLAB 平滑混合 | `a = w·a_adaptive + (1-w)·4` | 自适应, 近圆平滑退化 |
| C++ ω_d | `a = max(clamp(-m·ev/ep, ±ω_d·m), ω_d·m)` | **恒定** = ω_d·m |

### 关键参数

| 参数 | 默认值 | 说明 |
|------|-------|------|
| m | 2 | 双重积分器质量 |
| radius | 1 m | 安全圆半径 |
| h | 0.01 s | 仿真步长 (100 Hz) |
| Tmax | 30 s | 仿真时长 |
| gain_min / gain_max | 4 / 40 | MATLAB 自适应增益范围 |
| omega_d | 1.5 | C++ 阻尼带宽 |
| noise_pos_std | 0.0 | 位置测量噪声 (m) |
| noise_vel_std | 0.0 | 速度测量噪声 (m/s) |

### 对照 C++ 实现

| MATLAB | C++ |
|--------|-----|
| `demo_4d_cont.m` | `homo_controller_4d_cont.hpp` |
| `demo_4d_cont_omegad.m` | `calculate_klin()` in 同上 |
| `lpc_hpc_distance_square.m` | `homo_controller.hpp` (原版 LpcController) |
| `hnorm.m` | `hnorm.hpp` |
| `lpc2hpc.m` | `lpc2hpc.hpp` |

## 6d_analysis/ — 6D 运动学稳定性分析

详见 `6d_analysis/README.md`。

6D 状态：`x = [px, py, θ, vx, vy, ω]^T`（含车身朝向 + 全向轮运动学约束）。
用于分析时变系统（领航者速度变化时）的冻结时间稳定性和 HPC 鲁棒性。
