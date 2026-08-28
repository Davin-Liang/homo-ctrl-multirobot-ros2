# 6D map-frame HPC 与 Artstein 数值仿真设计

## 目标

为 `6D_齐次编队控制理论_Codex工程压缩版.md` 的 map-frame 固定偏移模型新增独立数值仿真。验证在相同工程正则化 HPC 控制律、初始条件和执行器限制下，Artstein 预测对输入纯延迟与一阶速度响应的改善。

本工作不修改 ROS 2 控制器，也不改变既有 `sim_6d_disc_artstein_compare.py`；旧脚本继续服务于离散 Leader-frame 编队点实现。

## 仿真模型

状态使用文档指定顺序：

```
e = [ex, ey, e_theta, evx, evy, e_omega]
```

位置偏移固定在 map 坐标系，默认 `d_p_map = [-1.0, 0.0] m`、`d_theta = 0`。误差按 map-frame 模型计算：位置与速度误差均在 map 系，航向误差使用 wrap。

控制器采用现有数值仿真的工程正则化形式，而非文档的理论 `K0` 控制律：

```
c = clamp(hnorm(e, Gd, P), hpc_c_min, 1)
u = c^(1 + mu) K exp(Gd (1 - log(c))) e
```

其中 `A` 是三通道双积分器，`B = [0; diag(1/m, 1/m, 1/I)]`，并使用文档规定的解析 `G0 = diag(-I3, 0)` 和 `Gd = I + mu G0`。每次运行将检查可控性、齐次代数恒等式、闭环 Hurwitz 裕度和 Lyapunov/dilation 正定裕度；不满足时终止并报告。

Follower plant 接收 body-frame `cmd_vel`，但其一阶执行器状态以 map-frame 平移速度和 yaw 角速度表达，以避免在预测模型中引入姿态相关的 LTI 误差。Plant 包含：控制周期 20 Hz、积分周期 100 Hz、`Td = 0.22 s` 纯输入延迟、平移与偏航一阶时间常数 `tau = 0.43 s`。

## 对照组

所有对照共享 Leader 圆轨迹、初值、HPC 参数、命令限幅和随机种子。

1. `ideal`：无延迟、无一阶滞后，用于给出工程控制律的性能上界。
2. `delayed`：含 `Td + tau`，直接用测量状态反馈。
3. `artstein`：与 `delayed` 的同一 plant，但反馈前采用 map-frame 4D 平移 Artstein 预测与 2D yaw Artstein 预测。

默认 Leader 以 `0.45 m/s` 沿半径 `2 m` 圆轨迹运行；其 yaw 与路径切线一致。Follower 从较大的位置和 yaw 偏差起步，以暴露延迟带来的相位滞后。

## 输出与判据

结果保存到新的、被 Git 忽略的 `analysis/results/6d_map_hpc_artstein/` 子目录：

- 轨迹、位置误差、yaw 误差、map-frame 命令的 PNG；
- 每组尾段均值/标准差、峰值与最终误差的 CSV；
- 全时序 CSV；
- 一份文本诊断，列出矩阵恒等式残差、稳定性和 LMI 裕度。

成功条件：脚本可重复运行；三项代数恒等式和可控性检查通过；延迟组与 Artstein 组实际使用完全相同的 plant；结果中不把正则化工程控制律表述为严格有限时间证明。

## 测试

先以 pytest 编写并运行失败测试，覆盖：

1. map-frame 固定偏移下的零误差平衡；
2. `G0 B = 0`、`A G0 - G0 A = A`、`A Gd - Gd A = mu A`；
3. 三组延迟设置分别符合其定义；
4. Artstein 预测器在零延迟/零初始误差极限下与测量状态一致。

随后实现最小脚本并使测试通过，再运行默认数值实验。

## 范围边界

本仿真不包括 Artstein 与完整非线性 ROS body-input 映射的严格等价证明、离散编队点切换、HOCBF、噪声鲁棒性扫描或 Gazebo 验证。这些在名义 map-frame 对照完成后分别扩展。
