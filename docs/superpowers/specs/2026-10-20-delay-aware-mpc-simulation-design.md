# Delay-Aware Constrained MPC 数值仿真设计

## 目标与范围

在 `homo_multirobot_formation_control/mpc_sim/` 新建一个纯 Python 的数值实验模块，验证独立的 Leader--Follower 线性 MPC。第一阶段不接入 ROS 2、Gazebo、既有 HPC/Artstein/HOCBF 控制器，也不修改它们。

Follower 控制器必须显式考虑固定的输入纯时延、三通道一阶速度响应、速度指令幅值约束及指令变化率约束。该模块的产出是可重复运行的 CSV、图表、指标汇总和 Markdown 实验报告。

## 名义模型

使用 map 坐标系中的 6 维编队误差：

```
e = [ex, ey, e_theta, evx, evy, e_omega]^T
```

其中位置偏移 `d_p` 与航向偏移 `d_theta` 固定在 map 系；平移速度同样使用 map 系。忽略由旋转引起的 `omega J v` 耦合，采用下列名义执行器误差模型：

```
e_q_dot  = e_nu
e_nu_dot = -T^-1 e_nu + T^-1 delta_u(t - Td)
```

`T = diag(tau_v, tau_v, tau_omega)`，控制输入为相对速度指令 `delta_u = u_f - u_l`；实际发送给 Follower 的命令为 `u_f = u_l + delta_u`。

离散采样周期固定为 `h = 0.05 s`，名义输入延迟为 `Td = 0.30 s`，故整数延迟为 `d = 6` 步。精确零阶保持矩阵为：

```
E  = exp(-h T^-1)
H  = T (I - E)
Ad = [[I, H], [0, E]]
Bd = [[hI - H], [I - E]]
```

因此一采样步状态更新是 `e[k+1] = Ad e[k] + Bd delta_u[k-d]`。这里位置块的 `Ad` 是 `H`，输入位置块的 `Bd` 是 `hI-H`；这与连续 ZOH 积分一致。

## 延迟时序与增广状态

在时刻 `k`、控制器求解前，保存的历史队列按最旧到最新排列：

```
w[k] = [delta_u[k-d], delta_u[k-d+1], ..., delta_u[k-1]]
```

前端元素 `w[k][0]` 在从 `k` 到 `k+1` 的对象更新中实际生效。MPC 决策的首个变量是 `delta_u[k]`，将被追加到队尾，并且仅在从 `k+d` 到 `k+d+1` 的对象更新中生效。每步更新后丢弃队首元素。

增广状态定义为 `z[k] = [e[k]; w[k]]`。增广转移的首行使用 `Ad e[k] + Bd w[k][0]`，队列其余元素左移，最后一项写入当前优化变量。这一约定必须由单元测试覆盖，避免常见的一步错位。

## 控制器与约束

每步构造凸二次规划，预测步数为 20。代价包含：

- 全预测域的 `e_x`、`e_y`、`e_theta`、`e_vx`、`e_vy`、`e_omega` 二次误差；
- 加大的终端误差二次代价；
- 相对命令 `delta_u` 的二次正则项；
- Follower 发送命令变化率 `u_f[j] - u_f[j-1]` 的二次惩罚。

幅值与变化率约束都施加于 Follower 发送命令 `u_f[j] = u_l[j] + delta_u[j]`：

```
u_min <= u_f[j] <= u_max
du_min <= u_f[j] - u_f[j-1] <= du_max
```

第一项变化率以仿真中上一时刻真正发送的 Follower 指令为基准。Leader 的未来名义命令序列视为已知；因此该问题保持凸 QP，优先使用 CVXPY/OSQP。结果不作严格闭环稳定性声明。

## 对比对象和数值对象

所有对照实验共用同一初值、Leader 参考、速度时间常数、约束和真实对象。Leader 与 Follower 均运行同一一阶滞后 + 固定输入延迟模型。

- Delay-aware MPC：内部模型使用 6 步名义延迟。
- 普通 MPC 基线：内部模型错误地使用 0 步输入延迟；真实对象仍保留 6 步延迟。
- 时延失配：控制器始终使用 6 步，真实对象分别用 4、6、8 步延迟。

初始实验提供静止 Leader、直线 Leader、含平移与角速度变化的曲线 Leader、严格输入约束，以及时延模型比较/失配。若某个试验 infeasible、数值异常或出现约束违反，保留完整结果并在报告中说明，不能筛除。

## 模块边界

- `model.py`：精确离散化、增广矩阵、队列与离散对象；不依赖 CVXPY。
- `controller.py`：CVXPY 参数化 QP、输入/变化率约束、求解状态和耗时。
- `scenarios.py`：Leader 指令、初始条件及场景配置。
- `simulation.py`：闭环时序、真实对象、记录。
- `reporting.py`：CSV、PNG、指标与 Markdown 报告。
- `config.py`：集中式 dataclass 参数。
- `tests/`：模型、时延时序、零误差不动点、约束和端到端短仿真测试。

默认输出不写入既有 `robot_traj/`，而是写至 `mpc_sim/results/<timestamp>/`（该路径将被忽略），以隔离新实验的结果。

## 验证标准

1. 数值积分与 `Ad`、`Bd` 的单步 ZOH 解在机器精度范围内一致。
2. 六步队列测试证明命令在第六个完整控制区间后才开始生效。
3. 零误差、零 Leader 命令、零历史输入下，MPC 和对象均保持零状态。
4. 每次 QP 均检查最优状态；非最优结果被记录而非静默使用。
5. 后处理验证 `u_f` 幅值、相邻实际发送命令变化率、CSV 完整性和图表生成。
6. 全部场景完成后生成相同口径的 RMSE、最大位置误差、尾段稳态误差、约束违反次数和求解耗时统计。

## 非目标与后续工作

本阶段不包括轮速逆运动学约束、障碍物、CBF/HOCBF、通信延迟、分布式多 Follower MPC、非线性 `omega Jv` 模型、理论稳定性证明或 ROS 2/Gazebo 集成。数值结果足以表明实现正确和工程潜力后，再独立开展稳定性分析与 ROS 2 集成设计。
