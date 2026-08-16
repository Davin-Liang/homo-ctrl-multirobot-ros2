# 4D Artstein-LQR 数值仿真说明

## 1. 目的

`scripts/sim_4d_artstein_lqr_compare.py` 用于验证 4D DARE-LQR 数值对照组。该对照组用于回答：
在同一 Artstein 输入时延补偿、一阶电机前向预测、速度/加速度限幅和离散编队点切换条件下，
标准离散 LQR 相对 4D HPC 的表现如何。

主对照组为：

```text
original_4d_delay
artstein_hpc_delay
artstein_lqr_delay
```

no-delay sanity check 为：

```text
artstein_hpc_no_delay
artstein_lqr_no_delay
```

本脚本不把 `artstein_mpc_delay` 纳入主对照。MPC 对照仍由
`scripts/sim_4d_artstein_mpc_compare.py` 独立维护。

## 2. 数学模型与状态定义

LQR 作用在预测补偿后的 4D 双积分状态：

```math
x=
\begin{bmatrix}
p_x & p_y & v_x & v_y
\end{bmatrix}^T
=
\begin{bmatrix}
p^T & v^T
\end{bmatrix}^T .
```

连续时间名义模型为：

```math
\dot p=v,\qquad \dot v=u/m ,
```

即：

```math
\dot x = A x + B u ,
```

其中：

```math
A =
\begin{bmatrix}
0&0&1&0\\
0&0&0&1\\
0&0&0&0\\
0&0&0&0
\end{bmatrix},
\qquad
B =
\begin{bmatrix}
0&0\\
0&0\\
1/m&0\\
0&1/m
\end{bmatrix}.
```

这里的 `u=[u_x,u_y]^T` 是与 4D HPC/MPC 一致的 force-like 等效输入，
实际加速度为：

```math
a = u/m .
```

因此 `mass` 在该对照组中是控制输入缩放参数，不强行解释为底盘真实物理质量。

## 3. ZOH 离散化

ROS 控制器以固定周期运行，数值仿真默认 `dt=0.05s`，对应 `control_rate=20Hz`。
因此 LQR 直接在离散系统上设计，而不是先用连续 CARE 再离散执行。

采样周期记为：

```math
h = \Delta t = 1/f_c .
```

对双积分模型做精确零阶保持（ZOH）离散化：

```math
x_{k+1}=A_d x_k+B_d u_k .
```

因为 `A^2=0`，有：

```math
A_d=e^{Ah}=I+Ah ,
```

且：

```math
B_d=\int_0^h e^{A s}B\,ds .
```

展开后得到：

```math
A_d =
\begin{bmatrix}
1&0&h&0\\
0&1&0&h\\
0&0&1&0\\
0&0&0&1
\end{bmatrix},
\quad
B_d =
\begin{bmatrix}
h^2/(2m)&0\\
0&h^2/(2m)\\
h/m&0\\
0&h/m
\end{bmatrix}.
```

这与 Python 数值仿真和 C++ `LqrController4DArtstein::zoh_matrices()` 使用的矩阵一致。

## 4. 编队误差系统

Leader 和 Follower 的预测补偿后状态分别记为：

```math
x_{1,h,k},\qquad x_{2,h,k}.
```

离散多边形编队点为：

```math
d_j =
\begin{bmatrix}
-r\cos(2\pi j/m_p)\\
-r\sin(2\pi j/m_p)\\
0\\
0
\end{bmatrix},
\qquad j=0,\ldots,m_p-1 .
```

控制器初始化时选择距离最近的 `d_j`，运行中仅当新目标比当前目标至少好 `tol` 时切换，
该逻辑与 4D HPC/MPC 对照保持一致。

对当前选中的编队偏移 `d`，定义误差：

```math
e_k=x_{2,h,k}-x_{1,h,k}-d .
```

若 Leader 在一个采样周期内近似匀速，且 `d` 在 map 系固定，则误差动力学可写作：

```math
e_{k+1}=A_d e_k+B_d u_k-w_k ,
```

其中 `w_k` 表示 Leader 加速度、目标切换、预测误差和实际限幅带来的等效扰动。
LQR 设计时采用名义调节模型：

```math
e_{k+1}=A_d e_k+B_d u_k .
```

因此该 LQR 是编队误差上的无限时域离散调节器，而不是显式求解整段 Leader 未来轨迹的跟踪型 MPC。

## 5. DARE-LQR 原理

对名义误差系统：

```math
e_{k+1}=A_d e_k+B_d u_k ,
```

定义无限时域二次型性能指标：

```math
J=\sum_{k=0}^{\infty}
\left(e_k^TQe_k+u_k^TRu_k\right),
```

其中：

```math
Q=\operatorname{diag}(q_{px},q_{py},q_{vx},q_{vy})\succeq 0,
\qquad
R=\operatorname{diag}(r_{ux},r_{uy})\succ 0 .
```

默认参数为：

```math
Q=\operatorname{diag}(40,40,1,1),
\qquad
R=\operatorname{diag}(0.02,0.02).
```

离散 LQR 的值函数取二次型：

```math
V(e)=e^TPe ,
```

其中 `P` 是对称半正定矩阵。由 Bellman 最优性方程可得离散代数 Riccati 方程（DARE）：

```math
P =
A_d^TPA_d
-A_d^TPB_d(R+B_d^TPB_d)^{-1}B_d^TPA_d
+Q .
```

求得稳定化解 `P` 后，最优反馈增益为：

```math
K =
(R+B_d^TPB_d)^{-1}B_d^TPA_d .
```

控制律为：

```math
u_k=-K e_k ,
```

闭环误差系统为：

```math
e_{k+1}=(A_d-B_dK)e_k .
```

在本项目的 4D 双积分模型中，`(A_d,B_d)` 可控；若 `Q\succeq0`、`R\succ0`，
并且 `Q` 对不可稳定模态有足够惩罚，则 DARE 存在稳定化解，闭环矩阵：

```math
A_{cl}=A_d-B_dK
```

的特征值位于单位圆内：

```math
|\lambda_i(A_{cl})|<1 .
```

C++ contract test `test_lqr_controller_4d_artstein.cpp` 会显式检查该闭环稳定性。

## 6. 从 LQR 输入到 ROS 速度命令

LQR 输出的 `u_k` 是 force-like 等效输入，不直接发布到底盘。ROS 全向底盘接收的是速度命令，
因此控制器按与数值仿真一致的方式把 `u_k` 积分为下一步 map 系速度：

```math
v_{cmd,raw}=v_{2,h}+h u_k/m .
```

其中 `v_{2,h}` 是预测补偿后的 Follower 速度分量。随后节点执行：

```text
map 系速度
  -> 按 follower yaw 旋转到 body 系
  -> max_linear_vel 分量限幅
  -> min_cmd_vel 死区补偿（默认 0）
  -> yaw P+feedforward 控制
  -> 轮速约束 + max_linear_accel/max_angular_accel 加速度限幅
  -> 发布 cmd_vel
```

发布后，节点再把最终 body 系速度旋回 map 系，回写 `vx_cmd_map_/vy_cmd_map_`，
并存入 Artstein 历史缓冲。这样 predictor 使用的是实际发布后的速度命令历史，
而不是未经限幅的理想 LQR 输出。

## 7. 与 Artstein 预测补偿层的关系

`artstein_lqr_delay` 不是严格 delayed LQR。它复用当前项目的补偿层：

```text
测量状态 + 历史 cmd_vel
  -> Artstein 输入时延补偿 Td
  -> 一阶电机响应前向预测 tau
  -> 等效 4D 双积分状态
  -> DARE-LQR
```

因此论文定位应写成“共享同一预测补偿层的离散 LQR 基线”，不是“原始时滞执行器系统的最优控制器”。

更准确的理论表述是：

```text
本文先利用 Artstein 型输入时延补偿和一阶执行器前向预测，将实测 follower 状态映射为
等效双积分预测状态；随后在该预测状态上构造无显式时延的 DARE-LQR 对照器。
```

这样做的目的不是证明 LQR 对真实延迟执行器严格最优，而是保证对照变量干净：

```text
同一 EKF/TF 状态通道
同一 Artstein Td 补偿
同一 tau 前向预测
同一离散编队点切换
同一 cmd_vel 后处理
仅替换上层平移控制律: HPC vs LQR
```

若后续要构造严格 delayed LQR，需要把输入队列、纯时延或执行器状态显式增广进离散系统，
再重新求解增广系统的 DARE。该工作量和理论定位不同，不属于当前第一版 LQR 对照组。

## 8. 为什么使用 DARE 而不是 CARE

本项目第一版 LQR 采用 DARE，原因如下：

1. ROS 控制器是采样控制系统，控制周期固定为 `h=1/control_rate`。
2. 数值仿真和 ROS 节点使用同一离散 ZOH 模型，避免“连续设计、离散执行”造成额外差异。
3. 速度命令生成本身是离散积分：

```math
v_{cmd,k+1}=v_{2,h,k}+h u_k/m .
```

因此 DARE-LQR 与实现语义更一致。若使用 CARE，需要额外说明连续反馈在离散采样和限幅后的近似误差。

## 9. 与 HPC 的公平对照边界

LQR 对照组用于回答“如果上层不使用齐次非线性 warping，而使用标准线性二次调节器，会得到怎样的表现”。
为了保证结论可解释，实验中保持以下内容一致：

```text
leader/follower namespace
EKF odometry + TF map 投影
Artstein 历史 cmd_vel 缓冲
tau/Td 预测补偿参数
m_p/radius/tol 编队点切换
max_linear_vel/max_linear_accel/wheel_max_omega 约束
yaw P+feedforward 控制
sim_motor_delay.py 延迟注入方式
```

不同之处仅为：

```text
4D Artstein-HPC: 上层使用齐次控制律
4D Artstein-LQR: 上层使用 DARE-LQR 线性反馈
```

因此，默认实验不把 `artstein_mpc_delay` 纳入主对照；MPC 是另一个优化控制基线，应单独成组比较。

## 10. 运行命令

从源码仓库根目录运行：

```bash
python3 homo_multirobot_formation_control/scripts/sim_4d_artstein_lqr_compare.py
```

默认输出目录：

```text
homo_multirobot_formation_control/analysis/results/4d_artstein_lqr/
```

主要输出：

```text
circle_no_delay_hpc_vs_lqr.png
circle_delay_hpc_vs_lqr.png
circle_lqr_all_compare.png
summary_metrics.csv
timeseries_circle_lqr_compare.csv
```

## 11. 默认参数

```text
dt=0.05
circle_tmax=45.0
tau=0.43
Td=0.22
mass=2.0
radius=2.0
m_p=4
max_linear_vel=0.5
max_linear_accel=0.4
Q_lqr=diag(40,40,1,1)
R_lqr=diag(0.02,0.02)
```

## 12. 默认结果

2026-08-16 默认参数下，关键指标如下：

```text
case                   tail_mean_distance  final_distance  mean_cmd_delta
artstein_hpc_no_delay  0.0141              0.0129          0.0197
artstein_lqr_no_delay  0.0100              0.0100          0.0022
original_4d_delay      0.0711              0.0788          0.0119
artstein_hpc_delay     0.0115              0.0119          0.0193
artstein_lqr_delay     0.0157              0.0157          0.0020
```

结论：

- 共享预测补偿层后，`artstein_lqr_delay` 明显优于无补偿的 `original_4d_delay`。
- 默认权重下，`artstein_lqr_delay` 稳态误差略高于 `artstein_hpc_delay`。
- LQR 命令变化量更小，说明该基线更平滑但更偏保守。

## 13. 验证命令

```bash
python3 -m pytest homo_multirobot_formation_control/test/test_sim_4d_artstein_mpc_compare.py -q
python3 homo_multirobot_formation_control/scripts/sim_4d_artstein_lqr_compare.py
```

## 14. ROS 实现

ROS 2 节点：

```text
formation_control_node_4d_artstein_lqr
```

Launch 文件：

```text
launch/formation_single_follower_4d_artstein_lqr.launch.py
```

启动示例：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_4d_artstein_lqr.launch.py \
  leader_ns:=/robot1 follower_ns:=/robot2 \
  tau:=0.43 Td:=0.22 control_rate:=20.0 \
  mass:=2.0 radius:=2.0 max_linear_vel:=0.5 max_linear_accel:=0.4 min_cmd_vel:=0.0 \
  q_px:=40.0 q_py:=40.0 q_vx:=1.0 q_vy:=1.0 r_ux:=0.02 r_uy:=0.02
```

该 ROS 节点沿用 4D Artstein-HPC/MPC 的 EKF/TF 状态获取、Artstein 输入延迟补偿、
一阶执行器前向预测、yaw 控制和 cmd_vel 后处理，只把上层平移控制律替换为 DARE-LQR。

延迟仿真示例：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_4d_artstein_lqr.launch.py \
  leader_ns:=/robot1 follower_ns:=/robot2 \
  tau:=0.43 Td:=0.22 control_rate:=20.0 \
  mass:=2.0 radius:=2.0 max_linear_vel:=0.5 max_linear_accel:=0.4 min_cmd_vel:=0.0 \
  q_px:=40.0 q_py:=40.0 q_vx:=1.0 q_vy:=1.0 r_ux:=0.02 r_uy:=0.02 \
  use_motor_delay:=true motor_tau:=0.43 transport_delay:=0.22 delay_max_accel:=2.0
```

`use_motor_delay:=true` 时链路为：

```text
formation_control_node_4d_artstein_lqr -> cmd_vel_raw
sim_motor_delay.py                     -> cmd_vel
```

`delay_max_accel` 建议先设为 `2.0` 验证一阶延迟补偿链路；若设为 `0.4`，
delay node 会额外加入速度斜率饱和，这已经超出当前 DARE-LQR predictor 模型。

## 15. 不同 leader 速度下的数值观察

固定噪声：

```text
pos_noise = 0.02 m
vel_noise = 0.03 m/s
```

### leader 速度 0.5 m/s

参数：

```text
leader_radius = 2.0 m
leader_omega = 0.25 rad/s
```

关键结果：

```text
case                         tail_mean_distance  final_distance
artstein_hpc_no_delay_noise  0.1410              0.1245
artstein_lqr_no_delay_noise  0.1741              0.1543
original_4d_delay_noise      1.1707              1.1785
artstein_hpc_delay_noise     0.5298              0.5344
artstein_lqr_delay_noise     0.4339              0.4328
```

此时 `max_linear_vel=0.5` 基本成为主导约束，`speed_clip_ratio` 接近 1。该工况可作为高速饱和压力测试，
但不适合作为纯控制律公平性结论。

### leader 速度 0.3 m/s

参数：

```text
leader_radius = 2.0 m
leader_omega = 0.15 rad/s
```

关键结果：

```text
case                         tail_mean_distance  final_distance
artstein_hpc_no_delay_noise  0.0566              0.0299
artstein_lqr_no_delay_noise  0.0444              0.0288
original_4d_delay_noise      0.0969              0.0910
artstein_hpc_delay_noise     0.0512              0.0354
artstein_lqr_delay_noise     0.0474              0.0400
```

0.3 m/s 下速度饱和显著减轻，HPC 与 LQR 差距较小；该工况更适合作为论文中的公平对照组之一。
