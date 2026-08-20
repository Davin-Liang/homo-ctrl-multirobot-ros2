# 4D Artstein-MPC 对照组预研与实现设计

## 1. 目标

本文档用于后续实现一个 **4D Artstein-MPC** 对照组。它不是替代当前 4D
Artstein-HPC 的主方法，而是作为论文和实验中的公平比较对象：

```text
4D Artstein-HPC:
  Artstein/电机预测补偿层 + 原始 4D 双积分齐次控制器

4D Artstein-MPC:
  Artstein/电机预测补偿层 + 4D 双积分线性 MPC
```

核心原则是：**延迟补偿、ROS 状态获取、车体系速度转换、yaw 控制、速度/加速度/轮速后处理都保持与
4D Artstein-HPC 一致，只替换上层平移控制律**。这样实验差异才主要来自
HPC 和 MPC，而不是来自工程链路不同。

## 2. 已查找的相关文献

### 2.1 Artstein / Predictor feedback

1. Z. Artstein, “Linear Systems with Delayed Controls: A Reduction,”
   IEEE Transactions on Automatic Control, 27(4):869-879, 1982.
   DOI: 10.1109/TAC.1982.1103023.
   资料页：
   https://weizmann.elsevierpure.com/en/publications/linear-systems-with-delayed-controls-a-reduction/

   这篇是本项目 Artstein 变换的根文献。它说明线性输入延迟系统可以通过状态变换转成无显式延迟的系统，
   并且可在 reduced system 上讨论可控性、稳定化和优化问题。对本项目的启发是：
   MPC 不必直接处理输入死区延迟，可以先把 follower 测量状态映射为 Artstein 预测状态，再让 MPC
   面对无延迟 4D 双积分模型。

2. A. Z. Manitius and A. W. Olbrot,
   “Finite Spectrum Assignment Problem for Systems with Delays,”
   IEEE Transactions on Automatic Control, 24(4):541-553, 1979.
   DOI: 10.1109/TAC.1979.1102124.
   资料页：
   https://ui.adsabs.harvard.edu/abs/1979ITAC...24..541M/abstract

   这篇是 predictor feedback / finite spectrum assignment 的经典工作。它和 Artstein reduction
   属于同一类思想：用预测状态或积分型变换处理输入延迟，而不是简单调小反馈增益硬扛延迟。

3. M. Krstic, *Delay Compensation for Nonlinear, Adaptive, and PDE Systems*,
   Birkhauser, 2009.

   这本书可作为延迟补偿理论背景。当前项目不需要照搬非线性 predictor 的完整证明，但可以在论文中说明：
   本文只使用线性执行器近似上的 Artstein/predictor layer，避免把延迟状态直接并入 4D HPC 或 MPC 主状态。

4. I. Karafyllis and M. Krstic,
   *Predictor Feedback for Delay Systems: Implementations and Approximations*,
   Birkhauser, 2017.

   这本书更偏实现和近似。对本项目有用的点是：真实系统中 predictor 通常需要数值积分、历史输入缓冲和近似预测；
   这和当前 4D Artstein-HPC 中 `follower_vcmd_history_` 的做法一致。

### 2.2 约束 MPC / 线性 MPC

5. D. Q. Mayne, J. B. Rawlings, C. V. Rao, and P. O. M. Scokaert,
   “Constrained Model Predictive Control: Stability and Optimality,”
   Automatica, 36(6):789-814, 2000.
   DOI: 10.1016/S0005-1098(99)00214-9.
   资料页：
   https://www.sciencedirect.com/science/article/abs/pii/S0005109899002149

   这是 constrained MPC 的经典综述。MPC 每个采样周期求解有限时域最优控制问题，只执行第一步输入；
   重要优势是可以显式处理输入和状态约束。对本项目而言，MPC 的价值不是“比 HPC 更创新”，而是给出一个
   约束优化基线，用来回答老师可能问的“为什么不用常见 MPC”。

6. P. O. M. Scokaert and J. B. Rawlings,
   “Constrained Linear Quadratic Regulation,”
   IEEE Transactions on Automatic Control, 43(8):1163-1169, 1998.
   DOI: 10.1109/9.704994.
   资料页：
   https://ieeexplore.ieee.org/document/704994/

   这篇可作为“有限维 QP + 线性二次调节 + 约束”的理论背景。4D Artstein-MPC 的 QP 形式与这类
   constrained LQR/MPC 十分接近。

7. J. B. Rawlings, D. Q. Mayne, and M. Diehl,
   *Model Predictive Control: Theory, Computation, and Design*, 2nd ed., 2017.
   在线书页：
   https://sites.engineering.ucsb.edu/~jbraw/mpc/

   推荐作为具体公式、稳定性条件、终端代价和实现细节的参考书。后续调权重时也可以参考其线性 MPC 结构。

8. B. Stellato, G. Banjac, P. Goulart, A. Bemporad, and S. Boyd,
   “OSQP: an Operator Splitting Solver for Quadratic Programs,”
   Mathematical Programming Computation, 12:637-672, 2020.
   DOI: 10.1007/s12532-020-00179-2.
   资料页：
   https://link.springer.com/article/10.1007/s12532-020-00179-2

   本项目已有 `osqp_interface.hpp`，6D MPC 已用 OSQP 构建 QP。4D Artstein-MPC 应直接复用
   OSQP 封装，不引入新求解器。

### 2.3 编队 / 多机器人 MPC

9. W. B. Dunbar and R. M. Murray,
   “Distributed Receding Horizon Control for Multi-Vehicle Formation Stabilization,”
   Automatica, 42(4):549-558, 2006.
   DOI: 10.1016/j.automatica.2005.12.008.
   资料页：
   https://www.sciencedirect.com/science/article/abs/pii/S0005109806000136

   这是多车编队 receding horizon control 的经典文献。它证明 MPC/RHC 用于多车编队是合理路线。
   但本项目第一版不做 distributed MPC，只做 single follower 对照组。

10. P. Wang and B. Ding,
    “Distributed RHC for Tracking and Formation of Nonholonomic Multi-Vehicle Systems,”
    IEEE Transactions on Automatic Control, 59(6):1439-1453, 2014.
    DOI: 10.1109/TAC.2014.2304175.
    资料页：
    https://ui.adsabs.harvard.edu/abs/2014ITAC...59.1439W/abstract

    这篇说明非完整约束车辆的 tracking/formation 可以用 distributed RHC 处理。它比本项目的 4D
    质点模型更复杂，因此适合作为“后续可扩展到非完整/多 follower”的参考，不适合作为第一版照搬对象。

11. H. Xiao, Z. Li, and C. L. P. Chen,
    “Formation Control of Leader-Follower Mobile Robots' Systems Using Model Predictive Control Based on Neural-Dynamic Optimization,”
    IEEE Transactions on Industrial Electronics, 63(9):5752-5762, 2016.
    DOI: 10.1109/TIE.2016.2542788.
    资料页：
    https://ieeexplore.ieee.org/document/7434605/

    这篇直接面向 leader-follower mobile robots，用 MPC 和优化方法处理编队。它可以支撑“移动机器人编队
    MPC 对照组”这个选题是合理存在的。

12. Z. Li, Y. Yuan, F. Ke, W. He, and C.-Y. Su,
    “Robust Vision-Based Tube Model Predictive Control of Multiple Mobile Robots for Leader-Follower Formation,”
    IEEE Transactions on Industrial Electronics, 67(4):3096-3106, 2020.
    DOI: 10.1109/TIE.2019.2913813.
    资料页：
    https://ieeexplore.ieee.org/document/8705683/

    这篇属于更强的 tube MPC / 鲁棒 MPC 路线。本项目暂不建议上 tube MPC，因为会把章节重心从
    Artstein-HPC 拉到鲁棒优化上，工作量会明显膨胀。

## 3. 推荐方案

推荐实现 **Artstein/执行器预测补偿状态上的 delay-free 4D linear MPC**：

```text
EKF + TF
  -> map 系 leader/follower 4D 测量状态 [px, py, vx, vy]
  -> leader 匀速预测 x1_h
  -> follower Artstein 积分 z2 = x2 + I(history)
  -> follower 电机前向预测 x2_h
  -> 4D linear MPC: x0 = x2_h, reference = leader prediction + selected formation offset
  -> 输出 map 系期望速度 out_map
  -> 与 4D Artstein-HPC 完全相同的后处理和 cmd_vel 发布
```

不要第一版就做“输入延迟增广 MPC”。原因：

- 增广延迟状态会把工程复杂度拉高，而且和现有 4D Artstein-HPC 不再是同一补偿层。
- Artstein 文献本身已经支持先做 reduction，再在 reduced system 上处理优化问题。
- 对照组目标是比较 HPC 和 MPC 的上层控制差异，不是比较两种延迟补偿结构。

也不要把 yaw 放入这个 4D Artstein-MPC。yaw 仍然沿用现有 4D 节点的 `Kp_yaw + K_ff`
独立控制。这样可以避免 6D Artstein 中已经遇到的 yaw/平移耦合问题。

### 3.1 MPC 是否必须需要 Artstein 延迟层

严格来说，**MPC 不天然必须使用 Artstein 延迟层**。MPC 处理输入延迟通常有三种选择：

```text
1. 不处理延迟：直接用无延迟模型做 MPC；
2. 增广延迟状态：把输入队列、电机一阶滞后或 Pade 近似写进 MPC 状态；
3. Predictor/Artstein reduction：先把测量状态映射成无显式延迟的预测状态，再做普通 MPC。
```

对本文项目，推荐第 3 种。原因不是“MPC 离不开 Artstein”，而是“为了公平比较 4D Artstein-HPC 和
4D Artstein-MPC，二者应共享同一套延迟补偿层”。这样实验变量只有上层控制律：

```text
同一 Artstein/prediction layer + HPC
vs
同一 Artstein/prediction layer + MPC
```

如果做一个普通 4D MPC baseline，也可以不加 Artstein；但它在有 `motor_tau/transport_delay`
的实验中会同时受到延迟建模不足影响，和 4D Artstein-HPC 的对比会变成：

```text
有延迟补偿的 HPC
vs
无延迟补偿的 MPC
```

这个对照不够干净。若想做更强的 MPC baseline，则应另开一个“delay-augmented MPC”版本，但那已经是
另一个工作量较大的控制器，而不是当前建议的第一版对照组。

### 3.2 理论成立条件与近似边界

严格理论上，Artstein reduction 处理的是线性输入时延系统。当前 4D Artstein-HPC 的工程实现还额外加入了
一阶电机滞后前向预测：

```text
measured x2
  -> Artstein integral compensates Td
  -> exp(A_a Td) back-mapping
  -> forward prediction over tau
  -> x2_h = [p_h, v_h]
```

因此，送入 MPC 的 `x2_h` 更准确地说是**延迟/执行器预测补偿后的等效 4D 双积分状态**，而不是纯粹的
Artstein 状态 `z`。在论文中建议这样表述：

```text
本文先利用 Artstein 型输入时延补偿和一阶执行器前向预测，将实测 follower 状态映射为等效双积分预测状态；
随后在该预测状态上构造无显式延迟的线性 MPC。
```

这个说法比“Artstein 直接把真实底盘完全变成无延迟双积分系统”更稳。真实底盘仍有模型误差、加速度限幅、
轮速限幅和低层控制器非线性，所以本 MPC 对照组的理论定位应是：

```text
基于同一预测补偿层的约束优化对照器，而不是对真实延迟执行器的严格最优控制器。
```

若后续要写严格的 delayed MPC 理论，则需要把输入队列或执行器状态纳入 MPC 预测模型，并重新处理状态/输入约束；
这超出当前 4D Artstein-MPC 对照组的范围。

## 4. 4D MPC 数学模型

状态定义：

```math
x = [p_x, p_y, v_x, v_y]^T .
```

输入定义：

```math
u = [u_x, u_y]^T .
```

为了和现有 4D Artstein-HPC 保持一致，`u` 仍解释为 force-like 的等效控制输入：

```math
\dot{p}_x = v_x,\quad
\dot{p}_y = v_y,\quad
\dot{v}_x = u_x/m,\quad
\dot{v}_y = u_y/m .
```

连续时间矩阵：

```math
A =
\begin{bmatrix}
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0
\end{bmatrix},
\qquad
B =
\begin{bmatrix}
0 & 0 \\
0 & 0 \\
1/m & 0 \\
0 & 1/m
\end{bmatrix}.
```

这里的 `m` 与当前 4D Artstein-HPC 中的 `mass` 一致，是双积分模型的输入缩放参数；它不应在论文中强行解释为
真实全向底盘的精确物理质量。若把 MPC 输入直接定义为加速度，则可改写为
`B=[0,0;0,0;1,0;0,1]`，但那会和现有 4D Artstein-HPC 的
`B=[0,0;0,0;1/m,0;0,1/m]` 不完全一致。为了公平比较，第一版建议保留 force-like 输入定义。

推荐第一版 MPC 使用精确 ZOH 离散化：

```math
A_d =
\begin{bmatrix}
1 & 0 & h & 0 \\
0 & 1 & 0 & h \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix},
\qquad
B_d =
\begin{bmatrix}
\frac{h^2}{2m} & 0 \\
0 & \frac{h^2}{2m} \\
\frac{h}{m} & 0 \\
0 & \frac{h}{m}
\end{bmatrix}.
```

这里 `h = 1/control_rate`。如果想和现有 HPC 数值积分更近，也可以提供
`mpc_discretization:=zoh|euler`，其中 Euler 为：

```math
A_d = I + hA,\qquad B_d = hB .
```

但文档建议第一版只实现 ZOH，少一个参数，调试更稳。

## 5. QP 形式

预测时域：

```math
N = \text{mpc\_horizon}, \qquad T = Nh .
```

决策变量采用非紧凑形式，和当前 6D MPC 一致：

```math
z = [x_0, u_0, x_1, u_1, \ldots, x_{N-1}, u_{N-1}, x_N]^T .
```

代价函数：

```math
\min_z
\sum_{k=0}^{N-1}
\left[
(x_k-x_{\mathrm{ref},k})^T Q (x_k-x_{\mathrm{ref},k})
+ u_k^T R u_k
\right]
+ (x_N-x_{\mathrm{ref},N})^T Q_f (x_N-x_{\mathrm{ref},N}) .
```

默认权重建议：

```text
mpc_horizon       = 30 or 40     # 20Hz 下 1.5-2.0s
q_px, q_py        = 10.0
q_vx, q_vy        = 1.0
r_ux, r_uy        = 0.05
terminal_factor   = 5.0 or 10.0
```

约束：

```math
x_0 = x_{2,h}
```

```math
x_{k+1} = A_d x_k + B_d u_k,\quad k=0,\ldots,N-1 .
```

```math
|u_{x,k}| \le u_{\max},\qquad |u_{y,k}| \le u_{\max}.
```

因为 `u` 是 force-like 等效输入，所以

```math
u_{\max}=m\cdot a_{\max}.
```

这里 `a_max` 对应 ROS 参数 `max_linear_accel`。这样 MPC 内部加速度约束和后处理中的
`KinematicConstraint` 加速度限幅在量纲上保持一致，但二者不是数学上完全相同的约束：

```text
MPC 内部约束: map 系预测模型中的 |u/m| <= a_max
后处理约束:  发布前车体系 cmd_vel 的 slew rate / 轮速约束
```

因此 MPC 内部约束用于提前抑制激进预测，最终实际发布命令仍以后处理结果为准。若日志中
`KinematicConstraint` 频繁触发，说明 MPC 内部约束、坐标变换和底盘真实约束仍未完全对齐，需要调低
`max_linear_vel/max_linear_accel` 或增大 `R`。

速度约束建议先用 box bound：

```math
|v_{x,k}| \le v_{\max},\qquad |v_{y,k}| \le v_{\max}.
```

其中 `v_max=max_linear_vel`。严格的圆形速度约束
`\sqrt{v_x^2+v_y^2}\le v_{\max}` 是二次约束，不适合直接放入 OSQP 线性 QP；第一版不要做。

## 6. 参考轨迹构造

为了和现有 4D Artstein-HPC 可比，MPC 的参考点应沿用原始离散多边形编队点：

```math
d_j =
\begin{bmatrix}
-r\cos(2\pi j/m_p) \\
-r\sin(2\pi j/m_p) \\
0 \\
0
\end{bmatrix},
\qquad j=0,\ldots,m_p-1 .
```

当前 follower 使用哪个 `d_j`，应尽量复用或复制 `LpcController` 的切换逻辑：

```text
1. 初始化时选择 4D 误差范数 ||x2 - x1 - d_j|| 最近的离散编队点；
2. 控制过程中只在新目标比旧目标至少好 tol 时切换；
3. 切换后记录 target index，日志打印 target。
```

这里的距离不是单纯位置距离，而是当前 `LpcController` 中使用的 4D 范数，速度误差也会参与目标选择。
如果后续为了可解释性改成纯位置距离，应在 HPC 和 MPC 两边同时改，否则对照不公平。

预测时域内的参考状态：

```math
x_{\mathrm{ref},k} = x_{1,h}(k) + d_j .
```

其中 leader 预测采用匀速模型：

```math
p_{1,h}(k) = p_{1,h}(0) + kh\,v_{1,h}(0),\qquad
v_{1,h}(k) = v_{1,h}(0).
```

因为 4D 状态全在 map 系，编队偏移也是 map 系固定偏移。这样 follower 轨迹会自然成为 leader
圆轨迹的平移版本，便于和 4D Artstein-HPC 对比。

需要注意：若 leader 实际做圆周运动，长时域内“匀速直线外推”会偏离真实圆轨迹。第一版仍推荐使用匀速外推，
因为当前 4D Artstein-HPC 也只基于当前预测状态和当前速度做反馈，这样对照更公平。若后续使用
`virtual_leader_circle.py` 的已知解析轨迹生成整段 MPC reference，MPC 可能会更强，但实验解释会变成
“HPC 当前反馈 vs MPC 已知未来参考”，不再是最干净的控制律对照。

## 7. 工程实现建议

新增文件建议：

```text
include/homo_multirobot_formation_control/mpc_controller_4d_artstein.hpp
include/homo_multirobot_formation_control/formation_control_node_4d_artstein_mpc.hpp
src/formation_control_node_4d_artstein_mpc.cpp
src/main_4d_artstein_mpc.cpp
launch/formation_single_follower_4d_artstein_mpc.launch.py
```

推荐直接复用：

```text
include/homo_multirobot_formation_control/osqp_interface.hpp
include/homo_multirobot_formation_control/homo_controller_4d_artstein.hpp
src/formation_control_node_4d_artstein.cpp 中的状态获取、Artstein预测、后处理链路
launch/formation_single_follower_4d_artstein.launch.py 中的 delay node 接线
```

`MpcController4DArtstein` 只负责：

```text
1. 保存 4D MPC 参数；
2. 维护当前离散编队 target；
3. 构建线性 MPC QP；
4. 调用 OSQP；
5. 返回第一步预测速度 x_{1|0}.tail<2>()。
```

推荐第一版让 MPC 直接输出 `out_map = x_{1|0}.tail<2>()`，而不是把优化输入 `u_0` 直接作为
`cmd_vel` 发布，也不是在节点外再重复积分一次。
理由是：

- MPC 优化变量中已经包含速度状态；
- 可以让速度约束直接作用在预测速度上；
- 与 `cmd_integrator_base:=pred` 的 4D Artstein-HPC 行为更接近：上层直接给出预测状态下的下一步速度命令。

这里 `u_0` 的物理意义是第一步 force-like 等效输入，对应加速度 `u_0/m`。它只用于预测和日志分析，
不直接发布到底盘。实际写入 Artstein 历史缓冲的仍应是经过车体系旋转、速度限幅、最小速度补偿、
yaw 叠加、轮速/加速度约束之后的最终发布速度，再旋回 map 系后的 `v_cmd_map`。

内部仍需记录 `u_0` 便于日志分析：

```text
MPC_DIAG target=... status=... iter=... solve_ms=...
         u0=(...,...) x1v=(...,...) ref_err=(...)
```

## 8. ROS 参数建议

沿用 4D Artstein-HPC 参数：

```text
leader_ns
follower_ns
use_sim_time
m_p
radius
tol
mass
tau
Td
Kp_yaw
K_ff
control_rate
leader_vel_lpf_tau
min_cmd_vel
wheel_radius
base_radius
wheel_max_omega
max_linear_vel
max_angular_vel
max_linear_accel
max_angular_accel
use_motor_delay
motor_tau
transport_delay
delay_max_accel
```

新增 MPC 参数：

```text
mpc_horizon:=40
q_px:=10.0
q_py:=10.0
q_vx:=1.0
q_vy:=1.0
r_ux:=0.05
r_uy:=0.05
terminal_factor:=10.0
mpc_max_iter:=2000        # 若后续扩展 osqp_interface，可接入
mpc_eps_abs:=1e-3         # 第一版可先写死在 osqp_interface
mpc_eps_rel:=1e-3
```

不再使用的 HPC 专属参数：

```text
use_hpc
hpc_c_min
initial_min_lambda
switch_min_lambda
omega_d
cmd_integrator_base
```

为了 launch 命令便于公平对比，可以保留这些参数但不传给 MPC 节点，或者在文档中明确
4D Artstein-MPC 的对应调参项是 `Q/R/N`，不是 `lambda/c_min/omega_d`。

## 9. 与 4D Artstein-HPC 的公平对照实验

建议实验组：

```text
A. 4D Artstein-HPC, no delay
B. 4D Artstein-MPC, no delay
C. 4D Artstein-HPC, motor_tau=0.43, transport_delay=0.22
D. 4D Artstein-MPC, motor_tau=0.43, transport_delay=0.22
```

固定共同参数：

```text
mass:=2.0
radius:=2.0
m_p:=4
tol:=0.1
control_rate:=20.0
max_linear_vel:=0.4 or 0.5
max_linear_accel:=0.25/0.3/0.4 分组测试
max_angular_vel:=0.3 or 0.5
max_angular_accel:=0.3 or 0.5
tau:=0.43
Td:=0.22
motor_tau:=0.43
transport_delay:=0.22
delay_max_accel:=与 max_linear_accel 一致
```

比较指标：

```text
1. follower 轨迹是否为 leader 轨迹的平移版本；
2. 当前选中编队点误差范数的均值、最大值、RMS；
3. cmd_vel map/body 速度曲线是否更平滑；
4. 加速度/轮速限幅触发比例；
5. OSQP solve time 和失败次数；
6. 延迟存在时的超调和震荡幅度。
```

预期现象：

- MPC 在硬约束下可能比 HPC 更平滑，但不一定更快收敛。
- 如果 `R` 太小，MPC 会很激进，速度曲线可能比 HPC 更尖。
- 如果 `R` 太大或 `N` 太短，MPC 会显得保守，跟踪滞后会变明显。
- 如果后处理的 `KinematicConstraint` 经常触发，说明 MPC 内部约束和外部真实约束不一致，需要降低
  `max_linear_vel/max_linear_accel` 或增大 `R`。

## 10. 论文写法建议

4D Artstein-MPC 在毕业论文中适合放在“对照实验方法”或“基准控制器设计”小节，而不是主创新章节。

可以这样表述：

```text
为验证 Artstein-HPC 的控制特性，本文进一步构造了一个共享相同延迟补偿层的
Artstein-MPC 对照器。该对照器使用同一 4D 双积分预测状态、同一编队目标切换逻辑和同一底盘约束后处理，
仅将上层齐次反馈替换为有限时域约束二次规划。由此可以把实验差异主要归因于齐次控制律与 MPC
优化控制律本身。
```

创新性定位要克制：

```text
Artstein-MPC 本身不是本文主创新；
它的价值是提供一个约束优化基线，增强 4D Artstein-HPC 实验论证的说服力。
```

如果后续实验发现 MPC 性能更好，也不影响课题结构。可以写成：

```text
HPC 的优势在于解析控制律、计算量低、结构清晰；
MPC 的优势在于约束表达直接、调参直观；
本文主方法强调 Artstein-HPC 在低算力 ROS 实车链路中的实时性和延迟鲁棒表现。
```

## 11. 下一轮实现检查清单

新开对话时可直接给 Codex 这段任务：

```text
根据 homo_multirobot_formation_control/doc/4d_artstein_mpc_design.md
实现 4D Artstein-MPC 对照组。要求复用 4D Artstein-HPC 的 Artstein 预测层、
状态获取和 cmd_vel 后处理链路，只把上层控制器替换成 4D 线性 MPC。
先做 C++ ROS 节点和 launch，构建通过后给出 Gazebo 测试命令。
```

实现顺序建议：

```text
1. 新增 MpcController4DArtstein，并用离线单元/小程序验证 QP 维度和单步输出；
2. 复制 4D Artstein 节点形成 MPC 节点，只替换步骤 5；
3. 新增 launch 文件，保持 delay node 接线一致；
4. CMake 增加 target，确认 osqp_vendor 可用；
5. workspace 根目录 colcon build；
6. no-delay Gazebo 冒烟测试；
7. delay Gazebo 对照测试；
8. 更新 README / BUG_RECORD / 文档。
```

第一版完成标准：

```text
1. 节点能以 20Hz 稳定输出 cmd_vel；
2. OSQP 大多数周期 status=solved；
3. 无延迟下 follower 能收敛到离散编队圆上的一个目标点；
4. 有延迟下轨迹可记录并能和 4D Artstein-HPC 同图比较；
5. 日志能看出 target、solve_ms、u0、速度限幅/轮速限幅触发情况。
```

## 12. 数值仿真预验证记录

2026-08-08 已完成 Python 数值仿真预验证，并已落地 ROS C++ 对照节点。新增脚本：

```text
homo_multirobot_formation_control/scripts/sim_4d_artstein_mpc_compare.py
```

对应说明文档：

```text
homo_multirobot_formation_control/doc/4d_artstein_mpc_simulation.md
```

ROS C++ 实现文件：

```text
include/homo_multirobot_formation_control/mpc_controller_4d_artstein.hpp
include/homo_multirobot_formation_control/formation_control_node_4d_artstein_mpc.hpp
src/formation_control_node_4d_artstein_mpc.cpp
src/main_4d_artstein_mpc.cpp
launch/formation_single_follower_4d_artstein_mpc.launch.py
```

仿真保持以下公平对照原则：

```text
4D Artstein-HPC = 同一 Artstein/执行器预测补偿层 + 4D HPC 上层控制律
4D Artstein-MPC = 同一 Artstein/执行器预测补偿层 + 4D linear MPC 上层控制律
```

MPC 使用 4D 双积分 ZOH 离散化，输出 `x_{1|0}.tail(2)` 作为下一步 map 系速度命令，
不直接输出 `u0`，也不在 MPC 外层重复积分。

默认 20Hz、`tau=0.43`、`Td=0.22`、`radius=2.0`、`mass=2.0`、
`max_linear_vel=0.5`、`max_linear_accel=0.4`、`Q=diag(40,40,1,1)`、
`R=diag(0.02,0.02)`、`mpc_horizon=30` 下的结果目录：

```text
homo_multirobot_formation_control/analysis/results/4d_artstein_mpc/
```

关键指标：

```text
case                   tail_mean_distance  final_distance  mean_cmd_delta  mean_solve_ms  solver_failures
artstein_hpc_no_delay  0.0141              0.0129          0.0197          0.000          0
artstein_mpc_no_delay  0.0100              0.0100          0.0020          0.998          0
original_4d_delay      0.0711              0.0788          0.0119          0.000          0
artstein_hpc_delay     0.0115              0.0119          0.0193          0.000          0
artstein_mpc_delay     0.0159              0.0159          0.0020          3.052          0
```

调参记录：初始 `Q=diag(10,10,1,1), R=diag(0.05,0.05)` 时，delay 场景
`artstein_mpc_delay` 的尾段误差约为 `0.0259`。将位置权重提高到 `q_px=q_py=40`，
并将输入惩罚降到 `r_ux=r_uy=0.02` 后，尾段误差降至约 `0.0159`，且
`solver_failures=0`。继续提高 `Q` 或降低 `R` 还能略微压低误差，但当前 Python ADMM
原型会出现更多未收敛周期或求解时间上升，因此暂不作为默认参数。

预验证结论：

- no-delay 下 4D Artstein-MPC 能稳定收敛到离散编队目标，尾段误差与 4D Artstein-HPC 同量级。
- delay 下 4D Artstein-MPC 使用同一 Artstein/执行器预测补偿层后未发散，尾段误差明显小于
  original 4D + delay。
- 调参后 MPC 命令仍明显更平滑，但 delay 场景稳态误差仍略大于 Artstein-HPC；这符合“第一版对照组，
  非最优调参”的定位。
- ROS C++ 节点已使用仓库已有 `osqp_interface.hpp` 和 `ros-humble-osqp-vendor`，QP 形式与第 5 节一致。

ROS 延迟仿真联调补充：

- 静止 Leader + `use_motor_delay:=true` 时，若只依赖 4D linear MPC 的速度/加速度盒约束，
  Follower 在接近编队半径时仍可能因 `Td + tau` 执行器滞后继续内切，出现穿过 Leader 的风险。
- C++ 节点因此增加了默认启用的 `enable_radial_safety` 后处理层。该层不改变 QP 形式，只对 MPC 输出的
  map 系速度做径向限幅：当速度朝向 Leader 时，要求当前半径余量能覆盖 `Td + tau` 延迟滑行距离和
  `max_linear_accel` 下的刹停距离。
- Leader 运动时，径向限幅必须使用 Follower 相对 Leader 的径向速度；使用 Follower 绝对 map 速度会在
  绕圈轨迹中误判随动速度，导致控制命令间歇性被压到接近零。
- 该层用于 ROS/Gazebo 延迟注入下的碰撞安全保护；若需要严格比较裸 4D Artstein-MPC 上层控制律，可在
  launch 中设 `enable_radial_safety:=false`。
- `sim_motor_delay.py` 的 `delay_max_accel` 若设得过低，会额外引入速度斜率饱和。当前 Artstein 预测层建模的是
  `motor_tau + transport_delay`，不包含该饱和非线性；因此理论补偿验证阶段应先使用较大的
  `delay_max_accel`（如 `2.0`），避免把执行器限幅误当成纯一阶滞后。

验证记录：

```bash
source /opt/ros/humble/setup.bash
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select homo_multirobot_formation_control --event-handlers console_direct+
colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
ros2 launch homo_multirobot_formation_control formation_single_follower_4d_artstein_mpc.launch.py --show-args
```

其中 C++ 单元测试 `test_mpc_controller_4d_artstein` 验证：

```text
1. 4D 双积分 ZOH 离散化矩阵正确；
2. MPC 输出等于 x_{1|0}.tail(2)，不是 u0；
3. 输出速度满足 max_linear_vel 约束。
```

2026-08-08 ROS delay 调试补充：

- 静止 leader + delay 仿真时，如果 `min_cmd_vel:=0.03`，MPC 平衡点附近的 OSQP 微小速度残差会被最小速度补偿放大；
  no-delay 下该残差能快速纠偏，但 delay 链路会引入相位滞后，可能导致 follower 向 leader 内冲。
- 4D Artstein-MPC 仿真默认已改为 `min_cmd_vel:=0.0`；实物若需要死区补偿，应先在低速、小半径外侧场景验证。
- OSQP 默认保持 `eps_abs=eps_rel=1e-3`，但开启 `osqp_polish:=true`。该组合能在平衡点把残差压到近零，
  同时避免 `eps=1e-5` 在部分非平衡状态下达到最大迭代。

## 13. 参考文献

后续写毕业论文或小论文时，可从下面条目中选择引用。第 1-4 条用于支撑
Artstein/predictor 延迟补偿，第 5-8 条用于支撑 constrained MPC / QP 求解，第 9-12
条用于支撑 MPC 在多机器人编队中的合理性。

[1] Z. Artstein, “Linear Systems with Delayed Controls: A Reduction,”
*IEEE Transactions on Automatic Control*, vol. 27, no. 4, pp. 869-879, 1982.
DOI: 10.1109/TAC.1982.1103023.
https://weizmann.elsevierpure.com/en/publications/linear-systems-with-delayed-controls-a-reduction/

[2] A. Z. Manitius and A. W. Olbrot, “Finite Spectrum Assignment Problem for Systems with Delays,”
*IEEE Transactions on Automatic Control*, vol. 24, no. 4, pp. 541-553, 1979.
DOI: 10.1109/TAC.1979.1102124.
https://ui.adsabs.harvard.edu/abs/1979ITAC...24..541M/abstract

[3] M. Krstic, *Delay Compensation for Nonlinear, Adaptive, and PDE Systems*.
Birkhauser, 2009.

[4] I. Karafyllis and M. Krstic,
*Predictor Feedback for Delay Systems: Implementations and Approximations*.
Birkhauser, 2017.

[5] D. Q. Mayne, J. B. Rawlings, C. V. Rao, and P. O. M. Scokaert,
“Constrained Model Predictive Control: Stability and Optimality,”
*Automatica*, vol. 36, no. 6, pp. 789-814, 2000.
DOI: 10.1016/S0005-1098(99)00214-9.
https://www.sciencedirect.com/science/article/abs/pii/S0005109899002149

[6] P. O. M. Scokaert and J. B. Rawlings,
“Constrained Linear Quadratic Regulation,”
*IEEE Transactions on Automatic Control*, vol. 43, no. 8, pp. 1163-1169, 1998.
DOI: 10.1109/9.704994.
https://ieeexplore.ieee.org/document/704994/

[7] J. B. Rawlings, D. Q. Mayne, and M. Diehl,
*Model Predictive Control: Theory, Computation, and Design*, 2nd ed., 2017.
https://sites.engineering.ucsb.edu/~jbraw/mpc/

[8] B. Stellato, G. Banjac, P. Goulart, A. Bemporad, and S. Boyd,
“OSQP: an Operator Splitting Solver for Quadratic Programs,”
*Mathematical Programming Computation*, vol. 12, pp. 637-672, 2020.
DOI: 10.1007/s12532-020-00179-2.
https://link.springer.com/article/10.1007/s12532-020-00179-2

[9] W. B. Dunbar and R. M. Murray,
“Distributed Receding Horizon Control for Multi-Vehicle Formation Stabilization,”
*Automatica*, vol. 42, no. 4, pp. 549-558, 2006.
DOI: 10.1016/j.automatica.2005.12.008.
https://www.sciencedirect.com/science/article/abs/pii/S0005109806000136

[10] P. Wang and B. Ding,
“Distributed RHC for Tracking and Formation of Nonholonomic Multi-Vehicle Systems,”
*IEEE Transactions on Automatic Control*, vol. 59, no. 6, pp. 1439-1453, 2014.
DOI: 10.1109/TAC.2014.2304175.
https://ui.adsabs.harvard.edu/abs/2014ITAC...59.1439W/abstract

[11] H. Xiao, Z. Li, and C. L. P. Chen,
“Formation Control of Leader-Follower Mobile Robots' Systems Using Model Predictive Control Based on Neural-Dynamic Optimization,”
*IEEE Transactions on Industrial Electronics*, vol. 63, no. 9, pp. 5752-5762, 2016.
DOI: 10.1109/TIE.2016.2542788.
https://ieeexplore.ieee.org/document/7434605/

[12] Z. Li, Y. Yuan, F. Ke, W. He, and C.-Y. Su,
“Robust Vision-Based Tube Model Predictive Control of Multiple Mobile Robots for Leader-Follower Formation,”
*IEEE Transactions on Industrial Electronics*, vol. 67, no. 4, pp. 3096-3106, 2020.
DOI: 10.1109/TIE.2019.2913813.
https://ieeexplore.ieee.org/document/8705683/
