# 6D Artstein Disc 控制器讨论接力笔记

日期: 2026-07-28

本文档整理本次关于 `formation_single_follower_6d_disc`、原始 4D 齐次控制、以及拟新增 `6D Artstein Disc` 控制器的讨论，便于后续在另一台电脑继续。

## 1. 讨论目标

用户希望把 `formation_single_follower_6d_disc` 相对于原始 4D 控制器扩展出的两维，即偏航角和角速度通道，加入当前 `4d_artstein` 的延迟补偿思想中。

澄清后的目标不是在 4D Artstein 上做小补丁，而是新增一个独立控制器:

```text
formation_control_node_6d_artstein_disc
formation_single_follower_6d_artstein_disc.launch.py
```

目标控制器应结合:

- 6D Disc 的状态、误差、离散编队点和 6D HPC 框架
- 4D Artstein 的输入时延补偿和一阶电机前向预测思想

## 2. 当前代码中的两个基础控制器

### 2.1 4D Artstein

相关文件:

```text
include/homo_multirobot_formation_control/homo_controller_4d_artstein.hpp
src/formation_control_node_4d_artstein.cpp
launch/formation_single_follower_4d_artstein.launch.py
```

4D Artstein 的上层 HPC 状态仍是原始双积分器:

```math
x_h = [p_x, p_y, v_x, v_y]^T
```

模型矩阵为:

```math
A_h =
\begin{bmatrix}
0&0&1&0\\
0&0&0&1\\
0&0&0&0\\
0&0&0&0
\end{bmatrix},
\qquad
B_h =
\begin{bmatrix}
0&0\\
0&0\\
1/m&0\\
0&1/m
\end{bmatrix}
```

因此:

```math
A_h^2=0
```

4D Artstein 的补偿层处理实际执行器:

```math
\dot p = v
```

```math
\dot v = -\frac{1}{\tau}v + \frac{1}{\tau}u_c(t-T_d)
```

其中 `u_c` 是发布给底盘的速度命令。补偿流程为:

```text
测量状态 [p, v_real]
  + 历史 cmd_vel
  -> Artstein 积分消除 Td
  -> exp(A_a Td) 反映射
  -> 一阶电机 tau 前向预测
  -> 等效 4D 双积分状态 x_h
  -> 原始 4D HPC
  -> 积分为速度命令
  -> 发布 cmd_vel
```

注意: 4D Artstein 中 yaw 没有进入 HPC。yaw 当前是节点侧独立 P + feedforward:

```text
cmd.angular.z = Kp_yaw * yaw_error + K_ff * leader_angular_z
```

### 2.2 6D Disc

相关文件:

```text
include/homo_multirobot_formation_control/homo_controller_6d_disc.hpp
src/formation_control_node_6d_disc.cpp
launch/formation_single_follower_6d_disc.launch.py
```

6D Disc 的状态定义为:

```math
x = [p_x, p_y, \theta, v_x^b, v_y^b, \omega]^T
```

其中位置和偏航角在 map 系，速度在车体系。控制输入是广义力/力矩:

```math
u = [F_x, F_y, M_z]^T
```

实际 ROS 输出仍是 `cmd_vel`:

```text
linear.x  = v_x_cmd_body
linear.y  = v_y_cmd_body
angular.z = omega_cmd
```

6D Disc 内部先计算广义力/力矩，再通过前向 Euler 积分变成速度命令:

```math
v_{x,cmd} = v_x + h F_x/m
```

```math
v_{y,cmd} = v_y + h F_y/m
```

```math
\omega_{cmd} = \omega + h M_z/I
```

所以控制理论内部的量仍是力/加速度，底层接口接收的是速度命令。

## 3. 原始 4D 论文证明方法

本次明确论文路径:

```text
homo_multirobot_formation_control/doc/homogeneous_control.pdf
```

论文主线如下。

### 3.1 建模

Leader/Follower 动力学:

```math
m\ddot r_1 + Q_1(r_1,\dot r_1)=u_1
```

```math
m\ddot r_2 + Q_2(r_2,\dot r_2)=u_2
```

定义固定安全点 `d_i` 下的编队误差:

```math
e = r_2-r_1-d_i
```

当 `d_i` 固定时:

```math
m\ddot e = u+\gamma
```

其中 `gamma` 包含 Leader 输入、未知广义力和扰动。

令:

```math
x=[e_x,e_y,\dot e_x,\dot e_y]^T
```

得到:

```math
\dot x = Ax+B(u+\gamma)
```

### 3.2 线性控制器

先设计:

```math
u=K_1e+K_2\dot e=K_{lin}x
```

论文引入辅助变量:

```math
g=m\dot e+\lambda e
```

可得:

```math
\dot e=\frac{-\lambda e+g}{m}
```

```math
\dot g=\frac{K_2+\lambda I}{m}g
```

只要:

```math
K_1=\frac{\lambda I(K_2+\lambda I)}{m},\qquad K_2<-\lambda I<0
```

并且初始条件满足:

```math
\lambda \ge
\max\left\{
-\frac{m\dot e_x(0)}{e_x(0)},
-\frac{m\dot e_y(0)}{e_y(0)}
\right\}
```

则 `g(t)>=0`、`e(t)>=0`，从而得到不超调和渐近稳定。

### 3.3 齐次化升级

在线性控制器基础上，论文使用 generalized homogeneous control:

```math
u=K_0x+\|x\|_d^{1+\mu}(K_{lin}-K_0)d(-\ln\|x\|_d)x
```

取齐次范数:

```math
V(x)=\|x\|_d
```

利用齐次性和 Lyapunov 不等式推出:

```math
\dot V \le -\rho V^{1+\mu}
```

当:

```math
\mu<0
```

可得有限时间收敛:

```math
T \le \frac{V(x_0)^{-\mu}}{-\mu\rho}
```

扰动存在时，论文给出 input-to-state stability 形式的结论。

## 4. 关于 A 矩阵是否必须幂零

本次讨论的关键问题是: 6D Leader 运动时 `A_L` 不再幂零，这是否破坏齐次控制结构。

结论:

```text
A 幂零不是 generalized homogeneous control 的必要条件。
```

但要区分两类理论:

1. 积分链/全相对阶系统的齐次控制
   - 常见于 chain of integrators
   - 原始 4D 双积分器属于这一类
   - 此时 `A^2=0`，结构特别规整

2. generalized homogeneous linear systems
   - 直接处理一般线性系统 `dot x = Ax+Bu`
   - 要求通常是 `(A,B)` 可控，且能找到 `K` 使 `A+BK` Hurwitz
   - 不要求 `A` 本身幂零

仓库中的:

```text
include/homo_multirobot_formation_control/lpc2hpc_nd.hpp
```

使用的是第二类思想:

- 检查 `(A,B)` 可控
- 检查 `A+B*K` Hurwitz
- 做 block controllable decomposition
- 构造 `G0`
- 解 Lyapunov 方程得到 `P`
- 计算 `nu_min/nu_max`

因此 6D 的 `A_L` 非幂零不直接否定 HPC。

## 5. 为什么 4D 不需要冻结 Leader 速度，6D 需要

原始 4D 中:

```math
e=r_2-r_1-d_i
```

Leader 的运动进入:

```math
\gamma
```

而系统矩阵 `A` 保持常数:

```math
A =
\begin{bmatrix}
0&0&1&0\\
0&0&0&1\\
0&0&0&0\\
0&0&0&0
\end{bmatrix}
```

所以 4D 不需要冻结 Leader 速度。

6D 中误差定义在 Leader 车体系:

```math
e_p^L=R(-\theta_l)(p_f-p_l)
```

对该误差求导时，`R(-theta_l)` 也在变化，因此引入 Leader 角速度和速度耦合。6D Disc 使用的矩阵为:

```math
A_L =
\begin{bmatrix}
0 & \omega_l & -v_{y,l} & 1 & 0 & 0 \\
-\omega_l & 0 & v_{x,l} & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}
```

因此:

```math
A_L=A_L(v_{x,l},v_{y,l},\omega_l)
```

所谓“冻结”不是让 Leader 停住，而是在一次 HPC 参数计算中把当前 Leader 速度视为常值参数:

```math
A_L(t)\approx A_L(t_k)
```

然后对这个冻结矩阵计算 `K/G0/P/Gd`。

## 6. 6D 的 A 矩阵是不是三条独立双积分链

Leader 静止时:

```math
v_{x,l}=v_{y,l}=\omega_l=0
```

此时:

```math
A_0 =
\begin{bmatrix}
0&0&0&1&0&0\\
0&0&0&0&1&0\\
0&0&0&0&0&1\\
0&0&0&0&0&0\\
0&0&0&0&0&0\\
0&0&0&0&0&0
\end{bmatrix}
```

这是三条独立双积分链，且:

```math
A_0^2=0
```

Leader 速度不为零时:

```math
\dot e_x = \omega_l e_y - v_{y,l}e_\theta + e_{v_x}
```

```math
\dot e_y = -\omega_l e_x + v_{x,l}e_\theta + e_{v_y}
```

```math
\dot e_\theta = e_\omega
```

此时 x/y/yaw 通道被 Leader 运动耦合，不再是三条独立双积分链，通常:

```math
A_L^2 \ne 0
```

## 7. 6D Artstein Disc 的可行结构

推荐新增独立控制器，不修改现有 4D Artstein 和 6D Disc 的行为。

HPC 状态:

```math
x_h=[p_x,p_y,\theta,v_x^b,v_y^b,\omega]^T
```

执行器命令:

```math
u_c=[v_{x,cmd}^b,v_{y,cmd}^b,\omega_{cmd}]^T
```

补偿层:

```text
测量 6D follower 状态 + 3D cmd_vel 历史
  -> 6D Artstein 输入时延补偿
  -> 一阶速度/角速度执行器预测
  -> 等效 6D HPC 状态
```

HPC 层:

```text
使用 6D Disc 的离散编队点、tol 切换、leader 车体系误差
使用 lpc2hpc_nd(A_L, B_6, K_lin)
计算广义力/力矩 [F_x,F_y,M_z]
通过 Euler 积分变为 [vx_cmd,vy_cmd,omega_cmd]
发布 cmd_vel
```

注意: Artstein/Prediction 层不应把电机状态直接增广进 HPC。否则会从 6D 变成更高维执行器增广系统，齐次结构和证明对象都会改变。

## 8. 理论表述建议

不要写成:

```text
6D Artstein Disc 保持原始 4D 的幂零双积分结构。
```

应该写成:

```text
6D Artstein Disc 在固定离散编队点和当前冻结 Leader 速度下，构造局部 6D 线性误差系统。若该系统可控且线性闭环 Hurwitz，则可通过 generalized homogeneous control 将线性反馈升级为齐次反馈，实现无扰动下的局部有限时间收敛；在 Leader 速度慢变化、目标点切换、执行器参数误差和测量噪声存在时，结论降级为 ISS/实用稳定。
```

## 9. 实现风险和建议

### 9.1 主要风险

当前 6D Disc 的 `K_lin` 是分块通道增益:

```text
x 通道: e_x, e_vx
y 通道: e_y, e_vy
yaw 通道: e_theta, e_omega
```

但 `A_L` 中存在 Leader 速度耦合。分块增益不一定对所有 Leader 速度都保证:

```math
A_L+B_6K_{lin}
```

Hurwitz。

### 9.2 建议保护

新增 6D Artstein Disc 时，每次重算 `A_L` 后都应检查闭环特征值:

```text
max(real(eig(A_L+B_6K_lin))) < -margin
```

若不满足:

- 增大 `omega_d` 或阻尼
- 降低 Leader 速度适用范围
- 或退回保守线性控制/上一次稳定 HPC 参数

### 9.3 更严谨的 K 设计

更严谨的后续方向是对当前 `A_L` 做完整 MIMO 极点配置或 LQR，得到包含耦合项的 `K_lin`，而不是只用分块二阶通道增益。

## 10. 后续待办

1. 决定第一版 `6D Artstein Disc` 是否继续沿用当前 6D Disc 的分块 `K_lin`。
2. 若沿用，必须加入闭环 Hurwitz 检查和失败 fallback。
3. 设计 6D Artstein kernel:
   - history 从 `Vector2d` 扩为 `Vector3d`
   - kernel 从 `4x2` 扩为 `6x3`
   - 对 `vx_body/vy_body/omega` 使用一阶滞后模型
4. 新增:
   - `homo_controller_6d_artstein_disc.hpp`
   - `formation_control_node_6d_artstein_disc.hpp/cpp`
   - `main_6d_artstein_disc.cpp`
   - `formation_single_follower_6d_artstein_disc.launch.py`
   - CMake 目标
5. 编译验证应在 workspace 根目录执行，不要在源码仓库内直接 `colcon build`。

推荐编译命令:

```bash
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

