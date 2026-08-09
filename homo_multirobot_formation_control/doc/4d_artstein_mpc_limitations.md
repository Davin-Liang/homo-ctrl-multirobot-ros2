# 4D Artstein-MPC 问题记录与局限说明

本文记录 4D Artstein-MPC 对照组在数值仿真与 ROS 联调中暴露出的主要问题、
适用边界和工程局限。它不是设计失败记录，而是为了在论文与后续实验中明确：
**该对照组适合作为基线比较，但不应被表述为“对所有绕圈/大延迟场景都稳定有效”的控制器。**

## 1. 结论先行

4D Artstein-MPC 的核心链路是可工作的：

```text
TF + EKF 状态获取
  -> follower Artstein / 执行器预测层
  -> 4D linear MPC
  -> map/body 旋转 + 速度限幅 + 约束后处理
```

但在以下条件同时存在时，性能会明显退化：

- leader 以较高速度绕圈运动
- follower 存在输入死区和一阶执行器滞后
- 还叠加速度/加速度限幅
- leader 预测仍采用恒速外推
- 目标点是离散多边形，而不是连续边界投影

换句话说，这个 MPC 对照组对“直线或缓慢变化 leader”较友好，
但对“绕圈 + delay + 饱和 + 离散切换”组合并不鲁棒。

## 2. 复现条件

### 2.1 ROS 场景

典型启动参数：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_4d_artstein_mpc.launch.py \
  leader_ns:=/robot1 follower_ns:=/robot2 \
  tau:=0.43 Td:=0.22 control_rate:=20.0 \
  mass:=2.0 radius:=2.0 max_linear_vel:=0.5 max_linear_accel:=0.4 min_cmd_vel:=0.0 \
  mpc_horizon:=30 q_px:=40.0 q_py:=40.0 q_vx:=1.0 q_vy:=1.0 r_ux:=0.02 r_uy:=0.02 \
  use_motor_delay:=true motor_tau:=0.43 transport_delay:=0.22 delay_max_accel:=2.0
```

### 2.2 数值仿真场景

用与 ROS 更接近的 leader 圆周速度测试时，MPC 也会退化：

```bash
python3 homo_multirobot_formation_control/scripts/sim_4d_artstein_mpc_compare.py \
  --leader-radius 2.0 --leader-omega 0.25 \
  --max-linear-vel 0.5 --max-linear-accel 0.4 \
  --tau 0.43 --Td 0.22
```

这说明问题不只是 ROS 工程实现，而是控制结构本身在该工况下的适应性有限。

## 3. 观察到的现象

### 3.1 目标点会切换

在绕圈 leader 下，follower 与离散编队点的相对几何关系会变化，最近编队点会发生切换。
日志中可以看到 `target` 从 2 跳到 3，随后误差定义和控制方向也随之变化。

### 3.2 不是简单的“切换导致失控”

更准确的顺序是：

```text
跟踪误差先变大
-> follower 跨过离散编队点边界
-> 最近目标点切换
-> 新误差定义再次放大观感
```

也就是说，离散切换更像放大器，而不是首因。

### 3.3 开启 `use_motor_delay` 后明显变差

不开 `use_motor_delay` 时，闭环几乎立刻吃到命令，tracking 会好很多。
一旦开启 delay，真实速度会持续滞后于控制器输出，绕圈 leader 下这种相位滞后特别明显。

## 4. 根因分析

### 4.1 leader 预测过于粗

当前 4D Artstein-MPC 的 leader 预测仍是恒速外推：

```cpp
ref.head<2>() += k * dt * leader_state.tail<2>();
```

对于圆周 leader，这相当于用直线预测去追弧线运动。  
预测时域越长，这个误差越明显。

### 4.2 长 horizon 在 delay 下容易“追未来”

MPC 的代价函数会倾向于把整段预测窗口都压向未来参考。  
当参考本身有系统性偏差时，MPC 容易形成一种“往前追、但始终晚半拍”的行为。

### 4.3 速度/加速度/执行器饱和叠加

ROS 链路里还存在：

- `max_linear_vel`
- `max_linear_accel`
- `delay_max_accel`
- 车体系旋转后的逐轴 clamp
- 实际执行器一阶滞后

这些约束会让 MPC 的计划动作和真实动作进一步分离。

### 4.4 离散编队点对绕圈不友好

离散多边形编队适合“目标点切换少”的场景。  
在 leader 持续绕圈时，编队点边界会频繁被触碰，导致目标索引变动。

## 5. 这个对照组的局限

### 5.1 不适合直接表述为“绕圈鲁棒控制器”

它更适合表述为：

```text
同一 Artstein 预测层上的线性 MPC 对照组
```

而不是：

```text
对任意 leader 轨迹都优于 HPC 的方案
```

### 5.2 不适合拿来证明“离延迟越近越好”

在本项目里，MPC 的弱点主要暴露于：

- 大圆周速度
- 离散目标切换
- 执行器滞后
- 低层限幅

所以它不能被当成“只要加了 Artstein 就自然好用”的正例。

### 5.3 不适合与 HPC 直接比“通用优越性”

HPC 更像局部阻尼式反馈，收手更快；  
MPC 更像未来窗口优化，规划味更强。  
两者的优势维度不同，不宜简单说谁全面更好。

## 6. 对后续工作的建议

1. 若要继续保留 4D Artstein-MPC，建议把它定位为“方法对照组”，不是最终推荐算法。
2. 若要在绕圈 leader 上提升表现，优先考虑：
   - 缩短 horizon
   - 减弱恒速 leader 外推长度
   - 提高目标保持性，减少离散切换
   - 或改成连续边界投影目标
3. 若目标是“先证明能稳定跟踪”，4D Artstein-LQR 可能比当前 MPC 更合适。
4. 若目标是“最大鲁棒性”，则需要重新设计成显式 delay-augmented MPC，而不是当前这版简化 QP。

## 7. 本次实验结论

综合 ROS 日志与数值仿真结果，可以认为：

- `use_motor_delay:=false` 时效果好，说明基线反馈链本身没有大问题
- 加入 delay 后，MPC 在绕圈 leader 下明显退化
- 这个退化在数值仿真中也能复现
- 因而问题主要来自控制结构与任务工况不匹配，而不是单纯的 ROS 实现错误

这份记录的意义在于：后续如果老师问“为什么不用 MPC”，可以明确回答：

```text
我们做过同一 Artstein 预测层下的 MPC 对照组，
它在绕圈 + delay + 饱和工况下并不优于 HPC，因此更适合作为基线而非主方法。
```
