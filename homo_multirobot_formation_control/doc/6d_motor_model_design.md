# 6D 电机感知模型：将 STM32 执行器延迟纳入齐次控制器状态方程

> **目标**: 从 4D 质点模型 `[px, py, vx, vy]` 扩展为 6D 电机感知模型
> `[px, py, vx_cmd, vy_cmd, vx_real, vy_real]`，使 HPC 齐次控制器天然感知
> "指令速度 ≠ 实际速度"的物理现实，消除过度补偿引起的震荡。

## 1. 问题诊断（实物数据）

### 1.1 系统延迟链路

```
网络(L→F)        EKF 滤波         电机响应(主瓶颈)
   35ms            38ms              1243ms
    ██              ██          ████████████████████████████████
├─────────┼─────────┼──────────────────────────────────────────┤
0ms      35ms      73ms                                      1316ms

总延迟: controller 感知到领导车状态 → 发出 cmd_vel → 跟随车速度到位 ≈ 1.3s
```

### 1.2 电机响应实测数据

| 指标 | Raw /odom | EKF /odometry/filtered | 单位 |
|------|-----------|------------------------|------|
| 起步延迟 (→0.01 m/s) | P50 267ms | P50 228ms | ms |
| 到达 90% 目标速度 (→0.27 m/s) | P50 1239ms | P50 1243ms | ms |
| EKF 滤波额外开销 | — | 48ms | ms |
| 等效加速度 | ~0.22-0.27 | — | m/s² |
| 数据源发布频率 | 20Hz (STM32 固件) | 20Hz (受限于 /odom) | Hz |

> 测试条件: cmd_vel=0.3 m/s, wheeltec mini_omni 全向底盘, 轮式里程计模式。
> 测量工具: `measure_motor_latency.py` (阶跃响应法, 本机 time.time() 计时, 不依赖时钟同步)。

### 1.3 网络延迟实测数据

| 指标 | 值 | 说明 |
|------|-----|------|
| 原始 ping RTT | 5.4ms | WiFi 物理层 |
| 跨机 topic delay (leader→follower) | 32ms | ros2 topic delay |
| 本地 topic delay (follower) | 13ms | 基线 |
| DDS 序列化/路由开销 | ~14ms | 非瓶颈 |

> 网络延迟仅占总延迟 ~3%，不是编队效果差的根因。

### 1.4 实物系统频率上限

```
STM32 固件 → /odom (20Hz) → EKF (20Hz 实际) → 控制器 (20Hz)
    ↑
  硬件上限，wheeltec_robot 驱动无频率设置参数
  串口 115200 baud，24 字节编码器帧
  EKF 配置写 30Hz 无效——输入只有 20Hz，纯预测无测量更新
```

### 1.5 核心矛盾

当前 4D 模型的系统方程:
```
状态: x = [px, py, vx, vy]
模型: ṗ = v,   v̇ = u / mass

→ 假设输入 u 直接对应加速度 v̇，无任何延迟
```

实物:
```
cmd_vel 发出 → 串口 → STM32 速度环 ~250ms 才起步 → ~1.2s 才接近目标
```

控制器每 50ms 查一次 EKF 速度，每次都看到"慢了"，
24 次过后电机才到位——前 23 次都在错误地加大输出，
导致 overshoot → 拉回 → 震荡。

## 2. 技术方案: 4D → 6D 电机感知模型

### 2.1 当前 4D 模型 (formation_control_node.cpp)

```
x = [px, py, vx, vy]ᵀ

A = [0 0 1 0;      B = [0  0;
     0 0 0 1;           0  0;
     0 0 0 0;          1/m 0;
     0 0 0 0]          0  1/m]

控制律: u = HPC(x1, x2)  →  cmd_vel_body  →  clamp  →  wheel/acccel 约束  →  /cmd_vel
```

### 2.2 扩展后 6D 模型

```
x = [px, py, vx_cmd, vy_cmd, vx_real, vy_real]ᵀ
          ↑ 控制器指令速度     ↑ 电机实际滤波后速度

系统方程:
  ṗ        = v_real                                    (位置由实际速度积分)
  v̇_cmd    = u / mass                                  (指令由控制力驱动，同原模型)
  v̇_real   = (v_cmd - v_real) / tau                    (一阶 LP 模拟电机响应)

A = [0 0  0  0  1  0 ;      B = [0  0 ;
     0 0  0  0  0  1 ;           0  0 ;
     0 0  0  0  0  0 ;          1/m 0 ;
     0 0  0  0  0  0 ;          0  1/m;
     0 0 1/τ 0 -1/τ 0 ;         0  0 ;
     0 0  0 1/τ 0 -1/τ]         0  0 ]
```

### 2.3 为什么这样建模

| 设计选择 | 原因 |
|---------|------|
| `ṗ = v_real` (非 v_cmd) | 位置是物理现实，由轮子实际转动决定 |
| `v̇_cmd = u/mass` | 控制力仍然作用于"期望"，保留原 HPC 语义 |
| `v̇_real = (v_cmd - v_real)/tau` | 一阶 LP: τ 越小响应越快，实物 τ≈0.5 |
| cmd_vel 取自 v_cmd | 发给 STM32 的是 "期望指令" |
| 不改变输入维度 | 仍为 2 输入（ux, uy），偏航控制独立 |

**⚠️ v_cmd 是控制器内部积分状态，不是测量量**（关键，误实现会使整个设计失效）：

- EKF 测到的永远是轮子实际速度 = v_real。**如果每周期用 EKF 速度覆盖 v_cmd，
  则 v_cmd ≡ v_real，模型退化回 4D**——控制器每周期"忘记"自己上周期已发过的
  指令，重新看到"指令=实际=太慢"而继续加码，正是 §1.5 描述的震荡根因。
- 正确管线:
  1. **初始化**: v_cmd ← EKF 速度（起点对齐）；
  2. **每周期**: v_cmd 由控制器自行积分维护（`v_cmd += h·u/m`），不再读 EKF；
  3. **发布后回写**: cmd_vel 经 clamp + 轮速约束后才发出，须把**最终发布值**
     （body 系旋转回 map 系）写回 v_cmd——否则饱和时内部记账虚高，模型预测
     失真（抗饱和 / anti-windup）。
- **Leader 的 x1**: leader 的 v_cmd 无法获知，取 v_cmd = v_real = leader EKF
  测量速度（稳态假设）。

**已知局限（v1）**: 一阶 LP 只建模 ~1s 的爬升过程，**不建模 ~250ms 起步死区**
（纯延迟）。死区补偿可后续叠加 `motor_predictor.hpp`（Smith，τ+Td 双模型）
对 v_real 做前推，v1 不做以便单变量归因。

**理论说明**: A 含特征值 −1/τ（v_real 自阻尼项），非幂零；B 不作用于 v_real 行，
lpc2hpc 的 K0 无法抵消它，故闭环齐次性只**近似**成立——与已有 6D 控制器
（时变 A 含 ω 耦合项）的近似程度同性质，且 −1/τ 为耗散项、偏安全侧。
好处是本模型 A 为**常值**，HPC 仅需在编队点切换时重算（比 6D 简单）。

### 2.4 参数物理含义

| 参数 | 默认值 | 实物标定 | 含义 |
|------|--------|---------|------|
| `mass` | 2.0 | 2-4（经仿真扫参） | 控制力→加速度增益（调参用，非物理质量；4D 用 8.0，6D Motor 降为 2.0 以匹配 0.25 加速度上限） |
| `tau` | 0.43 | ~0.43 (由实测 T_90% 推算) | 电机时间常数，越小响应越快 |

tau 与实物加速特性的关系（一阶系统 T_90% = ln(10)·τ ≈ 2.3τ，扣除死区）:
```
tau ≈ (T_90% - T_dead) / ln(10)
    ≈ (1.24 - 0.25) / 2.3
    ≈ 0.43 → 默认 0.43（对齐实测）
```
注: τ 不建议取 < 0.1——三阶极点配置中 k2 = m(1/τ − 3λ) 随 τ→0 发散（见 §3.2）。

### 2.4.1 附加参数（6D Motor 专属，经仿真标定）

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `hpc_c_min` | 0.9 | HPC warp 的 c-clamp 下界。6D 三阶链深度 [2,1,0] 在 c=0.5 时 warp~30×（4D 仅 5×），提高到 0.9（~1.17×）消除弛豫振荡 |
| `leader_vel_lpf_tau` | 0.0 | leader 速度低通时间常数 (s)。0.0=关断。rf2o vy 噪声过大时设 0.2-0.3 |
| `control_period` | 1/control_rate | v_cmd 积分步长（必须等于真实控制周期，否则等效 B 矩阵缩放 → 极点失真） |

### 2.5 现有 HPC 框架的复用

已有 N-D 泛化齐次控制库 (`hnorm_nd.hpp`, `lpc2hpc_nd.hpp`, `types_nd.hpp`)，
直接支持任意维度的状态:

```cpp
// 原 4D:  Vec4d x1, x2;  Mat4d A_;  Mat42d B_;   (固定尺寸 + hnorm/lpc2hpc)
// 新 6D:  Eigen::VectorXd x1(6), x2(6);  MatrixXd A_(6,6), B_(6,2);
//         (动态尺寸 + hnorm_nd/lpc2hpc_nd，与已有 LpcController6D 同一套路)

A_ << 0, 0,    0,    0,  1,  0,
      0, 0,    0,    0,  0,  1,
      0, 0,    0,    0,  0,  0,
      0, 0,    0,    0,  0,  0,
      0, 0, 1/tau_, 0, -1/tau_, 0,
      0, 0,    0, 1/tau_, 0, -1/tau_;

B_ << 0, 0,
      0, 0,
      1.0/mass_, 0,
      0, 1.0/mass_,
      0, 0,
      0, 0;
```

HPC 核心（hnorm, expm, lpc2hpc, 自适应增益 calculate_klin）直接工作在 Vec6d 上，无需改动算法。

## 3. 实现计划

### 3.1 文件变更

| 文件 | 变更 |
|------|------|
| `homo_controller_6d_motor.hpp` (新建) | 6D 电机感知控制器类 `LpcController6DMotor`，A/B 矩阵 6×6/6×2 |
| `formation_control_node_6d_motor.cpp` (新建) | 6D 节点实现，状态管线 (位置/v_real=TF+EKF, v_cmd=内部积分状态+发布后回写, leader 取 v_cmd=v_real=测量值) |
| `formation_control_node_6d_motor.hpp` (新建) | 节点头文件 |
| `main_6d_motor.cpp` (新建) | 入口 |
| `formation_single_follower_6d_motor.launch.py` (新建) | Launch 文件 |
| `CMakeLists.txt` | 新增 target `formation_control_node_6d_motor` |
| `README.md` | 新增控制器版本条目 |

### 3.2 核心改动

1. **`LpcController6DMotor`**: 仿 `LpcController6D` 用动态 `MatrixXd` + `_nd` 库
   （`LpcController` 不是模板类，"改模板参数"不可行）；编队点逻辑照抄 4D
   （离散多边形 `dl_` 6×m_p，速度分量全零 + tol 滞后切换）。A/B 常值，HPC 仅在
   初始化和编队点切换时重算
2. **`calculate_klin` 三阶极点配置**: 每轴从二阶链 [p, v] 变为三阶链
   [p, v_cmd, v_real]，原二阶双增益公式不适用，须重新极点配置。
   注意 4D 中 `a` 不是极点（4D 闭环为 s² + (2a/m)s + (a/m)²，极点 = a/m），
   先换算 λ = a/m（λ ≥ ω_d），对 (s+λ)³ 三重极点配置，解析解:
   ```
   λ  = a / m     (a 自适应沿用 4D: a = max(clamp(−m·e_v/e_p, ±ωd·m), ωd·m)，e_v 取 v_real 误差)
   k1 = −λ³·m·τ         (作用于 p)
   k2 = m·(1/τ − 3λ)    (作用于 v_cmd)
   k3 = −3λ²·m·τ − k2   (作用于 v_real)
   ```
   验证: 闭环特征多项式 s³ + (1/τ − k2/m)s² − (k2+k3)/(mτ)·s − k1/(mτ) = (s+λ)³。
   K 为 2×6（x/y 两输入通道）。写成独立函数 `compute_channel_3rd(e_p, e_v, M, tau, wd)`
   以便后续 8D 复用
3. **数据管线**: 位置/偏航来自 TF+EKF，v_real 来自 EKF 速度（旋转到 map 系）；
   **v_cmd 仅初始化时取 EKF 速度，之后为内部积分状态**（见 §2.3，每周期发布后
   用最终 cmd_vel 回写）；leader 的 v_cmd = v_real = 测量速度
4. **cmd_vel 输出**: `goal_v_cmd = v_cmd + h·u/m` 旋转到车体系 → clamp →
   运动学约束 → `/cmd_vel`，随后将最终发布值旋转回 map 系写回 v_cmd
5. **4D 退化说明**: τ→∞ 时 v̇_real → 0 会**冻结 v_real、位置通道失去驱动**，
   模型失效（不是退化为 4D）；正确的 4D 退化极限是 τ→0⁺（v_real 瞬时跟上
   v_cmd），但 1/τ 数值发散不可行。**结论: 不做 τ 退化开关**，需要 4D 行为时
   直接使用原 4D 节点 `formation_control_node`

### 3.3 验证计划

1. 先用简单圆轨迹 leader 跑 6D 控制器，不开电机延迟，确认基本收敛
2. 开电机延迟 (use_motor_delay:=true)，对比不开 Smith 的 4D → 预期震荡明显减少
3. 在 6D 模型上扫参数 (tau, mass)，找出编队距离标准差最小的组合
4. 对比实验: 4D (baseline) vs 4D+Smith vs 6D Motor，三组轨迹/距离图

## 4. 预期效果

### 4.1 仿真预测

| 指标 | 4D (baseline) | 4D+Smith | 6D Motor |
|------|--------------|----------|----------|
| 编队距离 σ | 0.35m | 0.28m | **<0.15m** |
| overshoot 次数 | ~8/min | ~4/min | **<2/min** |
| 最大速度 raw | >3 m/s | 1-2 m/s | **<1 m/s** |

### 4.2 创新点

- "将执行器动力学显式建模为状态方程扩展维度，使齐次控制器
  天然感知指令-执行延迟"——与 Smith 预估器不同，6D 模型在 HPC 的
  gain scheduling 和误差 warping 层面利用了延迟信息，而非仅在反馈层面补偿。

## 5. 其他工具与脚本

### 5.1 诊断工具

| 脚本 | 用途 |
|------|------|
| `measure_motor_latency.py` | 阶跃响应法测电机延迟，双源 (raw/EKF) 对比，支持 `--target-fraction` |
| `measure_cross_machine_delay.py` | 跨机器 ROS 2 话题延迟统计，CSV 导出 |
| `sim_motor_delay.py` | 仿真电机延迟节点 (LP+传输延迟+加速度限幅)，launch 集成 |
| `record_trajectory.py` | 轨迹记录，六子图+CSV 导出，自动读控制器参数生成标签 |

### 5.2 控制器在线输出

| 输出 | 频率 | 内容 |
|------|------|------|
| `raw/clamped/final/scale` | 每秒 | 速度三层对比 |
| `DIAG: freq/leader_age/ekf_age/[smith]` | 每 5 秒 | 控制频率+数据新鲜度+补偿量 |
| `轮速约束触发` | 触发时 | scale<0.99 时打印 |

## 6. 附录: 实物环境参数

| 参数 | 值 |
|------|-----|
| 平台 | wheeltec mini_omni 三轮全向底盘 |
| 上位机 | ARM (Ubuntu 22.04, ROS 2 Humble) |
| 下位机 | STM32, 串口 115200 baud |
| /odom 频率 | 20Hz (固件限制) |
| 轮半径 | 0.03m |
| 底盘半径 | 0.11m |
| 双机通信 | WiFi 5GHz + CycloneDDS (已弃用, 回退 Fast-DDS) |
| 时钟同步 | chrony (仅跨机延迟测量需要) |

## 7. 实施记录（2026-07-16）

### 7.1 实现过程中发现与修复的问题

详见 `BUG_RECORD.md` 第 29–33 条，概要：

| # | 问题 | 根因 | 修复 |
|---|------|------|------|
| 29 | 横向速度反馈恒零 | rf2o 硬编码 vy=0 + EKF 融合假测量 | 补丁 rf2o 发布 lin_speed_y |
| 30 | 欠阻尼慢震荡 (3-4s 周期) | v_cmd 积分步长 h=0.1 ≠ 控制周期 0.05 | `control_period` 参数 = 1/control_rate |
| 31 | HPC warp 过大导致弛豫振荡 | 6D 三阶链 c=0.5 时 warp~30× (4D 仅 5×) | `hpc_c_min` 默认 0.9（~1.17×）|
| 32 | leader_vel_lpf_tau=0 冻住 leader 速度 | alpha=0 而非直通 | tau≤0 时关断低通 |
| 33 | 三阶 λ 语义对齐 4D | 4D 的 a 不是极点，λ=a/m | `compute_channel_3rd` 内部换算 |

### 7.2 经仿真实测标定的最终默认参数

| 参数 | 设计文档原值 | 标定值 | 原因 |
|------|------------|--------|------|
| `mass` | 8.0 | **2.0** | 降增益适应 0.25 加速度上限 |
| `tau` | 0.5 | **0.43** | 对齐实测 T_90% ≈ 1.24s |
| `omega_d` | 1.5 | **0.7** | 闭环带宽必须 ≤ 物理可达值 |
| `hpc_c_min` | 0.5 (4D 默认) | **0.9** | 6D 三阶链 warp 约 6× 于 4D |
| `leader_vel_lpf_tau` | 0.3 | **0.0** | 默认关断，需要时设 0.2-0.3 |
| `max_linear_accel` | 2.0 | **0.25** | 控制器加速度约束对齐实物 |
| `transport_delay` | 0.05 | **0.0** | v1 不建模死区 |
| `delay_max_accel` | 2.0 | **0.25** | 对齐实物电机等效加速度 |

### 7.3 实现的 8D 扩展预留

为后续 6d_disc + 电机模型融合（8D）做了以下设计预留：
- `compute_channel_3rd` 为独立静态函数，x/y 通道原样复用
- 全量用动态 `MatrixXd` + `_nd` 库，换维度零改动
- v_cmd 内部状态 + `sync_cmd_vel` 接口，8D 车体系更简单
- 编队点策略选定离散多边形 + tol（与 6d_disc 同款）
