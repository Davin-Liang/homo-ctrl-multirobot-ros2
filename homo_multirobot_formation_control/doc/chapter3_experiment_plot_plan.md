# 第 3 章 4D Artstein 齐次编队控制实验与绘图规划

本文档用于规划硕士论文第 3 章的数值仿真、Gazebo 仿真和实物实验数据采集、绘图及性能统计。
主线是：

```text
无延迟下原始 4D HPC 基准
  -> 延迟下原始 4D HPC 性能退化
  -> Artstein 预测补偿恢复延迟性能
  -> Artstein-HPC 与 Artstein-LPC 消融对比
  -> 数值、Gazebo 和实物结果汇总
```

## 1. 实验分组

### 1.1 四种控制工况

| 工况编号 | 控制器 | 延迟 | 用途 |
|---|---|---|---|
| C1 | 原始 4D HPC | 无延迟 | 理想基准，验证基础控制器 |
| C2 | 原始 4D HPC | 有延迟 | 展示未补偿时的性能退化 |
| C3 | Artstein-HPC | 有延迟 | 验证预测补偿的主要效果 |
| C4 | Artstein-LPC | 有延迟 | 消融 HPC，分析齐次升级的独立作用 |

### 1.2 三类实验平台

| 平台 | C1 | C2 | C3 | C4 |
|---|---:|---:|---:|---:|
| 数值仿真 | 必做 | 必做 | 必做 | 必做 |
| Gazebo 仿真 | 必做 | 必做 | 必做 | 必做 |
| 实物实验 | 不做 | 必做 | 必做 | 必做 |

实物不做 C1 是合理的：真实机器人始终存在执行器响应、通信、定位和采样延迟。
论文中应将实物的 C2、C3、C4 统一称为“实际延迟条件下的实验”，不要把实物 C1 留空误解为实验缺失。

## 2. 参数与公平性

### 2.1 固定条件

四种工况应尽量统一以下条件；仅对某个控制器实现不存在的参数，应明确记录为“不适用”，
不能把不存在的参数默认为已经统一：

```text
leader 轨迹
follower 初始位置与初始速度
编队半径 radius
安全编队点数量 m_p
编队点切换容差 tol
控制频率 control_rate
mass
omega_d
最大速度和最大加速度
轮速约束
hpc_c_min、initial_min_lambda、switch_min_lambda
实验时长
```

推荐默认实验周期为 20 Hz，以匹配实物 STM32 `/odom` 的频率。

### 2.2 延迟参数

控制器侧预测参数和仿真注入的物理延迟参数必须分开记录：

| 参数 | 含义 |
|---|---|
| `tau` | Artstein/执行器模型使用的电机时间常数 |
| `Td` | 控制器使用的输入死区或传输延迟 |
| `motor_tau` | Gazebo 延迟节点注入的一阶响应时间常数 |
| `transport_delay` | Gazebo 延迟节点注入的纯传输延迟 |
| `delay_max_accel` | Gazebo 延迟节点的加速度限制 |
| `max_linear_accel` | 控制器/底盘约束层的线速度变化率限制 |

建议延迟实验使用实物标定得到的典型值，例如：

```text
tau = 0.43 s
Td = 0.22 s
control_rate = 20 Hz
```

Gazebo 中应设置：

```text
motor_tau = tau
transport_delay = Td
delay_max_accel = 实物加速度上限或保守等效值
```

C2、C3、C4 必须使用相同的实际延迟和限幅条件。这里的实际延迟指 Gazebo 或实物执行器链路的
`motor_tau`、`transport_delay` 和加速度限制，而不是控制器内部的预测参数。
否则无法把性能差异归因于控制器结构。

`delay_max_accel` 和 `max_linear_accel` 不是同一个参数：前者属于 Gazebo 延迟注入节点，
后者属于控制器输出后的底盘约束层。若两者都启用，必须分别记录；实物中还应记录实际生效的
加速度上限或其标定值。

### 2.3 原始 4D 基线说明

C1 应使用原始 4D HPC 节点，并在数值仿真和 Gazebo 中关闭实际延迟。

不要仅用 `tau=0.01, Td=0` 的 Artstein 节点代替原始 4D 基线。该设置可作为近似无延迟
快速执行器工况，但当前实现仍会保留预测层，不能严格代表原始 4D 控制器。

### 2.4 Artstein-LPC/HPC 公平性

C3 和 C4 除 HPC/LPC 开关外，其余参数必须一致：

```text
tau, Td, radius, m_p, tol, mass, omega_d
控制频率、最大速度、最大加速度、轮速约束
min_cmd_vel、cmd_integrator_base
初始状态、leader 轨迹、实验时长
```

`hpc_c_min` 是 HPC 专属参数，在 C4 的 LPC 工况中应标为“不适用”。
`initial_min_lambda` 和 `switch_min_lambda` 若会影响 LPC 线性反馈增益，则必须与 C3 保持一致；
若当前 LPC 实现不使用它们，也应在实验参数表中标为“不适用”。这样可以明确性能差异来自
HPC 升级，而不是共同参数或约束条件不一致。

C1/C2 的原始 4D 节点与 C3/C4 的 Artstein 节点现在都支持通过 launch 参数配置
`hpc_c_min`、`initial_min_lambda` 和 `switch_min_lambda`。因此，C1/C2 与 C3/C4
可以显式使用同一组控制器参数，实验计划应记录实际传入的数值，而不能只记录 launch
文件名：

```text
C1 与 C2：保证原始 4D 节点内部参数一致；
C2 与 C3：除补偿层外，尽量对齐 mass、omega_d、编队几何、限幅和实际延迟；
C3 与 C4：严格只切换 use_hpc，其余 Artstein 参数完全一致。
```

当前两个 launch 文件的共有参数默认值已经统一。原始 4D launch 现在采用 Artstein launch
的共有参数默认值：

```text
mass=2.0，omega_d=0.7，hpc_c_min=0.1
initial_min_lambda=1.0，switch_min_lambda=4.0
motor_tau=0.43，transport_delay=0.0，delay_max_accel=0.25
```

因此，仅从 launch 默认值看，C2/C3 的共有参数已经对齐；正式实验仍应在元数据中记录
实际传入值，避免后续命令行覆盖默认参数造成混淆。

Artstein launch 还默认启用 `min_cmd_vel=0.03`，原始 4D 节点没有同名补偿参数；
`cmd_integrator_base` 也只存在于 Artstein 节点。数值/Gazebo 公平对比时应将
`min_cmd_vel:=0.0`，并固定记录 `cmd_integrator_base`；实物对比若保留 `min_cmd_vel=0.03`，
必须在论文中说明 Artstein 组同时包含速度死区补偿，不能把改善全部归因于 Artstein 预测。

正文实验应采用以下配置原则：

```text
原始节点和 Artstein 节点均显式配置 hpc_c_min、
initial_min_lambda、switch_min_lambda，并让 C2/C3 使用完全相同的值。

如果需要复现实验历史中的其他原始 4D 参数，则必须在两条 launch 命令中显式传入相同值。
```

若不采用上述公平配置，必须在论文中明确说明 C2/C3 同时改变了控制增益或 HPC 变形参数，
不能把全部性能差异归因于 Artstein 补偿。

还必须统一控制器内部离散化周期。当前原始 4D 节点构造 `LpcController` 时没有传入
`control_rate` 对应的 `control_period`，因此使用类默认值 `0.1 s`；Artstein 节点则传入
`1/control_rate`，20 Hz 时为 `0.05 s`。这会使 C2/C3 同时改变控制器离散积分步长。
正式实验前应修改原始节点，使其使用 `control_period=1/control_rate`，推荐将该项作为公平性
检查的阻断条件。

当前 4D Artstein 使用固定 `tau` 作为等效执行器时间常数。本章不包含自适应
tau 实验，也不将在线参数调度作为理论结论。

## 3. 数据采集清单

每次实验使用 `record_trajectory.py` 保存一个原始 CSV，并自动生成一张初步检查图。
数值仿真、Gazebo 和实物的字段名称尽量一致。后续论文图使用 MATLAB 读取该 CSV
重新绘制，不把脚本生成的 PNG 直接放入论文。

推荐的文件组织方式如下：

```text
一次实验运行
├── raw.csv：record_trajectory.py 输出的原始轨迹和里程计数据
├── 参数文件（可选）：实验平台、工况编号和完整参数
├── check.png：record_trajectory.py 自动生成的初步检查图
└── metadata.yaml：record_trajectory.py 自动生成的实验元数据
```

第 3 章的主实验流程不使用 `record_velocity_diagnostics.py`。如果后续研究需要分析
原始速度指令、延迟节点输出或控制器内部诊断量，再单独使用该脚本作为补充工具。
同一工况若重复进行多次实验，每次运行会创建独立实验目录，不会直接覆盖。

本章共 11 个基础实验运行：数值仿真 C1-C4 共 4 个，Gazebo C1-C4 共 4 个，
实物 C2-C4 共 3 个；若每个工况重复多次，则按“每次运行一个主 CSV”继续增加。

### 3.1 `record_trajectory.py` 实际记录字段

```text
time_s
leader_x_m, leader_y_m
leader_vx_ms, leader_vy_ms, leader_v_ms
follower_x_m, follower_y_m
follower_vx_ms, follower_vy_ms, follower_v_ms
distance_m
```

其中位置由 TF/EKF 转换到 `map` 坐标系；CSV 中的速度来自
`nav_msgs/Odometry.twist.twist.linear.x/y`，通常是车体系速度。当前脚本同时保存
速度模长 `*_v_ms`，但不保存速度指令 `cmd_vel`。

### 3.2 MATLAB 后处理字段

```text
target_dx, target_dy
desired_follower_x, desired_follower_y
e_x, e_y
e_p_norm
```

这些字段不由当前 `record_trajectory.py` 直接输出，而是由 MATLAB 根据实验参数和
CSV 离线计算。对于第 3 章的公平对比，推荐所有工况使用 `m_p:=1`，此时
`target_dx`、`target_dy` 为固定值：

```text
desired_follower_x = leader_x_m + target_dx
desired_follower_y = leader_y_m + target_dy
e_x = follower_x_m - desired_follower_x
e_y = follower_y_m - desired_follower_y
e_p_norm = sqrt(e_x^2 + e_y^2)
```

如果保留 `m_p>1` 的多编队点切换，当前 CSV 没有 `target_index`，无法仅凭
`record_trajectory.py` 的数据可靠恢复每个时刻的当前目标点。因此多点切换实验需要
额外记录控制器目标点，或在实验设计中将 `m_p:=1` 作为固定条件。

4D 位置误差应使用实际测量的 map 系位置：

```math
e_p(t)=p_F(t)-p_L(t)-d_{\sigma(t)}
```

其中 `d_{\sigma(t)}` 是当前选中的编队偏置。论文中的主误差只使用当前目标点对应的
`e_x`、`e_y` 和 `e_p_norm`。

Artstein 控制器内部还会使用预测状态 `x_h`。预测误差可以作为机理分析数据单独保存，
但表 3-1 的跟踪性能必须使用实际状态计算，不能用预测状态误差代替实物跟踪误差。

主指标定义为：

```math
e_{\mathrm{p,norm}}(t)=\sqrt{e_x(t)^2+e_y(t)^2}
```

实际平移速度的大小定义为：

```math
v_{\mathrm{real,norm}}(t)=
\sqrt{(v_{x,\mathrm{real}}^{body})^2+(v_{y,\mathrm{real}}^{body})^2}
```

`record_trajectory.py` 保存 Leader/Follower 的实际速度分量和速度模长，因此可以统计
实际速度的均值、最大值和稳态波动。速度范数在刚体旋转下不变，因此也可使用 map 系分量
计算，但同一张图和同一张表必须统一坐标系。

### 3.3 当前脚本不记录的量

`record_trajectory.py` 不记录 `cmd_vel`、原始速度指令、延迟节点输出和控制器内部诊断量。
这些量不作为第 3 章主实验的必需数据，也不作为表 3-1 的必填指标。
如果后续小论文必须分析“指令速度与实际速度的差异”，再单独启用
`record_velocity_diagnostics.py` 补充采集。

### 3.4 当前脚本覆盖范围

本规划描述的是论文最终需要的数据格式，不代表当前数值仿真脚本已经全部实现：

| 需求 | 当前状态 |
|---|---|
| 数值 C1：原始 4D HPC 无延迟 | `sim_4d_hpc_artstein_compare.py` 已覆盖 |
| 数值 C2：原始 4D HPC + 延迟 | 已覆盖 |
| 数值 C3：Artstein-HPC + 延迟 | 已覆盖 |
| 数值 C4：Artstein-LPC + 延迟 | 当前脚本未覆盖，需要增加 `use_hpc=False` 的仿真分支 |
| Gazebo/实物基础轨迹 CSV | `record_trajectory.py` 已覆盖 |
| 期望 Follower `x(t)/y(t)` | MATLAB 根据 `target_dx/target_dy` 后处理 |
| 当前目标点的 `e_x/e_y` | MATLAB 根据固定目标偏置后处理 |

因此，正式实验前至少需要：为数值脚本增加 C4，并统一每个工况的
`target_dx/target_dy`。图 3-2 至图 3-7 的位置轨迹、x(t)、y(t)、误差和表 3-1
的跟踪指标均可由 `record_trajectory.py` 的 CSV 在 MATLAB 中后处理得到。

正式对比还必须处理编队点切换。推荐 C1-C4 将 `m_p:=1`，使用相同的固定目标偏置 `d`，
以保证所有 `x(t)/y(t)` 图的期望 Follower 轨迹相同。若必须保留 `m_p` 多点切换，则每个
工况都要保存 `target_index` 和 `target_dx/target_dy`，并在图中标出切换时刻；此时不同工况的
期望轨迹可能不同，不能仅凭 `x(t)/y(t)` 曲线直接比较，主结论应转向相对各自目标点的误差统计。

## 4. 统一绘图规范

### 4.1 方法颜色与线型

全章固定使用以下编码：

| 对象 | 推荐样式 |
|---|---|
| Leader/期望轨迹 | 黑色虚线 |
| 原始 4D HPC，无延迟 | 灰色点划线 |
| 原始 4D HPC，有延迟 | 红色实线 |
| Artstein-HPC，有延迟 | 蓝色实线 |
| Artstein-LPC，有延迟 | 橙色虚线 |

颜色和线型同时编码，保证黑白打印时仍能区分。

对同时绘制两种控制器的 `e_x(t)`、`e_y(t)` 子图，使用“颜色表示控制器、线型表示分量”的规则：

```text
e_x: 实线
e_y: 虚线
```

因此该子图最多包含四条可识别曲线。若实物曲线噪声使四条线仍难以阅读，将同一子图拆为上下两个
共享时间轴的小坐标轴：上方绘制 `e_x`，下方绘制 `e_y`；它们仍共同构成该图的一个子图编号。

### 4.2 坐标与时间范围

同一组跨平台对比图应尽量统一：

```text
x/y 坐标范围
时间范围
误差坐标范围
图例名称
单位
字体大小
线宽
```

数值仿真、Gazebo 和实物不要强行叠加到同一个时间序列坐标轴中。三类平台的采样率、
时钟、噪声和实验起止时间不同，推荐使用同构子图并列展示。

### 4.3 数据处理

- RMSE、最大误差、稳态误差等统计量使用原始数据计算。
- 实物曲线可以叠加滑动平均线用于观察趋势，但必须保留原始曲线或在图注中说明。
- 不要用平滑操作掩盖延迟振荡、峰值误差和控制输入突变。
- 若不同平台采样率不同，绘图时可插值到统一时间网格，但统计指标应基于原始采样数据或明确说明插值方法。
- 编队点切换时在图中标注切换时刻，或在图注中说明误差定义随 `target_index` 更新。

## 5. 第 3 章主图清单

### 图 3-1：无延迟下原始 4D HPC 的数值与 Gazebo 基准验证

采用 `2 x 2`，不放实物，不放速度图：

```text
(a) 数值仿真：Leader、期望 Follower 与实际 Follower 的 x-y 平面轨迹
(b) 数值仿真：e_x(t)、e_y(t)
(c) Gazebo：Leader、期望 Follower 与实际 Follower 的 x-y 平面轨迹
(d) Gazebo：e_x(t)、e_y(t)
```

目的：

```text
证明原始 4D HPC 在无延迟条件下能够形成正确编队；
证明数值仿真与 Gazebo 的基础结果一致。
```

`e_p_norm`、速度指令和收敛指标放入表 3-1，不在此图重复展开。

### 图 3-2：延迟下原始 4D HPC 与 Artstein-HPC 的数值仿真对比

采用 `2 x 2`：

```text
(a) Leader、期望 Follower、原始 4D HPC、Artstein-HPC 的 x-y 平面轨迹
(b) x(t)：Leader、期望 Follower 位置和实际 Follower 位置
(c) y(t)：Leader、期望 Follower 位置和实际 Follower 位置
(d) 两种控制器的 e_x(t)、e_y(t)
```

目的：

```text
展示延迟导致原始 4D HPC 跟踪退化；
展示 Artstein 预测补偿改善 x/y 方向位置跟踪和编队误差。
```

### 图 3-3：延迟下原始 4D HPC 与 Artstein-HPC 的 Gazebo 对比

与图 3-2 保持完全同构：

```text
(a) Leader、期望 Follower、原始 4D HPC、Artstein-HPC 的 x-y 平面轨迹
(b) x(t)：Leader、期望 Follower 位置和实际 Follower 位置
(c) y(t)：Leader、期望 Follower 位置和实际 Follower 位置
(d) 两种控制器的 e_x(t)、e_y(t)
```

目的：

```text
验证补偿效果在 ROS 2、Gazebo、全向底盘运动学约束和延迟注入下仍然成立。
```

### 图 3-4：延迟下原始 4D HPC 与 Artstein-HPC 的实物对比

仍采用 `2 x 2`，主图只展示位置跟踪和编队误差：

```text
(a) Leader、期望 Follower、原始 4D HPC、Artstein-HPC 的 x-y 平面轨迹
(b) x(t)：Leader、期望 Follower 位置和实际 Follower 位置
(c) y(t)：Leader、期望 Follower 位置和实际 Follower 位置
(d) 两种控制器的 e_x(t)、e_y(t)
```

实物图中可以用细线表示原始数据、粗线表示滑动平均趋势，但统计指标仍使用原始数据。

目的：

```text
证明补偿效果不是数值模型中的假象，而是在真实电机、串口、定位噪声和 20 Hz 闭环下仍存在。
```

### 图 3-5：Artstein-HPC 与 Artstein-LPC 的数值仿真对比

采用 `2 x 2`：

```text
(a) Leader、期望 Follower、Artstein-LPC、Artstein-HPC 的 x-y 平面轨迹
(b) x(t)：Leader、期望 Follower 位置和实际 Follower 位置
(c) y(t)：Leader、期望 Follower 位置和实际 Follower 位置
(d) 两种控制器的 e_x(t)、e_y(t)
```

目的：

```text
在相同预测补偿和延迟条件下，单独分析 HPC 相对 LPC 的作用。
```

### 图 3-6：Artstein-HPC 与 Artstein-LPC 的 Gazebo 对比

与图 3-5 同构：

```text
(a) Leader、期望 Follower、Artstein-LPC、Artstein-HPC 的 x-y 平面轨迹
(b) x(t)：Leader、期望 Follower 位置和实际 Follower 位置
(c) y(t)：Leader、期望 Follower 位置和实际 Follower 位置
(d) 两种控制器的 e_x(t)、e_y(t)
```

目的：

```text
验证 HPC 的优势经过 Gazebo 全向底盘和延迟链路后仍然存在。
```

### 图 3-7：Artstein-HPC 与 Artstein-LPC 的实物对比

采用 `2 x 2`：

```text
(a) Leader、期望 Follower、Artstein-LPC、Artstein-HPC 的 x-y 平面轨迹
(b) x(t)：Leader、期望 Follower 位置和实际 Follower 位置
(c) y(t)：Leader、期望 Follower 位置和实际 Follower 位置
(d) 两种控制器的 e_x(t)、e_y(t)
```

真实速度跟踪不作为本图主子图，保留在原始 CSV 和表 3-1 中。

## 6. 表 3-1 性能指标汇总

表 3-1 按“平台 × 控制工况”排列。实物不填写 C1：

| 平台 | 控制工况 | 控制器 |
|---|---|---|
| 数值仿真 | C1 无延迟 | 原始 4D HPC |
| Gazebo | C1 无延迟 | 原始 4D HPC |
| 数值仿真 | C2 有延迟 | 原始 4D HPC |
| Gazebo | C2 有延迟 | 原始 4D HPC |
| 实物 | 实际延迟 | 原始 4D HPC |
| 数值仿真 | C3 有延迟 | Artstein-HPC |
| Gazebo | C3 有延迟 | Artstein-HPC |
| 实物 | 实际延迟 | Artstein-HPC |
| 数值仿真 | C4 有延迟 | Artstein-LPC |
| Gazebo | C4 有延迟 | Artstein-LPC |
| 实物 | 实际延迟 | Artstein-LPC |

推荐列：

```text
RMSE(e_p_norm)
稳态平均 e_p_norm
稳态标准差
最大 e_p_norm
95% 分位 e_p_norm
收敛时间
Leader 平均实际速度
Follower 平均实际速度
Follower 最大实际速度
Follower 稳态速度标准差
```

因此，正文主图不再单独绘制 `e_p_norm(t)` 和速度模长曲线。
`e_p_norm` 以及实际速度指标可由 `record_trajectory.py` 的 CSV 计算并写入表 3-1。
`cmd_speed_norm`、原始速度指令和最终速度指令不属于本章主实验的必填指标，因为
`record_trajectory.py` 不记录 `cmd_vel`。

## 7. 对小论文数据的支持

在完成第 3.4 节所列的数值脚本补充后，按照本规划采集的数据可以支撑后续小论文中
常见的以下图表：

```text
Leader-Follower x-y 平面轨迹
x 方向位置跟踪 x(t)
y 方向位置跟踪 y(t)
x/y 分量误差 e_x(t)、e_y(t)
Leader/Follower 实际速度及其稳态波动
不同平台或不同控制器的 RMSE、稳态误差、最大误差对比
不同控制器的收敛时间对比
延迟补偿前后的性能统计
```

`record_trajectory.py` 已保存后续位置跟踪和实际速度分析所需的基础状态数据。
仅凭该 CSV 可以支撑轨迹、位置跟踪、实际速度和误差统计类图表，但不能支撑“命令速度与实际速度
对比”或“控制器内部延迟链路”类图表。当前第 3 章不要求额外保存：

```text
cmd_vel_raw, cmd_vel
leader_age, ekf_age
```

若后续小论文需要分析命令速度、实际速度响应或延迟链路，再额外启用
`record_velocity_diagnostics.py`。它是可选诊断工具，不是第 3 章主实验的前置条件。

小论文的轨迹和误差类图表可以直接复用本章 CSV；可能还需要以下实验元数据，建议同步记录：

```text
experiment_id
trial_id
platform
controller
tau, Td
motor_tau, transport_delay
max_linear_accel
target_index
desired_follower_x, desired_follower_y
```

其中 `trial_id` 和 `experiment_id` 用于多次重复实验的均值、标准差、箱线图或误差带绘制；
`desired_follower_x/y` 用于在小论文中直接画出目标轨迹，不依赖事后重新推导。

仅有 x/y 位置数据不足以完整支撑“延迟补偿机理”类小论文，因为它无法区分控制器误差、
执行器滞后和定位链路噪声。完整数据至少应包含位置、目标位置、速度命令、实际速度和时间戳。

定义建议：

```math
RMSE(e_p)=
\sqrt{\frac{1}{N}\sum_{k=1}^{N}\left\|e_p(k)\right\|_2^2}
```

稳态区间应在所有同类实验中使用相同规则，例如去掉启动阶段后最后 20% 的数据。
收敛时间应在实验开始前固定定义。推荐使用 `e_p_norm <= 0.10 m`，并要求之后连续
`2 s` 保持在阈值以内；如果编队半径或实验速度改变导致该阈值不合适，应在所有对比组中统一
修改并记录。不能根据单条曲线事后调整阈值。

实物实验建议至少重复 3 次，报告均值和标准差：

```text
均值 ± 标准差
```

数值仿真和 Gazebo 如果为确定性实验，可以报告单次结果；如果加入噪声，建议同样进行多次重复。

## 8. 推荐实验执行顺序

按照以下顺序采集，便于尽早发现参数或数据字段问题：

1. 数值仿真 C1：确认原始 4D HPC 无延迟基准。
2. Gazebo C1：确认基础 ROS/Gazebo 链路。
3. 数值仿真 C2：确认延迟会造成可观察退化。
4. 数值仿真 C3：确认 Artstein-HPC 能改善 C2。
5. 数值仿真 C4：完成 Artstein-LPC 消融。
6. Gazebo C2、C3、C4：保持延迟和控制参数完全一致。
7. 实物 C2、C3、C4：每个工况至少重复 3 次。
8. 统一离线计算误差和性能指标。
9. 按图 3-1 至图 3-7 绘图，最后生成表 3-1。

每次运行的目录位于 `robot_traj/sim/` 或 `robot_traj/real/` 下，命名为
`<tag>_<timestamp>/`。目录内建议包含：

```text
metadata.yaml
raw.csv
processed.csv
metrics.csv
check.png
notes.md
```

其中：

```text
raw.csv       record_trajectory.py 输出的原始轨迹和里程计数据
processed.csv 对齐时间戳、补齐期望轨迹并计算误差后的统一分析表
metrics.csv   从 processed.csv 汇总得到的 RMSE、稳态误差、最大误差等指标
check.png     record_trajectory.py 生成的快速检查图，不代替论文最终排版图
metadata.yaml record_trajectory.py 输出的平台、工况、控制器和参数元数据
```

因此，`raw.csv`、`processed.csv` 和 `metrics.csv` 是同一次实验的不同处理阶段，
不是第 3.1、3.2、3.3 三个数据小节各生成一个 CSV。第 3 章主实验中不需要把两个
ROS 记录器的 CSV 合并；如果后续启用可选速度诊断工具，再将其输出作为补充数据单独保存。

`record_trajectory.py` 会自动生成 `metadata.yaml`，至少记录：

```text
platform
case
controller
date
leader trajectory
initial state
control_rate
hpc_c_min
initial_min_lambda
switch_min_lambda
tau
Td
motor_tau
transport_delay
max_linear_accel
```

## 9. 章节叙述顺序

正文建议按照以下顺序解释图表：

```text
图 3-1：先证明基础控制器在理想条件下有效；
图 3-2：再说明延迟使原始控制性能退化，Artstein-HPC 能改善；
图 3-3：验证上述结论能够迁移到 Gazebo；
图 3-4：验证上述结论能够在实物闭环中成立；
图 3-5 至图 3-7：在相同延迟补偿条件下分析 HPC 相对 LPC 的独立贡献；
表 3-1：最后用统一指标进行横向汇总。
```

不要在每幅图中重复完整描述实验平台。每幅图只解释该图支持的结论，平台、参数和指标定义
集中放在实验设置小节及本章前面的统一说明中。
