# Homo-Control 多机器人编队控制项目：ChatGPT 技术讨论上下文

> 用途：本文件是供网页端 ChatGPT 阅读的项目上下文。讨论本项目的控制理论、ROS 实现、参数选择、仿真与实车实验时，应以本文件定义的术语、边界和当前代码状态为准。

## 1. 项目一句话说明

这是一个基于 **ROS 2 Humble + Gazebo Classic 11** 的两台三轮全向移动机器人协同系统。项目以 `mini_omni_robot` 为统一平台，提供从 URDF/Gazebo 仿真、激光/IMU/里程计融合、已知地图定位，到 Leader–Follower 圆形编队控制、输入延迟补偿和基于激光雷达的安全过滤的完整研究链路。

研究核心不是一般意义的路径规划，而是：在定位误差、通信/执行器延迟、速度与轮速约束、以及静态障碍物存在时，使 Follower 围绕 Leader 保持指定半径的协同编队，并比较不同状态模型和控制补偿结构的效果。

## 2. 技术栈与仓库边界

| 层级 | 技术/组件 | 在系统中的作用 |
|---|---|---|
| 操作系统与中间件 | Ubuntu 22.04、ROS 2 Humble、DDS | 节点通信、TF、launch、参数服务 |
| 仿真 | Gazebo Classic 11 | 双机器人动力学/传感器仿真 |
| 建模 | URDF/Xacro、STL | 三轮全向 `mini_omni_robot` 模型 |
| 状态估计 | rf2o laser odometry、`robot_localization` EKF | 从激光、IMU、轮式里程计估计状态 |
| 全局定位/建图 | slam_toolbox、AMCL | 提供 `map → odom`，使编队在统一地图坐标中讨论 |
| 控制实现 | C++17、Eigen、rclcpp | 齐次控制（HPC）、Artstein 补偿、运动学约束、安全滤波 |
| 实车接口 | Wheeltec STM32 串口驱动、Leishen 雷达 | 20 Hz 轮式里程计、IMU、速度命令下发 |

仓库是 colcon workspace 的一个 `src/` 子仓库。构建命令必须在工作空间根目录执行，而不是当前源码仓库内；否则会产生与工作空间环境脱节的 `build/ install/ log/`。

```bash
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select homo_multirobot_formation_control \
  --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

## 3. 软件包职责

| 包 | 职责 |
|---|---|
| `homo_multirobot_urdf` | `mini_omni_robot.xacro`、网格、机器人状态发布与 RViz 展示；模型总质量为 2.0 kg。 |
| `homo_multirobot_gazebo` | 单机/双机 Gazebo spawn、世界文件、planar_move 和 ros2_control 相关配置。 |
| `homo_multirobot_localization` | 仿真/实车的 rf2o、EKF 与启动组合，负责里程计估计链路。 |
| `homo_multirobot_nav` | 已知地图定位：AMCL 或 slam_toolbox 纯定位。 |
| `homo_multirobot_slam_toolbox` | 多机环境下由选定机器人建图、其他机器人复用地图的封装。 |
| `homo_multirobot_formation_control` | 本项目研究重点：多版本 Leader–Follower 控制器、领航轨迹、延迟仿真、数据记录与诊断。 |
| `turn_on_wheeltec_robot` | 实车串口、雷达、IMU 处理、EKF bringup；支持 namespace 与 TF 前缀隔离。 |
| `third_party/rf2o_laser_odometry` | 引入的激光里程计；项目补丁使其发布横向速度 `lin_speed_y`，以适配全向底盘。 |
| `third_party/robot_localization` | EKF/UKF 上游源码副本。 |
| `third_party/omnidirectional_controllers` | ros2_control 轮级全向控制器，供后续真实轮速控制使用。 |

## 4. 机器人命名、话题与坐标系

### 4.1 多机器人隔离规则

- **TF 前缀**：`robot1_`、`robot2_`。它附加到 link/joint/frame 名称，避免 TF 帧重名。
- **ROS namespace**：`/robot1`、`/robot2`。它隔离同名话题和节点。
- 例如，robot1 的扫描话题是 `/robot1/scan`，其底盘帧是 `robot1_base_footprint`。

两者不可混淆：namespace 处理 ROS 名称；prefix 处理 TF/URDF 帧名称。

### 4.2 TF 语义

典型链路为：

```text
world → robot*_odom → robot*_base_footprint → robot*_base_link → 传感器/轮子 link
map   → robot*_odom  （已知地图定位或建图时）
```

- `world`：Gazebo 仿真世界坐标。
- `odom`：局部连续里程计坐标，短时间平滑但可漂移。
- `map`：全局地图坐标；通常由 slam_toolbox 或 AMCL 对 `odom` 作全局校正。
- `base_footprint`：机器人在地面的平面基座帧；控制器最终发布的 `cmd_vel` 是该车体坐标系的速度命令。

**关键约束：同一个 `odom → base_footprint` 只能由一个节点发布。** 若 EKF 负责该 TF，则必须关闭 Gazebo planar_move 与 rf2o 的对应 TF 发布，否则会出现跳变、回环或 TF 冲突。

### 4.3 核心接口

| 话题/接口 | 类型/频率 | 语义 |
|---|---|---|
| `/<ns>/scan` | `sensor_msgs/LaserScan`，仿真约 10 Hz、720 点 | rf2o 与障碍圆柱感知输入。 |
| `/<ns>/imu` 或实车 `imu/data_raw` | `sensor_msgs/Imu`，仿真约 50 Hz | 姿态/角速度估计输入。 |
| `/<ns>/odom` | `nav_msgs/Odometry` | Gazebo 或 STM32 原始里程计。实车默认约 20 Hz。 |
| `/<ns>/rf2o/odom` | `nav_msgs/Odometry` | 激光里程计输出；全向机器人必须关注其中的横向速度。 |
| `/<ns>/odometry/filtered` | `nav_msgs/Odometry` | EKF 估计；编队控制器读取 Leader/Follower 状态的主要来源。 |
| `map → <prefix>odom` | TF | 将局部估计映射进同一全球坐标，供 map 系控制与评估使用。 |
| `/<ns>/cmd_vel` | `geometry_msgs/Twist` | 控制器的最终车体系速度命令。 |

## 5. 仿真、实车与定位数据流

### 5.1 Gazebo 的两条驱动路径

1. **planar_move（默认）**：`gazebo_ros_planar_move` 直接订阅 `/<ns>/cmd_vel` 并发布里程计；适合快速控制联调。模型中轮子摩擦被置零，避免轮子带来非期望偏航力矩。
2. **ros2_control**：使用 `gazebo_ros2_control` 暴露关节速度接口，需要额外启动 controller_manager/spawner；更接近轮级控制，但配置和调试复杂度更高。

因此，planar_move 的机器人并不等价于真实电机/轮地动力学。若讨论实车可迁移性，必须把执行器时延、加速度上限、轮速饱和和定位频率单独建模或测量。

### 5.2 状态估计链路

仿真中常用：

```text
Gazebo scan + imu + odom
       ↓
rf2o laser odometry + robot_localization EKF
       ↓
/<ns>/odometry/filtered 与 odom → base_footprint
       ↓
slam_toolbox/AMCL 提供 map → odom
       ↓
map 系 Leader/Follower 状态
```

实车中常用且推荐的默认链路为：

```text
编码器 → STM32（约 20 Hz）→ 串口 → /odom
IMU raw → IMU 处理 → /imu/data_filtered
                  ↓
                 EKF → /odometry/filtered + TF → 编队控制器 → cmd_vel → STM32 → 电机
```

实车默认推荐轮式里程计模式；rf2o 实车模式存在漂移风险，应作为实验性链路讨论。

### 5.3 已知地图定位

- **AMCL**：粒子滤波定位，发布定位结果；可能存在重采样带来的离散跳变。
- **slam_toolbox 纯定位**：更适合平滑控制联调；不发布 `amcl_pose`，而通过 `map → odom` TF 表达全局校正。

如果控制问题在 `map` 系定义圆形编队和障碍物，控制器必须正确获取 `map` 到机器人当前坐标的变换，不能把某个 `odom` 直接假定为全球坐标。

## 6. 编队问题与公共符号

系统至少包含一个 Leader 和一个 Follower。Leader 提供自身的估计状态和轨迹，Follower 以其状态构造相对误差并输出本车 `cmd_vel`。常见目标是让 Follower 位于以 Leader 为圆心、半径 `radius` 的圆周上。

- `m_p`：圆周上离散安全编队点的数量。
- `radius`：期望编队半径，单位 m。
- `tol`：当前目标点与候选目标点之间触发切换的容差。
- `omega_d`：名义闭环带宽/极点尺度，决定响应快慢；过大时更容易触发速度、加速度或轮速约束。
- HPC（homogeneous control）：在设计的线性极点配置/LPC 基础上做齐次升级，目标是在其名义适用条件下取得更强的有限时间收敛性质；这不自动涵盖饱和、延迟失配、噪声和离散切换。

离散多边形策略不是要求 Follower 固定在某一绝对角度，而是从圆周候选点中选取/切换目标；这可避免在圆心附近或目标跨越时的几何退化。连续边界投影则直接将相对状态投影到圆边界附近，不使用 `m_p/tol` 点切换。

## 7. 控制器谱系与当前构建状态

下表的“实现状态”必须与“代码存在”分开理解。当前 `homo_multirobot_formation_control/CMakeLists.txt` 默认仅编译 **基线 4D** 和 **4D Artstein**。4D Artstein-LQR、6D Artstein Disc、HOCBF 位于 `if(FALSE)` 块；其他旧版本还被注释。使用它们前需要按目标机内存条件显式恢复 CMake 目标并重新构建。

| 版本 | 名义状态/坐标 | 队形与控制 | 延迟/安全处理 | 实现状态 |
|---|---|---|---|---|
| 4D 基线 | `[p_x,p_y,v_x,v_y]`，map 系双积分器 | 离散多边形 + HPC；yaw 独立 P+前馈 | 无专门输入时延补偿 | 默认构建 |
| 4D Artstein | 同 4D | 原 HPC 核心不变 | Artstein 纯死区补偿 + 一阶速度响应预测 + 可选径向制动层 | 默认构建，推荐的当前主线之一 |
| 4D Artstein-LQR | 同 4D Artstein | 预测层后改为离散 DARE-LQR，对照组 | 同 Artstein 预测；非 HPC 基准 | 代码存在，默认不构建 |
| 4D Cont | 同 4D | 连续圆边界投影 | 无专门输入时延补偿 | 旧/可选，当前注释 |
| 6D | `[p_x,p_y,theta,v_x^b,v_y^b,omega]`，位置 map 系、速度 body 系 | 连续边界投影，yaw 纳入主回路 | 轮速与车体约束 | 旧/可选，当前注释 |
| 6D Disc | 同 6D | 离散多边形 + tol 切换 | 轮速与车体约束 | 旧/可选，当前注释 |
| 6D Artstein Disc | 预测 map 平移 + 预测 yaw 后重组 6D body 状态 | 保留 6D Disc HPC 核心 | 分通道 Artstein/执行器预测 | 代码存在，但 `if(FALSE)` 默认不构建 |
| 6D Artstein Disc + HOCBF | 同上，使用预测 map 平移状态 | 名义 6D Disc + 局部切向通行偏置 | 多圆柱 predictor-HOCBF 硬约束 QP | 代码存在，但 `if(FALSE)` 默认不构建 |
| 6D Motor | `[p_x,p_y,v_x^c,v_y^c,v_x^r,v_y^r]`，map 系 | 离散多边形 + HPC | 显式一阶电机滞后增广、可选自适应 `tau`/Smith 预测 | 代码存在，当前注释 |
| 6D+OA | 6D 运动学状态 | 6D 名义跟踪 | 基于 scan 的软惩罚 QP 避障 | 历史实现，当前注释 |

**讨论原则：**“能在 README 中启动”不代表当前构建配置已经生成对应可执行文件。任何复现实验前，先检查 `install/homo_multirobot_formation_control/lib/homo_multirobot_formation_control/` 与 CMake 的实际启用目标。

## 8. 4D Artstein：延迟补偿主线

原始 4D 模型把二维平移近似为双积分器：位置和速度在 `map` 系演化。它便于极点配置与齐次化，但不直接包含纯输入死区或电机一阶响应。

4D Artstein 在外层补偿这些非理想因素，而不改变原 4D HPC 内核：

1. **`Td`**：纯输入/传输死区，例如串口、执行器或通信导致“命令在 `Td` 秒后才开始作用”。Artstein 变换利用命令历史把含输入时延的模型转换为等效的无显式延迟状态反馈问题。
2. **`tau`**：一阶等效速度响应时间常数，用来把测得 Follower 状态前向预测到更接近“命令真正作用时”的状态。它是模型/标定参数，不应在所有电机状态和速度段被视为严格不变的物理常数。
3. **Leader 预测**：Follower 使用到达时刻的 Leader 状态需要与自身预测时域对齐，特别是在跨机器 DDS 通信时。
4. **径向制动安全层**：靠近编队圆、且相对 Leader 有过大向内速度时，按可用减速度和有效延迟限制继续朝圆心的速度，预留刹停距离。它在命令输出侧工作，不改写 HPC/Artstein 的理论定义。

典型 launch 默认量级为 `tau=0.43 s`、`Td=0.22 s`、`control_rate=20 Hz`、`radius=2.0 m`、`m_p=4`、`tol=0.1 m`、`omega_d=0.7 rad/s`。它们是当前工程起点，不是脱离场景的最优参数。

## 9. 6D Artstein Disc 与执行器增广模型

6D 运动学控制器显式使用车体朝向和车体系速度。其状态混合了 map 系位置与 body 系速度：

```text
x_6D = [p_x, p_y, theta, v_x^b, v_y^b, omega]
```

由于旋转矩阵 `R(theta)` 随状态变化，项目没有把所有量硬塞进一个全局常值 6×3 Artstein kernel。6D Artstein Disc 采用“方向 A”分层近似：

```text
map 系平移 Artstein/执行器预测
                 +
yaw Artstein/执行器预测
                 ↓
预测后的 map 速度按预测 yaw 旋到 body 系
                 ↓
原 6D Disc HPC 与离散编队策略
```

这保留了原 6D Disc 的车体级控制结构，但理论结论只能表述为：在模型参数合理、命令历史完整、采样足够快、预测窗口内命令近似常值，以及局部闭环条件成立时的**名义/局部**预测补偿结论。饱和、轮地接触、切换、时间常数失配和坐标耦合残差应视为扰动，而不是被直接纳入全局有限时间证明。

6D Motor 采取另一条路线：把平移命令速度 `v^c` 和实际速度 `v^r` 都显式放进状态，形成每轴三阶链。它适合讨论实车中“发出的速度命令不等于当前实际速度”的大滞后问题，但当前构建中并非默认目标。

## 10. HOCBF 安全过滤与障碍物模型

6D Artstein Disc + HOCBF 的目标是：先生成名义编队速度，再在最终发布前以安全约束修正平移命令。当前感知对象是**单线激光可见的静态圆柱**，不是任意形状、动态目标或完整语义地图。

数据路径为：

```text
/scan
  → 连续量测点聚类
  → 圆柱最小二乘拟合
  → 转换到 map 系，形成保守障碍圆盘
  → 基于预测平移状态构造二阶 HOCBF
  → 多障碍硬约束 QP
  → 安全平移命令（与 yaw 命令组合）
  → body-frame cmd_vel
```

安全半径不是只有障碍物物理半径。讨论时应分别说明：

- `follower_radius`：机器人外接/等效半径；
- 障碍拟合半径：由 scan 拟合得到；
- `clearance`：额外几何间隙；
- `perception_margin`：覆盖感知、拟合、坐标转换和离散采样误差的保守裕度。

HOCBF-QP 是硬安全约束的工程实现，但不应被描述为无条件安全保证。其可行性会受速度/加速度约束、20 Hz 离散采样、预测误差、传感器盲区、障碍物突然出现、圆柱拟合失败和狭窄通道几何影响。不可行时的降级/制动策略及实际制动距离仍应实验验证。

## 11. Leader 轨迹来源

| 方式 | 节点/脚本 | 特点与用途 |
|---|---|---|
| 开环圆或 8 字 | `leader_circle.py`、`leader_eight.py` | 向 Leader 发布 `cmd_vel`，依赖 Gazebo/EKF 给出实际里程计；适合基本联调。 |
| 虚拟 Leader | `virtual_leader_circle.py` | 直接发布 `<ns>/odometry/filtered` 与静态 TF，不需运行 Leader 实车/仿真；适合只验证 Follower。 |
| 里程计闭环圆 | `leader_circle_closed_loop.py` | 使用 Leader 里程计反馈生成闭环圆轨迹。 |
| 地图闭环圆 | `leader_circle_closed_loop_map.py` | 在 `map` 系跟踪圆轨迹；默认约 20 Hz，带 `Td` 与 `tau_v` 的领航预测参数，更适合和 map 系编队讨论。 |

比较控制器时，应明确 Leader 是“理想虚拟状态源”还是“真实机器人闭环状态源”。前者去除了 Leader 执行器/定位误差；后者把它们带入 Follower 的相对误差，因此两类结果不能直接当作相同实验条件。

## 12. 建议的最小实验拓扑

### 12.1 虚拟 Leader + 单 Follower（最适合先讨论控制器）

终端 1：启动 robot2 的仿真与 rf2o/EKF。

```bash
ros2 launch homo_multirobot_localization sim_rf2o_ekf_single_robot.launch.py \
  robot_namespace:=/robot2 robot_prefix:=robot2_ robot_x:=2.0 robot_y:=0.0
```

终端 2：为 Follower 启动已知地图定位。

```bash
ros2 launch homo_multirobot_nav slam_toolbox_loc_single_robot.launch.py \
  namespace:=/robot2 prefix:=robot2_ map_name:=sim_room1_map \
  map_start_x:=2.0 map_start_y:=0.0 map_start_yaw:=0.0
```

终端 3：运行虚拟 Leader。

```bash
ros2 run homo_multirobot_formation_control virtual_leader_circle.py \
  --ros-args -r __ns:=/virtual_leader -p radius:=2.0 -p speed:=0.5
```

终端 4：运行当前默认可构建的 4D Artstein 控制器。

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_4d_artstein.launch.py \
  leader_ns:=/virtual_leader follower_ns:=/robot2 tau:=0.43 Td:=0.22 control_rate:=20.0
```

若要运行 6D Artstein Disc/HOCBF，先修改 CMake 以启用相应目标并重新构建；启动命令本身存在，但当前默认构建不会提供所需可执行文件。

### 12.2 双实体机器人联调

1. 启动双机 Gazebo + rf2o + EKF；
2. 让两机获得同一 `map` 下的定位；
3. Leader 使用开环或地图闭环轨迹；
4. Follower 订阅 Leader 和自身的 filtered odometry，发布 `/robot2/cmd_vel`；
5. 记录轨迹并检查 TF、频率和数据时效性。

不应在未确认 `map → odom`、`odom → base_footprint` 唯一发布者以及两车初始位姿的情况下直接解释编队误差。

## 13. 延迟诊断与评价指标

真实闭环的延迟链可概括为：

```text
编码器 → STM32(20 Hz) → 串口 → /odom → EKF → /odometry/filtered → 控制器
Leader filtered odometry → DDS/Wi-Fi → Follower 回调
控制器 → cmd_vel → 串口 → STM32 → 电机
```

建议至少分别报告：

| 指标 | 含义 | 典型工具/观察点 |
|---|---|---|
| `avg_leader_age` | Follower 收到的 Leader 状态相对当前控制时刻的年龄 | 控制器 DIAG 日志；跨机前提是时钟同步。 |
| `avg_ekf_age` | Follower 使用的自身 EKF 状态新鲜度 | 控制器 DIAG 日志。 |
| 控制频率 | 实际 timer 频率，而非参数设定值 | 控制器 DIAG。 |
| 网络延迟 | Leader 状态经 DDS/Wi-Fi 到 Follower 的延迟 | `measure_cross_machine_delay.py`；需 chrony 时钟同步。 |
| 电机响应 | 从阶跃命令到实际运动的响应/死区 | `measure_motor_latency.py`。 |
| 轨迹误差 | 相对半径误差、相位/切换行为、稳态尾段误差 | `record_trajectory.py` 与离线分析。 |
| 约束活动 | 轮速缩放、速度/加速度限幅、安全 QP 是否频繁介入 | 日志中的 raw/clamped/final/scale 与安全诊断。 |

实车的 `/odom` 约 20 Hz 是 STM32 固件限制。因此 EKF 和控制器即使配置更高频，也不必然得到更高质量状态；参数比较应在相同状态更新和测量条件下进行。

## 14. 讨论时必须保留的限制

1. **理论范围**：HPC、Artstein 和 HOCBF 的陈述基于各自名义模型与假设。不能把局部/名义结论直接说成包含饱和、噪声、离散采样、拟合误差和所有时变延迟的全局实车保证。
2. **模型范围**：4D 是 map 系双积分近似；6D 包含车体朝向和 body/map 变换；6D Motor 用显式一阶执行器状态。选择模型应由研究问题决定，而不是“维度越高越好”。
3. **安全范围**：当前 HOCBF 仅基于 scan 拟合静态圆柱。动态人/车、非圆柱多面体、遮挡和不可见障碍不在其明确能力范围内。
4. **定位范围**：全局编队误差的可信度依赖 `map → odom` 质量。激光特征不足、动态环境和错误初始位姿都会污染结论。
5. **构建范围**：代码、README 与默认 CMake 不完全等价；运行前必须检查目标是否真的编译/安装。
6. **比较公平性**：比较基线、Artstein、LQR 或 HOCBF 时，必须保持 Leader 轨迹、采样率、`Td/tau`、速度/加速度/轮速上限、初始状态、障碍几何和评价窗口一致。
7. **实车安全**：任何实车参数修改必须从低速度、较大半径、空旷场地和独立急停条件开始；AI 的参数建议不能替代实测与现场安全评估。

## 15. 推荐向 ChatGPT 提问的方式

为得到可审查的技术回答，提问中最好显式给出：控制器版本、构建状态、Leader 类型、坐标系、状态来源、延迟参数、约束、障碍物模型、目标指标和观察到的日志/曲线。

可直接使用以下模板：

```text
我在本项目中使用 [控制器版本]，当前 [已默认构建 / 已手工启用目标]。
Leader 是 [虚拟 / Gazebo 开环 / 地图闭环 / 实车]，Follower 状态来自 [EKF + slam_toolbox/AMCL]。
控制频率为 [ ] Hz，Td=[ ] s，tau=[ ] s，最大线速度/加速度为 [ ]。
目标是 [半径保持 / 延迟对比 / HOCBF 安全绕行 / 实车迁移]。
观察到 [具体误差、日志、轨迹或约束触发]。
请先检查模型假设和坐标系是否一致；再提出可验证的原因排序、最小改动实验和应记录的指标。不要把未验证的名义理论当作实车保证。
```

适合深入讨论的问题包括：

- 在已测得的 `Td`、`tau` 和实际加速度限制下，4D Artstein 的预测与径向制动层是否自洽？
- 6D Artstein Disc 的 map/body 分层预测会在哪些大转角、饱和或快速切换情况下产生显著残差？
- 当前圆柱拟合和 HOCBF 半径裕度是否覆盖 20 Hz 离散实现与定位误差？应如何做保守性消融实验？
- 如何设计基线、Artstein、LQR 和 HOCBF 的公平对照，并避免 Leader 轨迹差异污染结论？
- 从仿真迁移到实车时，应该优先测量哪一段延迟，如何将测量结果映射到 `Td`、`tau`、限速和限加速度？

## 16. 结论性定位

本项目应被理解为一个可扩展的“**定位—延迟感知编队控制—安全过滤—仿真/实车诊断**”研究平台。当前最直接可构建和运行的控制主线是 4D 基线与 4D Artstein；6D、HOCBF 和电机增广版本保留了更丰富的研究实现与理论文档，但需要根据 CMake 目标和实际验证状态谨慎启用与表述。

在任何讨论中，优先区分以下四件事：**代码是否存在、当前是否编译、名义模型是否证明、实验是否在相同条件下验证**。这四者不能互相替代。
