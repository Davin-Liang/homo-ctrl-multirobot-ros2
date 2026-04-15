# homo_multirobot_gazebo

面向 **Gazebo Classic 11**（`gazebo_ros` / ROS 2 Humble）的多机仿真启动资源：在同一世界里 **spawn 两台** `mini_omni_robot`（模型与 mesh 来自姊妹包 **`homo_multirobot_urdf`**）。

本包负责 **仿真世界、Gazebo 进程、spawn、ROS 侧 robot/joint 状态、可选 RViz 与 world 静态 TF**；**底盘平面运动/里程计（`cmd_vel → odom`）** 由 `homo_multirobot_urdf` 的 Gazebo 插件提供（当前为 `gazebo_ros_planar_move`，后续可升级 `ros2_control`）。

---

## 依赖与关系

| 依赖 | 作用 |
|------|------|
| `homo_multirobot_urdf` | 提供 `mini_omni_robot.xacro`、mesh、各台机器人的 `prefix` 与 Gazebo 传感器插件 |
| `gazebo_ros` | `gzserver` / `gzclient` 官方 launch、`/spawn_entity`、环境路径 `GazeboRosPaths` |
| `robot_state_publisher` | 各命名空间内发布 `robot_description` 与 TF |
| `joint_state_publisher` | 为 **continuous** 轮关节提供 `/joint_states`，否则 TF 树中缺少轮子 link |
| `xacro` | spawn 前由 `spawn_entity` 经 `robot_state_publisher` 间接使用已展开的 URDF |
| `rviz2` | 可选，与本包 `rviz/two_robots_sim.rviz` 联调可视化 |
| `tf2_ros` | 可选，发布 `world -> <prefix>base_footprint` 静态 TF（见 `publish_world_tf`） |
| `gazebo_ros` | Gazebo Classic 与 ROS2 桥接（同时提供 `gazebo_ros_planar_move` 插件） |

---

## 编译

在工作空间根目录：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select homo_multirobot_gazebo homo_multirobot_urdf --symlink-install
source install/setup.bash
```

---

## 启动

```bash
ros2 launch homo_multirobot_gazebo sim_two_robots.launch.py
```

默认：两台实体名 `robot1`、`robot2`，命名空间 `/robot1`、`/robot2`，初始位姿 `(0,0)` 与 `(1,0)`，前缀 `robot1_` / `robot2_`（与 URDF 中 `prefix` 一致，避免 TF 重名）。

### 键盘控制（cmd_vel）

安装键盘遥控（若未安装）：

```bash
sudo apt update
sudo apt install -y ros-humble-teleop-twist-keyboard
```

分别控制两台车（开两个终端最直观）：

```bash
# robot1
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel

# robot2
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot2/cmd_vel
```

### 常用 Launch 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `use_sim_time` | `true` | 与 Gazebo 仿真时钟一致 |
| `world` | 本包 `worlds/empty.world` | 可换自定义 `.world` |
| `gui` | `true` | `false` 仅跑 `gzserver`，无 Gazebo 窗口 |
| `server` | `true` | `false` 不启 `gzserver`（一般保持 true） |
| `verbose` | `false` | `gzserver` 是否啰嗦输出 |
| `software_rendering` | `false` | WSL/WSLg **黑屏**时可试 `true`（CPU 软渲染，界面会更卡） |
| `robot{1,2}_{x,y,z,yaw}` | 见 launch | 初始位姿（与 `publish_world_tf` 静态 TF 一致时使用） |
| `robot{1,2}_namespace` | `/robot1`、`/robot2` | 与 spawn、`robot_state_publisher`、传感器插件命名空间一致 |
| `robot{1,2}_prefix` | `robot1_`、`robot2_` | 与 URDF link/joint 前缀一致 |
| `publish_world_tf` | `false` | 发布 `world -> <prefix>base_footprint` 静态 TF（仅用于“初始对齐/静态展示”）；接入里程计/动态 TF 后建议保持 `false`，避免与 `odom -> base_footprint` 等 TF 冲突 |
| `use_rviz` | `true` | 是否同时启动 RViz2 |
| `rviz_config` | 本包 `rviz/two_robots_sim.rviz` | RViz 配置文件路径 |

### 关闭仿真

在运行 `ros2 launch` 的终端 **Ctrl+C**；若 Gazebo 窗口卡死可另开终端：

```bash
pkill -9 gzserver
pkill -9 gzclient
```

---

## 话题与可视化（当前实现）

在 **`homo_multirobot_urdf`** 的 xacro 中已配置 Gazebo 插件（`use_gazebo:=true`），并通过 launch 传入 `ros_namespace:=/robot1` 等，话题按命名空间隔离，多机话题形如：

| 话题（默认命名空间） | 消息类型 | 说明 |
|----------------------|----------|------|
| `/robot1/scan`、`/robot2/scan` | `sensor_msgs/LaserScan` | 2D 激光（`gazebo_ros_ray_sensor` + `~/out:=scan`） |
| `/robot1/imu`、`/robot2/imu` | `sensor_msgs/Imu` | IMU（`gazebo_ros_imu_sensor` + `~/out:=imu`） |
| `/robot1/cmd_vel`、`/robot2/cmd_vel` | `geometry_msgs/Twist` | 底盘速度指令（`gazebo_ros_planar_move` 订阅） |
| `/robot1/odom`、`/robot2/odom` | `nav_msgs/Odometry` | 里程计（`gazebo_ros_planar_move` 发布） |
| `/robot1/robot_description`、… | `std_msgs/String` | 各机模型描述（RViz RobotModel 使用） |
| `/clock` | `rosgraph_msgs/Clock` | 仿真时钟（RViz / 节点需 `use_sim_time`） |

快速检查：

```bash
ros2 topic list | egrep 'robot(1|2)/(scan|imu|cmd_vel|odom)'
```

### RViz2

默认与仿真一同启动，并加载 **`rviz/two_robots_sim.rviz`**（双车模型 + 双 LaserScan + TF，**Fixed Frame = world**）。

当启用里程计后更推荐把 RViz 的 **Fixed Frame** 设为各自的 `odom`（本仓库默认随 `prefix` 变化：例如 `robot1_odom`、`robot2_odom`），或继续使用 `world` 但保持 `publish_world_tf:=false`（默认已是 `false`），避免出现静态 world TF 与动态 odom TF 的混淆。

仅跑 Gazebo、不启 RViz：

```bash
ros2 launch homo_multirobot_gazebo sim_two_robots.launch.py use_rviz:=false
```

单机仅用模型做 RViz 展示请用姊妹包：`ros2 launch homo_multirobot_urdf display.launch.py`。

---

## 本包已解决的设计要点（协同必读）

1. **`/spawn_entity` 与 `GazeboRosFactory`**  
   使用 `gazebo_ros` 自带的 **`gzserver.launch.py`**，由 `GazeboRosPaths` 设置 `GAZEBO_PLUGIN_PATH` 等，保证 **`libgazebo_ros_factory.so`** 能加载，否则 `spawn_entity.py` 等不到服务。

2. **`robot_description` 参数类型**  
   `robot_state_publisher` 的 `robot_description` 由 `xacro` 命令展开，须用 **`ParameterValue(..., value_type=str)`** 声明为字符串，否则 launch 会按 YAML 解析报错。

3. **`model://` 与 mesh 路径**  
   Gazebo 会把 `package://homo_multirobot_urdf/...` 转成 `model://homo_multirobot_urdf/...`。本 launch 在 **`share/homo_multirobot_gazebo/gazebo_model_root/`** 下建立指向 **`homo_multirobot_urdf` 包 share** 的符号链接，并把 **仅含该子目录** 的 `gazebo_model_root` 加入 `GAZEBO_MODEL_PATH`，避免把整个 `.../share` 加进去导致 Insert 面板误扫 `ament_index` 等目录。

4. **TF 与轮子**  
   轮关节为 **continuous**，必须有人发布 **`/robot{1,2}/joint_states`**。本包为每台机器人各起一个 **`joint_state_publisher`**（默认零位），与 `robot_state_publisher` 配合，使 `view_frames` 中可见轮子 link。若后续由 **Gazebo 关节状态插件** 或 **控制器** 发布真实关节角，应改为以仿真源为准（例如通过 `source_list` 或关闭占位用 JSP）。

5. **`publish_world_tf` 与 RViz**  
   默认发布 **`world` → `<prefix>base_footprint`** 静态 TF，与 spawn 初始位姿一致，便于两车在同一 **Fixed Frame** 下显示。车在仿真中运动后，该 TF **不会**随真实位姿更新；接入 **`gazebo_ros_planar_move` / 位姿插件 / `odom`** 后应 **`publish_world_tf:=false`**，改以插件发布的 TF 为准。

6. **WSL 环境**  
   可能仍有 ALSA/OpenAL 无关声卡告警，一般可忽略；可选用 `ALSOFT_DRIVERS=null`（launch 内已设）减轻刷屏。

---

## 当前能力边界

- **已有**：双机 spawn、空世界、命名空间隔离、`robot_description` / `joint_states`、Gazebo 激光与 IMU、以及 **`/<ns>/cmd_vel → /<ns>/odom`（`gazebo_ros_planar_move`）**。
- **尚未实现（下一步可做）**：更逼真的全向轮 **接触/力矩/打滑**、以及基于 `gazebo_ros2_control` 的驱动与关节状态闭环等。

---

## 下一步计划（与仓库整体路线对齐）

| 优先级 | 内容 | 建议落点 |
|--------|------|----------|
| 高 | 平面运动：`cmd_vel` → 底盘运动，并发布 `odom` 与 TF | **已完成**：`homo_multirobot_urdf` 中 `gazebo_ros_planar_move`，话题与 `robot{1,2}_namespace` 一致 |
| 高 | 关闭与本包默认静态 TF 的冲突 | 动态 TF 可用后，启动时 `publish_world_tf:=false`，必要时调整 RViz **Fixed Frame**（如 `odom`） |
| 中 | 更逼真的全向轮动力学 | `gazebo_ros2_control` + 三轮全向运动学（单独迭代） |
| 中 | 多机导航/协同栈 | 独立功能包，订阅各机 `scan`/`odom`，保持话题前缀约定 |

建议在仓库的整体设计文档中保持：**模型与插件（`homo_multirobot_urdf`）** ↔ **世界与进程（本包）** ↔ **导航/控制栈** 三层边界清晰。

---

## 文件结构

源码位于工作空间：`src/homo-ctrl-multirobot-ros2/homo_multirobot_gazebo/`（与 `homo_multirobot_urdf` 同属该目录，便于协同）。

```
homo_multirobot_gazebo/
├── CMakeLists.txt
├── package.xml
├── README.md
├── BUG_RECORD.md
├── launch/
│   └── sim_two_robots.launch.py
├── rviz/
│   └── two_robots_sim.rviz
└── worlds/
    └── empty.world
```

---

## 许可

与 `package.xml` 中声明一致（Apache-2.0）。
