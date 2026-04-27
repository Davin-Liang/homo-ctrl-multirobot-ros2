# homo_multirobot_gazebo

面向 **Gazebo Classic 11**（`gazebo_ros` / ROS 2 Humble）的多机仿真启动资源：在同一世界里 **spawn 两台** `mini_omni_robot`（模型与 mesh 来自姊妹包 **`homo_multirobot_urdf`**）。

本包负责 **仿真世界、Gazebo 进程、spawn、ROS 侧 robot/joint 状态、可选 RViz 与 world 静态 TF**；**底盘运动/里程计（`cmd_vel → odom`）** 由 `homo_multirobot_urdf` 的 Gazebo 插件路径提供：可在 `gazebo_ros_planar_move` 与 `gazebo_ros2_control`（`ros2_control` 接口）之间切换。

---

## 📖 依赖与关系

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

## 🛠️ 编译

在工作空间根目录：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select homo_multirobot_gazebo homo_multirobot_urdf omnidirectional_controllers --symlink-install
source install/setup.bash
```

> 若你曾在 **不同路径** 构建过同名包（例如把 `omnidirectional_controllers` 从 `src/` 移到 `src/homo-ctrl-multirobot-ros2/`），可能遇到 CMake cache “source does not match” 报错。可删除对应 `build/<pkg>` 后再重新 `colcon build`。

---

## 🚀 启动

```bash
ros2 launch homo_multirobot_gazebo sim_two_robots.launch.py
```

默认：两台实体名 `robot1`、`robot2`，命名空间 `/robot1`、`/robot2`，初始位姿 `(0,0)` 与 `(1,0)`，前缀 `robot1_` / `robot2_`（与 URDF 中 `prefix` 一致，避免 TF 重名）。

> 若你计划使用 `rf2o`（激光里程计）/EKF，推荐使用带墙体/结构的世界（例如 `sim_room1.world` / `test_world.world`），避免 `empty.world` 特征不足导致 rf2o 漂移：
>
> `ros2 launch homo_multirobot_gazebo sim_two_robots.launch.py world_name:=sim_room1.world`

### 单机仿真（只 spawn 一台车）

当你需要“只建图/只联调一台车”（避免第二台车对激光/建图造成动态干扰）时使用：

```bash
ros2 launch homo_multirobot_gazebo sim_single_robot.launch.py
```

默认：实体名 `robot1`、命名空间 `/robot1`、前缀 `robot1_`。

### 🧩 ros2_control 模式（关闭 planar_move）

```bash
ros2 launch homo_multirobot_gazebo sim_two_robots.launch.py use_ros2_control:=true
```

注意：启用 `ros2_control` 后，本仓库当前尚未在 launch 中启动控制器（如 `controller_manager` + spawner + `omnidirectional_controllers`），因此 **Gazebo 不会再因为 planar_move 而“自动订阅 cmd_vel 并发布 odom”**；此时应把“能不能动起来/能不能出 odom”交给后续控制器接入来完成。

---

## 🧭 ros2_control + OmnidirectionalController（下一步协同入口）

当 `use_ros2_control:=true` 时，本包已经确保 URDF 走 `gazebo_ros2_control` 路径（由 `homo_multirobot_urdf` 控制），接下来需要在 launch 中为每台机器人启动：

- `controller_manager`（由 `gazebo_ros2_control` 插件提供）
- `spawner`：
  - `joint_state_broadcaster`
  - `omnidirectional_controller`（来自包 `omnidirectional_controllers`）

### 📄 控制器 YAML（已准备好）

本包已提供两台机器人的控制器配置文件（几何参数与计划一致：**r=0.03, L=0.24**；三轮对称时 **gamma=60°**）：

- `config/omni_controller_robot1.yaml`
- `config/omni_controller_robot2.yaml`

关键点（协同约定）：

- **wheel_names**：必须与 URDF 的 `prefix` 对齐（本仓库默认 `robot1_`/`robot2_`），因此 YAML 使用：
  - `robot1_front_wheel_joint / robot1_left_wheel_joint / robot1_right_wheel_joint`
  - `robot2_front_wheel_joint / robot2_left_wheel_joint / robot2_right_wheel_joint`
- **base/odom frame**：YAML 里固定为 `robot{1,2}_base_footprint` 与 `robot{1,2}_odom`，避免多机 TF 重名。
- **cmd_vel**：控制器默认订阅相对话题 `~/cmd_vel_unstamped`（或 `~/cmd_vel`），因此在命名空间 `/robot1` 下会是 `/robot1/omnidirectional_controller/cmd_vel_unstamped`。

### 🔁 推荐的话题对接方式（建议写进下一步 launch）

为了让现有键盘遥控命令不变（仍然发 `/robot1/cmd_vel`），建议在启动控制器时做 remap：

- `/robot1/cmd_vel` → `/robot1/omnidirectional_controller/cmd_vel_unstamped`
- `/robot2/cmd_vel` → `/robot2/omnidirectional_controller/cmd_vel_unstamped`

控制器发布的里程计是相对话题 `~/odom`，在命名空间 `/robot1` 下会是 `/robot1/omnidirectional_controller/odom`。如希望外部统一用 `/robot1/odom`，也建议在 launch 中 remap：

- `/robot1/omnidirectional_controller/odom` → `/robot1/odom`（robot2 同理）

> 说明：`omnidirectional_controllers` 的 README 文档里也给出了默认的订阅/发布话题名（`~/cmd_vel_unstamped` 与 `~/odom`），本仓库 YAML 与之保持一致，方便直接 spawner 加载。

### ⌨️ 键盘控制（cmd_vel）

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

### ⚙️ 常用 Launch 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `use_sim_time` | `true` | 与 Gazebo 仿真时钟一致 |
| `use_ros2_control` | `false` | `true` 时在 URDF 中启用 `ros2_control` + `gazebo_ros2_control`，并关闭 `gazebo_ros_planar_move`（避免重复驱动） |
| `world_name` | `sim_room1.world` | 从本包 `worlds/` 目录中加载的世界文件名（例如 `test_world.world`、`empty.world`）。若需绝对路径，请直接用 `world:=/abs/path/to.world` 覆盖。 |
| `world` | 本包 `worlds/sim_room1.world` | 可换自定义 `.world` |
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

示例：从本包 `worlds/` 目录切换世界文件（注意 ROS 2 launch 参数格式为 `name:=value`）：

```bash
ros2 launch homo_multirobot_gazebo sim_two_robots.launch.py world_name:=test_world.world
```

### 🧹 关闭仿真

在运行 `ros2 launch` 的终端 **Ctrl+C**；若 Gazebo 窗口卡死可另开终端：

```bash
pkill -9 gzserver
pkill -9 gzclient
```

---

## 📡 话题与可视化（当前实现）

在 **`homo_multirobot_urdf`** 的 xacro 中已配置 Gazebo 插件（`use_gazebo:=true`），并通过 launch 传入 `ros_namespace:=/robot1` 等，话题按命名空间隔离，多机话题形如：

| 话题（默认命名空间） | 消息类型 | 说明 |
|----------------------|----------|------|
| `/robot1/scan`、`/robot2/scan` | `sensor_msgs/LaserScan` | 2D 激光（`gazebo_ros_ray_sensor` + `~/out:=scan`） |
| `/robot1/imu`、`/robot2/imu` | `sensor_msgs/Imu` | IMU（`gazebo_ros_imu_sensor` + `~/out:=imu`） |
| `/robot1/cmd_vel`、`/robot2/cmd_vel` | `geometry_msgs/Twist` | 底盘速度指令（**planar_move 模式**下由 `gazebo_ros_planar_move` 订阅；**ros2_control 模式**下需由后续控制器订阅） |
| `/robot1/odom`、`/robot2/odom` | `nav_msgs/Odometry` | 里程计（**planar_move 模式**下由 `gazebo_ros_planar_move` 发布；**ros2_control 模式**下需由后续控制器发布） |
| `/robot1/robot_description`、… | `std_msgs/String` | 各机模型描述（RViz RobotModel 使用） |
| `/clock` | `rosgraph_msgs/Clock` | 仿真时钟（RViz / 节点需 `use_sim_time`） |

快速检查：

```bash
ros2 topic list | egrep 'robot(1|2)/(scan|imu|cmd_vel|odom)'
```

### 🔍 确认当前使用的是哪条驱动路径（planar_move vs ros2_control）

`robot_description` 很长，建议使用 `--full-length`：

```bash
ros2 topic echo /robot1/robot_description --once --full-length \
| grep -E "gazebo_ros_planar_move|gazebo_ros2_control|ros2_control" | head
```

输出中出现 `libgazebo_ros_planar_move.so` 则为 planar_move 模式；出现 `libgazebo_ros2_control.so` 与 `<ros2_control ...>` 则为 ros2_control 模式。

### 🖼️ RViz2

默认与仿真一同启动，并加载 **`rviz/two_robots_sim.rviz`**（双车模型 + 双 LaserScan + TF，**Fixed Frame = world**）。

当启用里程计后更推荐把 RViz 的 **Fixed Frame** 设为各自的 `odom`（本仓库默认随 `prefix` 变化：例如 `robot1_odom`、`robot2_odom`），或继续使用 `world` 但保持 `publish_world_tf:=false`（默认已是 `false`），避免出现静态 world TF 与动态 odom TF 的混淆。

仅跑 Gazebo、不启 RViz：

```bash
ros2 launch homo_multirobot_gazebo sim_two_robots.launch.py use_rviz:=false
```

单机仅用模型做 RViz 展示请用姊妹包：`ros2 launch homo_multirobot_urdf display.launch.py`。

---

## ✅ 本包已解决的设计要点（协同必读）

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

## 🧱 当前能力边界

- **已有**：双机 spawn、空世界、命名空间隔离、`robot_description` / `joint_states`、Gazebo 激光与 IMU；底盘驱动支持两种路径：  
  - `use_ros2_control:=false`：**`/<ns>/cmd_vel → /<ns>/odom`（`gazebo_ros_planar_move`）**  
  - `use_ros2_control:=true`：**加载 `gazebo_ros2_control` + `ros2_control` 接口**（控制器接入见下一步）
- **尚未实现（下一步可做）**：启动每台机器人的 `controller_manager` + spawner，并接入全向轮控制器（如 `omnidirectional_controllers`）以驱动轮关节并发布 `odom/TF`。

---

## 🗺️ 下一步计划（与仓库整体路线对齐）

| 优先级 | 内容 | 建议落点 |
|--------|------|----------|
| 高 | 平面运动：`cmd_vel` → 底盘运动，并发布 `odom` 与 TF | **已完成**：`homo_multirobot_urdf` 中 `gazebo_ros_planar_move`，话题与 `robot{1,2}_namespace` 一致 |
| 高 | 关闭与本包默认静态 TF 的冲突 | 动态 TF 可用后，启动时 `publish_world_tf:=false`，必要时调整 RViz **Fixed Frame**（如 `odom`） |
| 中 | 更逼真的全向轮动力学 | `gazebo_ros2_control` + 三轮全向运动学（单独迭代） |
| 中 | 多机导航/协同栈 | 独立功能包，订阅各机 `scan`/`odom`，保持话题前缀约定 |

建议在仓库的整体设计文档中保持：**模型与插件（`homo_multirobot_urdf`）** ↔ **世界与进程（本包）** ↔ **导航/控制栈** 三层边界清晰。

---

## 🗂️ 文件结构

源码位于工作空间：`src/homo-ctrl-multirobot-ros2/homo_multirobot_gazebo/`（与 `homo_multirobot_urdf` 同属该目录，便于协同）。

```
homo_multirobot_gazebo/
├── CMakeLists.txt
├── package.xml
├── README.md
├── BUG_RECORD.md
├── config/
│   ├── omni_controller_robot1.yaml
│   └── omni_controller_robot2.yaml
├── launch/
│   ├── sim_two_robots.launch.py
│   └── sim_single_robot.launch.py
├── rviz/
│   └── two_robots_sim.rviz
└── worlds/
    ├── sim_room1.world
    ├── test_world.world
    └── empty.world
```

---

## 📄 许可

与 `package.xml` 中声明一致（Apache-2.0）。
