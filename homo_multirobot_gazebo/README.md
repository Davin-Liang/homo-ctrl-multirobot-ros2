# homo_multirobot_gazebo

面向 **Gazebo Classic 11**（`gazebo_ros` / ROS 2 Humble）的多机仿真启动资源：在同一世界里 **spawn 两台** `mini_omni_robot`（模型与 mesh 来自姊妹包 **`homo_multirobot_urdf`**）。

本包负责 **仿真世界、Gazebo 进程、spawn 流程与 ROS 侧 robot/joint 状态**；**传感器插件、里程计、运动控制** 等若在整体路线中规划为下一步，应在 URDF/Xacro 中增加 `<gazebo>` 插件后再与本 launch 协同（见下文「与后续工作的衔接」）。

---

## 依赖与关系

| 依赖 | 作用 |
|------|------|
| `homo_multirobot_urdf` | 提供 `mini_omni_robot.xacro`、mesh、各台机器人的 `prefix` |
| `gazebo_ros` | `gzserver` / `gzclient` 官方 launch、`/spawn_entity`、环境路径 `GazeboRosPaths` |
| `robot_state_publisher` | 各命名空间内发布 `robot_description` 与 TF |
| `joint_state_publisher` | 为 **continuous** 轮关节提供 `/joint_states`，否则 TF 树中缺少轮子 link |
| `xacro` | spawn 前由 `spawn_entity` 经 `robot_state_publisher` 间接使用已展开的 URDF |

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

### 常用 Launch 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `use_sim_time` | `true` | 与 Gazebo 仿真时钟一致 |
| `world` | 本包 `worlds/empty.world` | 可换自定义 `.world` |
| `gui` | `true` | `false` 仅跑 `gzserver`，无 Gazebo 窗口 |
| `server` | `true` | `false` 不启 `gzserver`（一般保持 true） |
| `verbose` | `false` | `gzserver` 是否啰嗦输出 |
| `software_rendering` | `false` | WSL/WSLg **黑屏**时可试 `true`（CPU 软渲染，界面会更卡） |
| `robot{1,2}_{x,y,z,yaw}` | 见 launch | 初始位姿 |
| `robot{1,2}_namespace` | `/robot1`、`/robot2` | 与 spawn、`robot_state_publisher` 一致 |
| `robot{1,2}_prefix` | `robot1_`、`robot2_` | 与 URDF link/joint 前缀一致 |

---

## 本包已解决的设计要点（协同必读）

1. **`/spawn_entity` 与 `GazeboRosFactory`**  
   使用 `gazebo_ros` 自带的 **`gzserver.launch.py`**，由 `GazeboRosPaths` 设置 `GAZEBO_PLUGIN_PATH` 等，保证 **`libgazebo_ros_factory.so`** 能加载，否则 `spawn_entity.py` 等不到服务。

2. **`model://` 与 mesh 路径**  
   Gazebo 会把 `package://homo_multirobot_urdf/...` 转成 `model://homo_multirobot_urdf/...`。本 launch 在 **`share/homo_multirobot_gazebo/gazebo_model_root/`** 下建立指向 **`homo_multirobot_urdf` 包 share** 的符号链接，并把 **仅含该子目录** 的 `gazebo_model_root` 加入 `GAZEBO_MODEL_PATH`，避免把整个 `.../share` 加进去导致 Insert 面板误扫 `ament_index` 等目录。

3. **TF 与轮子**  
   轮关节为 **continuous**，必须有人发布 **`/robot{1,2}/joint_states`**。本包为每台机器人各起一个 **`joint_state_publisher`**（默认零位），与 `robot_state_publisher` 配合，使 `view_frames` 中可见轮子 link。若后续由 **Gazebo 关节状态插件** 或 **控制器** 发布真实关节角，应改为以仿真源为准（例如通过 `source_list` 或关闭占位用 JSP）。

4. **WSL 环境**  
   可能仍有 ALSA/OpenAL 无关声卡告警，一般可忽略；可选用 `ALSOFT_DRIVERS=null`（launch 内已设）减轻刷屏。

---

## 当前能力边界（避免重复造轮子）

- **已有**：双机 spawn、空世界、命名空间隔离的 `robot_description` / `joint_states` / TF（含轮子占位）、RViz/算法侧可读的模型结构。
- **未包含（若整体路线下一步才做）**：在 URDF/Xacro 中按需添加 Gazebo 插件后的 **`/scan`、`/imu`、`/odom`、`cmd_vel`** 等；全向轮 **真实关节力矩/摩擦** 与 **ros2_control** 等。  
  这些应主要在 **`homo_multirobot_urdf`（或专用控制包）** 中描述，本 launch 仅需保证 **namespace、use_sim_time、spawn 顺序** 与之一致。

---

## 与后续工作的衔接（建议分工）

| 方向 | 建议落点 | 与本包关系 |
|------|----------|------------|
| 激光 / IMU / 平面运动 / `odom` 等 Gazebo 插件 | `homo_multirobot_urdf` 的 xacro 中按 `use_gazebo` 条件插入 `<gazebo>` | 保持 `robot{1,2}_namespace` 与插件 `topic` 一致；必要时在本 launch 增加 `remapping` 或参数 |
| 多机话题约定 `/<ns>/scan` 等 | 插件参数 + 命名空间 | 启动顺序不变：先 `gzserver`，再各机 `robot_state_publisher`，再 `spawn_entity` |
| 实机与仿真切换 | URDF 参数 | 本包继续只负责 Gazebo 侧 spawn 与环境变量 |

建议在仓库的 **整体仿真/协同设计文档** 中保持：  
**模型与插件（URDF）** ↔ **世界与进程（本包）** ↔ **导航/控制栈** 三层边界清晰，避免在 launch 里硬编码过长 URDF。

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
└── worlds/
    └── empty.world
```

---

## 许可

与 `package.xml` 中声明一致（Apache-2.0）。
