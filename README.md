# homo-ctrl-multirobot-ros2

面向 **ROS 2 Humble** 的多机器人协同与仿真相关代码：以 **`mini_omni_robot`** 全向底盘模型为核心，提供 **URDF/Xacro + mesh**、**Gazebo Classic 双机仿真** 与 **RViz** 可视化。

本仓库已收敛为 **`homo_multirobot_*` 包**；若你曾从旧工程合并过 Wheeltec 厂商的 `wheeltec_robot_urdf` / `turn_on_wheeltec_robot`，它们已从本仓库移除，避免与当前模型与话题约定混淆。实机与 Wheeltec 驱动栈请单独维护或使用上游发行包。

---

## 仓库结构

| 包名 | 说明 |
|------|------|
| **`homo_multirobot_urdf`** | `mini_omni_robot.xacro`、STL mesh、单机 RViz 展示 launch |
| **`homo_multirobot_gazebo`** | 空世界、双机 spawn、可选 RViz 配置与 `world` 静态 TF |
| **`omnidirectional_controllers`** | 引入的上游 ros2_control 控制器（订阅 `cmd_vel`，输出轮速，发布里程计等），用于后续三轮全向底盘轮子级控制 |

各包内另有 **`README.md`** 与 **`BUG_RECORD.md`**，用于细节与排障。

---

## 环境依赖

- **Ubuntu 22.04**（与 ROS 2 Humble 官方配套）
- **ROS 2 Humble**（`desktop` 或 `ros-base` + 按需组件）
- **Gazebo Classic 11**（`gazebo_ros` / `gazebo_plugins`）
- **ROS 包**（示例）：`xacro`、`robot_state_publisher`、`joint_state_publisher`、`rviz2`、`tf2_ros`

安装示例（按需调整）：

```bash
sudo apt update
sudo apt install -y ros-humble-gazebo-ros ros-humble-gazebo-plugins \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher ros-humble-rviz2
```

---

## 部署步骤

### 1. 获取代码

将本仓库置于工作空间 **src** 下，例如：

```text
<your_ws>/
  src/
    homo-ctrl-multirobot-ros2/    # 本仓库（git clone 到此路径）
      homo_multirobot_urdf/
      homo_multirobot_gazebo/
      omnidirectional_controllers/
      README.md
```

### 2. 编译

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
# ros2_control 与控制器基座依赖（提供 controller_interface 等）
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers

colcon build --packages-select homo_multirobot_gazebo homo_multirobot_urdf omnidirectional_controllers --symlink-install
source install/setup.bash
```

若你此前在同一工作空间里编译过 **已删除的第三方包**（如旧的 `wheeltec_*`），`install/` 里可能仍残留同名目录；可手动删除这些目录，或在工作空间根目录 **清空 `build/`、`install/`、`log/` 后做一次全量重编**，避免 `ament` 与路径混淆。

仅编译模型包（无需 Gazebo）：

```bash
colcon build --packages-select homo_multirobot_urdf --symlink-install
source install/setup.bash
```

### 3. 运行

**单机 RViz（无仿真）**：

```bash
ros2 launch homo_multirobot_urdf display.launch.py
```

**Gazebo 双机仿真（含 Gazebo 客户端与 RViz，可选）**：

```bash
ros2 launch homo_multirobot_gazebo sim_two_robots.launch.py
```

常用参数见 **`homo_multirobot_gazebo/README.md`**（如 `use_rviz`、`publish_world_tf`、`gui` 等）。

### 4. 验证话题（仿真）

在仿真运行后：

```bash
ros2 topic list | egrep 'robot(1|2)/(scan|imu)'
```

---

## 与上层工作空间的关系

本目录是 **独立 Git 仓库**，可单独 `git clone`；也可作为 **`homo_fleet_ws/src/homo-ctrl-multirobot-ros2`** 等大工作空间中的一份子。  
无论哪种方式，**编译与 `source install/setup.bash` 始终在 Colcon 工作空间根目录**进行。

---

## 许可

各子包以各自 `package.xml` 声明为准（Apache-2.0 等）。
