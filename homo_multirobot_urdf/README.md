# homo_multirobot_urdf

本包提供 `mini_omni_robot` 的 **URDF/Xacro + mesh 资源**，用于 **RViz 单机展示**、**Gazebo 多机仿真**（与姊妹包 **`homo_multirobot_gazebo`** 配合）。

## 功能

- **模型描述**：`urdf/mini_omni_robot.xacro`
  - **`prefix`**：多机时 link/joint 名加前缀，避免 TF 重名（如 `robot1_` → `robot1_base_link`）。
  - **`use_gazebo`**：为 `true` 时在 Gazebo 中插入 **激光**、**IMU** 等 `<gazebo>` 块；纯 RViz 可看需求设为 `false`。
  - **`ros_namespace`**：传给 Gazebo 插件的 ROS 命名空间，需与仿真 launch 里每台车的 `robot*_namespace` 一致（如 `/robot1`），才能得到约定话题 **`/<ns>/scan`**、**`/<ns>/imu`**。
  - 内置 **`base_footprint`**（相对 `base_link` 高度约 6 cm）。
- **网格资源**：`meshes/mini_omni_robot_meshes/*.STL`
- **单机 RViz**：`launch/display.launch.py`（默认加载 `rviz/mini_omni_robot.rviz`）

## Gazebo 传感器话题约定（ROS 2）

本 xacro 使用 `gazebo_plugins` 中的 **`libgazebo_ros_ray_sensor.so`** / **`libgazebo_ros_imu_sensor.so`**。在 ROS 2 中插件默认发布在 **`~/out`**，需使用 **`<remapping>~/out:=scan</remapping>`**（激光另加 **`sensor_msgs/LaserScan`** 的 `output_type`），IMU 使用 **`~/out:=imu`**，再配合 **`<ros><namespace>...</namespace></ros>`**，最终话题为：

- **`/<ros_namespace>/scan`** — `sensor_msgs/LaserScan`
- **`/<ros_namespace>/imu`** — `sensor_msgs/Imu`

勿再使用 ROS 1 风格的 `<topicName>`，否则会出现 `.../gazebo_ros_*_sensor/out` 等名称。

多机仿真请使用：

```bash
ros2 launch homo_multirobot_gazebo sim_two_robots.launch.py
```

该 launch 会为 xacro 传入与各车一致的 **`prefix`** 与 **`ros_namespace`**。

## 依赖

必需（ROS 2 Humble）：

- `xacro`
- `robot_state_publisher`
- `rviz2`（仅在使用 `display.launch.py` 或 Gazebo 包带 RViz 时）

可选：

- `joint_state_publisher` / `joint_state_publisher_gui`（连续关节 TF；Gazebo 多机 launch 已用 `joint_state_publisher`）
- `gazebo_plugins` / `gazebo_ros`（Gazebo 仿真时由仿真环境提供）

## 编译

在工作空间根目录：

```bash
colcon build --packages-select homo_multirobot_urdf --symlink-install
source install/setup.bash
```

与 Gazebo 包一起编译：

```bash
colcon build --packages-select homo_multirobot_gazebo homo_multirobot_urdf --symlink-install
```

## 使用

### 单机 RViz（无 Gazebo）

```bash
ros2 launch homo_multirobot_urdf display.launch.py
```

多机前缀示例（仅展示一辆车）：

```bash
ros2 launch homo_multirobot_urdf display.launch.py prefix:=robot1_
```

`display.launch.py` 会将 RViz 的 **Fixed Frame** 适配为 **`${prefix}base_link`**（无前缀时为 `base_link`）。

### 双机 Gazebo + RViz

见 **`homo_multirobot_gazebo`** 的 README：`sim_two_robots.launch.py` 会加载双车 RViz 配置，并处理命名空间与可选 `world` 静态 TF。

## 下一步（与本包相关的实现项）

| 项目 | 说明 |
|------|------|
| **`/<ns>/odom` 与 `cmd_vel`** | 在 xacro 中增加 `gazebo_ros_planar_move`（或等价插件），并与 `ros_namespace` 对齐；接入后仿真侧动态位姿应替代仅初始对齐的静态 TF（见 gazebo 包 `publish_world_tf` 说明）。 |
| **实机 / 仿真切换** | 通过 `use_gazebo`、专用宏或拆分 xacro 减少重复。 |
| **全向轮高保真** | 后续可引入 `ros2_control` 与独立控制器包，与本 URDF 解耦迭代。 |

## 常见问题

### RViz 能启动但看不到轮子等部件

若未安装 `joint_state_publisher`，连续关节（如轮子）的 TF 可能不完整。安装后重试：

```bash
sudo apt update
sudo apt install -y ros-humble-joint-state-publisher
```

### 查看 TF 树

```bash
ros2 run tf2_tools view_frames
```

将生成 `frames*.pdf` 用于查看 TF 结构。
