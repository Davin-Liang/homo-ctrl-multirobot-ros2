# homo_multirobot_urdf

本包提供 `mini_omni_robot` 的 **URDF/Xacro + mesh 资源**，用于 RViz 展示与后续多机器人仿真集成。

## 功能
- **模型描述**：`urdf/mini_omni_robot.xacro`
  - 支持 `prefix` 参数（多机时避免 TF frame 重名）
  - 内置 `base_footprint`（相对 `base_link` 高度约 6cm）
- **网格资源**：`meshes/mini_omni_robot_meshes/*.STL`
- **RViz 快速展示**：`launch/display.launch.py`（自动加载 `rviz/mini_omni_robot.rviz`）

## 依赖
必需（ROS 2 Humble）：
- `xacro`
- `robot_state_publisher`
- `rviz2`

可选（用于发布关节状态/让轮子等连续关节的 TF 正常更新）：
- `joint_state_publisher`
- `joint_state_publisher_gui`

## 编译
在工作空间根目录：

```bash
colcon build --packages-select homo_multirobot_urdf --symlink-install
source install/setup.bash
```

## 使用（RViz 展示）

```bash
ros2 launch homo_multirobot_urdf display.launch.py
```

### 多机前缀（推荐）
每台机器人用不同 `prefix`，例如：

```bash
ros2 launch homo_multirobot_urdf display.launch.py prefix:=robot1_
```

> `display.launch.py` 会自动把 RViz 的 Fixed Frame 适配为 `${prefix}base_link`，避免前缀模式下 TF 断链导致看不到模型。

## 常见问题
### RViz 能启动但看不到轮子等部件
如果系统未安装 `joint_state_publisher`，连续关节（如轮子）的 TF 可能不会发布/更新，导致部分部件不显示或 TF 报错。安装后再启动：

```bash
sudo apt update
sudo apt install -y ros-humble-joint-state-publisher
```

（可选 GUI）

```bash
sudo apt install -y ros-humble-joint-state-publisher-gui
```

### 查看 TF 树

```bash
ros2 run tf2_tools view_frames
```

将生成 `frames*.pdf` 用于查看 TF 结构。

