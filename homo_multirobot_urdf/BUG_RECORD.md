# BUG_RECORD — `homo_multirobot_urdf` 开发/联调记录

本文档汇总在搭建与使用 `homo_multirobot_urdf` 过程中遇到的问题、原因与处理方式，便于后续复现与排查。

---

## 1. `xacro` 命令不存在

**现象**  
`ros2 launch ...` 报错：`file not found: ... 'xacro'`。

**原因**  
未安装 `xacro` 包，或终端未正确加载 ROS 环境（`PATH` 中无 `xacro`）。

**处理**  
```bash
sudo apt update
sudo apt install -y ros-humble-xacro
source /opt/ros/humble/setup.bash
```

---

## 2. Xacro：`Undefined substitution argument robot_name`

**现象**  
Launch 调用 `xacro` 展开 URDF 时报错：`Undefined substitution argument robot_name`。

**原因**  
在 xacro 文件顶部 `<robot name="$(arg robot_name)">` 使用了 `$(arg robot_name)`，但 `<xacro:arg name="robot_name" ...>` 定义在文件后面。xacro 对 `$(arg ...)` 的解析顺序导致该参数在展开时尚未定义。

**处理**  
将 `<robot name="...">` 改为固定名称（如 `mini_omni_robot`），或保证所有 `<xacro:arg>` 在使用前已声明；本包采用固定 `name`，并移除未使用的 `robot_name` 参数。

---

## 3. `joint_state_publisher_gui` 包未找到

**现象**  
Launch 报错：`package 'joint_state_publisher_gui' not found`。

**原因**  
系统未安装该可选包，且 launch 中无条件引用该节点会导致启动失败。

**处理**  
- 代码侧：`display.launch.py` 中仅在检测到已安装 GUI 包时才启动对应节点；否则仅用 `joint_state_publisher` 或跳过（见下一条）。  
- 环境侧（可选）：`sudo apt install -y ros-humble-joint-state-publisher-gui`。

---

## 4. `ImportError: cannot import name 'get_package_share_directory' from 'launch_ros.utilities'`

**现象**  
Launch 文件加载失败，提示无法从 `launch_ros.utilities` 导入 `get_package_share_directory`。

**原因**  
ROS 2 Humble 中该函数不在 `launch_ros.utilities`，应使用 `ament_index_python.packages`。

**处理**  
```python
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
```

---

## 5. `joint_state_publisher` 包未找到

**现象**  
Launch 报错：`package 'joint_state_publisher' not found`。

**原因**  
系统未安装该包；若 launch 仍强制启动该节点会失败。

**处理**  
- 代码侧：检测包是否存在，不存在则不启动 `joint_state_publisher`（仅 `robot_state_publisher` + RViz 仍可运行）。  
- 环境侧（推荐用于完整关节/轮子 TF）：`sudo apt install -y ros-humble-joint-state-publisher`。

---

## 6. RViz 中看不到机器人模型

**现象**  
Gazebo/话题正常，但 RViz 主视图只有网格，无模型。

**原因**  
默认打开的 RViz 未添加 **RobotModel** 显示项，或未订阅 `/robot_description`。

**处理**  
在包内提供默认 RViz 配置（含 `RobotModel`、`TF`），并由 `display.launch.py` 通过 `-d` 加载；或手动在 RViz 中添加 RobotModel，Description Topic 指向 `/robot_description`。

---

## 7. 使用 `prefix:=robot1_` 后 RobotModel / TF 报错

**现象**  
使用 `prefix` 后，RViz 报 `No transform from [robot1_xxx] to [base_link]` 等错误。

**原因**  
URDF 中 link 名变为 `robot1_base_link` 等，但 RViz **Fixed Frame** 仍为 `base_link`，与带前缀的 TF 树不一致。

**处理**  
`display.launch.py` 在启动 RViz 前根据 `prefix` 生成临时 RViz 配置，将 **Fixed Frame** 设为 `${prefix}base_link`（无 prefix 时为 `base_link`）。

---

## 8. 「命名空间 + frame_prefix」多机方案已移除

**现象/背景**  
曾尝试用 `namespace` + `robot_state_publisher` 的 `frame_prefix` 做多机，与仅改 URDF `prefix` 的方式并存时易混淆，且用户反馈有问题。

**处理**  
已从 `display.launch.py` 中移除 `namespace`、`frame_prefix`、`PushRosNamespace` 及相关逻辑；多机统一使用 **URDF `prefix` 参数** 区分 frame 名称。

---

## 9. 移动功能包目录后 `colcon build` 失败

**现象**  
将 `homo_multirobot_urdf` 从 `src/` 挪到 `src/homo-ctrl-multirobot-ros2/` 后编译报错：CMake 仍指向旧源码路径（目录不存在）。

**原因**  
`build/`、`install/` 中缓存了旧路径的 CMake 配置。

**处理**  
清理该包的构建产物后重编，例如：
```bash
rm -rf build/homo_multirobot_urdf install/homo_multirobot_urdf
colcon build --packages-select homo_multirobot_urdf --symlink-install
```

---

## 备注：本机 `sudo` / `apt` 异常

在部分受限或损坏的环境中可能出现 `sudo` 配置异常、`apt` 无法加锁等，导致无法代为安装 deb 包。需在用户本机 WSL/实体机修复权限或 `sudo` 后再执行 `apt install`。

---

## 10. 双机仿真里程计 frame 同名（`odom`）导致 TF 混淆

**现象**  
双机仿真时虽然话题已按命名空间隔离（如 `/robot1/odom`、`/robot2/odom`），但 RViz/`view_frames` 中出现 TF frame 混乱：两台车看起来共用同一个 `odom`，或 Fixed Frame 选 `odom` 时表现异常。

**原因**  
TF 的 frame 名称不带 ROS 命名空间；若运动/里程计插件把 `odometry_frame` 固定为 `odom`，两台机器人会发布同名 `odom` frame。

**处理**  
让 `odometry_frame` 随 URDF 的 `prefix` 变化，例如 `${prefix}odom`（对应 `robot1_odom`、`robot2_odom`），并在 RViz 中选择对应的 Fixed Frame（或用 `world` 作为全局参考）。

---

## 11. Gazebo 轮子陷进地面（出生穿模 / 抖动）

**现象**  
仿真 spawn 后轮子明显“陷进地面”，或刚体接触抖动明显。

**原因**  
轮关节 `origin` 的 z 高度与轮半径/碰撞体不匹配，导致轮子碰撞体初始位姿与地面发生穿透；ODE 接触求解会强行分离重叠，表现为下陷、抖动或不稳定。

**处理**  
- 调整 `urdf/mini_omni_robot.xacro` 中三轮关节（`front/left/right_wheel_joint`）的 `<origin xyz=...>` 的 z，使轮子初始不穿地。  
- 若已将轮子 collision 简化为 primitive（如 cylinder/box），更要同步校准轮半径与关节 z。

---

## 12. Gazebo（planar_move 模式）走直线会走歪 / 慢慢偏航

**现象**  
`use_ros2_control:=false`（`gazebo_ros_planar_move`）模式下，即使 `cmd_vel` 仅给定直线速度，机器人也会持续产生 yaw 偏航。

**原因**  
在 planar_move 模式，底盘由插件直接驱动，但轮子若仍以“高摩擦”参与物理接触，会对底盘额外施加侧向力/力矩（接触/摩擦数值不对、初始穿模、几何误差都会放大该现象）。

**处理**  
将轮子接触摩擦降为接近 0，避免轮子物理接触主导运动：在 xacro 中为各轮子 link 添加 Gazebo 接触参数：

- `<mu1>0.0</mu1>`
- `<mu2>0.0</mu2>`

---

## 13. 将 `base_link` collision 从 mesh 改为 primitive 后 Gazebo 黑屏/崩溃（`gzserver` 段错误）

**现象**  
把 `base_link` 的 `<collision>` 由 `<mesh ...>` 替换为 `<box>`（或其他 primitive）后，Gazebo 可能在 spawn 后 `gzserver` 直接崩溃（`exit code -11`）；用户侧常表现为 Gazebo 黑屏、窗口卡死或直接退出。

**原因**  
在该模型/环境组合下（Gazebo Classic 11 + 当前 URDF/SDF 转换链路 + 接触/碰撞求解），`base_link` 的 primitive collision 会触发不稳定/崩溃（需要进一步上游定位；当前先以规避为主）。

**处理**  
- `base_link` collision 建议保持 mesh（与 visual 同源），避免触发 `gzserver` 段错误。  
- 若要提升接触稳定性，优先只对轮子等局部做简化，并配合轮高/摩擦参数调参。

---

## 14. rf2o 里程计 `twist.linear.x` 符号与前进方向相反（常见于 Laser frame 翻转）

**现象**  
在双机仿真中（或任意发布 `/scan` 的场景），给机器人正向速度（前进），rf2o 输出的里程计（`/robot*/rf2o/odom`）中 `twist.twist.linear.x` 却为负。

**原因**  
rf2o 会将 `LaserScan.header.frame_id`（通常为 `${prefix}laser_link`）通过 TF 变换到 `base_frame_id`（本仓库推荐 `${prefix}base_footprint`）来推算运动。  
如果 `laser_link` 相对底盘存在 \( \pi \)（180°）的 yaw 翻转（例如 `laser_joint` 的 `rpy` 设置为 `0 0 3.1416`），则“前进方向”在算法坐标中会被映射为相反方向，从而出现速度符号反号。

**处理**  
- 保证 `laser_link` 的 +X 与底盘前向一致：将 `urdf/mini_omni_robot.xacro` 中 `laser_joint` 的 `rpy` 改为 `0 0 0`（或等价的“去掉 180° 翻转”）。  
- 重启仿真（URDF/TF 在启动时加载），再验证：

```bash
ros2 run tf2_ros tf2_echo robot1_base_footprint robot1_laser_link
```

期望 yaw 接近 0°。
