# Bug / 问题记录（Gazebo 双机仿真集成过程）

本文档汇总在集成 **`homo_multirobot_gazebo` + `homo_multirobot_urdf` + Gazebo Classic** 过程中出现过的问题、原因与处理方式，便于复盘与后续协同。

**环境参考**：ROS 2 Humble、Gazebo Classic 11、`gazebo_ros`；部分现象出现在 **WSL2/WSLg**。

---

## 1. `spawn_entity.py` 使用 `-string` 传 URDF，Gazebo 中无机器人 / 终端打印大段 URDF

**现象**：启动后世界里没有模型，终端出现 URDF 片段或异常输出。

**原因**：ROS 2 Humble 的 `gazebo_ros/spawn_entity.py` **不支持** 用 `-string` 传入整段 URDF（与部分文档或习惯用法不一致）；参数被误解析，容易导致失败并打印 XML。

**处理**：每台机器人由 **`robot_state_publisher`** 在对应命名空间发布 `robot_description`（话题），`spawn_entity.py` 使用 **`-topic robot_description`** 取模型再 spawn。

---

## 2. `Service /spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?`

**现象**：`spawn_entity.py` 等待 `/spawn_entity` 超时，退出码非 0。

**原因**：`libgazebo_ros_factory.so` 未成功加载时，Gazebo **不提供** `/spawn_entity` 服务。常见诱因包括：

- 手写 `ExecuteProcess(gzserver ...)` **未** 使用 `gazebo_ros` 提供的 **`GazeboRosPaths`** 合并后的 `GAZEBO_PLUGIN_PATH` / 环境，导致工厂插件找不到或加载失败；
- 仅传短文件名 `-s libgazebo_ros_factory.so` 而搜索路径不完整时，也可能加载失败。

**处理**：改为 **`IncludeLaunchDescription`** 引入 **`gazebo_ros/launch/gzserver.launch.py`**（及官方 `gzclient.launch.py`），由官方脚本设置环境与 **`init`/`factory` 插件**；避免在 launch 里自行拼一个不完整的 `gzserver` 命令。

---

## 3. 已改 `extra_gazebo_args` 仍无 `/spawn_entity`

**现象**：仍报 `GazeboRosFactory` 相关错误。

**原因**：若仍通过外层 `gazebo.launch.py` 传参，需确认该 **Humble 版本** 的 `gazebo.launch.py` 是否把参数正确转给 **`gzserver.launch.py`**；历史上曾出现参数未生效、仍走不完整启动路径的情况。

**处理**：**直接使用 `gzserver.launch.py`**，不依赖中间层；保证与 `gazebo_ros` 版本一致。

---

## 4. Gazebo 黑屏 / 界面极卡

**现象**：`gzclient` 窗口全黑或帧率很低。

**原因（黑屏）**：WSL/WSLg 下 OpenGL **兼容性问题**，硬件路径失败时无画面。

**原因（卡顿）**：为缓解黑屏开启 **`LIBGL_ALWAYS_SOFTWARE=1`（软件渲染）** 后，CPU 负担大，界面明显变卡。

**处理**：

- 默认 **`software_rendering:=false`**，优先 GPU；
- WSL 黑屏时再 **`software_rendering:=true`**；
- 仅需后台仿真时使用 **`gui:=false`**。

---

## 5. Mesh 不显示 / `model://homo_multirobot_urdf/...` 找不到，`No mesh specified`

**现象**：模型“透明”或控制台报 `Fuel` / `SystemPaths` 找不到 `model://homo_multirobot_urdf/meshes/...`。

**原因**：Gazebo 将 `package://homo_multirobot_urdf/...` 转为 **`model://homo_multirobot_urdf/...`** 解析；必须在 **`GAZEBO_MODEL_PATH`** 下存在名为 **`homo_multirobot_urdf`** 的目录指向资源根。若路径配错，mesh 无法加载。

**处理**：为 `model://` 提供正确父路径（见下一条“路径过宽”的 refined 做法）。

---

## 6. 将 `.../install/<pkg>/share` 整段加入 `GAZEBO_MODEL_PATH` 的副作用

**现象**：`InsertModelWidget` 报错 **`Missing model.config`**，路径指向 `ament_index`、`colcon-core` 等。

**原因**：`GAZEBO_MODEL_PATH` 的每一项下的 **每个子目录** 都会被当成“可插入模型”；`share` 下除功能包外还有 **构建系统目录**，并非 Gazebo 模型结构，也没有 `model.config`。

**处理**：仅把 **只包含 `homo_multirobot_urdf` 单一模型目录** 的路径加入 `GAZEBO_MODEL_PATH`。当前实现为：在 **`gazebo_model_root`** 下对 **`homo_multirobot_urdf` 的 share** 建 **符号链接**，并把 **`gazebo_model_root`** 作为 model path（见 `sim_two_robots.launch.py`）。

---

## 7. 仍提示某路径下 `Missing model.config`（指向 `.../gazebo_model_root/homo_multirobot_urdf`）

**现象**：spawn 已成功，但 `gzclient` 对 `homo_multirobot_urdf` 报缺 `model.config`。

**原因**：该目录本质是 **ROS 包资源树**（URDF/mesh），**不是** Gazebo 官方“模型仓库”那种带 `model.config` 的包；**插入模型面板** 仍会按 Gazebo 模型规则检查。

**处理**：可忽略（不影响 `spawn_entity`）；若需消除日志，可在资源包中按 Gazebo 模型规范补 **最小 `model.config`**（可选）。

---

## 8. TF 树缺少轮子 link

**现象**：`view_frames` 中只有底盘与固定传感器，没有 `*_wheel_link`。

**原因**：轮关节为 **`continuous`**，`robot_state_publisher` 依赖 **`/joint_states`** 才能发布可动关节 TF；仅有 RSP、无 **关节状态源** 时，轮子不会出现在 TF 树中。

**处理**：每台机器人增加 **`joint_state_publisher`**（默认零位作占位）；后续若由 Gazebo 或控制器发布真实关节状态，再改为以仿真/控制源为准。

---

## 9. 双机里程计坐标系 `odom` 同名导致 TF/可视化混乱

**现象**：两台车均发布 `odom -> <prefix>base_footprint`，但 `odom` frame 名相同，导致 TF 树中出现同名 frame，RViz/工具中看起来“抢占/跳变/混淆”（尤其在同一 Fixed Frame 下显示两车时）。

**原因**：多机隔离了话题（`/robot1/odom`、`/robot2/odom`），但 **TF frame 名称本身不带命名空间**；若插件/控制器把 `odometry_frame` 固定写成 `odom`，两台机器人会发布同名 frame。

**处理**：让 `odometry_frame` 随 URDF `prefix` 变化，例如使用 `${prefix}odom`（最终为 `robot1_odom`、`robot2_odom`），并在 RViz 中分别选择对应 Fixed Frame 或保持 `world` 但避免引入冲突的静态 TF。

---

## 9. ALSA / OpenAL 音频相关报错（WSL）

**现象**：`Unknown PCM default`、`Unable to open audio device`、`Audio will be disabled`。

**原因**：WSL 常见 **无声卡**，OpenAL 回退失败。

**处理**：一般 **不影响** 运动学与传感器仿真；launch 中可设 **`ALSOFT_DRIVERS=null`** 等减轻刷屏（见当前 launch）。

---

## 10. 在线模型库 / `Getting models from ...` 卡顿或异常

**现象**：启动阶段长时间卡在拉取模型、或 `database.config` 相关警告。

**原因**：与 **网络**、**模型库 URI** 配置有关；若曾将 **`GAZEBO_MODEL_DATABASE_URI` 置空**，可能触发异常 URI（如 `//`），行为依版本而异。

**处理**：优先用 **本地 `GAZEBO_MODEL_PATH`** 解决自有 mesh；在线模型库相关配置改前需 **单点验证**，避免写进 launch 的默认值 unless 必要。

---

## 复盘建议

| 类别 | 教训 |
|------|------|
| Spawn | 以当前发行版 **`spawn_entity.py` 实际支持的参数** 为准；优先 `-topic` + RSP。 |
| 插件 / 服务 | 使用 **`gazebo_ros` 官方 gzserver launch**，保证 **factory/init** 与 **路径环境** 一致。 |
| 渲染 | WSL 上 **黑屏 vs 流畅** 往往与 **软渲染** 权衡有关，应用 launch 参数区分环境。 |
| 资源路径 | `model://` 与 **`GAZEBO_MODEL_PATH` 目录语义** 强相关；避免把整棵 `share` 当模型根。 |
| TF | **可动关节** 必须配合 **joint_states**（或等价数据源）。 |

---

*文档随仿真方案演进可继续追加条目。*
