# 配置文件说明

本目录集中存放编队控制、轨迹记录，以及双机器人定位/建图联调所需的默认配置。
各 YAML 都可由对应 launch 文件加载；启动命令中的同名参数会覆盖 YAML 默认值。
控制器具体参数含义以各 YAML 文件中的行内中文注释为准。

## 编队控制器

| 文件 | 对应 launch / 节点 | 用途 |
| --- | --- | --- |
| `formation_control.yaml` | 早期 `formation_control_node` 参数模板 | 4D 原始编队控制器的基础命名空间、圆形编队、yaw 和控制频率参数。当前优先使用下方的 `formation_single_follower.yaml`。 |
| `formation_single_follower.yaml` | `formation_single_follower.launch.py` / `formation_control_node` | 4D 原始 HPC 单 Follower 控制器；包含 EKF/动捕状态源、离散编队点、yaw PD 和轮速/加速度约束。 |
| `formation_single_follower_4d_artstein.yaml` | `formation_single_follower_4d_artstein.launch.py` / `formation_control_node_4d_artstein` | 4D Artstein 控制器；在原始 4D 参数基础上增加 `tau`、`Td` 延迟补偿、可选 Leader 命令前馈及径向安全层。 |
| `formation_single_follower_6d_disc.yaml` | `formation_single_follower_6d_disc.launch.py` / `formation_control_node_6d_disc` | 6D 运动学离散多边形编队控制器；使用平移/yaw 闭环带宽及运动学轮速约束。 |
| `formation_single_follower_6d_artstein_disc_hocbf.yaml` | `formation_single_follower_6d_artstein_disc_hocbf.launch.py` / `formation_control_node_6d_artstein_disc_hocbf` | 6D Artstein 离散编队加 HOCBF 避障；除延迟补偿外，还配置激光聚类、圆柱拟合与障碍物安全间隙。 |
| `formation_single_follower_6d_map_hpc.yaml` | `formation_single_follower_6d_map_hpc.launch.py` / `formation_control_node_6d_map_hpc` | 6D map-frame HPC 无补偿基线。直接使用当前 Leader/Follower 测量状态，适合实物上与 Artstein 版本对照；默认关闭仿真电机延迟。 |
| `formation_single_follower_6d_map_hpc_artstein.yaml` | `formation_single_follower_6d_map_hpc_artstein.launch.py` / `formation_control_node_6d_map_hpc_artstein` | 6D map-frame HPC Artstein 版本。使用固定 map 系编队偏移，并配置独立可开关的 Artstein 输入延迟补偿与 tau 前向预测，以及可选 Leader 命令前馈。 |

除 `formation_single_follower_6d_disc.yaml` 外，带 `use_motor_delay` 的控制器配置都可在
Gazebo 中经 `sim_motor_delay.py` 注入一阶电机滞后和传输延迟。实物测试应保持它为 `false`。

## 轨迹记录

| 文件 | 对应程序 | 用途 |
| --- | --- | --- |
| `record_trajectory.yaml` | `record_trajectory.py` | 轨迹采集默认值：Leader/Follower 命名空间、记录时长、输出目录、实验标签/试验编号、状态源，以及用于自动写入元数据的控制器节点名。 |

## 双机器人联调配置

这两个文件为了让编队联调的仿真、定位默认值集中管理而置于本目录；实际由其他包的 launch 加载。

| 文件 | 实际加载它的 launch | 用途 |
| --- | --- | --- |
| `sim_rf2o_ekf_two_robots.launch.yaml` | `homo_multirobot_localization/sim_rf2o_ekf_two_robots.launch.py` | 双机器人 Gazebo + rf2o + EKF 定位链路的默认世界、GUI/RViz、两车命名空间/TF 前缀/初始位姿，以及 planar/rf2o TF 发布开关。 |
| `slam_toolbox_loc_two_robots.launch.yaml` | `homo_multirobot_nav/slam_toolbox_loc_two_robots.launch.py` | 双机器人 slam_toolbox 纯定位的地图名、map 坐标系、scan 话题、Rviz、初始位姿发布对象及两车命名空间/TF 前缀。 |

## 常用方式

例如，以无补偿 6D Map HPC 做实物对照：

```bash
ros2 launch homo_multirobot_formation_control formation_single_follower_6d_map_hpc.launch.py \
  leader_ns:=/robot1 follower_ns:=/robot2 use_sim_time:=false use_motor_delay:=false
```

只需临时调参时，优先在 launch 命令中覆盖参数；需要作为长期实验默认值时，再修改相应 YAML 并记录实验条件。
