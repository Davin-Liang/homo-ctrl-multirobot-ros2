# 6D Map HPC 无补偿基线设计

## 目的

从现有 `6D Map HPC Artstein` 控制器拆出一个独立的实物对照节点，用于测量在
不使用 Artstein 输入延迟补偿、Follower 电机前向预测、Leader 状态前向外推和
Leader 命令前馈时的 6D map-frame HPC 表现。

## 架构

新增 `FormationController6DMapHpc`。它沿用 Artstein 版本的状态输入、map/odom
变换、固定 map 偏移的 `MapHpcController6DArtstein` 核心、离散目标切换、速度限幅
及 `KinematicConstraint`。每个控制周期直接将当前测得的 `leader.x` 和
`follower.x` 传入 `select_target`、`initialize` 和 `command`。

该节点不包含预测器、延迟命令历史或预测状态日志；不声明 `tau`、`tau_yaw`、`Td`
以及全部 Leader `cmd_vel` 前馈参数，也不订阅 Leader 的 `cmd_vel`。

## 对外入口

新增独立的头文件、源文件和 `main_6d_map_hpc.cpp`，并在 CMake 中安装可执行文件
`formation_control_node_6d_map_hpc`。新增同名 YAML 与
`formation_single_follower_6d_map_hpc.launch.py`；其参数集合与 Artstein launch 的
共同基础参数一致，但移除所有补偿和前馈参数。可选 `sim_motor_delay.py` 保持为
launch 级仿真装置，不属于控制器补偿。

README 增加该基线的入口、含义和实物启动命令。

## 验证

新增或扩展测试，验证纯基线的 launch/YAML 不再暴露补偿或前馈参数，并编译
`homo_multirobot_formation_control`。实物运行时，日志应明确标识 `6D Map HPC`
且不再输出预测位移字段。
