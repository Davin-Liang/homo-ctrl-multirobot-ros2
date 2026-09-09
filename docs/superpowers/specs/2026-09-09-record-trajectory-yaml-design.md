# 轨迹记录器 YAML 参数设计

## 目标

让 `record_trajectory.py` 能从包内或用户指定的 YAML 文件读取运行参数，以便复现实验配置，同时保留命令行 ROS 参数覆盖能力。

## YAML 格式与位置

- 新增 `homo_multirobot_formation_control/config/record_trajectory.yaml`。
- 文件采用现有控制器配置一致的 ROS 2 参数格式：`/**` → `ros__parameters`。
- YAML 包含记录器已有的全部可设置参数：命名空间、时长、输出目录、半径、模式、标签、控制器节点与实验元数据。

## 加载与优先级

- 新增字符串 ROS 参数 `config_file`。
- `config_file` 为空时，加载包内默认 `config/record_trajectory.yaml`。
- 非空时，加载该绝对或相对路径所指 YAML。
- YAML 中的值作为默认值；显式 `--ros-args -p <name>:=<value>` 具有更高优先级。

## 错误处理与验证

- 缺失文件、非映射根节点、缺失 `/**.ros__parameters` 或非映射参数节时，记录器应输出明确错误并退出。
- 新增单元测试验证默认配置、用户文件、命令行覆盖以及无效 YAML 的报错。
- README 增加默认启动与自定义配置文件的示例。

## 范围外

- 不改变轨迹订阅、数据保存、绘图、控制器参数自动查询或输出格式。
