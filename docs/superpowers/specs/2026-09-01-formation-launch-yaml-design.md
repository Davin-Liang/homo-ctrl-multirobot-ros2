# 编队控制 Launch YAML 参数加载设计

## 目标

让以下五个 launch 的默认参数由 `homo_multirobot_formation_control/config/`
内各自固定的 YAML 文件提供，同时保留 `ros2 launch` 命令行参数覆盖能力：

- `formation_single_follower_4d_artstein.launch.py`
- `formation_single_follower_6d_map_hpc_artstein.launch.py`
- `formation_single_follower_6d_disc.launch.py`
- `formation_single_follower_6d_artstein_disc_hocbf.launch.py`
- `formation_single_follower.launch.py`

## 架构与数据流

每个 launch 新增一份同模型对应的 YAML 参数文件。YAML 使用 ROS 2 参数文件格式：

```yaml
/**:
  ros__parameters:
    parameter_name: value
```

控制器节点的 `parameters` 按以下顺序传入：

1. 该 launch 对应的固定 YAML 文件。
2. 现有 launch argument 构成的参数字典。

ROS 2 按后项覆盖前项处理，因此 YAML 是启动默认值，而命令行中显式提供的
`name:=value` 会经由 launch argument 覆盖 YAML 中同名参数。辅助的
`sim_motor_delay.py` 节点继续使用同一批 launch argument，保证延迟模型参数与
控制器计算一致。

## 文件映射

| Launch | 固定 YAML |
| --- | --- |
| `formation_single_follower_4d_artstein.launch.py` | `config/formation_single_follower_4d_artstein.yaml` |
| `formation_single_follower_6d_map_hpc_artstein.launch.py` | `config/formation_single_follower_6d_map_hpc_artstein.yaml` |
| `formation_single_follower_6d_disc.launch.py` | `config/formation_single_follower_6d_disc.yaml` |
| `formation_single_follower_6d_artstein_disc_hocbf.launch.py` | `config/formation_single_follower_6d_artstein_disc_hocbf.yaml` |
| `formation_single_follower.launch.py` | `config/formation_single_follower.yaml` |

## 兼容性与边界

- 不新增 `params_file` 或其他运行时选文件参数；每个 launch 的 YAML 映射固定。
- 保留既有 launch argument 名称、描述和命令行接口。
- YAML 初始值与修改前 launch 默认值完全一致，避免改变未显式设置参数时的行为。
- 继续通过现有 CMake `install(DIRECTORY config launch ...)` 安装，无需更改安装规则。

## 验证

新增 Python 静态测试，覆盖：

1. 五份 YAML 文件存在，且为可解析的 ROS 2 参数文件。
2. 每份 YAML 的参数键与相应 launch 的 `DeclareLaunchArgument` 名称完全一致。
3. 每个控制器节点将 YAML 放在 `parameters` 的首项，并将 launch argument 参数字典置于其后，保证覆盖顺序。
4. 使用 `ros2 launch ... --show-args`（在 ROS 环境可用时）确认原有参数仍可从命令行传入。

