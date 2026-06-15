# turn_on_wheeltec_robot

Wheeltec 实车 ROS 2 硬件驱动包。通过串口与 STM32 底盘控制器通信，发布 `/odom`（轮式里程计）、`/imu/data_raw`（IMU），订阅 `/cmd_vel` 下发控制。

## 硬件支持

- **底盘**: mini_omni（三轮全向）
- **雷达**: Leishen N10Plus (UART)
- **IMU**: 内置 STM32
- **相机**: 不支持
- **自动回充**: 不使用

## 编译

```bash
colcon build --packages-select serial wheeltec_robot_msg lslidar_msgs lslidar_driver turn_on_wheeltec_robot --symlink-install
```

## 启动

```bash
# 轮式里程计模式（默认，静止不漂移，推荐）
ros2 launch turn_on_wheeltec_robot bringup_mini_omni.launch.py namespace:=robot1 prefix:=robot1_

# rf2o 激光里程计模式（实验性，有漂移问题）
ros2 launch turn_on_wheeltec_robot bringup_mini_omni.launch.py namespace:=robot1 prefix:=robot1_ odom_source:=rf2o
```

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `namespace` | `""` | 机器人命名空间（如 `robot1`） |
| `prefix` | `""` | TF 帧前缀（如 `robot1_`） |
| `odom_source` | `"wheel"` | 里程计来源: `wheel`(轮式) / `rf2o`(激光) |

## 数据流

```
STM32 串口 ──→ /<ns>/imu/data_raw ──→ ImuProcessor ──→ /<ns>/imu/data_filtered ──→ EKF
STM32 串口 ──→ /<ns>/odom (轮式里程计) ──────────────────────────────────────────→ EKF
N10 雷达 ────→ /<ns>/scan
EKF ────────→ /<ns>/odometry/filtered + TF (odom → base_footprint)
```

## 依赖

- `serial`（SDK 内串口库）
- `wheeltec_robot_msg`（SDK 内）
- `lslidar_driver` + `lslidar_msgs`（SDK 内，N10 雷达驱动）
- `homo_multirobot_localization`（EKF 定位）
- `homo_multirobot_urdf`（机器人模型）
- `ros-humble-serial-driver`、`ros-humble-nav2-msgs`、`libpcap-dev`、`libserial-dev`
