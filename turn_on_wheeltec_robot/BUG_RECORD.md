# BUG_RECORD — turn_on_wheeltec_robot

## 1. odom_frame_id 参数碰撞导致 rf2o 坐标系错误

**现象**: rf2o/odom 的 `header.frame_id` 为 `robot1_odom_combined` 而非 `robot1_odom`，EKF 无法变换数据。

**原因**: `base_serial.launch.py` 和 `rf2o_ekf_single_robot.launch.py` 都声明了 `odom_frame_id` 参数。在同一个 launch 上下文中，先声明的（base_serial 的 `odom_combined`）覆盖了 rf2o 的默认值（`odom`）。

**解决**: bringup 中显式传递 `odom_frame_id: <prefix>odom` 给 rf2o_ekf launch。

## 2. 轮式里程计 frame_id 与 EKF world_frame 不一致

**现象**: wheel 模式下 EKF 无法正确融合轮式里程计数据。

**原因**: 轮式里程计 `header.frame_id` 此前为 `odom_combined`，而 EKF 的 `world_frame` 为 `odom`。

**解决**: 将 `base_serial.launch.py` 中 `odom_frame_id` 默认值从 `odom_combined` 改为 `odom`。

## 3. lslidar_driver 参数未生效

**现象**: 雷达驱动报 `Lidar model error` 和 `poll() timeout`。

**原因**: `lslidar_x10.yaml` 参数包在 `x10/lslidar_driver_node/` 命名空间下，但节点 namespace 被改为 `robot1`，YAML 参数不匹配。同时缺失 `lidar_type` 关键参数。

**解决**: 直接在 launch 中传所有必需参数（`lidar_type: X10`、`lidar_model: N10Plus` 等），不再依赖带命名空间的 YAML。

## 4. N10Plus 雷达在 10Hz 下点数不足

**现象**: 仅 540 有效点/圈，91% 为 inf，rf2o 无法正常工作。

**原因**: N10Plus 固定 5400 点/秒，`N10Plus_hz=10` 时 540 点/圈。

**解决**: `N10Plus_hz` 设为 6，提升至 900 点/圈。

## 5. ImuProcessor 话题绝对路径问题

**现象**: namespace 下 ImuProcessor 订阅仍为 `/imu/data_raw`（全局），不自动加前缀。

**原因**: `ImuProcessor.cpp` 中话题名硬编码为绝对路径（`/imu/data_raw`、`/odom`、`/imu/data_filtered`）。

**解决**: 改为相对路径（去掉前导 `/`）。

## 6. wheeltec_robot_node 不发布 TF

**确认**: 该节点仅发布 `nav_msgs/Odometry` 消息，不发布 `odom → base_footprint` 的 TF。头文件中 `tf2_ros/transform_broadcaster.h` 的 include 为死代码。
