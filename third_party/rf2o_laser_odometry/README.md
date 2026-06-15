# rf2o_laser_odometry

Estimation of 2D odometry based on planar laser scans. rf2o is a fast and precise method to estimate the planar motion of a lidar from consecutive range scans. Useful for mobile robots with inaccurate wheel odometry.

For every scanned point we formulate the range flow constraint equation in terms of the sensor velocity, and minimize a robust function of the resulting geometric constraints to obtain the motion estimate. Conversely to traditional approaches, this method does not search for correspondences but performs dense scan alignment based on the scan gradients, in the fashion of dense 3D visual odometry.

The very low computational cost (0.9 milliseconds on a single CPU core) together whit its high precision, makes RF2O a suitable method for those robotic applications that require planar odometry.

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `laser_scan_topic` | `/scan` | 激光话题 |
| `odom_topic` | `/odom_rf2o` | 里程计输出话题 |
| `publish_tf` | `true` | 是否发布 TF |
| `base_frame_id` | `base_link` | 机器人基坐标系 |
| `odom_frame_id` | `odom` | 里程计坐标系 |
| `init_pose_from_topic` | `/base_pose_ground_truth` | 初始位姿来源（空 = 原点） |
| `freq` | `10.0` | 处理频率 (Hz) |
| `ctf_levels` | `5` | 金字塔层级（5=精度高, 3=适配稀疏雷达） |
| `iter_irls` | `5` | 迭代求解次数（5=收敛好, 3=适配稀疏雷达） |

## 已知修改

本 fork 对上游做了以下修改：

1. **源头 inf/NaN 过滤**: `odometryCalculation()` 入口将所有 inf/NaN 替换为 0
2. **金字塔 inf/NaN 过滤**: 高层降采样补上 `std::isfinite` 检查
3. **TF 等待**: 首次 `lookupTransform` 前加 `canTransform` 等待就绪
4. **可配置参数**: `ctf_levels` 和 `iter_irls` 改为 ROS 参数

## 参考

For a full description of the algorithm, please refer to: **Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA 2016** Available at: http://mapir.uma.es/papersrepo/2016/2016_Jaimez_ICRA_RF2O.pdf