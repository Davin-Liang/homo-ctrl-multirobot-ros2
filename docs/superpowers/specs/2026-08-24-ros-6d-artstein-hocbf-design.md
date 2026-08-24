# ROS 6D Artstein Predictor-HOCBF 静态圆柱避障设计

## 目标

新增一个 ROS 2 节点，将既有 6D Artstein Disc 编队控制器输出与多圆柱 HOCBF 硬 QP 相结合。节点不接收障碍物真值位置和半径；静态圆柱几何仅由 follower 的 `LaserScan` 估计。

## 范围

第一阶段仅验证 Gazebo 中静态圆柱。动态障碍物、任意形状的严格外接几何和离散时间严格安全证明不属于本次实现。连续 predictor-HOCBF 理论在 20 Hz 零阶保持控制器上作为工程实现使用，安全裕量显式吸收 scan/TF/离散误差。

## 节点与数据流

新增 `formation_control_node_6d_artstein_disc_hocbf` 与对应 launch；不修改既有 `formation_control_node_6d_artstein_disc` 或旧 OA 节点。

```text
leader/follower EKF + TF -> 6D Artstein predictor -> 6D Disc HPC nominal body cmd
/scan -> valid-range filter -> contiguous clustering -> static-cylinder fitting
      -> scan-stamp TF to map -> conservative obstacle disks
predicted map translation state + disks -> multi-HOCBF hard QP
safe map cmd -> body cmd_vel -> wheel/acceleration constraint -> publish
actual published map command -> Artstein command history
```

## 感知表示

每个有效 scan 簇用二维最小二乘圆拟合得到圆心与半径；仅接受足够点数、半径处在配置范围内且拟合残差不超过阈值的簇。圆心和半径以 scan 时间戳的 `map <- scan_frame` TF 变换到 map 系。实际用于 HOCBF 的半径为：

\[
R_{\mathrm{filter},j}=r_{\mathrm{fit},j}+r_{\mathrm{follower}}+d_{\mathrm{clearance}}+\epsilon_{\mathrm{perception}}.
\]

`epsilon_perception` 是对拟合、量测、TF、scan 延迟以及 20 Hz 零阶保持误差的保守总裕量。scan 过期时，不接受旧障碍物几何作为“当前安全证明”；节点进入保守停止模式（平移命令为零，保留 nominal yaw）。

## HOCBF 与 QP

对预测 map 状态 \([p_x,p_y,v_x,v_y]\) 和每个圆盘应用数值仿真相同的二阶 HOCBF 约束。平移 QP 目标为最小化与 Artstein-HPC 名义 map 命令的偏差，同时满足所有圆柱半空间、速度限幅和相邻控制周期的加速度盒约束。

二维 QP 采用候选集枚举：名义点、各单边界投影、边界对交点和盒约束边界；没有可行点即判定 infeasible，而不是用软惩罚掩盖。infeasible 时节点发零平移命令并节流告警。

## 旧 OA 的取舍

旧 `ObstacleAvoider` 仅可借鉴 scan 过滤、连续点聚类、最大障碍物数和超时逻辑。其最近点+半径表示、body 系跨帧速度估计、软惩罚投影梯度和旧 6D 非 Artstein 融合链路不复用。

## 验证

纯 C++ 单元测试覆盖圆拟合、map HOCBF 半空间、二维硬 QP 与命令坐标变换；构建目标后，以 Gazebo 静态圆柱和 `/scan` 联调。启动参数不提供圆柱坐标或真实半径。
