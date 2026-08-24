# 多半径圆柱 HOCBF 数值仿真设计

## 目标

在既有 6D Artstein + predictor-HOCBF Python 数值仿真中，支持多个具有不同真实半径的静态圆柱障碍物；每个圆柱形成一条独立的 HOCBF 硬约束，并输出可复现的安全指标。

## 输入与半径定义

命令行新增 `--cylinder-radii r1,r2,...`，其元素是 Gazebo/数值场景中圆柱截面的真实半径。圆柱数必须与障碍物中心数一致。

对第 \(j\) 个圆柱，控制器使用：

\[
R_{\mathrm{physical},j}=r_{\mathrm{follower}}+r_{\mathrm{cylinder},j}+d_{\mathrm{clearance}},
\qquad
R_{\mathrm{filter},j}=R_{\mathrm{physical},j}+\epsilon_{\mathrm{filter}}.
\]

其中 follower 半径、净空和滤波裕量为全局参数。数值 HOCBF 使用 \(R_{\mathrm{filter},j}\)，图中同时画出每个 \(R_{\mathrm{physical},j}\)（红色实线）与 \(R_{\mathrm{filter},j}\)（橙色虚线）。

## 仿真数据流

自动双圆柱场景先按照现有 follower 名义轨迹选取两个不重合的位置；`--auto-two-offset` 只控制中心相对名义轨迹的法向偏移。随后将中心与逐圆柱滤波半径传给 HOCBF 过滤器，QP 对每个圆柱各加入一条半空间约束。

## 输出和判据

`coupled_timeseries.csv` 保留现有总体字段，新增每个圆柱的距离、物理安全余量与滤波余量列。控制台打印每个圆柱的最小中心距离、最小物理余量、最小滤波余量，以及是否进入物理安全圆。

通过标准：任何圆柱的物理余量不得为负；滤波余量允许在 20 Hz 离散实现与耦合预测误差下短暂为负，但必须明确报告，不能被当作物理碰撞。

## 测试

测试覆盖：不同圆柱半径被转换成相应的 \(R_{\mathrm{filter},j}\)；过滤器把不同半径逐一传给 HOCBF 半空间；短时两圆柱仿真生成逐圆柱指标。

## 范围限制

本次仅扩展 Python 数值仿真，不改 ROS 控制器，也不改变“障碍物几何已由数值场景给定”的 Oracle 验证假设。
