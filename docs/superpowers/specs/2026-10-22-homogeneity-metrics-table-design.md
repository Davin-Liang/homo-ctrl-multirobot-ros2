# 不同齐次度实验指标表设计

## 目标

新增一个 MATLAB 脚本，汇总不同齐次度 `nu` 的三次实物试验，并输出可直接插入论文的组级指标表 PNG 和矢量 PDF。

## 数据与试验分组

- 脚本置于 `homo_multirobot_formation_control/robot_traj/real/`，从脚本所在目录读取各试验子目录的 `raw.csv`。
- 顶部用 `nu` 与三条试验目录名组成显式分组配置，覆盖现有 `nu=-1.0` 至 `0.0` 的无前馈实验。
- 每组必须恰有三份可读 CSV；缺少文件或关键列时脚本报出对应的 `nu` 与路径，避免生成不完整统计表。
- 理想 Leader–Follower 距离使用脚本配置的 `ideal_radius_m=1.0` m；相对航向期望为零（同向编队）。

## 六项指标

令距离误差为 `e_d=abs(distance_m-ideal_radius_m)`，相对航向误差为
`e_theta=wrapToPi(follower_yaw_rad-leader_yaw_rad)`，速度差模长为
`e_v=norm([follower_vx_map_ms-leader_vx_map_ms, follower_vy_map_ms-leader_vy_map_ms])`。

每个试验计算：

1. 全程距离 MAE：`mean(e_d)`。
2. 末 10 s 距离 MAE：最后 10 s 样本上的 `mean(e_d)`；不足 10 s 时使用该试验所有样本。
3. 全程相对航向 RMS：`sqrt(mean(e_theta.^2))`。
4. 末 10 s 相对航向 RMS。
5. 全程速度差 MAE：`mean(e_v)`。
6. 末 10 s 速度差 MAE。

每个 `nu` 单元格显示三次试验的算术均值与样本标准差：`mean ± std`。

## 表格与导出

- 行为 `nu`，列为上述六项指标，单位明确标注为 `m`、`rad`、`m/s`。
- 使用 MATLAB figure/uitable 风格渲染，统一 Times New Roman 字体、适合单页横向论文版面。
- 导出到脚本目录：`homogeneity_metrics_table.png`（300 dpi）和 `homogeneity_metrics_table.pdf`（矢量）。
- 在命令窗口显示同一张组级汇总 `table`，便于核查数值与复制到论文源文件。

## 范围

仅新增独立 MATLAB 脚本及其自动化静态检查；不修改 `raw.csv` 格式、记录器、现有 `plot_paper_figures.m`、实验数据或已有统计文档。
