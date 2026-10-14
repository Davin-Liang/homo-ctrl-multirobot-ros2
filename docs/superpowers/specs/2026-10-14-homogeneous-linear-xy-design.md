# 齐次/线性 XY 轨迹图设计

## 目标

在 `homo_multirobot_formation_control/robot_traj/real/` 中新增一个 MATLAB 脚本，生成一张用于论文的齐次控制与线性控制 XY 轨迹对比图。

## 输入

脚本开头提供两个可编辑变量，分别指定齐次控制与线性控制实验目录。默认目录为：

- 齐次：`6d_artstein_no_forw_vs_forw_20260922_112130`
- 线性：`6d_artstein_lpc_vs_hpc(no_fora)_20260922_150051`

每个目录必须包含列名一致的 `raw.csv`，其中使用 `leader_x_m`、`leader_y_m`、`follower_x_m` 和 `follower_y_m`。

## 输出

同一张等比例 XY 图中绘制四条原始 map 系轨迹：

- 齐次控制的 Leader 和 Follower；
- 线性控制的 Leader 和 Follower。

颜色区分控制器，线型区分 Leader 与 Follower。脚本在自身目录输出
`homogeneous_vs_linear_xy.png` 和 `homogeneous_vs_linear_xy.pdf`。

## 范围

不对两次实验坐标进行对齐、平移或重采样；不计算统计指标；不绘制理想编队偏移、速度或误差曲线。

## 验收标准

1. 更改两个目录变量后可选择任意包含有效 `raw.csv` 的实验组。
2. 默认目录运行后生成 PNG 和 PDF。
3. 图例清晰区分齐次/线性与 Leader/Follower，坐标轴单位为 m，比例为 `axis equal`。
4. 缺少文件或所需列时脚本报出明确错误。
