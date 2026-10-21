# MPC 圆周 Leader 3 m 固定偏移实验设计

## 目标

在已有 1 m map 系固定偏移圆周实验旁，新增 Leader--Follower 期望平移偏移为 `d_p=[-3.0, 0.0] m` 的数值对照。该实验评估更大编队尺度下的延迟感知 MPC，不替换已有 1 m 结果。

## 语义

偏移固定在 map 坐标系中：

```text
p_f_desired(t) = p_l(t) + [-3.0, 0.0]
```

因此 Follower 期望始终在 Leader 的 map-x 负方向 3 m 处。它不是“相对于 Leader 航向的车后方 3 m”，也不随 Leader 绕圆产生旋转偏移。

## 公平对照

- Leader 圆轨迹沿用 `R=2.0 m`、`Omega=0.25 rad/s` 的 map 系预定义命令。
- Leader/Follower 真实对象均保留一阶速度滞后和 6 步输入延迟。
- 初始误差、MPC 权重、20 步预测域、输入幅值约束、输入变化率约束均与 1 m 圆周实验一致。
- 对照组为内部假定 0 步时延的普通 MPC；主实验组为内部 6 步时延的 MPC。

## 输出

生成 `circle_far3_ordinary_mpc/` 和 `circle_far3_delay_aware/`，各自包含轨迹、误差、输入/求解耗时 PNG、完整 CSV 与指标 CSV；汇总报告与 1 m 圆周实验放在独立 `circle_far3_20261021/` 结果目录中。

测试须验证该场景的 map 偏移确为 `[-3,0,0]`，并验证两组结果文件生成。不得修改现有 `circle_ordinary_mpc` 或 `circle_delay_aware` 结果。
