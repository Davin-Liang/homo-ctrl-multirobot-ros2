# MPC 圆周 Leader 数值场景设计

## 目标

为独立的 `homo_multirobot_formation_control/mpc_sim/` 数值实验新增一个圆周 Leader 场景，并在完全相同的真实延迟对象上对比普通 MPC 与 6 步延迟感知 MPC。

## 场景定义

- 圆半径：`R = 2.0 m`。
- 角速度：`Omega = 0.25 rad/s`。
- 理想圆周周期：约 `25.13 s`。
- Leader 的预定义 map 系速度命令为：

  ```text
  u_l(t) = [-R Omega sin(Omega t), R Omega cos(Omega t), Omega]
  ```

  该选择以 Leader 初始位置 `(R, 0)` 为圆周起点，初始切向速度沿正 map-y 方向；前两通道积分后满足 `x=R cos(Omega t)`、`y=R sin(Omega t)`。

- Follower 保持现有固定 map 系偏移 `[-1, 0, 0]`，初始误差与现有曲线场景一致。
- Leader 和 Follower 均继续使用同一个一阶速度滞后和固定输入延迟数值对象；不会用逆模型或提前输入使 Leader 的实际轨迹“完美圆形”。因此起始阶段的轨迹偏差是刻意保留的执行器动态影响。

## 实现与验证

新增 `make_circle_scenario()`，并把以下两行加入完整实验：

- `circle_ordinary_mpc`：控制器模型 0 步延迟、真实对象 6 步。
- `circle_delay_aware`：控制器和真实对象均为 6 步延迟。

测试须验证初始输入为 `[0, R*Omega, Omega]`，四分之一周期处 map 平移命令方向正确，并验证完整圆周案例产生所有 CSV/PNG/指标输出。原有场景和指标定义不变。
