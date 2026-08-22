# 6D Artstein 局部绕障参考与回归设计

## 目标

在理想静态圆障碍物数值仿真中，为 6D Artstein Disc 提供临时局部绕障参考，使 follower 绕开障碍物后平滑回到原离散编队点；HOCBF 保持最终硬安全过滤。

## 三层职责

1. 局部参考生成器选择绕行侧并生成临时 map 系位置/速度参考。
2. 6D Artstein Disc 追踪该临时参考，原 HPC、预测和执行器历史不变。
3. HOCBF-QP 对最终平移命令施加环境障碍硬约束。

## 状态机

- NORMAL：使用原始 Disc 编队点。
- BYPASS：障碍物距 follower 小于 activation_radius 时冻结当前 Disc 点，选择绕行侧，目标为障碍物外侧切点。
- RETURN：follower 位于障碍物远侧且距离大于 release_radius 时，临时参考按平滑插值回到冻结的 Disc 点。
- NORMAL：回归误差小于 return_tolerance 后恢复原 Disc 切换。

绕行侧根据原始编队目标方向与障碍物法向的二维叉积选择；若接近零，采用上一周期侧向以防抖动。

## 边界

局部参考层不证明安全；安全仅由 HOCBF 给出。该设计只用于 Python 数值闭环，不接 scan、ROS 或动态障碍物。
