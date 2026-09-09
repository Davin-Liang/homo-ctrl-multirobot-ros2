# 4D Artstein 偏航 PD 控制设计

## 目标

将 4D Artstein 控制器的独立偏航通道从“航向比例 + Leader 角速度前馈”替换为线性 PD 状态反馈。

## 控制律

控制器在每个周期读取 Leader、Follower 的 map 系 yaw 与角速度，并计算：

\[
e_\theta = \operatorname{wrap}(\theta_l - \theta_f),\qquad
e_\omega = \omega_l - \omega_f
\]

\[
\omega_{cmd}=\operatorname{sat}_{[-\omega_{max},\omega_{max}]}
\left(K_{p,yaw}e_\theta+K_{d,yaw}e_\omega\right)
\]

`wrap` 继续使用 `atan2(sin(), cos())`，将误差归一到 \([-\pi,\pi]\)。D 项直接采用已测量的相对角速度，不对角度误差做数值差分。

## 参数与兼容性

- 保留 `Kp_yaw`，默认 `4.0`。
- 删除旧的 `K_ff`，新增 `Kd_yaw`，默认 `1.0`。
- 同步更新控制器声明、头文件成员、launch 参数、默认 YAML、README，以及轨迹记录器自动采集的控制器参数列表。
- 这是明确的参数接口变更：旧 launch/YAML 中的 `K_ff` 不再传递给 4D Artstein 节点。

## 保持不变的行为

- 4D Artstein 平移的延迟补偿、HPC 与预测器不变；yaw 仍是独立通道，不加入 Artstein 预测。
- `max_angular_vel` 限幅、全向轮约束和 `max_angular_accel` 加速度约束继续作用于最终 `cmd_vel.angular.z`。
- EKF+TF 与 mocap 状态源均使用各自已有的 Leader/Follower `angular.z` 测量值。

## 验证

- 为 yaw PD 控制律添加单元测试：零误差、正负角速度误差、角度环绕和限幅。
- 检查 launch/YAML 不再暴露 `K_ff`，改为 `Kd_yaw`。
- 构建 `homo_multirobot_formation_control`，并运行相关测试。
