# 原始 4D 偏航 PD 控制设计

## 目标

将原始 4D 编队控制器的 yaw 通道从“P + Leader 角速度前馈”改为与 4D Artstein 一致的线性 PD 状态反馈。

## 控制律

\[
\omega_{cmd}=\operatorname{sat}_{[-\omega_{max},\omega_{max}]}
\left(K_{p,yaw}\operatorname{wrap}(\theta_l-\theta_f)+K_{d,yaw}(\omega_l-\omega_f)\right)
\]

复用 `yaw_pd_controller.hpp` 的 `yaw_pd_command`。下游轮速和角加速度约束不变。

## 必要修正

原始节点当前在 `ekf_tf` 路径读取 Follower 状态时错误复用了 `leader_az` 输出变量。修改为独立的 `follower_az`，使 EKF+TF 与 mocap 两条路径都能提供真实的相对角速度误差。

## 参数与验证

- 删除原始 4D 的 `K_ff` 接口，改为 `Kd_yaw`，默认 `1.0`；保留 `Kp_yaw: 4.0`。
- 同步原始 4D 的节点头文件、launch、YAML、README 和 launch/YAML 测试。
- 使用已存在的 yaw PD 单元测试，并增加原始节点源码接口检查，构建目标包验证。
