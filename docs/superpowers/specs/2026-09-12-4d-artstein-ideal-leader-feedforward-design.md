# 4D Artstein 理想 Leader 前馈数值仿真设计

## 目标

在现有 Python 数值仿真中比较 4D Artstein-HPC 在有无理想 Leader 输入前馈时的编队误差。仿真直接使用 Leader 模型的真实控制输入；不修改 ROS 2 节点、话题或 launch 文件。

## 范围

修改 `homo_multirobot_formation_control/scripts/sim_4d_hpc_artstein_compare.py`。

在既有 MATLAB 激励和圆轨迹两种场景中，为 Artstein + prediction 基线增加一组理想前馈案例。Follower 命令的构造为：先沿用现有 HPC 加速度积分得到的 map 系速度命令，再叠加由 Leader 已知加速度在一个控制周期内产生的速度增量。叠加结果仍走同一速度限幅、输入死区和一阶电机模型。

## 对照与输出

保留既有案例，仅新增对应的 `artstein_prediction_ideal_leader_feedforward` 案例。汇总 CSV 增加该案例的最大、尾段平均、尾段标准差和最终编队误差；比较图展示该案例的轨迹、误差和命令。结果写入用户指定的独立 `--out-dir`，不覆盖已有结果。

## 验证

先添加 Python 单元测试，验证理想前馈速度增量等于 `h * u_leader / mass`，并验证关闭前馈时命令不变。测试先失败，再实现最小改动使其通过。随后用无噪声、固定随机种子的圆轨迹运行脚本，并检查 CSV 和 PNG 均生成、前馈案例存在且全部指标为有限数。
