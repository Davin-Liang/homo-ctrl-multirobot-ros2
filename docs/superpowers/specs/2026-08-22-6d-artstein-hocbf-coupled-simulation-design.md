# 6D Artstein Disc 与理想 HOCBF 耦合数值仿真设计

## 目标

在已校正 220 ms 死区的 6D Artstein Disc Python 仿真基础上，比较原始延迟补偿编队控制和加入理想静态圆障碍物 HOCBF 安全滤波后的闭环轨迹、最小距离、编队恢复和命令修正。

## 复用边界

直接复用 scripts/sim_6d_disc_artstein_compare.py 中的 Hpc6DDisc、Artstein 平移/偏航预测、Leader 外推、step_plant、命令历史、绘图与 CSV 结构。新脚本只增加 HOCBF 层和对应图表，不复制 6D HPC 或 Artstein 算法。

## 每周期数据流

1. 原脚本生成预测 leader 和 follower 状态，并计算 body 系名义命令 cmd_nom。
2. 用当前测量 follower yaw 把 cmd_nom 的平移部分变为 map 系名义命令。
3. HOCBF 状态使用预测 follower 的 map 系位置和 map 系平移速度；障碍物是精确已知的 map 系静态圆。
4. HOCBF-QP 输出 map 系安全平移命令，再按当前测量 yaw 转回 body 系。
5. yaw 命令保留 cmd_nom 的原值。
6. 最终 body 系命令进入死区、一阶执行器植物，并按当前测量 yaw 转为 map 系写回 Artstein 历史。

因此，历史中的命令始终是实际发布的安全命令，而不是滤波前名义命令。

## 理想障碍物与参数

- 障碍物中心和半径通过脚本参数给出，第一版使用一个静态圆。
- HOCBF 使用预测状态和精确 tau、Td。
- 不使用 scan、定位、TF、噪声、障碍物外接或模型失配。
- 内部安全半径等于机器人/障碍物基础合并半径；不使用经验 5 mm 膨胀。

## 对照与图表

必须比较 compensated 6D Artstein Disc 与 compensated 6D Artstein Disc + HOCBF：

- map 系 Leader/Follower 轨迹、障碍物和安全圆；
- 最小障碍距离与 h(t)；
- 编队位置误差和 yaw 误差；
- map 系名义/安全平移命令；
- 命令修正范数；
- HOCBF 激活、QP 不可行和制动次数。

验收条件：无 HOCBF 对照进入障碍安全圆；HOCBF 对照保持在安全圆外且能在绕障后恢复编队。若 QP 不可行，必须显式记录，不能用松弛变量掩盖。

## 结论边界

该结果验证 6D Artstein 名义控制器和理想 HOCBF 层的耦合数值行为，不构成含 scan、定位、TF 或未知参数误差的安全证明。
