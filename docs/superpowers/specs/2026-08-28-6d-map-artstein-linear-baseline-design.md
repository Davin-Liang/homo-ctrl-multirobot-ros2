# 6D map-frame Artstein 线性基线设计

## 目标

在每个已有 map-frame 6D 数值对照中增加 `artstein_linear` 组，用于独立评估齐次升级相对于同一 Artstein 预测层的贡献。

## 组定义

`artstein_linear` 与 `artstein` 共享 Leader 状态预测、Follower Artstein 预测、输入延迟、一阶执行器、初值、命令限幅和质量/惯量参数。唯一差异是控制器：

\[
u_{linear}=Ke,
\]

其中 (K) 与正则化 HPC 组使用的线性基础反馈完全相同；不计算齐次范数、(c) 截断或矩阵指数翘曲。

## 输出与边界

轨迹图、误差曲线、CSV 与所有连续 yaw 场景均加入该组。比较结论可归因于“齐次升级”，但仍不构成正则化工程控制律的严格有限时间证明。
