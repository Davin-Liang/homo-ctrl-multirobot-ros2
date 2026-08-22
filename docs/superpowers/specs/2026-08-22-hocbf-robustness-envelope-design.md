# HOCBF 数值鲁棒性包络设计规格

## 目标

在现有静态圆障碍物 Oracle 仿真上，量化预测状态 HOCBF 安全滤波器对初始净空、径向接近速度、横向速度、执行器时间常数、输入时延失配和控制采样率的数值敏感性。输出可行性边界和经验采样裕度，为后续感知在环仿真选择保守半径提供依据。

## 范围

- 仅使用已有二维 map 系一阶执行器、常值输入延迟和静态圆障碍物模型。
- 控制周期固定为 50 ms；植物积分周期固定为 10 ms；1 ms 控制参考只用于对照。
- 每个场景记录高频最小距离、最小安全函数、最小 HOCBF 余量、QP 不可行步数和后备制动步数。
- 不接入 ROS、Gazebo、LaserScan、动态障碍物、定位噪声或 TF 延迟。

## 两种裕度必须分开

对基础安全半径 R，定义扫描观测裕度 m=d_min-R。对相同场景的 20 Hz 和 1 kHz 参考，定义经验采样差：

$$
\Delta_{\mathrm{sample,obs}}=
\max\left(0,\ d_{\min,1\,\mathrm{kHz}}-d_{\min,20\,\mathrm{Hz}}\right).
$$

输出的推荐数值膨胀仅为

$$
\epsilon_{\mathrm{numeric,obs}}=
\max_{\mathrm{tested\ scenarios}}\Delta_{\mathrm{sample,obs}}.
$$

它只覆盖已扫描的采样率、模型和初值，不覆盖感知、定位、时延辨识或障碍物几何误差。真实 ROS 安全半径仍需要额外的感知误差项：

$$
R_{\mathrm{ROS}}=R_{\mathrm{base}}+
\epsilon_{\mathrm{numeric,obs}}+
\epsilon_{\mathrm{perception}}+
\epsilon_{\mathrm{localization}}+
\epsilon_{\mathrm{geometry}}.
$$

在未证明所有误差上界前，这些公式是工程选参规则，不是鲁棒 HOCBF 定理。

## 扫描设计

扫描笛卡尔积：

- tau_actual：0.30、0.43、0.55 s；
- tau_model：tau_actual，以及相对误差 -20%、+20%；
- delay_model：0、0.22 s；
- delay_actual：delay_model，以及 +20、+50 ms；
- 初始净空：0.40、0.80、1.20 m；
- 初始径向接近速度：0.10、0.30、0.50 m/s；
- 初始横向速度：0、0.20 m/s；
- 名义径向接近命令：0.40、0.80 m/s。

所有延迟必须是 10 ms 积分周期的整数倍。若 HOCBF-QP 无解，仍保留该行并记录制动结果；它属于可行域边界，不计入准确模型的安全结论。

## 输出与验收

新增 robustness_envelope.csv。每行至少包含真实和预测的 tau、两种时延、初始状态、最小距离、最小 h、最小 psi2、不可行/制动次数，以及对应 1 kHz 参考的最小距离和观察采样差。

额外输出 robustness_summary.csv，按准确模型可行场景和全部场景分别汇总：

- 场景数；
- min_h 与最小距离；
- 不可行场景数；
- 最大 Delta_sample_obs。

验收条件：

1. 小扫描单元测试的行数、列名、模型失配字段和观察采样差计算正确。
2. 所有准确模型且 QP 可行的扫描行中，必须报告是否存在 min_h 小于 0；不允许静默筛掉失败行。
3. 所有结论文字明确把 epsilon_numeric_obs 限定为经验数值项。

## 非目标

本阶段不自动选择最终 ROS 安全半径，不宣称延迟失配下鲁棒安全，也不允许由“无碰撞的有限扫描”推出所有初值下的安全性。
