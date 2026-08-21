# 延迟原生级联 ILF 设计规格

**目标：** 建立一条不使用 Artstein 预测器、但直接面向一阶执行器和输入时滞的局部 6D 编队控制候选路线；先完成理论边界与离线 DDE 验证，再决定是否进入 ROS。

## 1. 结论定位

本设计不是“直接 6D MIMO-ILKF 定理”的替代证明。它将 6D 误差状态重新组织为三个
SISO 时滞齐次通道和三个稳定滤波器，因此可把 SISO 延迟 ILF/负次数齐次文献作为主
理论基础。最终可追求的论文结论为：在固定编队目标、冻结或慢变 Leader、局部无饱和
和有界时延条件下，编队误差局部实用稳定/最终有界；不得提前宣称全局、任意时延或
严格有限时间收敛。

## 2. 坐标、模型与命令

沿用已有误差状态

```math
e_p=[e_x,e_y,e_\theta]^\mathsf T,\qquad
e_v=[e_{v_x},e_{v_y},e_\omega]^\mathsf T,
\qquad \Tau=\operatorname{diag}(\tau_x,\tau_y,\tau_\omega).
```

在 `rho=0`、固定目标、`r=0` 时，定义可测组合坐标

```math
z=e_p+\Tau e_v. \tag{D1}
```

由真实一阶执行器模型

```math
\dot e_p=e_v,
\qquad
\dot e_v=-\Tau^{-1}e_v+\Tau^{-1}\delta u(t-d(t))
```

严格得到

```math
\dot z(t)=\delta u(t-d(t)),
\qquad
\dot e_p(t)=\Tau^{-1}[z(t)-e_p(t)]. \tag{D2}
```

因此 `z_i` 是第 `i` 个纯积分器输入时滞通道，而 `e_{p,i}` 是由 `z_i` 驱动的指数稳定
滤波器；随后 `e_v=Tau^{-1}(z-e_p)`。绝对发布命令仍为

```math
u_f=u_star+\delta u.
```

## 3. 控制器候选

第一版使用三个独立、连续的负次数齐次标量反馈：

```math
\delta u_i(t)=-k_i|z_i(t)|^\alpha\operatorname{sgn}(z_i(t)),
\qquad k_i>0,\quad 0<\alpha<1,\quad i\in\{x,y,\omega\}. \tag{D3}
```

无时滞时，每个通道满足有限时间稳定的标量系统
`dot z_i=-k_i|z_i|^alpha sign(z_i)`。有时滞时，控制输入只依赖 `z_i(t-d)`，直接匹配
SISO 控制通道时滞问题；其隐式 Lyapunov 形式可在实施阶段作为式 (D3) 的 SISO ILF
参数化重写。第一阶段不使用共享的 6D 隐式根 `V`，也不使用 Artstein 历史积分。

默认数值点：`alpha=0.5`、`tau_x=tau_y=tau_omega=0.43 s`，初始增益仅在无饱和离线
仿真中选择；速度/角速度约束不参与第一轮理论结论。

## 4. 非零 Leader 与扰动边界

当冻结 Leader twist `rho` 非零时，已有局部模型的位姿耦合可写为

```math
\dot e_p=F(\rho)e_p+e_v,
\qquad
\dot z=F(\rho)e_p+\delta u(t-d)+r_z. \tag{D4}
```

其中 `F(rho)` 是已有 6D 局部矩阵上方的 Leader 耦合块，`r_z` 汇集线性化余项、Leader
加速度、目标切换、饱和和观测误差。D4 不能再声称三个通道严格独立；第一阶段把
`F(rho)e_p+r_z` 作为有界扰动，验证局部实用稳定。若需写正式定理，必须给出 `rho`、
Leader 加速度、`r_z` 与工作域的显式上界。

## 5. 理论依据与不适用边界

- 对 `rho=0`，D2--D3 是三个标量负次数齐次时滞闭环。文献给出：无时滞渐近稳定的
  负次数齐次系统，对任意时滞可收敛至包含原点的紧集；小时滞可获得更强结论。
- Zimenko 等的延迟 ILF 论文直接针对 SISO 规范形，而 D2 每轴恰是其一阶特例；它不
  自动证明 D4 或完整机器人混杂系统。
- `e_p` 子系统是 ISS 稳定滤波器：若 `z` 最终有界，则 `e_p,e_v` 最终有界。该级联
  推论需在论文中写成引理，并给出相应范数界。
- 时变 `d(t)`、采样、饱和、目标切换和非零 `rho` 先进入数值扰动包络；未完成独立
  泛函/小增益证明前，不把它们包含进严格定理。

## 6. 离线验证顺序与验收条件

1. **C1：标量模型。** 对 `dot z=u(t-d)` + D3 做 DDE 扫描；输出最终幅值、峰值、控制
   峰值与延迟—最终误差曲线。验证 `d=0` 的有限时间数值收敛和非零时滞的紧集行为。
2. **C2：三通道执行器模型。** 用 D2 的 `e_p,e_v,z` 重建，核对 D1 的代数残差；扫描
   三个不同 `tau_i` 与恒定/分段时变延迟。
3. **C3：局部 Leader 扰动。** 加入 D4 的常值小 `rho` 和有界 `r_z`，报告最终误差对
   `||rho||`、`||r_z||` 的变化；不通过则缩小局部域。
4. **C4：20 Hz 离散化。** 仅在 C1--C3 通过后，比较 `dt=1 ms` 参考解和 `Ts=0.05 s`
   零阶保持实现；任何稳定性结论不能从 1 ms 仿真直接移植到 20 Hz。

进入 ROS 的最低条件：C1--C4 均无未解释发散，`d=0.22 s` 及其失配附近有可重复的
实用有界结果，且控制峰值在实际 `cmd_vel`/轮速约束内。否则该路线仅保留为理论/数值
探索，不替换 6D Artstein。

## 7. 预期文件与测试边界

- 扩展 `homo_multirobot_formation_control/scripts/ilf_6d_feasibility.py`，仅加入离线 SISO
  级联模型、DDE 和 CSV；不改 ROS 或 C++ 控制器。
- 扩展 `homo_multirobot_formation_control/test/test_6d_ilf_feasibility.py`：验证 D1--D2
  恒等式、无时滞收敛、延迟历史索引、三通道代数残差和 CSV 行尾。
- 结果置于 `analysis/results/6d_ilf_feasibility/`；文档只记录可复现数值结果与理论条件。

## 8. 设计决策

选择“延迟原生级联 ILF”而不继续直接 MIMO-ILKF，理由是 D2 消除了此前无法闭合的
执行器历史差分项，并把延迟结构降到有直接 SISO 文献基础的形式。代价是控制器由
共享 6D ILF 改为三通道级联，创新点应表述为“执行器感知坐标重构 + 时滞原生齐次级联
控制”，而非“直接 6D MIMO ILKF”。
