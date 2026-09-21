# 4D Artstein 齐次度 Override 设计

## 目标

为 4D Artstein 的底层 `LpcController` 增加受理论可行区间约束的固定齐次度实验参数，使 4D 与 6D Map HPC 使用一致的实验接口。

## 参数与默认行为

新增 `use_hpc_nu_override`（默认 `false`）及 `hpc_nu_override`（默认 `-0.30`，仅在启用时使用）。关闭时继续采用 `lpc2hpc()` 自动给出的 `nu_min`，保持当前行为。

## 初始化、切换与校验

首次 `controller_initial()` 和离散编队点切换触发的 HPC 重建，都会重新计算 `[nu_min, nu_max]`。启用 override 时，指定值必须为有限数并位于该闭区间内（带微小浮点容差）；否则抛出运行时错误，不静默回退。通过后以指定值构建 `Gd` 和 4D 齐次控制律。

## 可观测性和验证

初始化和切换日志输出 `nu_mode`、`nu_used` 与 `[nu_min, nu_max]`。单元测试验证默认自动值、有效 override、越界和 NaN 拒绝；YAML、launch、README 暴露参数和使用限制。

## 非目标

不覆盖用户定义的理论边界；不增加运行时动态更新；不修改非 Artstein 的 4D 节点或其他控制器。
