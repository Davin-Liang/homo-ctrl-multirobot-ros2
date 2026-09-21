# 6D Map HPC 齐次度 Override 设计

## 目标

为 `MapHpcController6DArtstein` 提供受理论可行区间约束的固定齐次度实验参数，使研究者能够在不改变默认控制行为的前提下比较不同 `nu`。

## 参数与默认行为

新增 `use_hpc_nu_override`（默认 `false`）和 `hpc_nu_override`（默认 `-0.30`，仅在开关启用时使用）。默认路径保持不变：控制器采用 `lpc2hpc_nd()` 自动计算的 `nu_min`。

## 初始化与安全约束

每次首次初始化及离散编队点切换重建时，控制器先计算自动理论可行区间 `[nu_min, nu_max]`。override 启用时，`hpc_nu_override` 必须为有限值，且落在该闭区间内（允许小的浮点比较容差）；否则抛出运行时错误并拒绝本次控制器初始化/重建。通过校验后，以指定值构建 `Gd = I + nu * G0` 和齐次控制律。

## 可观测性与验证

每次初始化或切换都记录 `nu_mode`、实际使用的 `nu` 和自动可行区间。单元测试覆盖：关闭 override 时使用 `nu_min`、区间内 override 被采用、非有限/越界 override 被拒绝。YAML、launch 和 README 公开该高级实验参数及其限制。

## 非目标

不将 `nu_min/nu_max` 变为用户可覆盖的理论边界；不提供运行时动态参数更新；不改变 4D 或其他 6D 控制器。
