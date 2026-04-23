# third_party

这里放的是本仓库用到的**上游源码包**（vendor / 引入副本），用于：

- 在缺少 apt 二进制包时补齐依赖并参与工作空间编译；
- 或用于上游源码联调（改 bug、对齐接口、快速验证）。

> 注意：这些包的 README 往往是“上游仓库的通用安装说明”，不一定适配本工作空间的目录结构与启动方式。  
> 复现本项目请优先看仓库级 `README.md` 与各功能包的 README。

当前包含（以工作区 `ros2 pkg list` 中的包名为准）：

- `rf2o_laser_odometry`
- `robot_localization`
- `omnidirectional_controllers`

