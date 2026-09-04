# VRPN 测试服务端设计

## 目标

提供一个由当前电脑运行的独立 VRPN Tracker 服务端，使局域网中的笔记本能够用已部署的 `vrpn_listener` 验证 VRPN 连通性、刚体发现及 pose、twist、accel 三类话题。

## 架构

新增项目自有 ROS 2 ament 包 `homo_multirobot_mocap_tools`。其可执行程序 `vrpn_test_server` 只使用 VRPN C++ API，不创建 ROS 节点、不发布 ROS 话题。服务端监听指定 TCP 端口，并通过一个 `vrpn_Tracker_Server` 周期性发布测试刚体状态。

```text
当前电脑: vrpn_test_server (VRPN TCP 0.0.0.0:3883, tracker robot1)
  -> 局域网
笔记本: vrpn_listener
  -> /vrpn/robot1/pose, /twist, /accel
```

## 行为与接口

默认参数：

- `--port 3883`
- `--tracker-name robot1`
- `--radius 1.0`（m）
- `--speed 0.5`（m/s）
- `--rate 100.0`（Hz）

服务端发布以原点为圆心的逆时针圆轨迹。令 `omega = speed / radius`、`t` 为启动后的单调时间：

```text
x = radius * cos(omega * t)
y = radius * sin(omega * t)
yaw = omega * t + pi/2
vx = -speed * sin(omega * t)
vy =  speed * cos(omega * t)
ax = -speed * omega * cos(omega * t)
ay = -speed * omega * sin(omega * t)
wz = omega
```

pose 使用 `(x, y, 0)` 与绕 z 轴的四元数。VRPN 速度消息使用全局测试坐标系的 `vx, vy, 0`；旋转速度消息使用表示 `wz * dt` 的 z 轴增量四元数和 `dt`。加速度消息同样使用全局测试坐标系的 `ax, ay, 0`。

启动时打印绑定端口、刚体名和完整客户端地址格式；无效参数（端口范围、非正 radius/rate、负 speed）打印用法并以非零状态退出。收到 `Ctrl+C` 后结束主循环并正常退出。

## 范围与约束

- 新源码只放入 `homo_multirobot_mocap_tools/`，不修改 `third_party/vrpn_client_ros2` 的行为。
- 包声明对 `ament_cmake` 与 VRPN 的构建依赖，使用系统的 `ros-humble-vrpn`。
- 构建必须从 `/home/l1anggmgo/ros-projects/homo_multirobot_ws` 执行。
- 不接入现有 EKF、TF 或编队控制器。
- 服务端默认只提供一个 `robot1` 刚体；多刚体仿真不属于本次范围。

## 验证

当前电脑：

```bash
ros2 run homo_multirobot_mocap_tools vrpn_test_server
```

笔记本将 `vrpn_listener` 的 `server` 设置为当前电脑局域网 IP、`port` 设为 `3883` 后启动 bridge。笔记本应发现 `robot1` 并看到 `/vrpn/robot1/pose`、`/vrpn/robot1/twist`、`/vrpn/robot1/accel`。对 pose 执行 `ros2 topic hz` 的频率应接近服务端 `--rate`。
