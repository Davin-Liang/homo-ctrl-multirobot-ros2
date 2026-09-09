# 轨迹记录器等待控制器就绪 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 控制器参数服务尚未就绪时阻止轨迹记录器创建订阅与开始记录；服务就绪后自动继续。

**Architecture:** 将服务等待抽为可测试的 `wait_for_controller_service` 辅助函数。控制器节点名为空时直接返回空参数；非空时先创建客户端并无限循环进行 1 秒服务等待，随后读取参数。`TrajectoryRecorder.__init__` 只有在该调用返回后才创建订阅与计时器，保留首条里程计开始计时的现有语义。

**Tech Stack:** Python 3、rclpy、pytest、ament_cmake。

## Global Constraints

- `controller_node_name` 非空时，每秒等待 `<follower_ns>/<controller_node_name>/get_parameters`，直到服务就绪或 Ctrl-C 终止。
- `controller_node_name` 为空时跳过控制器参数查询并直接开始记录。
- 不设置超时，不改变 CSV、绘图、元数据结构、延迟节点查询或现有输出格式。
- 在工作空间根目录 `/home/l1anggmgo/ros-projects/homo_multirobot_ws` 构建。

---

### Task 1: 等待控制器服务后开始记录

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/record_trajectory.py:167-197`
- Modify: `homo_multirobot_formation_control/test/test_record_trajectory_yaml.py:1-37`
- Modify: `homo_multirobot_formation_control/README.md:596-620`

**Interfaces:**
- Produces: `wait_for_controller_service(client, service_name, logger) -> bool`; returns `True` only once `client.wait_for_service(timeout_sec=1.0)` succeeds, and `False` if ROS shuts down.
- Consumes: a `controller_node_name`; an empty string means no parameter service is queried.

- [ ] **Step 1: Write failing tests**

Append this test code:

```python
class FakeClient:
    def __init__(self, results):
        self.results = iter(results)
        self.timeouts = []

    def wait_for_service(self, timeout_sec):
        self.timeouts.append(timeout_sec)
        return next(self.results)


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)


def test_wait_for_controller_service_retries_until_ready(monkeypatch):
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    client = FakeClient([False, False, True])
    logger = FakeLogger()

    assert module.wait_for_controller_service(client, "/robot2/controller/get_parameters", logger)
    assert client.timeouts == [1.0, 1.0, 1.0]
    assert len(logger.messages) == 2


def test_wait_for_controller_service_stops_when_ros_shuts_down(monkeypatch):
    states = iter([True, False])
    monkeypatch.setattr(module.rclpy, "ok", lambda: next(states))
    client = FakeClient([False])

    assert not module.wait_for_controller_service(client, "/robot2/controller/get_parameters", FakeLogger())
    assert client.timeouts == [1.0]
```

- [ ] **Step 2: Run tests to verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest homo_multirobot_formation_control/test/test_record_trajectory_yaml.py -q
```

Expected: FAIL because `wait_for_controller_service` is absent.

- [ ] **Step 3: Implement the minimal waiting behavior**

1. Define this helper next to the existing parameter helpers:

```python
def wait_for_controller_service(client, service_name, logger):
    while rclpy.ok():
        if client.wait_for_service(timeout_sec=1.0):
            return True
        logger.info(f'等待控制器参数服务就绪: {service_name}')
    return False
```

2. At the beginning of `_query_controller_params`, return `{}` and log an informational message if `self.ctrl_node_name` is empty.
3. For a nonempty node name, construct the service name exactly once, create the client, and call `wait_for_controller_service`. If it returns `False`, return `{}` without calling the service.
4. Retain the existing single `GetParameters` request and its 2-second response timeout after service readiness.
5. Update README to state that a nonempty `controller_node_name` delays subscription/recording until its parameter service is available; an empty value skips parameter collection and begins recording immediately.

- [ ] **Step 4: Run focused verification**

Run:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest homo_multirobot_formation_control/test/test_record_trajectory_yaml.py -q
python3 -m py_compile homo_multirobot_formation_control/scripts/record_trajectory.py
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

Expected: all commands succeed; tests verify retries, Ctrl-C shutdown handling, YAML behavior, and syntax validity.

- [ ] **Step 5: Commit**

```bash
git add homo_multirobot_formation_control/scripts/record_trajectory.py \
  homo_multirobot_formation_control/test/test_record_trajectory_yaml.py \
  homo_multirobot_formation_control/README.md
git commit -m "轨迹记录等待控制器就绪"
```
