# 轨迹记录器等待参数响应 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 服务可发现但控制器仍未处理参数请求时，轨迹记录器继续等待有效参数响应后才开始记录。

**Architecture:** 保持已有服务发现循环，新增可测试的 `wait_for_controller_parameters`：每轮发起 `GetParameters` 请求并等待最多 2 秒；无响应则记录日志、重新等待服务并再次请求。返回有效响应后由既有解析逻辑提取数值参数；ROS 关闭时返回 `None`。

**Tech Stack:** Python 3、rclpy、pytest、ament_cmake。

## Global Constraints

- 服务可用后必须成功收到一次 `GetParameters` 响应，才可创建订阅并开始记录。
- 每次参数响应最多等待 2 秒；失败后无限重试，无总超时。
- `controller_node_name` 为空时仍跳过参数查询；不改变 CSV、绘图、元数据结构、延迟节点查询或输出格式。
- 在工作空间根目录 `/home/l1anggmgo/ros-projects/homo_multirobot_ws` 构建。

---

### Task 1: 重试控制器参数请求

**Files:**
- Modify: `homo_multirobot_formation_control/scripts/record_trajectory.py:90-220`
- Modify: `homo_multirobot_formation_control/test/test_record_trajectory_yaml.py:38-90`
- Modify: `homo_multirobot_formation_control/README.md:620-625`

**Interfaces:**
- Produces: `wait_for_controller_parameters(node, client, service_name, logger) -> GetParameters.Response | None`.
- Consumes: an available service client; it sends one `GetParameters.Request` with `CTRL_PARAM_NAMES` per attempt.

- [ ] **Step 1: Write failing tests**

Add fakes for a client with `call_async`, futures with `done()`/`result()`, and a monkeypatched `rclpy.spin_until_future_complete`. Add these behavior tests:

```python
def test_wait_for_controller_parameters_retries_after_timeout(monkeypatch):
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    client = FakeParameterClient([FakeFuture(False, None), FakeFuture(True, "response")])
    logger = FakeLogger()

    assert module.wait_for_controller_parameters(object(), client, "/robot2/controller/get_parameters", logger) == "response"
    assert client.request_names == [module.CTRL_PARAM_NAMES, module.CTRL_PARAM_NAMES]
    assert len(logger.messages) == 1


def test_wait_for_controller_parameters_stops_when_ros_shuts_down(monkeypatch):
    states = iter([True, False])
    monkeypatch.setattr(module.rclpy, "ok", lambda: next(states))
    client = FakeParameterClient([FakeFuture(False, None)])

    assert module.wait_for_controller_parameters(object(), client, "/robot2/controller/get_parameters", FakeLogger()) is None
    assert len(client.request_names) == 1
```

- [ ] **Step 2: Run tests to verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest homo_multirobot_formation_control/test/test_record_trajectory_yaml.py -q
```

Expected: FAIL because `wait_for_controller_parameters` is absent.

- [ ] **Step 3: Implement the minimal retry loop**

1. Define `wait_for_controller_parameters(node, client, service_name, logger)` next to `wait_for_controller_service`.
2. While `rclpy.ok()`, build a `GetParameters.Request` with `req.names = list(CTRL_PARAM_NAMES)`, call the client, and spin the supplied node for 2 seconds.
3. Return `future.result()` only when the future is done and result is not `None`; otherwise log `等待控制器参数响应: <service_name>` and begin the next iteration.
4. In `_query_controller_params`, replace the one-shot request and warning branch with this helper. If it returns `None`, return `{}`. Keep the existing parameter-type parsing unchanged.
5. Update README: `controller_node_name` nonempty now waits for both service discovery and one successful parameter response before recording starts.

- [ ] **Step 4: Run focused verification**

Run:

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest homo_multirobot_formation_control/test/test_record_trajectory_yaml.py -q
python3 -m py_compile homo_multirobot_formation_control/scripts/record_trajectory.py
cd /home/l1anggmgo/ros-projects/homo_multirobot_ws
colcon build --packages-select homo_multirobot_formation_control --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

Expected: all commands succeed; tests verify retry after request timeout, successful response, shutdown handling, and the existing service-discovery/YAML behavior.

- [ ] **Step 5: Commit**

```bash
git add homo_multirobot_formation_control/scripts/record_trajectory.py \
  homo_multirobot_formation_control/test/test_record_trajectory_yaml.py \
  homo_multirobot_formation_control/README.md
git commit -m "轨迹记录等待控制器参数响应"
```
