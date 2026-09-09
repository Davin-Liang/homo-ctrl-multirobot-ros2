import importlib.util
from pathlib import Path

import pytest
import yaml
from geometry_msgs.msg import PoseStamped, TwistStamped


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = PACKAGE_DIR / "scripts" / "record_trajectory.py"
SPEC = importlib.util.spec_from_file_location("record_trajectory", SCRIPT_PATH)
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


def test_default_yaml_has_all_recorder_parameters():
    defaults = module.load_recorder_parameters(
        PACKAGE_DIR / "config" / "record_trajectory.yaml")

    assert defaults["leader_ns"] == "/robot1"
    assert isinstance(defaults["duration"], (int, float))
    assert defaults["trial_id"] == "trial_01"
    assert set(defaults) == set(module.RECORDER_PARAMETER_DEFAULTS)


def test_explicit_overrides_win_over_yaml_defaults():
    values = module.merge_parameter_overrides(
        {"duration": 30.0, "mode": "sim"}, {"duration": 60.0})

    assert values == {"duration": 60.0, "mode": "sim"}


def test_invalid_yaml_structure_raises_value_error(tmp_path):
    path = tmp_path / "invalid.yaml"
    path.write_text("ros__parameters: {}\n", encoding="utf-8")

    with pytest.raises(ValueError, match=r"/\*\*/ros__parameters"):
        module.load_recorder_parameters(path)


def test_legacy_yaml_without_state_source_gets_ekf_tf_default(tmp_path):
    parameters = dict(module.RECORDER_PARAMETER_DEFAULTS)
    parameters.pop("state_source")
    path = tmp_path / "legacy.yaml"
    path.write_text(
        yaml.safe_dump({"/**": {"ros__parameters": parameters}}),
        encoding="utf-8")

    assert module.load_recorder_parameters(path)["state_source"] == "ekf_tf"


def test_validate_state_source_rejects_unknown_value():
    with pytest.raises(ValueError, match="state_source"):
        module.validate_state_source("wheel_odom")


def test_mocap_sample_uses_map_pose_and_linear_twist_directly():
    pose = PoseStamped()
    pose.pose.position.x = 1.25
    pose.pose.position.y = -0.75
    twist = TwistStamped()
    twist.twist.linear.x = 0.4
    twist.twist.linear.y = -0.2

    assert module.mocap_sample(pose, twist) == (1.25, -0.75, 0.4, -0.2)


def test_mocap_recording_topics_include_pose_and_twist():
    assert module.recording_topics("/robot1", "/robot2", "mocap") == {
        "leader_topic": "/robot1/mocap/pose",
        "follower_topic": "/robot2/mocap/pose",
        "leader_twist_topic": "/robot1/mocap/twist",
        "follower_twist_topic": "/robot2/mocap/twist",
    }


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

    assert module.wait_for_controller_service(
        client, "/robot2/controller/get_parameters", logger)
    assert client.timeouts == [1.0, 1.0, 1.0]
    assert len(logger.messages) == 2


def test_wait_for_controller_service_stops_when_ros_shuts_down(monkeypatch):
    states = iter([True, False])
    monkeypatch.setattr(module.rclpy, "ok", lambda: next(states))
    client = FakeClient([False])

    assert not module.wait_for_controller_service(
        client, "/robot2/controller/get_parameters", FakeLogger())
    assert client.timeouts == [1.0]


def test_cleanup_node_handles_interrupted_construction(monkeypatch):
    calls = []
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(module.rclpy, "shutdown", lambda: calls.append("shutdown"))

    module.cleanup_node(None)

    assert calls == ["shutdown"]


class FakeFuture:
    def __init__(self, done, result, error=None):
        self._done = done
        self._result = result
        self._error = error
        self.cancel_calls = 0

    def done(self):
        return self._done

    def result(self):
        if self._error is not None:
            raise self._error
        return self._result

    def cancel(self):
        self.cancel_calls += 1


class FakeParameterClient:
    def __init__(self, futures):
        self.futures = iter(futures)
        self.request_names = []

    def call_async(self, request):
        self.request_names.append(request.names)
        future = next(self.futures)
        if isinstance(future, Exception):
            raise future
        return future

    def wait_for_service(self, timeout_sec):
        return True


def test_wait_for_controller_parameters_retries_after_timeout(monkeypatch):
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(module.rclpy, "spin_until_future_complete", lambda *args, **kwargs: None)
    timed_out = FakeFuture(False, None)
    client = FakeParameterClient([timed_out, FakeFuture(True, "response")])
    logger = FakeLogger()

    assert module.wait_for_controller_parameters(
        object(), client, "/robot2/controller/get_parameters", logger) == "response"
    assert client.request_names == [module.CTRL_PARAM_NAMES, module.CTRL_PARAM_NAMES]
    assert timed_out.cancel_calls == 1
    assert len(logger.messages) == 1


def test_wait_for_controller_parameters_stops_when_ros_shuts_down(monkeypatch):
    states = iter([True, False])
    monkeypatch.setattr(module.rclpy, "ok", lambda: next(states))
    monkeypatch.setattr(module.rclpy, "spin_until_future_complete", lambda *args, **kwargs: None)
    client = FakeParameterClient([FakeFuture(False, None)])

    assert module.wait_for_controller_parameters(
        object(), client, "/robot2/controller/get_parameters", FakeLogger()) is None
    assert len(client.request_names) == 1


def test_wait_for_controller_parameters_retries_after_request_error(monkeypatch):
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(module.rclpy, "spin_until_future_complete", lambda *args, **kwargs: None)
    client = FakeParameterClient([
        RuntimeError("service vanished"), FakeFuture(True, "response")])

    assert module.wait_for_controller_parameters(
        object(), client, "/robot2/controller/get_parameters", FakeLogger()) == "response"
    assert len(client.request_names) == 2
