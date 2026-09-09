import importlib.util
from pathlib import Path

import pytest


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
