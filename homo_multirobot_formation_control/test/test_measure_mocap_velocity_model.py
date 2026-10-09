import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "measure_mocap_velocity_model.py"
SPEC = importlib.util.spec_from_file_location("mocap_velocity_model", SCRIPT)
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


def test_scalar_response_uses_planar_magnitude():
    assert module.scalar_response("move", 0.3, 0.4, 9.0) == pytest.approx(0.5)
    assert module.scalar_response("turn", 0.3, 0.4, -0.7) == pytest.approx(-0.7)


def test_first_order_wins_for_first_order_samples():
    time_s = np.linspace(0.0, 1.0, 51)
    speed = 0.2 * (1.0 - np.exp(-time_s / 0.25))
    first = module.fit_first_order(time_s, speed, 0.2)
    accel = module.fit_constant_accel(time_s, speed, 0.2)
    assert module.choose_model(first, accel) == "first_order"


def test_write_outputs_creates_replayable_files(tmp_path):
    samples = [{"trial": 1, "phase": "hold", "elapsed_s": 0.0,
                "cmd_vx": 0.15, "cmd_vy": 0.0, "cmd_wz": 0.0,
                "vx_map": 0.0, "vy_map": 0.0, "wz": 0.0,
                "response": 0.0}]
    module.write_outputs(tmp_path, samples, [], {"mode": "move"})
    assert (tmp_path / "samples.csv").is_file()
    assert (tmp_path / "fit_summary.csv").is_file()
    assert json.loads((tmp_path / "metadata.json").read_text())["mode"] == "move"


def test_command_values_activate_one_axis():
    assert module.command_values("move", 0.15) == (0.15, 0.0, 0.0)
    assert module.command_values("turn", 0.3) == (0.0, 0.0, 0.3)
