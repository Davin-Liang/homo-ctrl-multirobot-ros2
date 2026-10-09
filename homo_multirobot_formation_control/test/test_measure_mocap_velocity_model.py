import importlib.util
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
