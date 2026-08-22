import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "hocbf_6d_feasibility.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location(
        "hocbf_6d_feasibility", SCRIPT_PATH
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_exact_predictor_matches_future_delayed_zoh_state():
    feasibility = load_module()
    params = feasibility.PlantParams(tau=0.4, delay=0.1, dt=0.05)
    ad, bd = feasibility.zoh_matrices(params)
    x0 = np.array([0.0, 0.0, 0.2, -0.1])
    queued = [np.array([0.3, 0.0]), np.array([0.1, -0.2])]

    predicted = feasibility.predict_delayed_state(x0, queued, ad, bd)
    expected = ad @ (ad @ x0 + bd @ queued[0]) + bd @ queued[1]

    np.testing.assert_allclose(predicted, expected, atol=1e-12)


def test_hocbf_halfspace_tightens_head_on_approach():
    feasibility = load_module()

    a, b, h, psi1 = feasibility.hocbf_halfspace(
        x_pred=np.array([2.0, 0.0, -0.5, 0.0]),
        obstacle=np.zeros(2),
        safe_radius=1.0,
        tau=0.5,
        c1=1.0,
        c2=1.0,
    )

    np.testing.assert_allclose(a, [8.0, 0.0])
    assert h == pytest.approx(3.0)
    assert psi1 == pytest.approx(1.0)
    assert a @ np.array([-1.0, 0.0]) < b
    assert a @ np.zeros(2) >= b
