import importlib.util
import sys
from pathlib import Path

import numpy as np


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "sim_6d_disc_artstein_compare.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location(
        "sim_6d_disc_artstein_compare", SCRIPT_PATH
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_delay_line_represents_220ms_exactly_at_10ms_plant_step():
    simulation = load_module()
    initial = np.zeros(3)
    command = np.array([0.4, -0.2, 0.1])

    delay_line = simulation.make_delay_line(initial, delay=0.22, plant_dt=0.01)

    for _ in range(22):
        applied = simulation.advance_delay_line(delay_line, command)
        np.testing.assert_allclose(applied, initial)

    applied = simulation.advance_delay_line(delay_line, command)

    np.testing.assert_allclose(applied, command)


def test_plant_dt_must_divide_control_period():
    simulation = load_module()

    try:
        simulation.simulate_case(
            "compensated",
            Tmax=0.1,
            h=0.05,
            tau_v=0.43,
            tau_w=0.43,
            Td=0.22,
            plant_dt=0.03,
        )
    except ValueError as exc:
        assert "integer multiple" in str(exc)
    else:
        raise AssertionError("simulate_case accepted a nonintegral plant substep")
