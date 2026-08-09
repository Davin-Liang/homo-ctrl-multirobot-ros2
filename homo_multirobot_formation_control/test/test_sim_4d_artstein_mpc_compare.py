import importlib.util
import sys
from pathlib import Path

import numpy as np


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "sim_4d_artstein_mpc_compare.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("sim_4d_artstein_mpc_compare", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_zoh_discretization_matches_double_integrator_model():
    sim = load_module()

    Ad, Bd = sim.double_integrator_zoh(mass=2.0, dt=0.05)

    np.testing.assert_allclose(
        Ad,
        np.array(
            [
                [1.0, 0.0, 0.05, 0.0],
                [0.0, 1.0, 0.0, 0.05],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        ),
    )
    np.testing.assert_allclose(
        Bd,
        np.array(
            [
                [0.000625, 0.0],
                [0.0, 0.000625],
                [0.025, 0.0],
                [0.0, 0.025],
            ]
        ),
    )


def test_mpc_command_is_next_predicted_velocity_not_force_input():
    sim = load_module()
    controller = sim.Mpc4D(
        mass=2.0,
        dt=0.05,
        horizon=12,
        radius=2.0,
        max_speed=0.6,
        max_accel=0.4,
        terminal_factor=8.0,
    )
    leader = np.array([0.0, 0.0, 0.2, 0.0])
    follower = np.array([2.8, 0.2, 0.0, 0.0])
    controller.init(leader, follower)

    command = controller.command(leader, follower)
    predicted_next = controller.last_solution.x_pred[1, 2:4]
    force_input = controller.last_solution.u_pred[0]

    np.testing.assert_allclose(command, predicted_next, atol=1e-9)
    assert not np.allclose(command, force_input)
    assert np.linalg.norm(command) <= 0.6 + 1e-9


def test_short_no_delay_mpc_closed_loop_reduces_formation_error():
    sim = load_module()

    rows = sim.simulate_circle_case(
        controller_kind="mpc",
        compensation_kind="none",
        tmax=6.0,
        dt=0.05,
        tau=0.43,
        Td=0.22,
        max_speed=0.6,
        max_accel=0.4,
    )

    dist = np.array([row.distance for row in rows])
    assert dist[-1] < 0.65 * dist[0]
