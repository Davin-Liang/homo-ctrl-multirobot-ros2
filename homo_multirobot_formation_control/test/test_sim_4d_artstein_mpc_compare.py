import argparse
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


def test_lqr_dare_feedback_is_discrete_stable():
    sim = load_module()
    controller = sim.Lqr4D(
        mass=2.0,
        dt=0.05,
        radius=2.0,
        m_p=4,
        tol=0.1,
        q=(40.0, 40.0, 1.0, 1.0),
        r=(0.02, 0.02),
    )

    eigvals = np.linalg.eigvals(controller.Ad - controller.Bd @ controller.K)

    assert controller.K.shape == (2, 4)
    assert np.max(np.abs(eigvals)) < 1.0


def test_lqr_factory_uses_lqr_parameters():
    sim = load_module()
    args = argparse.Namespace(
        mass=2.0,
        radius=2.0,
        m_p=4,
        tol=0.1,
        lqr_q_px=40.0,
        lqr_q_py=30.0,
        lqr_q_vx=2.0,
        lqr_q_vy=1.5,
        lqr_r_ux=0.03,
        lqr_r_uy=0.04,
    )

    controller = sim.make_controller("lqr", 0.05, args)

    assert isinstance(controller, sim.Lqr4D)
    assert np.allclose(np.diag(controller.Q), [40.0, 30.0, 2.0, 1.5])
    assert np.allclose(np.diag(controller.R), [0.03, 0.04])


def test_short_no_delay_lqr_closed_loop_reduces_formation_error():
    sim = load_module()
    args = argparse.Namespace(
        mass=2.0,
        radius=2.0,
        m_p=4,
        tol=0.1,
        hpc_c_min=0.1,
        initial_min_lambda=1.5,
        switch_min_lambda=4.0,
        mpc_horizon=30,
        q_px=40.0,
        q_py=40.0,
        q_vx=1.0,
        q_vy=1.0,
        r_ux=0.02,
        r_uy=0.02,
        terminal_factor=10.0,
        max_linear_vel=0.8,
        max_linear_accel=1.0,
        mpc_max_iter=320,
        mpc_eps_abs=1e-4,
        mpc_eps_rel=1e-3,
        mpc_rho=1.0,
        lqr_q_px=40.0,
        lqr_q_py=40.0,
        lqr_q_vx=1.0,
        lqr_q_vy=1.0,
        lqr_r_ux=0.02,
        lqr_r_uy=0.02,
        leader_radius=2.0,
        leader_omega=0.1,
    )

    rows = sim.simulate_circle_case(
        controller_kind="lqr",
        compensation_kind="none",
        tmax=8.0,
        dt=0.05,
        tau=0.43,
        Td=0.22,
        max_speed=args.max_linear_vel,
        max_accel=args.max_linear_accel,
        args=args,
    )

    assert rows[-1].distance < rows[0].distance
