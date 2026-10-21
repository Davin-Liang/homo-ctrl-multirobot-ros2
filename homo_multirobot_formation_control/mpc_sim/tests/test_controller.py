import sys
from pathlib import Path

import numpy as np
import pytest
from numpy.testing import assert_allclose


PACKAGE_PARENT = Path(__file__).resolve().parents[2]
if str(PACKAGE_PARENT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_PARENT))

from mpc_sim.config import MpcConfig, PlantConfig
from mpc_sim.controller import DelayAwareMpc, augment_delay_model, is_acceptable_status
from mpc_sim.model import exact_zoh


def make_controller(delay_steps=6, input_min=(-1.0, -1.0, -1.0), input_max=(1.0, 1.0, 1.0)):
    plant = PlantConfig(0.05, np.array([0.43, 0.43, 0.30]), delay_steps)
    mpc = MpcConfig(
        horizon=20,
        q=np.array([30.0, 30.0, 15.0, 3.0, 3.0, 1.5]),
        q_terminal=np.array([90.0, 90.0, 45.0, 9.0, 9.0, 4.5]),
        r_relative=np.array([0.2, 0.2, 0.1]),
        r_slew=np.array([1.0, 1.0, 0.5]),
        input_min=np.asarray(input_min),
        input_max=np.asarray(input_max),
        slew_min=np.array([-0.12, -0.12, -0.20]),
        slew_max=np.array([0.12, 0.12, 0.20]),
        model_delay_steps=delay_steps,
    )
    return DelayAwareMpc(plant, mpc)


def test_augmented_model_uses_oldest_queued_input_in_first_state_update():
    ad, bd = exact_zoh(PlantConfig(0.05, np.ones(3), 2))
    az, bz = augment_delay_model(ad, bd, 2)
    oldest = np.array([1.0, 2.0, 3.0])
    newest = np.array([4.0, 5.0, 6.0])
    state = np.r_[np.zeros(6), oldest, newest]
    next_state = az @ state + bz @ np.array([7.0, 8.0, 9.0])
    assert_allclose(next_state[:6], bd @ oldest)
    assert_allclose(next_state[6:9], newest)
    assert_allclose(next_state[9:12], [7.0, 8.0, 9.0])


def test_zero_error_and_zero_leader_returns_zero_follower_command():
    controller = make_controller(delay_steps=6)
    result = controller.solve(
        np.zeros(6), np.zeros((6, 3)), np.zeros((20, 3)), np.zeros(3)
    )
    assert result.status in {"optimal", "optimal_inaccurate"}
    assert_allclose(result.follower_command, np.zeros(3), atol=1e-7)


def test_constraints_apply_to_follower_not_relative_command():
    controller = make_controller(
        delay_steps=0, input_min=(-0.2, -0.2, -0.2), input_max=(0.2, 0.2, 0.2)
    )
    leader = np.tile([0.5, 0.0, 0.0], (20, 1))
    result = controller.solve(np.zeros(6), np.empty((0, 3)), leader, np.zeros(3))
    assert result.status in {"optimal", "optimal_inaccurate"}
    assert result.follower_command[0] == pytest.approx(0.12, abs=1e-5)
    assert np.all(result.follower_command <= 0.2 + 1e-8)


def test_only_optimal_statuses_are_accepted_for_command_output():
    assert is_acceptable_status("optimal")
    assert is_acceptable_status("optimal_inaccurate")
    assert not is_acceptable_status("user_limit")
    assert not is_acceptable_status("infeasible")
