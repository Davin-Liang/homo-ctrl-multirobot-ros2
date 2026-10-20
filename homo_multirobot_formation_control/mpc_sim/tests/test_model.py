import sys
from pathlib import Path

import numpy as np
import pytest
from numpy.testing import assert_allclose
from scipy.linalg import expm


PACKAGE_PARENT = Path(__file__).resolve().parents[2]
if str(PACKAGE_PARENT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_PARENT))

from mpc_sim.config import PlantConfig
from mpc_sim.model import DelayedPlant, exact_zoh


def continuous_zoh_reference(state, command, config):
    tau = np.asarray(config.tau)
    a = np.block([
        [np.zeros((3, 3)), np.eye(3)],
        [np.zeros((3, 3)), -np.diag(1.0 / tau)],
    ])
    b = np.vstack([np.zeros((3, 3)), np.diag(1.0 / tau)])
    augmented = np.zeros((10, 10))
    augmented[:6, :6] = a
    augmented[:6, 6:9] = b
    transition = expm(augmented * config.sample_time)
    return (transition @ np.r_[state, command, 1.0])[:6]


def test_exact_zoh_matches_scalar_analytic_blocks():
    config = PlantConfig(0.05, np.array([0.4, 0.4, 0.2]), 6)
    ad, bd = exact_zoh(config)
    decay = np.exp(-0.05 / 0.4)
    H = 0.4 * (1.0 - decay)
    assert ad[0, 3] == pytest.approx(H)
    assert bd[0, 0] == pytest.approx(0.05 - H)
    assert ad[3, 3] == pytest.approx(decay)
    assert bd[3, 0] == pytest.approx(1.0 - decay)


def test_zoh_step_matches_scipy_augmented_exponential():
    config = PlantConfig(0.05, np.array([0.43, 0.43, 0.30]), 0)
    state = np.array([0.3, -0.1, 0.2, 0.4, -0.2, 0.1])
    command = np.array([0.1, 0.2, -0.05])
    ad, bd = exact_zoh(config)
    assert_allclose(ad @ state + bd @ command, continuous_zoh_reference(state, command, config))


def test_six_step_delay_applies_new_command_on_seventh_step_call():
    plant = DelayedPlant(PlantConfig(0.05, np.ones(3), 6), np.zeros(6), np.zeros(3))
    command = np.array([1.0, 0.0, 0.0])
    for _ in range(6):
        _, applied = plant.step(command)
        assert_allclose(applied, np.zeros(3))
    _, applied = plant.step(command)
    assert_allclose(applied, command)


def test_zero_delay_applies_command_immediately():
    plant = DelayedPlant(PlantConfig(0.05, np.ones(3), 0), np.zeros(6), np.zeros(3))
    command = np.array([1.0, 0.0, 0.0])
    _, applied = plant.step(command)
    assert_allclose(applied, command)
