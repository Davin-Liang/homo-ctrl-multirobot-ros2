import sys
from pathlib import Path

import numpy as np
from numpy.testing import assert_allclose


PACKAGE_PARENT = Path(__file__).resolve().parents[2]
if str(PACKAGE_PARENT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_PARENT))

from mpc_sim.scenarios import make_curve_scenario, make_static_scenario, make_straight_scenario
from mpc_sim.simulation import default_config, run_case, strict_default_config


def test_scenarios_expose_static_straight_and_curved_leader_inputs():
    assert make_static_scenario().leader_command_at(0.0).shape == (3,)
    assert make_straight_scenario().leader_command_at(1.0)[0] > 0.0
    assert abs(make_curve_scenario().leader_command_at(1.0)[2]) > 0.0


def test_zero_state_is_invariant_for_delay_aware_closed_loop():
    scenario = make_static_scenario(initial_error=np.zeros(6), duration=0.5)
    log = run_case(scenario, controller_delay_steps=6, plant_delay_steps=6, config=default_config())
    assert_allclose(log.error, 0.0, atol=1e-7)
    assert set(log.statuses) <= {"optimal", "optimal_inaccurate"}


def test_recorded_sent_follower_commands_obey_amplitude_and_slew_limits():
    log = run_case(
        make_straight_scenario(duration=8.0), 6, 6, config=strict_default_config()
    )
    assert np.all(log.sent_follower <= log.input_max + 1e-8)
    assert np.all(log.sent_follower >= log.input_min - 1e-8)
    assert np.all(np.abs(np.diff(log.sent_follower, axis=0)) <= log.slew_max + 1e-8)
