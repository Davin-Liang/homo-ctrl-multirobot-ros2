"""Closed-loop Leader--Follower numerical experiment runner."""

from collections import deque
from dataclasses import dataclass

import numpy as np

from .config import MpcConfig, PlantConfig
from .controller import DelayAwareMpc
from .model import DelayedPlant
from .scenarios import Scenario


@dataclass(frozen=True)
class SimulationConfig:
    sample_time: float = 0.05
    tau: tuple[float, float, float] = (0.43, 0.43, 0.30)
    horizon: int = 20
    input_min: tuple[float, float, float] = (-0.65, -0.65, -1.2)
    input_max: tuple[float, float, float] = (0.65, 0.65, 1.2)
    slew_max: tuple[float, float, float] = (0.10, 0.10, 0.18)


@dataclass
class SimulationLog:
    scenario: str
    time: np.ndarray
    leader_state: np.ndarray
    follower_state: np.ndarray
    error: np.ndarray
    sent_leader: np.ndarray
    sent_follower: np.ndarray
    applied_leader: np.ndarray
    applied_follower: np.ndarray
    statuses: list[str]
    solve_seconds: np.ndarray
    input_min: np.ndarray
    input_max: np.ndarray
    slew_max: np.ndarray
    controller_delay_steps: int
    plant_delay_steps: int


def default_config() -> SimulationConfig:
    return SimulationConfig()


def strict_default_config() -> SimulationConfig:
    return SimulationConfig(
        input_min=(-0.24, -0.20, -0.45),
        input_max=(0.24, 0.20, 0.45),
        slew_max=(0.045, 0.040, 0.075),
    )


def _make_mpc_config(config: SimulationConfig, delay_steps: int) -> MpcConfig:
    slew_max = np.asarray(config.slew_max, dtype=float)
    return MpcConfig(
        horizon=config.horizon,
        q=np.array([32.0, 32.0, 16.0, 4.0, 4.0, 2.0]),
        q_terminal=np.array([120.0, 120.0, 60.0, 14.0, 14.0, 7.0]),
        r_relative=np.array([0.18, 0.18, 0.10]),
        r_slew=np.array([1.2, 1.2, 0.7]),
        input_min=np.asarray(config.input_min, dtype=float),
        input_max=np.asarray(config.input_max, dtype=float),
        slew_min=-slew_max,
        slew_max=slew_max,
        model_delay_steps=delay_steps,
    )


def run_case(
    scenario: Scenario,
    controller_delay_steps: int,
    plant_delay_steps: int,
    config: SimulationConfig,
) -> SimulationLog:
    """Run independent delayed Leader/Follower plants with one MPC follower."""
    if controller_delay_steps < 0 or plant_delay_steps < 0:
        raise ValueError("delay steps must be non-negative")
    tau = np.asarray(config.tau, dtype=float)
    model_plant = PlantConfig(config.sample_time, tau, controller_delay_steps)
    physical_plant = PlantConfig(config.sample_time, tau, plant_delay_steps)
    controller = DelayAwareMpc(model_plant, _make_mpc_config(config, controller_delay_steps))

    leader = DelayedPlant(physical_plant, np.zeros(6), np.zeros(3))
    follower_initial = np.r_[scenario.offset_map + scenario.initial_error[:3], scenario.initial_error[3:]]
    follower = DelayedPlant(physical_plant, follower_initial, np.zeros(3))
    relative_queue: deque[np.ndarray] = deque(
        [np.zeros(3) for _ in range(controller_delay_steps)]
    )
    previous_follower = np.zeros(3)
    steps = int(round(scenario.duration / config.sample_time))

    times = [0.0]
    leader_states = [leader.error.copy()]
    follower_states = [follower.error.copy()]
    errors = [scenario.initial_error.copy()]
    sent_leader: list[np.ndarray] = []
    sent_follower: list[np.ndarray] = []
    applied_leader: list[np.ndarray] = []
    applied_follower: list[np.ndarray] = []
    statuses: list[str] = []
    solve_seconds: list[float] = []

    for step in range(steps):
        time = step * config.sample_time
        leader_horizon = np.array(
            [scenario.leader_command_at(time + index * config.sample_time) for index in range(config.horizon)]
        )
        error = follower.error - leader.error - np.r_[scenario.offset_map, np.zeros(3)]
        queue_array = np.asarray(relative_queue, dtype=float).reshape(controller_delay_steps, 3)
        result = controller.solve(error, queue_array, leader_horizon, previous_follower)
        leader_command = leader_horizon[0]
        follower_command = result.follower_command
        _, actual_leader = leader.step(leader_command)
        _, actual_follower = follower.step(follower_command)
        if relative_queue:
            relative_queue.popleft()
            relative_queue.append((follower_command - leader_command).copy())
        previous_follower = follower_command.copy()

        sent_leader.append(leader_command)
        sent_follower.append(follower_command)
        applied_leader.append(actual_leader)
        applied_follower.append(actual_follower)
        statuses.append(result.status)
        solve_seconds.append(result.solve_seconds)
        times.append((step + 1) * config.sample_time)
        leader_states.append(leader.error.copy())
        follower_states.append(follower.error.copy())
        errors.append(follower.error - leader.error - np.r_[scenario.offset_map, np.zeros(3)])

    return SimulationLog(
        scenario.name, np.asarray(times), np.asarray(leader_states), np.asarray(follower_states), np.asarray(errors),
        np.asarray(sent_leader), np.asarray(sent_follower), np.asarray(applied_leader), np.asarray(applied_follower),
        statuses, np.asarray(solve_seconds), np.asarray(config.input_min), np.asarray(config.input_max),
        np.asarray(config.slew_max), controller_delay_steps, plant_delay_steps,
    )
