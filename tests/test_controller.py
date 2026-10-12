from __future__ import annotations

import math

import numpy as np
import pytest

from talon import (
    DT,
    DT_CONTROL,
    GRAVITY,
    MASS,
    CascadedController,
    ControlCommand,
    ControlGains,
    State,
    VehicleParams,
    propagate,
)

_DEG = math.pi / 180.0


def _closed_loop(
    initial: State,
    gains: ControlGains,
    params: VehicleParams,
    pos_desired: np.ndarray,
    vel_desired: np.ndarray,
    yaw_desired: float,
    duration: float,
    control_dt: float = DT_CONTROL,
) -> tuple[State, list[np.ndarray]]:
    """Closed loop with instantaneous motors.

    The controller runs at `control_dt` (default DT_CONTROL = 100 Hz, ZOH
    between ticks) and the plant propagates at DT = 200 Hz. Motor lag is
    covered by sims/scenario_hover.py; this mirrors the C++ GTest setup but
    with the realistic 100 Hz control period.
    """
    controller = CascadedController(gains, params)
    state = initial
    steps_per_control = max(1, round(control_dt / DT))
    n_steps = round(duration / DT)
    command = ControlCommand()
    command.collective_thrust = params.mass * params.gravity
    zeros = np.zeros(3)
    positions: list[np.ndarray] = []
    for i in range(n_steps):
        if i % steps_per_control == 0:
            command = controller.step(
                state, pos_desired, vel_desired, yaw_desired, control_dt
            )
        state = propagate(
            state,
            command.collective_thrust,
            np.asarray(command.moment),
            zeros,
            DT,
            params,
        )
        positions.append(np.asarray(state.position))
    return state, positions


def test_default_gains_all_positive() -> None:
    gains = ControlGains.defaults()
    for name in ("kp_pos", "kd_pos", "kp_att", "kd_att"):
        values = np.asarray(getattr(gains, name))
        assert np.all(values > 0.0), f"{name} has non-positive entries: {values}"


def test_control_command_defaults() -> None:
    cmd = ControlCommand()
    assert cmd.collective_thrust == 0.0
    np.testing.assert_array_equal(np.asarray(cmd.moment), np.zeros(3))


def test_closed_loop_hover_ten_seconds() -> None:
    # 10 s hover at 10 m altitude (NED z = -10); final error < 0.05 m.
    gains = ControlGains.defaults()
    params = VehicleParams()
    pos_desired = np.array([0.0, 0.0, -10.0])
    final, _ = _closed_loop(
        State(), gains, params, pos_desired, np.zeros(3), 0.0, 10.0
    )
    error = pos_desired - np.asarray(final.position)
    assert np.max(np.abs(error)) < 0.05
    assert np.linalg.norm(np.asarray(final.velocity)) < 0.05


def test_position_step_metrics() -> None:
    # 1 m north step: rise time (0 to 90%) < 2.5 s (slightly looser than the
    # GTest's 2.0 s to absorb the 100 Hz ZOH control period), overshoot < 10%,
    # steady-state error < 0.05 m after 10 s.
    gains = ControlGains.defaults()
    params = VehicleParams()
    final, positions = _closed_loop(
        State(), gains, params, np.array([1.0, 0.0, 0.0]), np.zeros(3), 0.0, 10.0
    )
    xs = np.array([p[0] for p in positions])
    above = np.flatnonzero(xs >= 0.9)
    assert above.size > 0, "position never reached 90% of the step"
    rise_time = float(above[0]) * DT
    assert rise_time < 2.5
    assert float(np.max(xs)) < 1.1
    error = final.position[0] - 1.0
    assert abs(error) < 0.05
    assert abs(final.position[1]) < 0.05
    assert abs(final.position[2]) < 0.05
    assert np.linalg.norm(np.asarray(final.velocity)) < 0.05


def test_yaw_tracking_thirty_degrees() -> None:
    gains = ControlGains.defaults()
    params = VehicleParams()
    final, _ = _closed_loop(
        State(), gains, params, np.zeros(3), np.zeros(3), 30.0 * _DEG, 5.0
    )
    rot = _dcm(final)
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    assert abs(yaw - 30.0 * _DEG) < 2.0 * _DEG
    assert np.linalg.norm(np.asarray(final.position)) < 0.02


def test_thrust_saturation_insane_command_no_nan() -> None:
    gains = ControlGains.defaults()
    params = VehicleParams()
    controller = CascadedController(gains, params)
    state = State()
    cmd = controller.step(
        state, np.array([1000.0, 1000.0, -1000.0]), np.zeros(3), 0.0, DT
    )
    assert cmd.collective_thrust <= controller.thrust_max() + 1e-12
    assert cmd.collective_thrust >= 0.0
    assert cmd.collective_thrust == params.mass * params.gravity * 4.0
    moment = np.asarray(cmd.moment)
    assert np.all(np.isfinite(moment))
    assert np.all(np.isfinite(np.asarray(controller.last_a_des())))


def test_hover_equilibrium_thrust_and_zero_moment() -> None:
    gains = ControlGains.defaults()
    params = VehicleParams()
    controller = CascadedController(gains, params)
    cmd = controller.step(State(), np.zeros(3), np.zeros(3), 0.0, DT)
    assert cmd.collective_thrust == pytest.approx(MASS * GRAVITY, abs=1e-12)
    np.testing.assert_allclose(np.asarray(cmd.moment), np.zeros(3), atol=1e-12)
    a_des = np.asarray(controller.last_a_des())
    np.testing.assert_allclose(a_des, [0.0, 0.0, GRAVITY], atol=1e-12)


def test_reset_makes_output_repeatable() -> None:
    gains = ControlGains.defaults()
    params = VehicleParams()
    controller = CascadedController(gains, params)
    state = State()
    state.position = np.array([0.3, -0.2, -1.7])
    state.velocity = np.array([0.4, 0.1, -0.3])
    pos_des = np.array([1.0, 0.5, -2.0])
    vel_des = np.array([0.1, 0.0, 0.0])
    first = controller.step(state, pos_des, vel_des, 0.4, DT)
    controller.step(State(), np.full(3, 5.0), np.ones(3), -1.0, DT)
    controller.set_omega_desired(np.array([1.0, 2.0, 3.0]))
    controller.reset()
    second = controller.step(state, pos_des, vel_des, 0.4, DT)
    assert first.collective_thrust == second.collective_thrust
    np.testing.assert_array_equal(
        np.asarray(first.moment), np.asarray(second.moment)
    )


def _dcm(state: State) -> np.ndarray:
    q = state.orientation
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )
