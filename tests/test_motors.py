from __future__ import annotations

import math

import numpy as np

from talon.constants import (
    ARM_LENGTH,
    DT,
    MASS,
    GRAVITY,
    MOTOR_KF,
    MOTOR_KM,
    MOTOR_TIME_CONST,
)
from talon.motors import MotorModel, forward_mix, motor_mixing


def test_motor_mixing_hover_splits_evenly() -> None:
    thrust = MASS * GRAVITY
    forces = motor_mixing(thrust, np.zeros(3))
    np.testing.assert_allclose(forces, np.full(4, thrust / 4.0), atol=1e-12)


def test_motor_mixing_pure_roll_signs() -> None:
    # Mix matrix row: Mx = l * (-f1 - f2 + f3 + f4). Mx > 0 means motors 3 and 4
    # work harder than 1 and 2.
    forces = motor_mixing(0.0, np.array([0.05, 0.0, 0.0]))
    assert forces[2] > forces[0]
    assert forces[3] > forces[1]
    assert forces[0] == forces[1]
    assert forces[2] == forces[3]
    # Zero net thrust.
    assert abs(float(np.sum(forces))) < 1e-12


def test_motor_lag_reaches_63_percent_at_tau() -> None:
    model = MotorModel()
    thrust = 4.0  # 1 N per motor commanded, moment zero
    forces = model.step(thrust, np.zeros(3), dt=MOTOR_TIME_CONST)
    expected = (thrust / 4.0) * (1.0 - math.exp(-1.0))
    np.testing.assert_allclose(forces, np.full(4, expected), rtol=1e-12)


def test_motor_lag_exact_exponential_accumulation() -> None:
    # Constant command over N steps: exact update gives 1 - exp(-T/tau).
    model = MotorModel()
    thrust = 8.0
    n_steps = 200
    dt = MOTOR_TIME_CONST / n_steps
    for _ in range(n_steps):
        forces = model.step(thrust, np.zeros(3), dt=dt)
    expected = (thrust / 4.0) * (1.0 - math.exp(-1.0))
    np.testing.assert_allclose(forces, np.full(4, expected), rtol=1e-12)


def test_motor_lag_clamps_negative_forces() -> None:
    model = MotorModel()
    # Pure yaw moment with zero collective. The mix inverse gives alternating
    # f = +/- Mz * Kf / (4 * Km): motors 1 and 4 are commanded negative and
    # must clamp to exactly zero at steady state.
    moment = np.array([0.0, 0.0, 50.0])
    magnitude = 50.0 * MOTOR_KF / (4.0 * MOTOR_KM)
    commanded = motor_mixing(0.0, moment)
    np.testing.assert_allclose(
        commanded, [-magnitude, magnitude, magnitude, -magnitude], rtol=1e-12
    )

    # Large dt so the lag converges to steady state within float precision.
    for _ in range(100):
        forces = model.step(0.0, moment, dt=MOTOR_TIME_CONST)

    # Negative motors clamp to exactly zero; positive ones settle to command.
    np.testing.assert_array_equal(forces[[0, 3]], np.zeros(2))
    np.testing.assert_allclose(forces[[1, 2]], np.full(2, magnitude), rtol=1e-12)
    assert np.all(forces >= 0.0)


def test_motor_model_reset() -> None:
    model = MotorModel()
    model.step(10.0, np.zeros(3), dt=0.1)
    assert np.any(model.forces > 0.0)
    model.reset()
    assert np.all(model.forces == 0.0)


def test_forward_mix_round_trip() -> None:
    thrust = MASS * GRAVITY
    moment = np.array([0.02, -0.01, 0.005])
    forces = motor_mixing(thrust, moment)
    assert np.all(forces > 0.0)  # no saturation in this regime
    thrust_out, moment_out = forward_mix(forces)
    np.testing.assert_allclose(thrust_out, thrust, atol=1e-10)
    np.testing.assert_allclose(moment_out, moment, atol=1e-10)


def test_forward_mix_known_values() -> None:
    forces = np.array([1.0, 2.0, 3.0, 4.0])
    thrust, moment = forward_mix(forces)
    assert thrust == 10.0
    lever = ARM_LENGTH / math.sqrt(2.0)
    expected_mx = lever * (-(forces[0] + forces[1]) + (forces[2] + forces[3]))
    np.testing.assert_allclose(moment[0], expected_mx, atol=1e-12)


def test_motor_step_uses_default_dt() -> None:
    model = MotorModel()
    forces = model.step(4.0, np.zeros(3))
    expected = (4.0 / 4.0) * (1.0 - math.exp(-DT / MOTOR_TIME_CONST))
    np.testing.assert_allclose(forces, np.full(4, expected), rtol=1e-12)
