from __future__ import annotations

import math

import numpy as np
import pytest

from talon.constants import (
    ARM_LENGTH,
    DT,
    MASS,
    GRAVITY,
    MOTOR_KF,
    MOTOR_KM,
    MOTOR_TIME_CONST,
)
from talon.motors import (
    MIX_MATRIX,
    MOTOR_FORCE_MAX,
    MotorModel,
    forward_mix,
    motor_mixing,
)


def test_yaw_mix_coefficient_is_physically_realistic() -> None:
    # c = KM/KF is the prop-drag yaw moment arm. The old draft constant
    # (KM = 5.96e-3) implied c ~ 4656 m, physically absurd; the fixed
    # KM = 1.92e-8 gives c = 0.015 m, realistic for a 15 cm quadrotor.
    c = MOTOR_KM / MOTOR_KF
    assert c == pytest.approx(0.015, rel=1e-9)
    # The mix matrix carries this c in its yaw row.
    assert MIX_MATRIX[3, 0] == pytest.approx(-c, rel=1e-12)
    assert MIX_MATRIX[3, 1] == pytest.approx(c, rel=1e-12)


def test_motor_force_max_is_physically_realistic() -> None:
    # Per-motor force cap from max RPM: KF * M_MAX^2.
    assert MOTOR_FORCE_MAX == pytest.approx(MOTOR_KF * 20000.0**2)
    # Hover thrust per motor must be well below the cap (no saturation
    # at hover).
    hover_per_motor = MASS * GRAVITY / 4.0
    assert hover_per_motor < 0.1 * MOTOR_FORCE_MAX


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
    # Symmetric pairs are equal up to float round-off in the mix inverse.
    np.testing.assert_allclose(forces[0], forces[1], rtol=1e-12)
    np.testing.assert_allclose(forces[2], forces[3], rtol=1e-12)
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
    # f = +/- Mz / (4c): motors 1 and 4 are commanded negative and must
    # clamp to exactly zero at steady state. Mz = 30 N*m keeps the positive
    # side (500 N) under MOTOR_FORCE_MAX (512 N) so only the lower clamp
    # binds.
    moment = np.array([0.0, 0.0, 30.0])
    magnitude = 30.0 / (4.0 * (MOTOR_KM / MOTOR_KF))
    assert magnitude < MOTOR_FORCE_MAX
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


def test_motor_lag_clamps_forces_at_max() -> None:
    # A pure yaw moment well above the physical envelope drives the
    # positive motors past MOTOR_FORCE_MAX; the upper clamp must bind
    # and the negatives to zero.
    model = MotorModel()
    moment = np.array([0.0, 0.0, 100.0])
    magnitude = 100.0 / (4.0 * (MOTOR_KM / MOTOR_KF))
    assert magnitude > MOTOR_FORCE_MAX
    for _ in range(100):
        forces = model.step(0.0, moment, dt=MOTOR_TIME_CONST)
    np.testing.assert_array_equal(forces[[0, 3]], np.zeros(2))
    np.testing.assert_allclose(
        forces[[1, 2]], np.full(2, MOTOR_FORCE_MAX), rtol=1e-12
    )


def test_motor_lag_accepts_moment_error_under_saturation() -> None:
    # When the mixer clamps, the forward mix of the converged motor forces
    # does NOT recover the commanded moment (real-actuator behavior: the
    # moment error is accepted, not redistributed).
    model = MotorModel()
    thrust = MASS * GRAVITY
    moment = np.array([1.1, -0.5, 0.3])
    for _ in range(200):
        forces = model.step(thrust, moment, dt=MOTOR_TIME_CONST)
    thrust_out, moment_out = forward_mix(forces)
    assert np.all(forces >= 0.0)
    # Roll/pitch demands exceed the hover envelope, so the recovered
    # moment differs from the command.
    assert np.linalg.norm(moment_out - moment) > 1e-3


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
