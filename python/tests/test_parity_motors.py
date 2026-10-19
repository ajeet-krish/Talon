"""Python vs ``talon._core`` parity for motor mixing and the motor model.

The inverse mix, the forward mix, the first-order lag against its closed
form and both saturation edges are exercised on both implementations, plus
the three registered API-edge divergences: a negative ``dt`` (Python
raises, Rust clamps to zero), a NaN ``dt`` (Python yields NaN forces,
Rust clamps to zero), and a NaN ``time_const`` (Python accepts, Rust
rejects).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from parity_util import (
    assert_exact,
    assert_near,
    assert_same_error,
    assert_vec_near,
    expect_divergence,
)
from talon import _core
from talon.constants import DT, GRAVITY, MASS, MOTOR_KF, MOTOR_KM, MOTOR_TIME_CONST
from talon.motors import MIX_MATRIX, MOTOR_FORCE_MAX, MotorModel, forward_mix, motor_mixing

WRENCHES = [
    pytest.param(MASS * GRAVITY, [0.0, 0.0, 0.0], id="hover"),
    pytest.param(12.0, [0.05, -0.02, 0.01], id="mixed"),
    pytest.param(16.0, [-0.1, 0.1, -0.02], id="large_moments"),
    pytest.param(0.0, [0.0, 0.0, 0.1], id="pure_yaw"),
    pytest.param(4.0, [0.0, 0.0, 0.0], id="low_thrust"),
]


def test_mix_matrix_parity() -> None:
    rs_matrix = _core.mix_matrix()
    assert len(rs_matrix) == 4
    for row in range(4):
        assert_vec_near(rs_matrix[row], MIX_MATRIX[row], label=f"mix row {row}")


@pytest.mark.parametrize(("thrust", "moment"), WRENCHES)
def test_motor_mixing_parity(thrust: float, moment: list[float]) -> None:
    py_forces = motor_mixing(thrust, moment)
    rs_forces = _core.motor_mixing(thrust, moment)
    # The two sides invert the mix matrix with different libraries (LAPACK
    # vs nalgebra), so agreement is at the ULP level, not bit level. The
    # raw inverse mix may legitimately go negative (motors are only clamped
    # by MotorModel, which the saturation tests below pin).
    assert_vec_near(rs_forces, py_forces, label="motor forces")


@pytest.mark.parametrize(("thrust", "moment"), WRENCHES)
def test_forward_mix_round_trip(thrust: float, moment: list[float]) -> None:
    py_forces = motor_mixing(thrust, moment)
    rs_forces = _core.motor_mixing(thrust, moment)
    py_t, py_m = forward_mix(py_forces)
    rs_t, rs_m = _core.forward_mix(rs_forces)
    # forward_mix(motor_mixing(w)) recovers w on both sides.
    assert_near(py_t, thrust, label="python thrust round trip")
    assert_near(rs_t, thrust, label="rust thrust round trip")
    assert_vec_near(py_m, moment, label="python moment round trip")
    assert_vec_near(rs_m, moment, label="rust moment round trip")
    # And the two sides agree with each other.
    assert_near(rs_t, py_t, label="thrust parity")
    assert_vec_near(rs_m, py_m, label="moment parity")


def test_forward_mix_known_values() -> None:
    forces = [1.0, 2.0, 3.0, 4.0]
    py_t, py_m = forward_mix(np.asarray(forces))
    rs_t, rs_m = _core.forward_mix(forces)
    # The thrust row is all ones over small integers: exact on both sides.
    assert_exact(py_t, 10.0, label="python thrust")
    assert_exact(rs_t, 10.0, label="rust thrust")
    lever = 0.15 / math.sqrt(2.0)  # ARM_LENGTH / sqrt(2)
    expected_mx = lever * (-(1.0 + 2.0) + (3.0 + 4.0))
    assert_near(py_m[0], expected_mx, label="python roll moment")
    assert_near(rs_m[0], expected_mx, label="rust roll moment")
    assert_vec_near(rs_m, py_m, label="moment parity")


def test_lag_matches_the_exponential_closed_form() -> None:
    # One step of length tau reaches 1 - 1/e of the command.
    tau = 0.02
    thrust = 4.0
    py_model, rs_model = MotorModel(tau), _core.MotorModel(tau)
    py_forces = py_model.step(thrust, np.zeros(3), dt=tau)
    rs_forces = rs_model.step(thrust, [0.0, 0.0, 0.0], dt=tau)
    expected = (thrust / 4.0) * (1.0 - math.exp(-1.0))
    for index in range(4):
        assert_near(py_forces[index], expected, label=f"python motor {index}")
        assert_near(rs_forces[index], expected, label=f"rust motor {index}")
    assert_vec_near(rs_forces, py_forces, label="lag parity")


def test_lag_accumulation_matches_the_closed_form() -> None:
    # A constant command over N steps gives the exact 1 - exp(-T/tau).
    tau = MOTOR_TIME_CONST
    thrust = 8.0
    n_steps = 200
    dt = tau / n_steps
    py_model, rs_model = MotorModel(), _core.MotorModel()
    for _ in range(n_steps):
        py_forces = py_model.step(thrust, np.zeros(3), dt=dt)
        rs_forces = rs_model.step(thrust, [0.0, 0.0, 0.0], dt=dt)
    expected = (thrust / 4.0) * (1.0 - math.exp(-1.0))
    for index in range(4):
        assert_near(py_forces[index], expected, label=f"python motor {index}")
        assert_near(rs_forces[index], expected, label=f"rust motor {index}")
    assert_vec_near(rs_forces, py_forces, label="accumulated lag parity")


def test_default_dt_parity() -> None:
    # Both default-dt paths use the same DT constant from the same core.
    py_model, rs_model = MotorModel(), _core.MotorModel()
    py_forces = py_model.step(4.0, np.zeros(3))
    rs_forces = rs_model.step(4.0, [0.0, 0.0, 0.0])
    expected = 1.0 * (1.0 - math.exp(-DT / MOTOR_TIME_CONST))
    assert_near(py_forces[0], expected, label="python default dt")
    assert_near(rs_forces[0], expected, label="rust default dt")
    assert_vec_near(rs_forces, py_forces, label="default dt parity")


def test_lower_saturation_clamps_to_zero() -> None:
    # A pure yaw moment of 30 N*m commands alternating +/-500 N: the
    # negative motors clamp to exactly zero, the positive ones settle at
    # the command, and both sides agree.
    moment = [0.0, 0.0, 30.0]
    magnitude = 30.0 / (4.0 * (MOTOR_KM / MOTOR_KF))
    py_commanded = motor_mixing(0.0, np.asarray(moment))
    rs_commanded = _core.motor_mixing(0.0, moment)
    assert_vec_near(rs_commanded, py_commanded, label="commanded forces")

    py_model, rs_model = MotorModel(), _core.MotorModel()
    for _ in range(100):
        py_forces = py_model.step(0.0, np.asarray(moment), dt=MOTOR_TIME_CONST)
        rs_forces = rs_model.step(0.0, moment, dt=MOTOR_TIME_CONST)
    for model_forces in (py_forces, rs_forces):
        assert_exact(model_forces[0], 0.0, label="lower clamp motor 0")
        assert_exact(model_forces[3], 0.0, label="lower clamp motor 3")
        assert_near(model_forces[1], magnitude, label="settled positive motor")
    assert_vec_near(rs_forces, py_forces, label="lower saturation parity")


def test_upper_saturation_clamps_at_the_force_ceiling() -> None:
    # The commanded moment is far outside the envelope, so the positive
    # motors pin at MOTOR_FORCE_MAX (exactly 512 N, identical on both
    # sides) and the negative ones at zero.
    moment = [0.0, 0.0, 100.0]
    magnitude = 100.0 / (4.0 * (MOTOR_KM / MOTOR_KF))
    assert magnitude > MOTOR_FORCE_MAX
    py_model, rs_model = MotorModel(), _core.MotorModel()
    for _ in range(100):
        py_forces = py_model.step(0.0, np.asarray(moment), dt=MOTOR_TIME_CONST)
        rs_forces = rs_model.step(0.0, moment, dt=MOTOR_TIME_CONST)
    for model_forces in (py_forces, rs_forces):
        assert_exact(model_forces[1], MOTOR_FORCE_MAX, label="upper clamp motor 1")
        assert_exact(model_forces[2], MOTOR_FORCE_MAX, label="upper clamp motor 2")
        assert_exact(model_forces[0], 0.0, label="lower clamp motor 0")
    assert_vec_near(rs_forces, py_forces, label="upper saturation parity")


def test_zero_dt_leaves_the_forces_unchanged() -> None:
    py_model, rs_model = MotorModel(0.02), _core.MotorModel(0.02)
    py_before = py_model.step(10.0, np.zeros(3), dt=0.02)
    rs_before = rs_model.step(10.0, [0.0, 0.0, 0.0], dt=0.02)
    py_after = py_model.step(0.0, np.zeros(3), dt=0.0)
    rs_after = rs_model.step(0.0, [0.0, 0.0, 0.0], dt=0.0)
    assert list(py_after) == list(py_before)
    assert list(rs_after) == list(rs_before)  # exact, not merely close
    assert_vec_near(rs_after, py_after, label="dt = 0 parity")


def test_reset_parity() -> None:
    py_model, rs_model = MotorModel(), _core.MotorModel()
    py_model.step(10.0, np.zeros(3), dt=0.1)
    rs_model.step(10.0, [0.0, 0.0, 0.0], dt=0.1)
    assert any(f > 0.0 for f in py_model.forces)
    assert any(f > 0.0 for f in rs_model.forces)
    py_model.reset()
    rs_model.reset()
    assert list(py_model.forces) == [0.0, 0.0, 0.0, 0.0]
    assert list(rs_model.forces) == [0.0, 0.0, 0.0, 0.0]


def test_rust_only_accessors() -> None:
    # time_const mirrors the Rust core surface; the Python module has no
    # accessor, so only the binding side is asserted here.
    model = _core.MotorModel(0.05)
    assert_exact(model.time_const, 0.05, label="time_const")
    assert list(model.forces) == [0.0, 0.0, 0.0, 0.0]
    default_model = _core.MotorModel()
    assert_exact(default_model.time_const, MOTOR_TIME_CONST, label="default time_const")


@pytest.mark.parametrize(
    ("value", "message"),
    [
        pytest.param(0.0, "time_const must be positive, got 0.0", id="zero"),
        pytest.param(-0.02, "time_const must be positive, got -0.02", id="negative"),
        pytest.param(-0.0, "time_const must be positive, got -0.0", id="negative_zero"),
    ],
)
def test_time_const_error_parity(value: float, message: str) -> None:
    with pytest.raises(ValueError) as py_error:
        MotorModel(value)
    with pytest.raises(ValueError) as rust_error:
        _core.MotorModel(value)
    assert_same_error(py_error.value, rust_error.value, label=f"MotorModel({value!r})")
    assert str(rust_error.value) == message


def test_nan_time_const_is_a_registered_divergence() -> None:
    reason = expect_divergence("motors.time_const_nan")
    # Python's guard is `time_const <= 0.0`, and NaN comparisons are all
    # False, so a NaN lag is silently accepted and poisons the forces.
    py_model = MotorModel(float("nan"))
    py_forces = py_model.step(4.0, np.zeros(3), dt=0.01)
    assert all(math.isnan(f) for f in py_forces)
    # The Rust core rejects a NaN lag explicitly; the binding says so with
    # the same message shape the Python module uses for other bad lags.
    with pytest.raises(ValueError, match="time_const must be positive"):
        _core.MotorModel(float("nan"))
    assert reason.startswith("Python MotorModel(float('nan')) is accepted")


def test_negative_dt_is_a_registered_divergence() -> None:
    reason = expect_divergence("motors.step_negative_dt")
    py_model, rs_model = MotorModel(0.02), _core.MotorModel(0.02)
    py_before = py_model.step(4.0, np.zeros(3), dt=0.02)
    rs_before = rs_model.step(4.0, [0.0, 0.0, 0.0], dt=0.02)
    assert_vec_near(rs_before, py_before, label="first step parity")
    with pytest.raises(ValueError, match="dt must be non-negative"):
        py_model.step(0.0, np.zeros(3), dt=-1.0)
    # The Rust core clamps dt to zero: forces unchanged, no exception.
    rs_after = rs_model.step(0.0, [0.0, 0.0, 0.0], dt=-1.0)
    assert list(rs_after) == list(rs_before)
    assert reason.startswith("Python MotorModel.step raises")


def test_nan_dt_is_a_registered_divergence() -> None:
    reason = expect_divergence("motors.step_nan_dt")
    # Python's step guard is `dt < 0.0`, and NaN comparisons are all
    # False, so a NaN dt slips through and every force goes NaN; the
    # model's stored state is poisoned along with the returned tuple.
    py_model, rs_model = MotorModel(0.02), _core.MotorModel(0.02)
    py_before = py_model.step(4.0, np.zeros(3), dt=0.02)
    rs_before = rs_model.step(4.0, [0.0, 0.0, 0.0], dt=0.02)
    assert_vec_near(rs_before, py_before, label="first step parity")
    py_nan = py_model.step(0.0, np.zeros(3), dt=float("nan"))
    assert all(math.isnan(force) for force in py_nan)
    assert all(math.isnan(force) for force in py_model.forces)
    # The Rust core treats a NaN dt like a negative one (`NaN > 0.0` is
    # False): clamped to zero, forces returned and stored unchanged.
    rs_nan = rs_model.step(0.0, [0.0, 0.0, 0.0], dt=float("nan"))
    assert list(rs_nan) == list(rs_before)
    assert list(rs_model.forces) == list(rs_before)
    assert reason.startswith("Python MotorModel.step guards only dt < 0.0")
