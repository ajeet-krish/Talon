from __future__ import annotations

import numpy as np
import pytest

from talon import (
    DT,
    GRAVITY,
    Quaternion,
    State,
    VehicleParams,
    hover_thrust,
    propagate,
    rotation_matrix,
)

# ---------------------------------------------------------------------------
# Pure-Python reference implementation of the same equations, written from
# docs/conventions.md. This catches extension marshalling/sign bugs that a
# self-consistent Rust-only test would miss.
# ---------------------------------------------------------------------------


def _ref_quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def _ref_dcm(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def _ref_derivative(
    y: np.ndarray,
    thrust: float,
    moment: np.ndarray,
    wind: np.ndarray,
    p: dict[str, np.ndarray | float],
) -> np.ndarray:
    """y = [pos(3), quat(4), vel(3), omega(3)]."""
    pos, quat, vel, omega = y[0:3], y[3:7], y[7:10], y[10:13]
    rot = _ref_dcm(quat)
    thrust_body = np.array([0.0, 0.0, -thrust])
    v_rel = vel - wind
    k_drag = 0.5 * p["air_density"] * p["drag_coeff"] * p["vehicle_area"]
    accel = (
        rot @ thrust_body / p["mass"]
        + np.array([0.0, 0.0, p["gravity"]])
        - k_drag * np.linalg.norm(v_rel) * v_rel / p["mass"]
    )
    quat_dot = 0.5 * _ref_quat_multiply(quat, np.array([0.0, *omega]))
    inertia = p["inertia_diag"]
    gyro = np.cross(omega, inertia * omega)
    omega_dot = (moment - gyro) / inertia
    return np.concatenate([vel, quat_dot, accel, omega_dot])


def _ref_propagate(
    state: State,
    thrust: float,
    moment: np.ndarray,
    wind: np.ndarray,
    dt: float,
    p: dict[str, np.ndarray | float],
) -> np.ndarray:
    y = np.concatenate(
        [
            np.asarray(state.position),
            np.array(
                [
                    state.orientation.w,
                    state.orientation.x,
                    state.orientation.y,
                    state.orientation.z,
                ]
            ),
            np.asarray(state.velocity),
            np.asarray(state.angular_rate),
        ]
    )
    k1 = _ref_derivative(y, thrust, moment, wind, p)
    k2 = _ref_derivative(y + 0.5 * dt * k1, thrust, moment, wind, p)
    k3 = _ref_derivative(y + 0.5 * dt * k2, thrust, moment, wind, p)
    k4 = _ref_derivative(y + dt * k3, thrust, moment, wind, p)
    y_next = y + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    y_next[3:7] /= np.linalg.norm(y_next[3:7])
    return y_next


def _params_dict(params: VehicleParams) -> dict[str, np.ndarray | float]:
    return {
        "mass": params.mass,
        "inertia_diag": np.asarray(params.inertia_diag),
        "drag_coeff": params.drag_coeff,
        "gravity": params.gravity,
        "air_density": params.air_density,
        "vehicle_area": params.vehicle_area,
    }


def _random_state(rng: np.random.Generator) -> State:
    state = State()
    state.position = rng.uniform(-50.0, 50.0, 3)
    state.velocity = rng.uniform(-10.0, 10.0, 3)
    state.angular_rate = rng.uniform(-3.0, 3.0, 3)
    q = rng.normal(0.0, 1.0, 4)
    q /= np.linalg.norm(q)
    state.orientation = Quaternion(float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    return state


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_parity_one_step_against_python_reference() -> None:
    rng = np.random.default_rng(2026)
    base = VehicleParams()
    hover = hover_thrust()

    # Default parameters, then a batch with varied parameters to catch
    # VehicleParams marshalling bugs.
    param_sets = [base]
    for _ in range(2):
        p = VehicleParams()
        p.mass = float(rng.uniform(0.8, 2.5))
        p.inertia_diag = rng.uniform(1e-3, 6e-3, 3)
        p.drag_coeff = float(rng.uniform(0.0, 0.6))
        p.air_density = float(rng.uniform(0.8, 1.4))
        p.vehicle_area = float(rng.uniform(0.01, 0.08))
        param_sets.append(p)

    for params in param_sets:
        p_ref = _params_dict(params)
        for _ in range(40):
            state = _random_state(rng)
            thrust = float(rng.uniform(0.0, 2.0 * hover))
            moment = rng.uniform(-0.3, 0.3, 3)
            wind = rng.uniform(-8.0, 8.0, 3)

            cpp = propagate(state, thrust, moment, wind, DT, params)
            ref = _ref_propagate(state, thrust, moment, wind, DT, p_ref)

            np.testing.assert_allclose(
                np.asarray(cpp.position), ref[0:3], rtol=1e-10, atol=1e-10
            )
            np.testing.assert_allclose(
                np.asarray(cpp.velocity), ref[7:10], rtol=1e-10, atol=1e-10
            )
            np.testing.assert_allclose(
                np.asarray(cpp.angular_rate), ref[10:13], rtol=1e-10, atol=1e-10
            )
            q_cpp = np.array(
                [
                    cpp.orientation.w,
                    cpp.orientation.x,
                    cpp.orientation.y,
                    cpp.orientation.z,
                ]
            )
            q_ref = ref[3:7]
            # Quaternions q and -q are the same rotation.
            err = min(np.linalg.norm(q_cpp - q_ref), np.linalg.norm(q_cpp + q_ref))
            assert err < 1e-10


def test_hover_ten_seconds_via_python_api() -> None:
    state = State()
    thrust = hover_thrust()
    zeros = np.zeros(3)
    for _ in range(2000):  # 10 s at DT = 0.005
        state = propagate(state, thrust, zeros, zeros, DT)
    assert np.linalg.norm(np.asarray(state.position)) < 1e-4
    assert np.linalg.norm(np.asarray(state.velocity)) < 1e-4
    assert abs(state.orientation.norm() - 1.0) < 1e-12


def test_rotation_matrix_orthonormality() -> None:
    rng = np.random.default_rng(11)
    for _ in range(100):
        q = rng.normal(0.0, 1.0, 4)
        quat = Quaternion(*(float(v) for v in q)).normalized()
        rot = np.asarray(rotation_matrix(quat))
        np.testing.assert_allclose(rot @ rot.T, np.eye(3), atol=1e-12)
        assert abs(np.linalg.det(rot) - 1.0) < 1e-12


def test_rotate_vector_matches_dcm() -> None:
    rng = np.random.default_rng(23)
    for _ in range(100):
        q = rng.normal(0.0, 1.0, 4)
        quat = Quaternion(*(float(v) for v in q)).normalized()
        v = rng.uniform(-5.0, 5.0, 3)
        via_quat = np.asarray(quat.rotate_vector(v))
        via_dcm = np.asarray(rotation_matrix(quat)) @ v
        np.testing.assert_allclose(via_quat, via_dcm, atol=1e-12)


def test_free_fall_matches_analytic(default_params: VehicleParams) -> None:
    default_params.drag_coeff = 0.0
    state = State()
    zeros = np.zeros(3)
    steps = 400  # 2 s
    for _ in range(steps):
        state = propagate(state, 0.0, zeros, zeros, DT, default_params)
    t = steps * DT
    assert abs(state.position[2] - 0.5 * GRAVITY * t * t) < 1e-6
    assert abs(state.velocity[2] - GRAVITY * t) < 1e-6


def test_hover_thrust_default_params(default_params: VehicleParams) -> None:
    expected = 1.5 * 9.81
    assert hover_thrust() == pytest.approx(expected, abs=1e-12)
    assert hover_thrust(default_params) == pytest.approx(expected, abs=1e-12)
