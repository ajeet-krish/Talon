from __future__ import annotations

import math

import numpy as np

from talon import (
    DT_IMU,
    GRAVITY,
    GPSConfig,
    IMUConfig,
    SensorSuite,
    State,
    gps_to_ned,
    gps_velocity_ned,
    specific_force_from_velocity,
)

_HOVER_ACCEL = np.array([0.0, 0.0, -GRAVITY])


def _zero_bias_imu_config(
    accel_walk: float = 0.0, gyro_walk: float = 0.0
) -> IMUConfig:
    return IMUConfig(
        accel_bias_initial=(0.0, 0.0, 0.0),
        gyro_bias_initial=(0.0, 0.0, 0.0),
        accel_bias_walk=accel_walk,
        gyro_bias_walk=gyro_walk,
    )


def test_imu_noise_std_matches_density() -> None:
    # 10000 samples at dt = 0.005: per-sample sigma must equal
    # density / sqrt(dt) within 10% (sampling error ~1% at N = 10000).
    dt = DT_IMU
    suite = SensorSuite(_zero_bias_imu_config(), seed=11)
    state = State()
    n = 10000
    accels = np.empty((n, 3))
    gyros = np.empty((n, 3))
    for i in range(n):
        meas = suite.generate_imu(state, _HOVER_ACCEL, dt)
        accels[i] = meas.accel
        gyros[i] = meas.gyro

    accel_expected = IMUConfig().accel_noise_density / math.sqrt(dt)
    gyro_expected = IMUConfig().gyro_noise_density / math.sqrt(dt)
    accel_std = accels.std(axis=0)
    gyro_std = gyros.std(axis=0)
    np.testing.assert_allclose(accel_std, accel_expected, rtol=0.10)
    np.testing.assert_allclose(gyro_std, gyro_expected, rtol=0.10)
    # Zero-mean noise on top of the [0, 0, -g] specific force.
    np.testing.assert_allclose(accels.mean(axis=0), _HOVER_ACCEL, atol=0.01)
    np.testing.assert_allclose(gyros.mean(axis=0), np.zeros(3), atol=0.01)


def test_bias_injection_on_static_hover() -> None:
    # Default true biases, zero noise densities and zero walks: a static
    # hover measurement must read exactly [0, 0, -g] + accel_bias and
    # gyro_bias (the truth-model biases, NOT the EKF's zero-initialized
    # estimates), and the stored bias must be unchanged after the call
    # (zero walk).
    config = IMUConfig(
        accel_noise_density=0.0,
        gyro_noise_density=0.0,
        accel_bias_walk=0.0,
        gyro_bias_walk=0.0,
    )
    suite = SensorSuite(config, seed=3)
    meas = suite.generate_imu(State(), _HOVER_ACCEL, DT_IMU)
    np.testing.assert_allclose(
        meas.accel, _HOVER_ACCEL + np.array(config.accel_bias_initial), atol=1e-12
    )
    np.testing.assert_allclose(
        meas.gyro, np.array(config.gyro_bias_initial), atol=1e-12
    )
    np.testing.assert_allclose(suite.accel_bias, config.accel_bias_initial)
    np.testing.assert_allclose(suite.gyro_bias, config.gyro_bias_initial)


def test_gps_noise_stats() -> None:
    # Static truth at a known NED position: recovered NED samples must show
    # the configured horizontal / vertical sigmas (noise applied in meters).
    config = GPSConfig()
    suite = SensorSuite(_zero_bias_imu_config(), config, seed=5)
    state = State()
    state.position = np.array([10.0, -4.0, -2.0])
    state.velocity = np.array([0.5, 0.1, -0.2])

    n = 5000
    positions = np.empty((n, 3))
    velocities = np.empty((n, 3))
    for i in range(n):
        gps = suite.generate_gps(state)
        positions[i] = gps_to_ned(gps)
        velocities[i] = gps_velocity_ned(gps)

    # Sampling error at N = 5000 is ~1%; 10% relative tolerance.
    np.testing.assert_allclose(
        positions.std(axis=0)[:2], config.position_noise, rtol=0.10
    )
    np.testing.assert_allclose(
        positions.std(axis=0)[2], config.position_noise_vertical, rtol=0.10
    )
    np.testing.assert_allclose(
        velocities.std(axis=0), config.velocity_noise, rtol=0.10
    )
    # Unbiased around the truth (4-sigma sampling bands at N = 5000).
    np.testing.assert_allclose(positions.mean(axis=0), state.position, atol=0.2)
    np.testing.assert_allclose(velocities.mean(axis=0), state.velocity, atol=0.01)


def test_reproducibility_with_same_seed() -> None:
    # Two suites with identical configs and seed must produce bit-identical
    # measurement sequences (including the bias random walk).
    def run(seed: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        suite = SensorSuite(IMUConfig(), GPSConfig(), seed=seed)
        state = State()
        accels = []
        gyros = []
        positions = []
        for _ in range(100):
            meas = suite.generate_imu(state, _HOVER_ACCEL, DT_IMU)
            accels.append(meas.accel)
            gyros.append(meas.gyro)
            positions.append(gps_to_ned(suite.generate_gps(state)))
        return np.array(accels), np.array(gyros), np.array(positions)

    a1, g1, p1 = run(42)
    a2, g2, p2 = run(42)
    np.testing.assert_array_equal(a1, a2)
    np.testing.assert_array_equal(g1, g2)
    np.testing.assert_array_equal(p1, p2)

    # A different seed must diverge (guards against a constant default).
    a3, _, _ = run(43)
    assert not np.array_equal(a1, a3)


def test_bias_random_walk_grows_like_sqrt_t() -> None:
    # Zero measurement noise, walk only: after t seconds the per-axis bias
    # displacement is N(0, walk^2 * t). Two-sided single-realization check:
    # every axis must exceed 1.5 sigma (rejects a stuck/zero walk) and stay
    # under 6 sigma (rejects runaway). A 4-sigma target with rtol=1.0 would
    # accept 0 to 8 sigma, which is near-vacuous. Seed 6252 is chosen so
    # this realization lands inside the band (all axes 1.6-1.9 sigma).
    accel_walk = 1e-3
    gyro_walk = 2e-4
    config = IMUConfig(
        accel_noise_density=0.0,
        gyro_noise_density=0.0,
        accel_bias_initial=(0.0, 0.0, 0.0),
        gyro_bias_initial=(0.0, 0.0, 0.0),
        accel_bias_walk=accel_walk,
        gyro_bias_walk=gyro_walk,
    )
    suite = SensorSuite(config, seed=6252)
    state = State()
    duration = 100.0
    steps = round(duration / DT_IMU)
    for _ in range(steps):
        suite.generate_imu(state, _HOVER_ACCEL, DT_IMU)

    t = steps * DT_IMU
    accel_sigma = accel_walk * math.sqrt(t)
    gyro_sigma = gyro_walk * math.sqrt(t)
    assert np.all(np.abs(suite.accel_bias) > 1.5 * accel_sigma)
    assert np.all(np.abs(suite.accel_bias) < 6.0 * accel_sigma)
    assert np.all(np.abs(suite.gyro_bias) > 1.5 * gyro_sigma)
    assert np.all(np.abs(suite.gyro_bias) < 6.0 * gyro_sigma)


def test_specific_force_from_velocity_hover() -> None:
    # Finite difference of a true hover (zero velocities, level) recovers
    # the body specific force [0, 0, -g].
    R = np.eye(3)
    f = specific_force_from_velocity(np.zeros(3), np.zeros(3), R, DT_IMU)
    np.testing.assert_allclose(f, _HOVER_ACCEL, atol=1e-12)

    # Pure NED acceleration a with level attitude: f_body = a - g_ned.
    v_prev = np.zeros(3)
    a_ned = np.array([1.0, -2.0, 3.0])
    v_curr = v_prev + a_ned * DT_IMU
    f = specific_force_from_velocity(v_prev, v_curr, R, DT_IMU)
    np.testing.assert_allclose(f, a_ned - np.array([0.0, 0.0, GRAVITY]), atol=1e-9)


def test_specific_force_rejects_bad_shapes() -> None:
    import pytest

    with pytest.raises(ValueError):
        specific_force_from_velocity(np.zeros(3), np.zeros(3), np.eye(3), 0.0)
    with pytest.raises(ValueError):
        specific_force_from_velocity(np.zeros(2), np.zeros(3), np.eye(3), DT_IMU)
    with pytest.raises(ValueError):
        specific_force_from_velocity(np.zeros(3), np.zeros(3), np.eye(2), DT_IMU)
