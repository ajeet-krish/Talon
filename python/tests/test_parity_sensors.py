"""Python vs ``talon._core`` parity for the truth-side sensor models.

Config defaults and setters, the specific-force helper, the GPS NED
mappings and a zero-noise ``SensorSuite`` are exercised on both
implementations. Noisy realisations are deliberately NOT compared:
numpy's ziggurat stream and the core's ``Rng`` are different generators by
design (docs/decisions.md, decision D18), so parity is pinned on the
deterministic parts only: zero noise densities and walks, the bias
initials, the sensor clock, and the exact IEEE field copies of the GPS
mapping. Three registered divergences: the two non-positive ``dt``
policies and the config snapshot policy.
"""

from __future__ import annotations

import numpy as np
import pytest

from parity_util import assert_exact, assert_near, assert_vec_near, expect_divergence
from talon import State, _core
from talon.constants import GRAVITY
from talon.sensors import (
    M_PER_DEG,
    GPSConfig,
    IMUConfig,
    SensorSuite,
    gps_to_ned,
    gps_velocity_ned,
    specific_force_from_velocity,
)
from talon.types import GPSMeasurement

HOVER_ACCEL = [0.0, 0.0, -GRAVITY]
IDENTITY = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def zero_noise_configs() -> tuple:
    py_imu = IMUConfig(
        accel_noise_density=0.0,
        gyro_noise_density=0.0,
        accel_bias_walk=0.0,
        gyro_bias_walk=0.0,
    )
    rs_imu = _core.IMUConfig(
        accel_noise_density=0.0,
        gyro_noise_density=0.0,
        accel_bias_walk=0.0,
        gyro_bias_walk=0.0,
    )
    py_gps = GPSConfig(
        position_noise=0.0, position_noise_vertical=0.0, velocity_noise=0.0
    )
    rs_gps = _core.GPSConfig(
        position_noise=0.0, position_noise_vertical=0.0, velocity_noise=0.0
    )
    return py_imu, rs_imu, py_gps, rs_gps


# ---------------------------------------------------------------------------
# Config defaults and setters
# ---------------------------------------------------------------------------


def test_imu_config_defaults_parity() -> None:
    py_cfg, rs_cfg = IMUConfig(), _core.IMUConfig()
    assert_exact(rs_cfg.accel_noise_density, py_cfg.accel_noise_density)
    assert_exact(rs_cfg.gyro_noise_density, py_cfg.gyro_noise_density)
    assert_exact(rs_cfg.accel_bias_walk, py_cfg.accel_bias_walk)
    assert_exact(rs_cfg.gyro_bias_walk, py_cfg.gyro_bias_walk)
    assert_exact(list(rs_cfg.accel_bias_initial), list(py_cfg.accel_bias_initial))
    assert_exact(list(rs_cfg.gyro_bias_initial), list(py_cfg.gyro_bias_initial))
    # And the defaults are the documented literals, bit for bit.
    assert_exact(py_cfg.accel_noise_density, 0.004)
    assert_exact(py_cfg.gyro_noise_density, 0.003)
    assert_exact(list(py_cfg.accel_bias_initial), [0.05, -0.03, 0.02])
    assert_exact(list(py_cfg.gyro_bias_initial), [0.001, -0.002, 0.0015])
    assert_exact(py_cfg.accel_bias_walk, 5e-4)
    assert_exact(py_cfg.gyro_bias_walk, 1e-5)


def test_gps_config_defaults_parity() -> None:
    py_cfg, rs_cfg = GPSConfig(), _core.GPSConfig()
    assert_exact(rs_cfg.position_noise, py_cfg.position_noise)
    assert_exact(rs_cfg.position_noise_vertical, py_cfg.position_noise_vertical)
    assert_exact(rs_cfg.velocity_noise, py_cfg.velocity_noise)
    assert_exact(rs_cfg.update_rate, py_cfg.update_rate)
    assert_exact(py_cfg.position_noise, 1.5)
    assert_exact(py_cfg.position_noise_vertical, 3.0)
    assert_exact(py_cfg.velocity_noise, 0.1)
    assert_exact(py_cfg.update_rate, 10.0)


def test_imu_config_constructor_and_setter_parity() -> None:
    py_cfg = IMUConfig(accel_noise_density=0.01, accel_bias_walk=0.0)
    rs_cfg = _core.IMUConfig(accel_noise_density=0.01, accel_bias_walk=0.0)
    assert_exact(rs_cfg.accel_noise_density, py_cfg.accel_noise_density)
    assert_exact(rs_cfg.accel_bias_walk, py_cfg.accel_bias_walk)
    # Unset fields keep the defaults on both sides.
    assert_exact(rs_cfg.gyro_noise_density, py_cfg.gyro_noise_density)

    py_cfg.accel_bias_initial = (1.0, 2.0, 3.0)
    rs_cfg.accel_bias_initial = [1.0, 2.0, 3.0]
    assert_exact(list(rs_cfg.accel_bias_initial), list(py_cfg.accel_bias_initial))
    py_cfg.gyro_bias_walk = 2e-5
    rs_cfg.gyro_bias_walk = 2e-5
    assert_exact(rs_cfg.gyro_bias_walk, py_cfg.gyro_bias_walk)


def test_gps_config_setter_parity() -> None:
    py_cfg, rs_cfg = GPSConfig(), _core.GPSConfig()
    py_cfg.velocity_noise = 0.25
    rs_cfg.velocity_noise = 0.25
    py_cfg.update_rate = 5.0
    rs_cfg.update_rate = 5.0
    assert_exact(rs_cfg.velocity_noise, py_cfg.velocity_noise)
    assert_exact(rs_cfg.update_rate, py_cfg.update_rate)


# ---------------------------------------------------------------------------
# specific_force_from_velocity and the GPS NED mappings
# ---------------------------------------------------------------------------


def test_specific_force_hover_parity() -> None:
    # Level, not accelerating: both sides read [0, 0, -g].
    py_force = specific_force_from_velocity(np.zeros(3), np.zeros(3), np.eye(3), 0.005)
    rs_force = _core.specific_force_from_velocity(
        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], IDENTITY, 0.005
    )
    assert_near(py_force, HOVER_ACCEL, label="python hover specific force")
    assert_near(rs_force, HOVER_ACCEL, label="rust hover specific force")
    assert_vec_near(rs_force, py_force, label="hover parity")


def test_specific_force_general_case_parity() -> None:
    # Level attitude, NED acceleration [1, -2, 3] over 5 ms:
    # f_body = a_ned - g_ned.
    a_ned = [1.0, -2.0, 3.0]
    dt = 0.005
    v_curr = [a * dt for a in a_ned]
    py_force = specific_force_from_velocity(
        np.zeros(3), np.asarray(v_curr), np.eye(3), dt
    )
    rs_force = _core.specific_force_from_velocity(
        [0.0, 0.0, 0.0], v_curr, IDENTITY, dt
    )
    expected = [1.0, -2.0, 3.0 - GRAVITY]
    assert_near(py_force, expected, label="python expected force")
    assert_near(rs_force, expected, label="rust expected force")
    assert_vec_near(rs_force, py_force, label="general parity")


def test_specific_force_yawed_attitude_parity() -> None:
    # Body +x points east (yaw +90 deg about down):
    # R = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]], so f_body = R^T (a_ned - g_ned)
    # = [-a_east, a_north, a_down - g].
    rotation = [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    a_ned = [2.0, -1.0, 0.5]
    dt = 0.005
    v_curr = [a * dt for a in a_ned]
    py_force = specific_force_from_velocity(
        np.zeros(3), np.asarray(v_curr), np.asarray(rotation), dt
    )
    rs_force = _core.specific_force_from_velocity([0.0, 0.0, 0.0], v_curr, rotation, dt)
    expected = [1.0, 2.0, 0.5 - GRAVITY]
    assert_near(py_force, expected, label="python yawed force")
    assert_near(rs_force, expected, label="rust yawed force")
    assert_vec_near(rs_force, py_force, label="yawed parity")


def _sample_gps() -> GPSMeasurement:
    return GPSMeasurement(
        timestamp=1.0, lat=0.001, lon=-0.002, alt=12.0, vn=3.0, ve=-1.0, vd=0.5
    )


def test_gps_to_ned_parity() -> None:
    gps = _sample_gps()
    py_ned = gps_to_ned(gps)
    rs_ned = _core.gps_to_ned(gps)
    # The flat-Earth mapping is one multiply (and one negate) per axis on
    # both sides, so the copies are bit-exact.
    assert_exact(list(rs_ned), list(py_ned), label="gps_to_ned parity")
    assert_exact(
        list(py_ned),
        [0.001 * M_PER_DEG, -0.002 * M_PER_DEG, -12.0],
        label="gps_to_ned known values",
    )


def test_gps_velocity_ned_parity() -> None:
    gps = _sample_gps()
    py_velocity = gps_velocity_ned(gps)
    rs_velocity = _core.gps_velocity_ned(gps)
    assert_exact(list(rs_velocity), list(py_velocity), label="gps_velocity_ned parity")
    assert_exact(list(py_velocity), [3.0, -1.0, 0.5])


def test_broken_optional_gps_property_is_propagated() -> None:
    # Security review close-out: the duck-typed GPS reader defaults the
    # optional timestamp only on AttributeError. A timestamp that exists
    # but raises on access must propagate, not silently read as 0.0. The
    # mapping helpers ignore the timestamp either way, so only the
    # binding side is pinned here (the Python functions read only the
    # lat/lon/alt/vn/ve/vd fields).
    class BrokenTimestampGps:
        lat, lon, alt, vn, ve, vd = 0.001, -0.002, 12.0, 3.0, -1.0, 0.5

        @property
        def timestamp(self) -> float:
            raise RuntimeError("timestamp read failed")

    broken = BrokenTimestampGps()
    with pytest.raises(RuntimeError, match="timestamp read failed"):
        _core.gps_to_ned(broken)
    with pytest.raises(RuntimeError, match="timestamp read failed"):
        _core.gps_velocity_ned(broken)

    # A genuinely absent timestamp still defaults (to 0.0) and the
    # mapping is unaffected.
    class NoTimestampGps:
        lat, lon, alt, vn, ve, vd = 0.001, -0.002, 12.0, 3.0, -1.0, 0.5

    gps = NoTimestampGps()
    assert_exact(
        list(_core.gps_to_ned(gps)),
        [0.001 * M_PER_DEG, -0.002 * M_PER_DEG, -12.0],
        label="absent timestamp defaults",
    )
    assert_exact(list(_core.gps_velocity_ned(gps)), [3.0, -1.0, 0.5])


# ---------------------------------------------------------------------------
# Zero-noise SensorSuite: the deterministic half of the generator
# ---------------------------------------------------------------------------


def test_zero_noise_suite_parity() -> None:
    # All noise densities and walks zero, identical bias initials, one
    # seed on both sides. The RNG streams differ by design (decision D18),
    # but with zero densities and walks no draw can reach the outputs, so
    # every sample must agree exactly: accel = f_true + bias,
    # gyro = omega + bias, GPS equals truth.
    py_imu, rs_imu, py_gps_cfg, rs_gps_cfg = zero_noise_configs()
    py_suite = SensorSuite(py_imu, py_gps_cfg, seed=7)
    rs_suite = _core.SensorSuite(rs_imu, rs_gps_cfg, seed=7)

    accel_bias = list(py_imu.accel_bias_initial)
    gyro_bias = list(py_imu.gyro_bias_initial)
    omega = [0.1, 0.2, 0.3]
    state = State()
    state.angular_rate = np.asarray(omega)
    dt = 0.005
    truth_position = [10.0, -4.0, -12.0]
    truth_velocity = [2.0, 1.0, 0.0]
    state.position = np.asarray(truth_position)
    state.velocity = np.asarray(truth_velocity)

    for _ in range(25):
        py_meas = py_suite.generate_imu(state, HOVER_ACCEL, dt)
        rs_meas = rs_suite.generate_imu(omega, HOVER_ACCEL, dt)
        expected_accel = [f + b for f, b in zip(HOVER_ACCEL, accel_bias)]
        expected_gyro = [w + b for w, b in zip(omega, gyro_bias)]
        assert_exact(list(py_meas.accel), expected_accel, label="python accel")
        assert_exact(list(rs_meas.accel), expected_accel, label="rust accel")
        assert_exact(list(py_meas.gyro), expected_gyro, label="python gyro")
        assert_exact(list(rs_meas.gyro), expected_gyro, label="rust gyro")
        assert_exact(list(rs_meas.accel), list(py_meas.accel), label="accel parity")
        assert_exact(list(rs_meas.gyro), list(py_meas.gyro), label="gyro parity")
        assert_exact(rs_meas.timestamp, py_meas.timestamp, label="imu timestamp")

        py_fix = py_suite.generate_gps(state)
        rs_fix = rs_suite.generate_gps(truth_position, truth_velocity)
        # The noisy position is truth + 0.0 * draw, so the degree fields
        # are the truth divided by M_PER_DEG: correctly rounded IEEE
        # division, bit-identical across the two languages.
        assert_exact(rs_fix.lat, py_fix.lat, label="gps lat parity")
        assert_exact(rs_fix.lon, py_fix.lon, label="gps lon parity")
        assert_exact(rs_fix.alt, py_fix.alt, label="gps alt parity")
        assert_exact(py_fix.lat, truth_position[0] / M_PER_DEG)
        assert_exact(py_fix.lon, truth_position[1] / M_PER_DEG)
        assert_exact(py_fix.alt, -truth_position[2])
        assert_exact(
            [rs_fix.vn, rs_fix.ve, rs_fix.vd], [py_fix.vn, py_fix.ve, py_fix.vd]
        )
        assert_exact([py_fix.vn, py_fix.ve, py_fix.vd], truth_velocity)
        assert_exact(rs_fix.timestamp, py_fix.timestamp, label="gps timestamp")

    # With zero walks the stored true biases never move on either side.
    assert_exact(list(py_suite.accel_bias), accel_bias)
    assert_exact(list(rs_suite.accel_bias), accel_bias)
    assert_exact(list(py_suite.gyro_bias), gyro_bias)
    assert_exact(list(rs_suite.gyro_bias), gyro_bias)
    assert_exact(rs_suite.time, py_suite.time)


def test_bias_accessors_after_steps_parity() -> None:
    # Zero-walk configs: after N IMU steps both accessors still report the
    # configured initial biases, bit for bit.
    py_imu, rs_imu, py_gps_cfg, rs_gps_cfg = zero_noise_configs()
    py_suite = SensorSuite(py_imu, py_gps_cfg, seed=11)
    rs_suite = _core.SensorSuite(rs_imu, rs_gps_cfg, seed=11)
    state = State()
    state.angular_rate = np.asarray([0.0, 0.1, 0.0])
    for _ in range(50):
        py_suite.generate_imu(state, HOVER_ACCEL, 0.005)
        rs_suite.generate_imu([0.0, 0.1, 0.0], HOVER_ACCEL, 0.005)
    assert_exact(list(py_suite.accel_bias), list(py_imu.accel_bias_initial))
    assert_exact(list(rs_suite.accel_bias), list(py_imu.accel_bias_initial))
    assert_exact(list(py_suite.gyro_bias), list(py_imu.gyro_bias_initial))
    assert_exact(list(rs_suite.gyro_bias), list(py_imu.gyro_bias_initial))
    assert_exact(rs_suite.time, py_suite.time)


def test_bias_random_walk_drifts_on_each_side() -> None:
    # Default (nonzero) walks: each side's stored true bias drifts with
    # its OWN generator. The two realisations are deliberately not
    # compared with each other (decision D18: the RNG streams differ by
    # design); only the per-side statistical bound is pinned, mirroring
    # core/src/sensors.rs.
    py_suite = SensorSuite(IMUConfig(), GPSConfig(), seed=6252)
    rs_suite = _core.SensorSuite(_core.IMUConfig(), _core.GPSConfig(), seed=6252)
    state = State()
    accel_initial = np.asarray(py_suite.accel_bias)
    gyro_initial = np.asarray(py_suite.gyro_bias)
    rs_accel_initial = list(rs_suite.accel_bias)
    rs_gyro_initial = list(rs_suite.gyro_bias)
    for _ in range(500):  # 2.5 s at dt = 0.005
        py_suite.generate_imu(state, HOVER_ACCEL, 0.005)
        rs_suite.generate_imu([0.0, 0.0, 0.0], HOVER_ACCEL, 0.005)
    py_drift = np.linalg.norm(np.asarray(py_suite.accel_bias) - accel_initial)
    assert 0.0 < py_drift < 0.05, py_drift
    rs_drift = np.linalg.norm(
        np.asarray(rs_suite.accel_bias) - np.asarray(rs_accel_initial)
    )
    assert 0.0 < rs_drift < 0.05, rs_drift
    py_gyro_drift = np.linalg.norm(np.asarray(py_suite.gyro_bias) - gyro_initial)
    assert 0.0 < py_gyro_drift < 0.001, py_gyro_drift
    rs_gyro_drift = np.linalg.norm(
        np.asarray(rs_suite.gyro_bias) - np.asarray(rs_gyro_initial)
    )
    assert 0.0 < rs_gyro_drift < 0.001, rs_gyro_drift


def test_clock_advances_by_dt_parity() -> None:
    py_suite = SensorSuite(seed=3)
    rs_suite = _core.SensorSuite(seed=3)
    state = State()
    for _ in range(10):
        py_suite.generate_imu(state, HOVER_ACCEL, 0.005)
        rs_suite.generate_imu([0.0, 0.0, 0.0], HOVER_ACCEL, 0.005)
    assert_exact(rs_suite.time, py_suite.time, label="clock parity")
    assert_near(py_suite.time, 0.05, label="python clock")
    assert_near(rs_suite.time, 0.05, label="rust clock")


# ---------------------------------------------------------------------------
# Registered divergences
# ---------------------------------------------------------------------------


def test_specific_force_non_positive_dt_is_a_registered_divergence() -> None:
    reason = expect_divergence("sensors.specific_force_non_positive_dt")
    with pytest.raises(ValueError, match="dt must be positive"):
        specific_force_from_velocity(np.zeros(3), np.ones(3), np.eye(3), 0.0)
    # The Rust core yields the zero vector rather than raising
    # (core/src/sensors.rs), and the binding mirrors the core.
    zero = _core.specific_force_from_velocity(
        [0.0, 0.0, 0.0], [1.0, 1.0, 1.0], IDENTITY, 0.0
    )
    assert list(zero) == [0.0, 0.0, 0.0]
    assert reason.startswith("Python specific_force_from_velocity raises ValueError")


def test_generate_imu_non_positive_dt_is_a_registered_divergence() -> None:
    reason = expect_divergence("sensors.generate_imu_non_positive_dt")
    py_suite = SensorSuite(seed=5)
    with pytest.raises(ValueError, match="dt must be positive"):
        py_suite.generate_imu(State(), HOVER_ACCEL, -0.1)
    # The Rust core returns a zero sample and does not advance the clock.
    rs_suite = _core.SensorSuite(seed=5)
    measurement = rs_suite.generate_imu([0.0, 0.0, 0.0], HOVER_ACCEL, -0.1)
    assert list(measurement.accel) == [0.0, 0.0, 0.0]
    assert list(measurement.gyro) == [0.0, 0.0, 0.0]
    assert rs_suite.time == 0.0
    assert reason.startswith("Python SensorSuite.generate_imu raises ValueError")


def test_config_is_a_snapshot_divergence() -> None:
    reason = expect_divergence("sensors.config_is_a_snapshot")
    # Python re-reads the passed config object every call; the binding
    # copies it at construction (the same snapshot policy as
    # NLGLGuidance.config), so the post-construction write only reaches
    # the Python suite.
    py_cfg, rs_cfg = IMUConfig(), _core.IMUConfig()
    py_suite = SensorSuite(py_cfg, seed=1)
    rs_suite = _core.SensorSuite(rs_cfg, seed=1)
    py_cfg.accel_noise_density = 0.0
    rs_cfg.accel_noise_density = 0.0
    py_meas = py_suite.generate_imu(State(), HOVER_ACCEL, 0.005)
    rs_meas = rs_suite.generate_imu([0.0, 0.0, 0.0], HOVER_ACCEL, 0.005)
    bias = list(IMUConfig().accel_bias_initial)
    expected = [f + b for f, b in zip(HOVER_ACCEL, bias)]
    assert_exact(list(py_meas.accel), expected, label="python follows the write")
    assert not np.allclose(rs_meas.accel, expected), (
        "the binding snapshot must keep the constructor-time density"
    )
    assert reason.startswith("Python SensorSuite keeps the passed IMUConfig")
