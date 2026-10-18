"""Sensor measurement models for the Phase 3 INS/GPS stack.

Truth-side models used by SIL tests and scenarios: the EKF itself lives in
Rust (core/src/ekf.rs, exposed via talon._core); this module only
GENERATES measurements from a true vehicle state, so its sign conventions
must match docs/conventions.md exactly:

- IMU accel is the specific force in body frame. In hover it reads
  [0, 0, -g] (the reaction against NED gravity g_ned = [0, 0, +g]). The SIL
  caller is responsible for producing the true specific force: either from
  the thrust model (f_body = [0, 0, -T/m] without drag) or, robustly with
  drag and any acceleration, by finite-differencing the true NED velocity
  through :func:`specific_force_from_velocity`.
- IMU gyro is the true body angular rate plus a true (simulated) bias plus
  white noise. The true bias itself random-walks each IMU step, so the
  EKF's bias-estimation path is exercised against a realistic target.
- Noise is converted from continuous densities the standard way:
  sigma_sample = density / sqrt(dt) (density * sqrt(sample rate)).

GPS reuses the Phase 1 :class:`~talon.types.GPSMeasurement` (lat/lon in
degrees, alt in m, NED velocity). Phase 3 is a flat-Earth sim with the
origin at lat = lon = 0: north (m) = lat * M_PER_DEG, east (m) =
lon * M_PER_DEG, down (m) = -alt. Noise is applied in METERS in the local
NED frame before the degenerate degree conversion, so the horizontal /
vertical sigmas in :class:`GPSConfig` are directly in meters; use
:func:`gps_to_ned` / :func:`gps_velocity_ned` to hand NED arrays to
``Ekf.update``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from talon.constants import GRAVITY
from talon.types import GPSMeasurement, IMUMeasurement

# Flat-Earth meters-per-degree at the (degenerate) origin latitude. Only the
# ratio matters: noise is applied in meters, converted to degrees for the
# Phase 1 GPSMeasurement fields, and converted straight back by gps_to_ned.
M_PER_DEG = 111320.0

_G_NED = np.array([0.0, 0.0, GRAVITY], dtype=np.float64)


@dataclass(slots=True)
class IMUConfig:
    """IMU noise / bias parameters.

    The *_initial biases are the TRUE biases of the simulated sensor at
    t = 0 (the SensorSuite truth model); the EKF's own (separate) bias
    estimates start at zero in EKFConfig. The *_walk values drive the
    per-axis random walk of the true bias, in units per sqrt(s).
    """

    accel_noise_density: float = 0.004
    gyro_noise_density: float = 0.003
    accel_bias_initial: tuple[float, float, float] = (0.05, -0.03, 0.02)  # m/s^2
    gyro_bias_initial: tuple[float, float, float] = (0.001, -0.002, 0.0015)  # rad/s
    accel_bias_walk: float = 5e-4
    gyro_bias_walk: float = 1e-5


@dataclass(slots=True)
class GPSConfig:
    """GPS noise parameters (1-sigma, METERS / m/s in local NED)."""

    position_noise: float = 1.5
    position_noise_vertical: float = 3.0
    velocity_noise: float = 0.1
    update_rate: float = 10.0


def specific_force_from_velocity(
    v_prev_ned: ArrayLike,
    v_curr_ned: ArrayLike,
    rotation_body_to_ned: ArrayLike,
    dt: float,
) -> NDArray[np.float64]:
    """True body-frame specific force from finite-differenced true velocity.

    Robust SIL approach that works with drag, thrust, and any acceleration:
    a_ned = (v_curr - v_prev) / dt, then f_body = R^T (a_ned - g_ned) with
    R the body-to-NED DCM (docs/conventions.md). In hover this returns
    [0, 0, -g] up to the finite-difference error.
    """
    if dt <= 0.0:
        raise ValueError(f"dt must be positive, got {dt}")
    v_prev = np.asarray(v_prev_ned, dtype=np.float64)
    v_curr = np.asarray(v_curr_ned, dtype=np.float64)
    R = np.asarray(rotation_body_to_ned, dtype=np.float64)
    if v_prev.shape != (3,) or v_curr.shape != (3,):
        raise ValueError("velocities must have shape (3,)")
    if R.shape != (3, 3):
        raise ValueError(f"rotation_body_to_ned must have shape (3, 3), got {R.shape}")
    a_ned = (v_curr - v_prev) / dt
    return R.T @ (a_ned - _G_NED)


def gps_to_ned(gps: GPSMeasurement) -> NDArray[np.float64]:
    """GPS fix -> local NED position [n, e, d] (m), flat-Earth mapping."""
    return np.array(
        [
            gps.lat * M_PER_DEG,
            gps.lon * M_PER_DEG,
            -gps.alt,
        ],
        dtype=np.float64,
    )


def gps_velocity_ned(gps: GPSMeasurement) -> NDArray[np.float64]:
    """GPS fix -> NED velocity [vn, ve, vd] (m/s)."""
    return np.array([gps.vn, gps.ve, gps.vd], dtype=np.float64)


class SensorSuite:
    """Seeded IMU + GPS truth-side measurement generator.

    Holds the TRUE bias states (initialized from :class:`IMUConfig`,
    random-walked on every ``generate_imu`` call); tests assert against
    ``accel_bias`` / ``gyro_bias`` accessors, NOT against the EKF config.
    One instance = one sensor unit; reproducible given the same seed and
    call sequence.
    """

    def __init__(
        self,
        imu_config: IMUConfig | None = None,
        gps_config: GPSConfig | None = None,
        seed: int = 0,
    ) -> None:
        self._imu_config = imu_config if imu_config is not None else IMUConfig()
        self._gps_config = gps_config if gps_config is not None else GPSConfig()
        self._rng = np.random.default_rng(seed)
        self._accel_bias = np.asarray(
            self._imu_config.accel_bias_initial, dtype=np.float64
        )
        self._gyro_bias = np.asarray(
            self._imu_config.gyro_bias_initial, dtype=np.float64
        )
        self._time = 0.0

    @property
    def accel_bias(self) -> NDArray[np.float64]:
        """Current TRUE accelerometer bias (m/s^2, body)."""
        return self._accel_bias.copy()

    @property
    def gyro_bias(self) -> NDArray[np.float64]:
        """Current TRUE gyro bias (rad/s, body)."""
        return self._gyro_bias.copy()

    @property
    def time(self) -> float:
        """Internal sensor clock (s), advanced by generate_imu."""
        return self._time

    def generate_imu(
        self,
        true_state: object,
        true_specific_force_body: ArrayLike,
        dt: float,
    ) -> IMUMeasurement:
        """One IMU sample from the true vehicle state.

        true_state: the truth ``talon.State`` (its angular_rate is the true
        body rate); true_specific_force_body: the caller-supplied true
        specific force in body frame, e.g. from
        :func:`specific_force_from_velocity`. Output:
        accel = f_true + true_bias + sigma_a/sqrt(dt) * N(0, 1) (same for
        gyro); the TRUE biases then random-walk by walk * sqrt(dt) * N(0,1).
        """
        if dt <= 0.0:
            raise ValueError(f"dt must be positive, got {dt}")
        f_true = np.asarray(true_specific_force_body, dtype=np.float64)
        if f_true.shape != (3,):
            raise ValueError(f"true_specific_force_body must have shape (3,)")
        omega_true = np.asarray(true_state.angular_rate, dtype=np.float64)

        accel_sigma = self._imu_config.accel_noise_density / math.sqrt(dt)
        gyro_sigma = self._imu_config.gyro_noise_density / math.sqrt(dt)
        accel = f_true + self._accel_bias + accel_sigma * self._rng.standard_normal(3)
        gyro = omega_true + self._gyro_bias + gyro_sigma * self._rng.standard_normal(3)

        # True bias random walk (units per sqrt(s) * sqrt(dt)).
        walk_dt = math.sqrt(dt)
        self._accel_bias += (
            self._imu_config.accel_bias_walk * walk_dt * self._rng.standard_normal(3)
        )
        self._gyro_bias += (
            self._imu_config.gyro_bias_walk * walk_dt * self._rng.standard_normal(3)
        )

        measurement = IMUMeasurement(timestamp=self._time, accel=accel, gyro=gyro)
        self._time += dt
        return measurement

    def generate_gps(self, true_state: object) -> GPSMeasurement:
        """One GPS fix from the true vehicle state (noise applied in meters)."""
        position = np.asarray(true_state.position, dtype=np.float64)
        velocity = np.asarray(true_state.velocity, dtype=np.float64)
        pos_sigma = np.array(
            [
                self._gps_config.position_noise,
                self._gps_config.position_noise,
                self._gps_config.position_noise_vertical,
            ],
            dtype=np.float64,
        )
        noisy_position = position + pos_sigma * self._rng.standard_normal(3)
        noisy_velocity = (
            velocity
            + self._gps_config.velocity_noise * self._rng.standard_normal(3)
        )
        return GPSMeasurement(
            timestamp=self._time,
            lat=float(noisy_position[0] / M_PER_DEG),
            lon=float(noisy_position[1] / M_PER_DEG),
            alt=float(-noisy_position[2]),
            vn=float(noisy_velocity[0]),
            ve=float(noisy_velocity[1]),
            vd=float(noisy_velocity[2]),
        )


__all__ = [
    "GPSConfig",
    "IMUConfig",
    "M_PER_DEG",
    "SensorSuite",
    "gps_to_ned",
    "gps_velocity_ned",
    "specific_force_from_velocity",
]
