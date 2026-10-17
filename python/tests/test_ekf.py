from __future__ import annotations

import math

import numpy as np

from talon import (
    DT,
    DT_CONTROL,
    DT_GPS,
    DT_IMU,
    GRAVITY,
    GPSConfig,
    CascadedController,
    ControlGains,
    EKFConfig,
    EKFNominalState,
    Ekf,
    IMUConfig,
    Quaternion,
    SensorSuite,
    State,
    VehicleParams,
    gps_to_ned,
    gps_velocity_ned,
    hover_thrust,
    propagate,
    specific_force_from_velocity,
)
from talon.motors import MotorModel, forward_mix

_DEG = math.pi / 180.0
_HOVER_ACCEL = np.array([0.0, 0.0, -GRAVITY])


def _quiet_config() -> EKFConfig:
    """Low-noise EKF config for deterministic perfect-sensor tests.

    Mirrors the C++ quiet_config() in core/tests/test_ekf.cpp so the Python
    convergence checks do not depend on the noise floor.
    """
    config = EKFConfig()
    config.accel_noise_density = 1e-4
    config.gyro_noise_density = 1e-4
    config.accel_bias_walk = 1e-8
    config.gyro_bias_walk = 1e-8
    config.gps_pos_noise = 0.05
    config.gps_pos_noise_vertical = 0.05
    config.gps_vel_noise = 0.01
    return config


def test_truth_injection_position_convergence() -> None:
    # Wrong initial position (5 m class offset), perfect hover IMU and
    # perfect GPS at the origin: position error < 0.1 m after 10 s.
    ekf = Ekf(_quiet_config())
    init = EKFNominalState()
    init.position = np.array([5.0, -3.0, -12.0])
    init.velocity = np.array([0.3, -0.2, 0.1])
    ekf.initialize(init)

    steps = round(10.0 / DT_IMU)
    for i in range(steps):
        ekf.predict(_HOVER_ACCEL, np.zeros(3), DT_IMU)
        if i % round(DT_GPS / DT_IMU) == 0:
            assert ekf.update(np.zeros(3), np.zeros(3))

    nominal = ekf.nominal()
    assert np.linalg.norm(np.asarray(nominal.position)) < 0.1
    assert np.linalg.norm(np.asarray(nominal.velocity)) < 0.05
    assert ekf.last_update_accepted()


def test_attitude_error_injection_convergence() -> None:
    # 5 deg roll init error, perfect sensors: roll < 0.5 deg after 20 s.
    phi0 = 5.0 * _DEG
    ekf = Ekf(_quiet_config())
    init = EKFNominalState()
    # Hamilton quaternion for a rotation of phi0 about body x.
    init.attitude = Quaternion(
        math.cos(0.5 * phi0), math.sin(0.5 * phi0), 0.0, 0.0
    )
    ekf.initialize(init)

    steps = round(20.0 / DT_IMU)
    for i in range(steps):
        ekf.predict(_HOVER_ACCEL, np.zeros(3), DT_IMU)
        if i % round(DT_GPS / DT_IMU) == 0:
            assert ekf.update(np.zeros(3), np.zeros(3))

    R = np.asarray(ekf.nominal().attitude.to_dcm())
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = math.asin(-R[2, 0])
    assert abs(roll) < 0.5 * _DEG
    assert abs(pitch) < 0.5 * _DEG


def test_gating_rejects_outlier() -> None:
    # Converge on perfect sensors; a 50 m GPS position jump must be gated
    # out without touching the nominal state.
    config = _quiet_config()
    ekf = Ekf(config)
    ekf.initialize(EKFNominalState())

    steps = round(10.0 / DT_IMU)
    for i in range(steps):
        ekf.predict(_HOVER_ACCEL, np.zeros(3), DT_IMU)
        if i % round(DT_GPS / DT_IMU) == 0:
            assert ekf.update(np.zeros(3), np.zeros(3))

    position_before = np.asarray(ekf.nominal().position).copy()
    accepted = ekf.update(np.array([50.0, 0.0, 0.0]), np.zeros(3))
    assert accepted is False
    assert ekf.last_update_accepted() is False
    assert ekf.last_innovation_chi2() > config.innovation_gate_chi2
    np.testing.assert_array_equal(
        np.asarray(ekf.nominal().position), position_before
    )
    # The filter accepts the next clean fix.
    assert ekf.update(np.zeros(3), np.zeros(3)) is True
    assert ekf.last_update_accepted() is True


def test_covariance_symmetry_after_noisy_run() -> None:
    # 20 s of default-noise propagation + updates: P stays symmetric.
    rng = np.random.default_rng(2026)
    config = EKFConfig()
    ekf = Ekf(config)
    ekf.initialize(EKFNominalState())

    accel_sigma = config.accel_noise_density / math.sqrt(DT_IMU)
    gyro_sigma = config.gyro_noise_density / math.sqrt(DT_IMU)
    steps = round(20.0 / DT_IMU)
    for i in range(steps):
        accel = _HOVER_ACCEL + accel_sigma * rng.standard_normal(3)
        gyro = gyro_sigma * rng.standard_normal(3)
        ekf.predict(accel, gyro, DT_IMU)
        if i % round(DT_GPS / DT_IMU) == 0:
            gps_pos = np.array(
                [
                    config.gps_pos_noise * rng.standard_normal(),
                    config.gps_pos_noise * rng.standard_normal(),
                    config.gps_pos_noise_vertical * rng.standard_normal(),
                ]
            )
            gps_vel = config.gps_vel_noise * rng.standard_normal(3)
            assert ekf.update(gps_pos, gps_vel)

    P = np.asarray(ekf.covariance())
    assert P.shape == (15, 15)
    np.testing.assert_allclose(P, P.T, atol=1e-12)


def test_initialize_level_roll_pitch_near_zero() -> None:
    # initialize_level assumes the stationary specific force [0, 0, -g]:
    # roll/pitch come out ~0 and yaw passes through.
    ekf = Ekf(_quiet_config())
    ekf.initialize_level(np.array([1.0, 2.0, -3.0]), np.zeros(3), yaw=0.3)
    R = np.asarray(ekf.nominal().attitude.to_dcm())
    assert abs(math.atan2(R[2, 1], R[2, 2])) < 1e-9
    assert abs(math.asin(-R[2, 0])) < 1e-9
    assert abs(math.atan2(R[1, 0], R[0, 0]) - 0.3) < 1e-9

    # The same statement through initialize_from_accel with the hover reading.
    ekf2 = Ekf(_quiet_config())
    ekf2.initialize_from_accel(
        np.array([1.0, 2.0, -3.0]), np.zeros(3), _HOVER_ACCEL, yaw=0.0
    )
    R2 = np.asarray(ekf2.nominal().attitude.to_dcm())
    assert abs(math.atan2(R2[2, 1], R2[2, 2])) < 1e-9
    assert abs(math.asin(-R2[2, 0])) < 1e-9


def test_estimated_body_rates_subtract_bias() -> None:
    ekf = Ekf(_quiet_config())
    init = EKFNominalState()
    init.gyro_bias = np.array([0.01, -0.02, 0.005])
    ekf.initialize(init)
    gyro_meas = np.array([0.5, -0.3, 0.25])
    np.testing.assert_allclose(
        np.asarray(ekf.estimated_body_rates(gyro_meas)),
        gyro_meas - init.gyro_bias,
    )


def test_closed_loop_hover_with_ekf() -> None:
    # THE Phase 1-3 integration test: truth-driven sensors -> EKF ->
    # controller -> motor lag -> truth dynamics, 60 s hover at 10 m.
    #
    # Architecture (see sims/scenario_hover.py for the truth-fed Phase 2
    # counterpart):
    #   1. SensorSuite.generate_imu with the true specific force from
    #      finite-differenced truth velocity (includes drag + motor lag
    #      effects automatically).
    #   2. ekf.predict every DT_IMU (200 Hz).
    #   3. GPS update every DT_GPS (10 Hz) from generate_gps (NED via the
    #      flat-Earth helpers).
    #   4. Controller fed the EKF nominal state + estimated body rates
    #      (gyro - estimated bias) at DT_CONTROL (100 Hz, ZOH).
    #   5. Command through inverse mix + first-order motor lag + forward
    #      mix, then one RK4 truth step.
    #
    # Start: BOTH truth and EKF at the hover equilibrium 10 m (NED z = -10)
    # with motors pre-spooled to hover thrust, to isolate the
    # estimator-controller interaction from takeoff transients. GPS noise is
    # tightened (0.3 m horizontal / 0.5 m vertical) so the ~0.15 m
    # steady-state TRUE position envelope measures estimator-controller
    # coupling rather than the 1.5 m-class consumer GPS error floor (the
    # realistic-noise steady state is covered by the C++ NoisySteadyState
    # hover RMSE test). IMU keeps production densities and the SensorSuite's
    # true accel/gyro biases, so the EKF bias-estimation path is live.
    params = VehicleParams()
    gains = ControlGains.defaults()
    controller = CascadedController(gains, params)

    altitude = 10.0
    pos_desired = np.array([0.0, 0.0, -altitude])
    vel_desired = np.zeros(3)
    yaw_desired = 0.0

    truth = State()
    truth.position = pos_desired.copy()

    motors = MotorModel()
    hover = hover_thrust(params)
    for _ in range(200):  # 1 s pre-spin: motors converge to hover thrust
        motors.step(hover, np.zeros(3), dt=DT)
    assert abs(forward_mix(motors.forces)[0] - hover) < 1e-6

    gps_config = GPSConfig(
        position_noise=0.3,
        position_noise_vertical=0.5,
        velocity_noise=0.05,
        update_rate=1.0 / DT_GPS,
    )
    sensors = SensorSuite(IMUConfig(), gps_config, seed=20261012)

    ekf = Ekf(EKFConfig())
    ekf.initialize_level(pos_desired, np.zeros(3))

    duration = 60.0
    steps = round(duration / DT)
    steps_per_gps = round(DT_GPS / DT)
    steps_per_control = round(DT_CONTROL / DT)
    check_start = round((duration - 20.0) / DT)  # steady-state window

    command_thrust = hover
    command_moment = np.zeros(3)
    v_prev = np.asarray(truth.velocity).copy()

    max_steady_error = 0.0
    final_error_norm = math.inf
    for i in range(steps):
        # 1-2. IMU truth + EKF propagation (200 Hz).
        R_truth = np.asarray(truth.orientation.to_dcm())
        f_body = specific_force_from_velocity(
            v_prev, np.asarray(truth.velocity), R_truth, DT
        )
        v_prev = np.asarray(truth.velocity).copy()
        imu = sensors.generate_imu(truth, f_body, DT)
        ekf.predict(np.asarray(imu.accel), np.asarray(imu.gyro), DT)

        # 3. GPS update (10 Hz).
        if i % steps_per_gps == 0:
            gps = sensors.generate_gps(truth)
            ekf.update(gps_to_ned(gps), gps_velocity_ned(gps))

        # 4. Controller on the EKF state view (100 Hz, ZOH).
        if i % steps_per_control == 0:
            nominal = ekf.nominal()
            estimated = State()
            estimated.position = np.asarray(nominal.position)
            estimated.velocity = np.asarray(nominal.velocity)
            estimated.orientation = nominal.attitude
            estimated.angular_rate = ekf.estimated_body_rates(
                np.asarray(imu.gyro)
            )
            cmd = controller.step(
                estimated, pos_desired, vel_desired, yaw_desired, DT_CONTROL
            )
            command_thrust = cmd.collective_thrust
            command_moment = np.asarray(cmd.moment)

        # 5. Motors + truth propagation.
        forces = motors.step(command_thrust, command_moment, dt=DT)
        thrust_actual, moment_actual = forward_mix(forces)
        truth = propagate(truth, thrust_actual, moment_actual, np.zeros(3), DT, params)

        error = float(np.linalg.norm(pos_desired - np.asarray(truth.position)))
        if i >= check_start:
            max_steady_error = max(max_steady_error, error)
        final_error_norm = error

    print(
        f"[hover-with-EKF] final position error = {final_error_norm:.4f} m, "
        f"max over last 20 s = {max_steady_error:.4f} m, "
        f"final velocity norm = {np.linalg.norm(np.asarray(truth.velocity)):.4f} m/s"
    )
    # Thresholds set to 0.25 m: the observed steady-state envelope is
    # ~0.11-0.14 m across seeds (1.07-1.19x margin at 0.15 m, which is
    # brittle to numpy RNG stream changes); 0.25 m keeps ~1.8-2.3x margin
    # while still meaningful against the 1.5 m GPS error floor.
    assert final_error_norm < 0.25
    assert max_steady_error < 0.25
    assert np.linalg.norm(np.asarray(truth.velocity)) < 0.15

    # The EKF itself also tracks truth closely over the same window.
    est_error = np.asarray(ekf.nominal().position) - np.asarray(truth.position)
    assert np.linalg.norm(est_error) < 0.3
