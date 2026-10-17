"""End-to-end abbreviated integration tests (Phase 5).

Each test runs the full SIL stack (truth dynamics, motor lag, sensors,
EKF, state machine, guidance, controller) at reduced duration and asserts
the mission-level outcome. The hover test reuses the Phase 3
closed-loop pattern from tests/test_ekf.py; the rest drive
talon.mavlink_sil.SILRunner directly (the same object the sims/ scenarios
wrap in run_scenario()), so no subprocess is involved.
"""

from __future__ import annotations


import numpy as np

from talon import (
    DT,
    DT_CONTROL,
    DT_GPS,
    CascadedController,
    ControlGains,
    EKFConfig,
    Ekf,
    GPSConfig,
    IMUConfig,
    SensorSuite,
    State,
    VehicleParams,
    Waypoint,
    gps_to_ned,
    gps_velocity_ned,
    hover_thrust,
    propagate,
    specific_force_from_velocity,
)
from talon.mavlink_sil import SILConfig, SILRunner
from talon.motors import MotorModel, forward_mix
from talon.types import FlightPhase

ALTITUDE_M = 10.0


def _hover_with_ekf(duration_s: float = 15.0) -> tuple[float, float]:
    """Phase 3 closed-loop pattern: hover at 10 m with the EKF in the loop.

    Returns (final position error norm, EKF-vs-truth position error norm).
    """
    params = VehicleParams()
    gains = ControlGains.defaults()
    controller = CascadedController(gains, params)

    pos_desired = np.array([0.0, 0.0, -ALTITUDE_M])
    truth = State()
    truth.position = pos_desired.copy()

    motors = MotorModel()
    hover = hover_thrust(params)
    for _ in range(200):
        motors.step(hover, np.zeros(3), dt=DT)

    sensors = SensorSuite(
        IMUConfig(),
        GPSConfig(
            position_noise=0.3,
            position_noise_vertical=0.5,
            velocity_noise=0.05,
            update_rate=1.0 / DT_GPS,
        ),
        seed=20261012,
    )
    ekf = Ekf(EKFConfig())
    ekf.initialize_level(pos_desired, np.zeros(3))

    steps = round(duration_s / DT)
    steps_per_gps = round(DT_GPS / DT)
    steps_per_control = round(DT_CONTROL / DT)

    command_thrust = hover
    command_moment = np.zeros(3)
    v_prev = np.asarray(truth.velocity).copy()

    for i in range(steps):
        rotation = np.asarray(truth.orientation.to_dcm())
        f_body = specific_force_from_velocity(
            v_prev, np.asarray(truth.velocity), rotation, DT
        )
        v_prev = np.asarray(truth.velocity).copy()
        imu = sensors.generate_imu(truth, f_body, DT)
        ekf.predict(np.asarray(imu.accel), np.asarray(imu.gyro), DT)

        if i % steps_per_gps == 0:
            gps = sensors.generate_gps(truth)
            ekf.update(gps_to_ned(gps), gps_velocity_ned(gps))

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
                estimated, pos_desired, np.zeros(3), 0.0, DT_CONTROL
            )
            command_thrust = cmd.collective_thrust
            command_moment = np.asarray(cmd.moment)

        forces = motors.step(command_thrust, command_moment, dt=DT)
        thrust_actual, moment_actual = forward_mix(forces)
        truth = propagate(
            truth, thrust_actual, moment_actual, np.zeros(3), DT, params
        )

    final_error = float(
        np.linalg.norm(pos_desired - np.asarray(truth.position))
    )
    ekf_error = float(
        np.linalg.norm(
            np.asarray(ekf.nominal().position) - np.asarray(truth.position)
        )
    )
    return final_error, ekf_error


def test_hover_with_ekf_15s() -> None:
    final_error, ekf_error = _hover_with_ekf(15.0)
    print(
        f"[hover-abbrev] final error {final_error:.4f} m, "
        f"ekf error {ekf_error:.4f} m"
    )
    assert final_error < 0.25
    assert ekf_error < 0.30


def test_waypoints_abbrev_3wp() -> None:
    waypoints = [
        Waypoint(np.array([20.0, 0.0, -ALTITUDE_M]), radius=2.0),
        Waypoint(np.array([20.0, 20.0, -ALTITUDE_M]), radius=2.0),
        Waypoint(np.array([0.0, 20.0, -ALTITUDE_M]), radius=2.0),
    ]
    runner = SILRunner(SILConfig(enable_udp=False, waypoints=waypoints))
    runner.request_event("arm")
    runner.request_event("takeoff")
    result = runner.run(50.0)

    assert result.metrics["mission_complete"] == 1.0
    assert result.metrics["waypoints_reached"] == 3
    assert FlightPhase.WAYPOINT in result.phases_visited()
    assert FlightPhase.EMERGENCY not in result.phases_visited()
    # NLGL cross-track stayed sane over the whole mission.
    assert result.e_ct.size > 0
    assert float(np.mean(result.e_ct)) < 1.5


def test_rtl_abbrev_battery_injection() -> None:
    waypoints = [
        Waypoint(np.array([30.0, 0.0, -ALTITUDE_M]), radius=2.0),
        Waypoint(np.array([30.0, 30.0, -ALTITUDE_M]), radius=2.0),
    ]
    runner = SILRunner(SILConfig(enable_udp=False, waypoints=waypoints))
    runner.request_event("arm")
    runner.request_event("takeoff")
    # Inject while the mission is still in WAYPOINT (mission takes ~14 s
    # of cruise; injection at 12 s lands mid-leg).
    runner.schedule(12.0, lambda: runner.inject_battery_soc(0.15))
    result = runner.run(70.0)

    low_battery = [
        e for e in result.event_log if e.event == "low_battery" and e.guard_ok
    ]
    assert low_battery, "low_battery RTL trigger never accepted"
    assert low_battery[0].time < 13.0  # within 1 s of the injection

    final_horizontal = float(
        np.linalg.norm(result.truth_position[-1, :2])
    )
    final_altitude = float(-result.truth_position[-1, 2])
    final_vz = float(abs(result.truth_velocity[-1, 2]))
    print(
        f"[rtl-abbrev] final horizontal {final_horizontal:.3f} m, "
        f"altitude {final_altitude:.3f} m, |vz| {final_vz:.3f} m/s"
    )
    assert final_horizontal < 3.0
    assert final_altitude < 0.5
    assert final_vz < 1.0
    assert result.final_phase is FlightPhase.IDLE
    assert FlightPhase.RTL in result.phases_visited()
    assert FlightPhase.LAND in result.phases_visited()


def test_gps_outage_abbrev_2s() -> None:
    waypoints = [
        Waypoint(np.array([60.0, 0.0, -ALTITUDE_M]), radius=5.0),
    ]
    runner = SILRunner(SILConfig(enable_udp=False, waypoints=waypoints))
    runner.request_event("arm")
    runner.request_event("takeoff")
    runner.schedule(20.0, lambda: runner.inject_gps_outage(2.0))
    result = runner.run(50.0)

    time_s = result.time_s
    error = np.linalg.norm(
        result.ekf_position - result.truth_position, axis=1
    )
    i_end = int(np.searchsorted(time_s, 22.0, side="right") - 1)
    err_at_outage_end = float(error[i_end])

    after = time_s >= 22.0
    err_after_resume = float(np.max(error[after][:50]))  # first 0.5 s
    final_err = float(error[after][-1])

    print(
        f"[gps-outage-abbrev] error at outage end {err_at_outage_end:.3f} m, "
        f"max after resume {err_after_resume:.3f} m, final {final_err:.3f} m"
    )
    assert FlightPhase.EMERGENCY not in result.phases_visited()
    assert err_at_outage_end < 5.0
    assert final_err < 1.0
    assert result.gps_skipped >= 15


def test_emergency_abbrev_divergence_at_10s() -> None:
    runner = SILRunner(SILConfig(enable_udp=False))
    runner.request_event("arm")
    runner.request_event("takeoff")
    runner.schedule(10.0, lambda: runner.inject_ekf_divergence("corrupt_bias"))
    result = runner.run(40.0)

    emergency_times = [
        t for t, p in result.phase_timeline if p is FlightPhase.EMERGENCY
    ]
    assert emergency_times, "never entered EMERGENCY"
    detection_delay = emergency_times[0] - 10.0
    print(
        f"[emergency-abbrev] detected {detection_delay:.3f} s after injection"
    )
    assert detection_delay < 1.0

    assert FlightPhase.LAND in result.phases_visited()
    accepted = {e.event for e in result.event_log if e.guard_ok}
    assert "critical_fault" in accepted
    assert "emergency_landing" in accepted

    final_altitude = float(-result.truth_position[-1, 2])
    final_vz = float(abs(result.truth_velocity[-1, 2]))
    print(
        f"[emergency-abbrev] final altitude {final_altitude:.3f} m, "
        f"|vz| {final_vz:.3f} m/s, phase {result.final_phase.value}"
    )
    assert final_altitude < 0.5
    assert final_vz < 1.0
    assert result.final_phase is FlightPhase.IDLE
    assert "landed" in accepted
