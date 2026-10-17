"""Phase 4 low-battery RTL scenario: short 3-leg mission, battery fault
injection at t = 40 s, return-to-launch and gentle landing.

Scenario state machine (simple; Phase 5 formalizes it):
- MISSION: fly a 3-leg 20 m-class circuit at 10 m (same full stack as
  scenario_waypoints: truth + motor lag with the mixer clamp + sensors
  + EKF + NLGL + mission + cascaded PD), then LOITER (position hold) on
  the final waypoint.
- At t = 40 s: battery.force_soc(0.15), below the 20% RTL threshold.
- RTL_TRANSIT: on the next control tick with battery.is_low, snapshot the
  current EKF position as the start of a single-waypoint mission to the
  home hover point (0, 0, -10) and fly it with NLGL.
- RTL_DESCEND: within the 2 m radius of home, reset the horizontal
  reference to home and descend at DESCENT_RATE (0.4 m/s, under the
  0.5 m/s touchdown budget) until truth altitude crosses 0.15 m.

Acceptance:
- RTL triggers within 1 s of the injection.
- Final horizontal distance to home < 3 m (truth).
- Final |z| < 0.3 m (truth; sim ends at the 0.15 m crossing, ground
  contact itself is not simulated).
- Vertical speed at the 0.5 m altitude crossing < 0.5 m/s (truth).

Wind: calm air. Turbulence is exercised by the waypoint-circuit
scenario; here the acceptance is about RTL logic and touchdown quality,
and a deterministic calm-air run keeps the vertical-speed margin honest.
Battery: updated at the control rate from the actual (lagged, mixed)
collective thrust.
"""

from __future__ import annotations

import sys

import numpy as np

from talon import (
    DT,
    DT_CONTROL,
    DT_GPS,
    BatteryModel,
    CascadedController,
    ControlGains,
    EKFConfig,
    Ekf,
    GPSConfig,
    IMUConfig,
    NLGLConfig,
    NLGLGuidance,
    SensorSuite,
    State,
    VehicleParams,
    Waypoint,
    WaypointMission,
    gps_to_ned,
    gps_velocity_ned,
    hover_thrust,
    propagate,
    specific_force_from_velocity,
)
from talon.motors import MotorModel, forward_mix

RTL_INJECT_S = 40.0
SIM_END_S = 120.0
ALTITUDE_M = 10.0
WP_RADIUS_M = 2.0
DESCENT_RATE = 0.4  # m/s, NED positive down; under the 0.5 m/s budget
TOUCHDOWN_Z_M = -0.15  # sim ends when truth z crosses this (0.15 m AGL)

MISSION_POSITIONS = [
    (20.0, 0.0),
    (20.0, 20.0),
    (0.0, 20.0),
]

TRIGGER_MAX_DELAY_S = 1.0
FINAL_HORIZONTAL_MAX_M = 3.0
FINAL_ALTITUDE_MAX_M = 0.3
TOUCHDOWN_VZ_MAX = 0.5

# state names for the printout
STATE_MISSION = "MISSION"
STATE_LOITER = "LOITER"
STATE_RTL_TRANSIT = "RTL_TRANSIT"
STATE_RTL_DESCEND = "RTL_DESCEND"
STATE_LANDED = "LANDED"


def main() -> int:
    params = VehicleParams()
    gains = ControlGains.defaults()
    controller = CascadedController(gains, params)
    guidance = NLGLGuidance(NLGLConfig())

    home = np.array([0.0, 0.0, 0.0])  # ground level home (NED)
    hover_point = np.array([0.0, 0.0, -ALTITUDE_M])

    mission = WaypointMission(
        [
            Waypoint(
                np.array([north, east, -ALTITUDE_M]),
                radius=WP_RADIUS_M,
            )
            for north, east in MISSION_POSITIONS
        ]
    )

    truth = State()
    truth.position = hover_point.copy()

    hover = hover_thrust(params)
    motors = MotorModel()
    for _ in range(round(1.0 / DT)):  # pre-spin (Phase 3 convention)
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
    ekf.initialize_level(hover_point, np.zeros(3))

    battery = BatteryModel()

    reference = hover_point.copy()
    v_cmd = np.zeros(3)
    command_thrust = hover
    command_moment = np.zeros(3)

    steps_per_control = round(DT_CONTROL / DT)
    steps_per_gps = round(DT_GPS / DT)

    v_prev = np.asarray(truth.velocity).copy()
    state = STATE_MISSION
    rtl_mission: WaypointMission | None = None
    rtl_trigger_time: float | None = None
    injected = False
    vz_at_touchdown: float | None = None
    time_s = 0.0
    steps = round(SIM_END_S / DT)

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

        # 4. Fault injection + state machine + guidance + controller.
        if i % steps_per_control == 0:
            if not injected and time_s >= RTL_INJECT_S:
                battery.force_soc(0.15)
                injected = True

            nominal = ekf.nominal()
            ekf_position = np.asarray(nominal.position)
            ekf_velocity = np.asarray(nominal.velocity)
            yaw_desired = 0.0
            diag = None

            if state in (STATE_MISSION, STATE_LOITER):
                if battery.is_low:
                    # RTL trigger: single-waypoint mission home from the
                    # current estimated position.
                    rtl_mission = WaypointMission(
                        [Waypoint(hover_point.copy(), radius=WP_RADIUS_M)]
                    )
                    rtl_mission.set_start(ekf_position)
                    rtl_trigger_time = time_s
                    state = STATE_RTL_TRANSIT
                elif not mission.complete:
                    mission.update(ekf_position)
                    v_des, diag = guidance.compute_mission(
                        ekf_position, ekf_velocity, mission
                    )
                    yaw_desired = diag.chi_d
                else:
                    state = STATE_LOITER
                    # Position hold: frozen reference, P on altitude.
                    cfg = NLGLConfig()
                    v_des = np.array(
                        [
                            0.0,
                            0.0,
                            float(
                                np.clip(
                                    cfg.kp_alt
                                    * (hover_point[2] - ekf_position[2]),
                                    -cfg.climb_rate_max,
                                    cfg.climb_rate_max,
                                )
                            ),
                        ]
                    )

            if state == STATE_RTL_TRANSIT:
                assert rtl_mission is not None
                rtl_mission.update(ekf_position)
                v_des, diag = guidance.compute_mission(
                    ekf_position, ekf_velocity, rtl_mission
                )
                yaw_desired = diag.chi_d
                if rtl_mission.complete:
                    state = STATE_RTL_DESCEND
                    # Re-center the horizontal reference on home so the
                    # descent also closes the last ~2 m of horizontal
                    # error, then descend at DESCENT_RATE.
                    reference = np.array(
                        [home[0], home[1], ekf_position[2]]
                    )
                    v_des = np.array([0.0, 0.0, DESCENT_RATE])

            if state == STATE_RTL_DESCEND:
                reference[2] = reference[2] + DESCENT_RATE * DT_CONTROL
                v_des = np.array([0.0, 0.0, DESCENT_RATE])

            # Velocity command goes straight to the PD and the reference
            # integrator; with the physical motor constants the mixer
            # clamp no longer manufactures yaw spikes, so no slew limit
            # is needed for stability.
            v_cmd = np.asarray(v_des)
            if (state == STATE_MISSION and not mission.complete) or (
                state == STATE_RTL_TRANSIT
            ):
                reference = reference + v_cmd * DT_CONTROL

            estimated = State()
            estimated.position = ekf_position
            estimated.velocity = ekf_velocity
            estimated.orientation = nominal.attitude
            estimated.angular_rate = ekf.estimated_body_rates(
                np.asarray(imu.gyro)
            )
            cmd = controller.step(
                estimated, reference, v_cmd, yaw_desired, DT_CONTROL
            )
            command_thrust = cmd.collective_thrust
            command_moment = np.asarray(cmd.moment)

            # Battery update at the control rate from the actual thrust
            # of the last dynamics step.
            battery.update(
                forward_mix(motors.forces)[0], DT_CONTROL
            )

        # 5. Motors + truth propagation (calm air). The mixer clamps each
        # motor to [0, MOTOR_FORCE_MAX] and accepts the moment error
        # (real-actuator behavior; no redistribution).
        forces = motors.step(command_thrust, command_moment, dt=DT)
        thrust_actual, moment_actual = forward_mix(forces)
        truth = propagate(
            truth, thrust_actual, moment_actual, np.zeros(3), DT, params
        )
        time_s += DT

        truth_z = float(np.asarray(truth.position)[2])
        truth_vz = float(np.asarray(truth.velocity)[2])
        if vz_at_touchdown is None and truth_z >= -0.5:
            vz_at_touchdown = truth_vz
        if truth_z >= TOUCHDOWN_Z_M:
            state = STATE_LANDED
            if vz_at_touchdown is None:
                vz_at_touchdown = truth_vz
            break

    # ---- Metrics -----------------------------------------------------
    truth_position = np.asarray(truth.position)
    final_horizontal = float(np.linalg.norm(truth_position[:2]))
    final_altitude = float(abs(truth_position[2]))

    print("=== Phase 4 low-battery RTL ===")
    print(
        f"duration: {time_s:.1f} s, final state: {state}, "
        f"soc: {battery.soc:.3f}"
    )
    if rtl_trigger_time is not None:
        print(
            f"RTL triggered at t = {rtl_trigger_time:.2f} s "
            f"(injection at {RTL_INJECT_S:.1f} s, "
            f"delay {rtl_trigger_time - RTL_INJECT_S:.3f} s)"
        )
    else:
        print("RTL WAS NEVER TRIGGERED")
    print(f"final position [n, e, d] (m): {truth_position}")
    print(f"final horizontal distance to home: {final_horizontal:.3f} m")
    print(f"final altitude |z|: {final_altitude:.3f} m")
    print(
        f"vertical speed at 0.5 m crossing: "
        f"{vz_at_touchdown if vz_at_touchdown is not None else float('nan'):.3f} m/s"
    )

    ok = True
    if rtl_trigger_time is None or (
        rtl_trigger_time - RTL_INJECT_S >= TRIGGER_MAX_DELAY_S
    ):
        print(f"FAIL: RTL trigger delay >= {TRIGGER_MAX_DELAY_S} s")
        ok = False
    if final_horizontal >= FINAL_HORIZONTAL_MAX_M:
        print(f"FAIL: final horizontal {final_horizontal:.3f} m "
              f">= {FINAL_HORIZONTAL_MAX_M} m")
        ok = False
    if final_altitude >= FINAL_ALTITUDE_MAX_M:
        print(f"FAIL: final altitude {final_altitude:.3f} m "
              f">= {FINAL_ALTITUDE_MAX_M} m")
        ok = False
    if vz_at_touchdown is None or vz_at_touchdown >= TOUCHDOWN_VZ_MAX:
        print(
            f"FAIL: touchdown vertical speed "
            f"{vz_at_touchdown} >= {TOUCHDOWN_VZ_MAX} m/s"
        )
        ok = False
    if state != STATE_LANDED:
        print(f"FAIL: final state {state} != LANDED")
        ok = False
    if not ok:
        return 1
    print("PASS: RTL within thresholds")
    return 0


if __name__ == "__main__":
    sys.exit(main())
