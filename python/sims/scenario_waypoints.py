"""Phase 4 waypoint circuit scenario: 40 m x 30 m rectangle at 10 m with
Dryden wind, full EKF-in-the-loop stack.

Architecture (documented choices):
- Truth dynamics: Rust propagate (RK4, DT = 0.005 s / 200 Hz) with the
  Phase 1 DrydenWind (sigma_u = sigma_v = 2.0, sigma_w = 1.0 m/s) stepped
  every dynamics tick at the current ground-speed magnitude; the returned
  NED wind vector is fed to propagate.
- Motors: Python MotorModel first-order lag at DT, inverse mix -> lag ->
  clamp -> forward mix before the plant. With the realistic yaw drag arm
  c = KM/KF = 0.015 m the clamp residual is a normal actuator moment
  error (no amplification), so the mixer needs no scenario-side limiting.
- Sensors: SensorSuite IMU at 200 Hz (production densities + true bias
  walk), GPS at 10 Hz. GPS noise tightened to 0.3 m horizontal / 0.5 m
  vertical / 0.05 m/s (same choice as the Phase 3 closed-loop test): the
  1.5 m consumer floor would dominate the EKF-vs-truth RMSE metric and
  measure the GPS rather than the estimator. IMU keeps production noise.
- EKF: predict every IMU step, GPS update every DT_GPS.
- Guidance: NLGLGuidance (ground-velocity Nu2 from the EKF NED velocity)
  + WaypointMission, recomputed at the control rate (100 Hz, ZOH).
- Cascaded controller fed the EKF state view (position, velocity,
  attitude, bias-corrected rates), pos_desired = an integrated reference
  point p_ref += v_cmd * DT_CONTROL (the standard way to feed a velocity
  command into a position PD: the reference runs at v_cmd and the PD
  tracks it with its own second-order lag; initialized at the vehicle
  position so there is no initial step). yaw_desired = chi_d (body-x
  along course; cos/sin of the yaw command are continuous through the
  +/- pi wrap, so chi_d wraps are harmless).
- Start: truth + EKF at 10 m hover above wp0, motors pre-spuned to hover
  thrust (Phase 3 test convention: isolates guidance from takeoff).
- On mission completion the reference freezes (position hold on the final
  waypoint) for the settle window; cross-track sampling stops.

Metrics: per-leg and overall |e_ct| (from GuidanceDiagnostics, after a
15 s transient), waypoint hits, mean ground speed, EKF-vs-truth position
RMSE.

Acceptance: all 5 waypoints reached; mean |e_ct| < 1.0 m; max |e_ct| <
3.0 m after the transient; EKF position RMSE < 2.0 m.

Public API (Sprint 2.2): run_scenario() runs one simulation with
perturbable noise scales, wind scales, start offsets, and seeds, and
returns the headline WaypointMetrics; main() runs the nominal scenario
(defaults) and prints the acceptance report. sims/monte_carlo.py wraps
run_scenario() for the Monte Carlo sweep.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass

import numpy as np

from talon import (
    DT,
    DT_CONTROL,
    DT_GPS,
    CascadedController,
    ControlGains,
    DrydenWind,
    EKFConfig,
    EKFNominalState,
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

MAX_DURATION_S = 120.0
SETTLE_S = 5.0
TRANSIENT_S = 15.0  # excluded from cross-track statistics
ALTITUDE_M = 10.0
CRUISE_SPEED = 5.0
WP_RADIUS_M = 2.0

# Rectangle at 10 m: start/finish at the origin, 40 m north, 30 m east.
WAYPOINT_POSITIONS = [
    (0.0, 0.0),
    (40.0, 0.0),
    (40.0, 30.0),
    (0.0, 30.0),
    (0.0, 0.0),
]

# Acceptance thresholds.
MEAN_E_CT_MAX_M = 1.0
MAX_E_CT_MAX_M = 3.0
EKF_RMSE_MAX_M = 2.0

# Nominal GPS noise densities (scaled by gps_noise_scale) and Dryden
# wind sigmas (scaled by wind_scale); defaults reproduce the original
# fixed-seed scenario exactly.
NOMINAL_GPS_POSITION_NOISE_M = 0.3
NOMINAL_GPS_POSITION_NOISE_VERTICAL_M = 0.5
NOMINAL_GPS_VELOCITY_NOISE_MPS = 0.05
NOMINAL_WIND_SIGMA_HORIZONTAL = 2.0
NOMINAL_WIND_SIGMA_VERTICAL = 1.0

# Lower bound for the GPS noise scale: at 0 the GPS would be unrealistically
# perfect and the EKF-vs-truth RMSE would measure the IMU drift alone.
MIN_GPS_NOISE_SCALE = 0.01


@dataclass(slots=True)
class WaypointMetrics:
    """Headline metrics of one waypoint-circuit run.

    success is the AND of the four acceptance criteria: all waypoints
    reached, mean |e_ct| < 1.0 m, max |e_ct| < 3.0 m after the 15 s
    transient, and EKF-vs-truth position RMSE < 2.0 m.
    """

    mean_e_ct: float
    max_e_ct: float
    ekf_rmse: float
    waypoints_reached: int
    mean_speed: float
    duration_s: float
    success: bool


def _estimated_state(nominal: EKFNominalState, ekf: Ekf, gyro: np.ndarray) -> State:
    """Build the controller-facing State view from the EKF estimate."""
    estimated = State()
    estimated.position = np.asarray(nominal.position)
    estimated.velocity = np.asarray(nominal.velocity)
    estimated.orientation = nominal.attitude
    estimated.angular_rate = ekf.estimated_body_rates(gyro)
    return estimated


def run_scenario(
    *,
    gps_noise_scale: float = 1.0,
    wind_scale: float = 1.0,
    init_offset_ne: tuple[float, float] = (0.0, 0.0),
    sensor_seed: int = 20261012,
    wind_seed: int = 42,
) -> WaypointMetrics:
    """Run one waypoint-circuit simulation and return its metrics.

    All arguments are keyword-only. The defaults reproduce the nominal
    fixed-seed scenario (the one main() reports on). Perturbations for
    the Monte Carlo sweep:

    - gps_noise_scale: multiplier on the nominal GPS noise densities
      (clipped to >= 0.01 so the GPS can never become unrealistically
      perfect);
    - wind_scale: multiplier on the Dryden turbulence sigmas;
    - init_offset_ne: north/east offset (m) of the start hover point;
    - sensor_seed / wind_seed: seeds of the sensor suite and the wind
      realization.
    """
    gps_noise_scale = max(gps_noise_scale, MIN_GPS_NOISE_SCALE)
    offset_n, offset_e = init_offset_ne

    params = VehicleParams()
    gains = ControlGains.defaults()
    controller = CascadedController(gains, params)
    guidance = NLGLGuidance(NLGLConfig())

    mission = WaypointMission(
        [
            Waypoint(
                np.array([north, east, -ALTITUDE_M]),
                speed=CRUISE_SPEED,
                radius=WP_RADIUS_M,
            )
            for north, east in WAYPOINT_POSITIONS
        ]
    )

    start_position = np.array([offset_n, offset_e, -ALTITUDE_M])
    truth = State()
    truth.position = start_position.copy()

    # Pre-spin the LOOP motors to hover thrust (1 s), as in the Phase 3
    # test: starting from rest, the first asymmetric command would clamp
    # motor forces and the resulting moment error would kick the vehicle.
    hover = hover_thrust(params)
    motors = MotorModel()
    for _ in range(round(1.0 / DT)):
        motors.step(hover, np.zeros(3), dt=DT)
    assert abs(forward_mix(motors.forces)[0] - hover) < 1e-6

    sensors = SensorSuite(
        IMUConfig(),
        GPSConfig(
            position_noise=NOMINAL_GPS_POSITION_NOISE_M * gps_noise_scale,
            position_noise_vertical=(
                NOMINAL_GPS_POSITION_NOISE_VERTICAL_M * gps_noise_scale
            ),
            velocity_noise=NOMINAL_GPS_VELOCITY_NOISE_MPS * gps_noise_scale,
            update_rate=1.0 / DT_GPS,
        ),
        seed=sensor_seed,
    )

    ekf = Ekf(EKFConfig())
    ekf.initialize_level(start_position, np.zeros(3))
    wind = DrydenWind(
        NOMINAL_WIND_SIGMA_HORIZONTAL * wind_scale,
        NOMINAL_WIND_SIGMA_HORIZONTAL * wind_scale,
        NOMINAL_WIND_SIGMA_VERTICAL * wind_scale,
        200.0,
        200.0,
        50.0,
        DT,
        seed=wind_seed,
    )

    # Integrated guidance reference point (module docstring) and the
    # current velocity command (overwritten each control tick).
    reference = start_position.copy()
    v_cmd = np.zeros(3)
    command_thrust = hover
    command_moment = np.zeros(3)

    steps = round(MAX_DURATION_S / DT)
    steps_per_control = round(DT_CONTROL / DT)
    steps_per_gps = round(DT_GPS / DT)
    transient_controls = round(TRANSIENT_S / DT_CONTROL)

    v_prev = np.asarray(truth.velocity).copy()
    mission_complete_time: float | None = None
    waypoints_reached = 1  # parked on wp0 at start; first update skips it
    last_index = 0
    time_s = 0.0

    e_ct_samples: list[float] = []
    ground_speeds: list[float] = []
    est_errors: list[float] = []

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

        # 4. Mission + guidance + controller (100 Hz, ZOH).
        if i % steps_per_control == 0:
            nominal = ekf.nominal()
            ekf_position = np.asarray(nominal.position)
            ekf_velocity = np.asarray(nominal.velocity)
            diag = None

            if not mission.complete:
                mission.update(ekf_position)
                if mission.active_index != last_index:
                    waypoints_reached += 1
                    last_index = mission.active_index
                if mission.complete:
                    mission_complete_time = time_s
                v_des, diag = guidance.compute_mission(
                    ekf_position, ekf_velocity, mission
                )
            else:
                # Position hold on the final waypoint for the settle
                # window: freeze the horizontal reference, P on altitude.
                cfg = NLGLConfig()
                v_des = np.array(
                    [
                        0.0,
                        0.0,
                        float(
                            np.clip(
                                cfg.kp_alt * (-ALTITUDE_M - ekf_position[2]),
                                -cfg.climb_rate_max,
                                cfg.climb_rate_max,
                            )
                        ),
                    ]
                )

            # Velocity command goes straight to the PD and the reference
            # integrator; with the physical motor constants the mixer
            # clamp no longer manufactures yaw spikes, so no slew limit
            # is needed for stability.
            v_cmd = np.asarray(v_des)
            if not mission.complete:
                reference = reference + v_cmd * DT_CONTROL

            estimated = _estimated_state(nominal, ekf, np.asarray(imu.gyro))
            yaw_desired = diag.chi_d if diag is not None else 0.0
            cmd = controller.step(
                estimated, reference, v_cmd, yaw_desired, DT_CONTROL
            )
            command_thrust = cmd.collective_thrust
            command_moment = np.asarray(cmd.moment)

            # Metrics at the control rate.
            est_errors.append(
                float(
                    np.linalg.norm(ekf_position - np.asarray(truth.position))
                )
            )
            ground_speeds.append(
                float(np.linalg.norm(np.asarray(truth.velocity)[:2]))
            )
            if diag is not None and i >= transient_controls * steps_per_control:
                e_ct_samples.append(abs(diag.e_ct))

        # 5. Motors + wind + truth propagation. The mixer clamps each
        # motor to [0, MOTOR_FORCE_MAX] and accepts the moment error
        # (real-actuator behavior; no redistribution).
        forces = motors.step(command_thrust, command_moment, dt=DT)
        thrust_actual, moment_actual = forward_mix(forces)
        wind_ned = wind.step(float(np.linalg.norm(np.asarray(truth.velocity))))
        truth = propagate(
            truth, thrust_actual, moment_actual, wind_ned, DT, params
        )
        time_s += DT

        if mission_complete_time is not None and time_s >= (
            mission_complete_time + SETTLE_S
        ):
            break

    # ---- Metrics -----------------------------------------------------
    mean_e_ct = float(np.mean(e_ct_samples)) if e_ct_samples else math.inf
    max_e_ct = float(np.max(e_ct_samples)) if e_ct_samples else math.inf
    ekf_rmse = float(np.sqrt(np.mean(np.square(est_errors))))
    mean_speed = float(np.mean(ground_speeds[transient_controls:]))

    success = (
        waypoints_reached >= len(WAYPOINT_POSITIONS)
        and mean_e_ct < MEAN_E_CT_MAX_M
        and max_e_ct < MAX_E_CT_MAX_M
        and ekf_rmse < EKF_RMSE_MAX_M
    )
    return WaypointMetrics(
        mean_e_ct=mean_e_ct,
        max_e_ct=max_e_ct,
        ekf_rmse=ekf_rmse,
        waypoints_reached=waypoints_reached,
        mean_speed=mean_speed,
        duration_s=time_s,
        success=success,
    )


def main() -> int:
    metrics = run_scenario()

    print("=== Phase 4 waypoint circuit (40x30 m @ 10 m, Dryden wind) ===")
    print(f"duration: {metrics.duration_s:.1f} s")
    print(
        f"waypoints reached: {metrics.waypoints_reached}"
        f"/{len(WAYPOINT_POSITIONS)}"
    )
    print(f"mean ground speed after transient: {metrics.mean_speed:.2f} m/s")
    print(
        f"cross-track |e_ct| after {TRANSIENT_S:.0f} s: "
        f"mean {metrics.mean_e_ct:.3f} m, max {metrics.max_e_ct:.3f} m"
    )
    print(f"EKF-vs-truth position RMSE: {metrics.ekf_rmse:.3f} m")

    ok = True
    if metrics.waypoints_reached < len(WAYPOINT_POSITIONS):
        print(
            f"FAIL: only {metrics.waypoints_reached} waypoints reached"
        )
        ok = False
    if metrics.mean_e_ct >= MEAN_E_CT_MAX_M:
        print(
            f"FAIL: mean |e_ct| {metrics.mean_e_ct:.3f} m"
            f" >= {MEAN_E_CT_MAX_M} m"
        )
        ok = False
    if metrics.max_e_ct >= MAX_E_CT_MAX_M:
        print(
            f"FAIL: max |e_ct| {metrics.max_e_ct:.3f} m"
            f" >= {MAX_E_CT_MAX_M} m"
        )
        ok = False
    if metrics.ekf_rmse >= EKF_RMSE_MAX_M:
        print(
            f"FAIL: EKF RMSE {metrics.ekf_rmse:.3f} m"
            f" >= {EKF_RMSE_MAX_M} m"
        )
        ok = False
    if not ok:
        return 1
    print("PASS: waypoint circuit within thresholds")
    return 0


if __name__ == "__main__":
    sys.exit(main())
