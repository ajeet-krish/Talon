"""UDP MAVLink software-in-the-loop runner (Phase 5).

SILRunner owns the complete flight stack and closes the loop against the
C++ truth plant:

    dynamics (truth, C++ RK4) -> motors -> wind -> sensors -> EKF -> state
    machine -> phase setpoints -> NLGL guidance -> cascaded PD -> motors

All new Phase 5 scenarios (wind, GPS outage, emergency) and the
abbreviated integration tests are thin wrappers over this class.

Documented design choices:

SIM TIME. The clock is simulation time (steps * DT), never wall clock.
``run()`` starts with a convergence pre-roll at rest on the ground (motors
idle, ground clamp active, IMU + GPS fed to the EKF) that runs until the
filter reports converged or 5 s elapse; pre-roll is stamped with NEGATIVE
sim time so the main loop starts at t = 0 and scheduled events / fault
injection times are plain "sim seconds after run() begins". This is what
makes request_event("arm") at t = 0 honest: the arm guard requires
``ekf_converged``.

ORDER PER 200 Hz STEP. Scheduled faults, command socket drain, IMU +
EKF predict, GPS update (10 Hz, skippable), derived-estimate flags,
state machine events, phase setpoints + controller (100 Hz ZOH), motors +
wind + truth propagate + ground clamp, battery (100 Hz), telemetry
broadcast (configurable rate), history logging. The spec's ordering lists
events before sensors; the SIL evaluates guards on FRESH estimates, so the
sensor/estimator half-step runs first.

DERIVED ESTIMATE FLAGS (Telemetry contract):

- ekf_converged: at least 20 GPS updates accepted AND the position block
  trace of P (P[0,0] + P[1,1] + P[2,2]) < 4.0 m^2 (a ~1.15 m position
  1-sigma ellipsoid). Rejected by construction in the first 2 s.
- ekf_diverged: sim time >= 5 s AND (|ekf position - truth position| >
  20 m OR 5 consecutive GPS updates gated out by the innovation chi2
  gate). The 5 s floor keeps normal start-up transients from firing it.
  Note on the gated-streak clause: a rejected update leaves P untouched
  (only a successful update contracts it); P grows between updates via
  the process noise Q added in predict(), so a healthy filter widens its
  gate and re-admits fixes, while a corrupted nominal state or an
  adversarial GPS stream keeps every innovation outside the gate no
  matter how wide P grows.
- gps_outage_duration: 0.0 when no outage is active, else sim time since
  inject_gps_outage() was called (outages are only ever fault-injected).

GROUND. The truth plant has no ground plane in C++; the SIL applies a
simple contact clamp after every propagate step: z is capped at the home
altitude and any downward velocity is zeroed. That is what lets ARM/IDLE
rest on the deck and LAND -> IDLE detect a settled touchdown.

UDP. Two sockets, both bound on the loopback interface: the command
socket (bind, non-blocking recv every step) and the telemetry socket
(send-only). COMMAND_LONG mapping to state machine events:

    MAV_CMD_NAV_TAKEOFF (22)          -> "takeoff"
    MAV_CMD_NAV_RETURN_TO_LAUNCH (20) -> "rtl"      (operator extension)
    custom 40001                      -> "arm"
    custom 40002                      -> "mission_start"

Telemetry stream (per-message dividers off telemetry_rate_hz, see
SILConfig): HEARTBEAT (truth phase + armed) at 1 Hz, ATTITUDE and
LOCAL_POSITION_NED (truth state) at telemetry_rate_hz, GPS_RAW_INT at
10 Hz (fresh fixes only, never a stale repeat during an outage),
BATTERY_STATUS at 1 Hz.

FDIR. The automatic critical_fault fires on the RISING EDGE of
ekf_diverged (False -> True): exactly one automatic response per fault
episode. While ekf_diverged stays True the trigger is quiet, so the
emergency sequence owns the vehicle until landed; when ekf_diverged
returns False the trigger re-arms for a new fault episode. An EXTERNAL
critical_fault request works at any time (the state machine itself
allows ANY -> EMERGENCY).

THREADING. Histories are appended only by the SIL thread under a lock;
``stop()`` is a threading.Event polled every step; external readers
should ``thread.join()`` before consuming ``result`` / ``event_log``.
"""

from __future__ import annotations

import math
import socket
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np
from pymavlink.dialects.v20 import common as mavlink2

from talon._core import (
    DT,
    DT_CONTROL,
    DT_GPS,
    CascadedController,
    ControlGains,
    DrydenWind,
    EKFConfig,
    Ekf,
    State,
    VehicleParams,
    propagate,
)
from talon.battery import BatteryModel, BatteryState
from talon.guidance import NLGLConfig, NLGLGuidance
from talon.mavlink_codec import MavlinkCodec
from talon.mission import WaypointMission
from talon.motors import MotorModel, forward_mix
from talon.sensors import (
    GPSConfig,
    IMUConfig,
    SensorSuite,
    gps_to_ned,
    gps_velocity_ned,
    specific_force_from_velocity,
)
from talon.state_machine import (
    ArrayF,
    FlightPhaseMachine,
    PhaseTransition,
    Telemetry,
)
from talon.types import FlightPhase, GPSMeasurement, Waypoint

# Derived-estimate thresholds (module docstring).
CONVERGENCE_MIN_GPS_UPDATES = 20
CONVERGENCE_POS_TRACE_M2 = 4.0
DIVERGENCE_MIN_TIME_S = 5.0
DIVERGENCE_POS_ERROR_M = 20.0
DIVERGENCE_GATED_STREAK = 5

# Pre-roll cap: at 10 Hz GPS, 20 updates take 2 s; 5 s is generous.
PREROLL_MAX_S = 5.0

# COMMAND_LONG command id -> state machine event (module docstring).
COMMAND_EVENT_MAP: dict[int, str] = {
    mavlink2.MAV_CMD_NAV_TAKEOFF: "takeoff",
    mavlink2.MAV_CMD_NAV_RETURN_TO_LAUNCH: "rtl",
    40001: "arm",
    40002: "mission_start",
}


@dataclass(slots=True)
class SILConfig:
    """Everything needed to configure one SIL run.

    Module tuning (EKF / IMU / GPS / guidance / gains) is intentionally
    kept as the production defaults plus the scenario-facing noise knobs
    below: the Phase 4 scenarios established 0.3 m / 0.5 m / 0.05 m/s GPS
    noise as the "measure the estimator, not the GPS floor" setting, and
    every Phase 5 scenario reuses it.

    telemetry_rate_hz is the burst rate; each MAVLink message has its own
    counter-based divider off that burst (SILRunner._broadcast_telemetry):

        HEARTBEAT          1 Hz
        ATTITUDE           telemetry_rate_hz
        LOCAL_POSITION_NED telemetry_rate_hz
        GPS_RAW_INT        10 Hz, fresh fixes only (no stale repeats)
        BATTERY_STATUS     1 Hz
    """

    telemetry_host: str = "127.0.0.1"
    telemetry_port: int = 14550
    command_port: int = 14551
    telemetry_rate_hz: float = 20.0
    waypoints: list[Waypoint] = field(default_factory=list)
    takeoff_altitude: float = 10.0
    start_position_ned: ArrayF = field(
        default_factory=lambda: np.zeros(3, dtype=np.float64)
    )
    wind_sigma: tuple[float, float, float] = (0.0, 0.0, 0.0)
    wind_steady_ned: tuple[float, float, float] = (0.0, 0.0, 0.0)
    battery_initial_soc: float = 1.0
    gps_position_noise: float = 0.3
    gps_position_noise_vertical: float = 0.5
    gps_velocity_noise: float = 0.05
    cruise_hold_s: float = 2.0
    enable_udp: bool = True
    seed: int = 20261012

    def __post_init__(self) -> None:
        self.start_position_ned = np.asarray(
            self.start_position_ned, dtype=np.float64
        )
        if self.start_position_ned.shape != (3,):
            raise ValueError(
                "start_position_ned must have shape (3,), "
                f"got {self.start_position_ned.shape}"
            )
        if self.takeoff_altitude <= 0.0:
            raise ValueError(
                "takeoff_altitude must be positive, "
                f"got {self.takeoff_altitude}"
            )
        if self.telemetry_rate_hz <= 0.0:
            raise ValueError(
                f"telemetry_rate_hz must be positive, got {self.telemetry_rate_hz}"
            )


@dataclass(slots=True)
class SILResult:
    """Numpy histories + logs from one SILRunner.run() call.

    Histories are sampled at the CONTROL rate (100 Hz) except
    gps_update_times (one entry per ACCEPTED GPS update, at 10 Hz).
    phase_timeline is the list of (sim_time, phase) CHANGE events; the
    phase at any control sample is the last timeline entry at or before
    that sample's time.
    """

    time_s: ArrayF
    truth_position: ArrayF
    truth_velocity: ArrayF
    ekf_position: ArrayF
    ekf_velocity: ArrayF
    phase_timeline: list[tuple[float, FlightPhase]]
    event_log: list[PhaseTransition]
    e_ct: ArrayF
    gps_update_times: ArrayF
    gps_skipped: int
    gps_gated: int
    metrics: dict

    @property
    def final_phase(self) -> FlightPhase:
        """The last phase on the timeline."""
        return self.phase_timeline[-1][1]

    def phases_visited(self) -> set[FlightPhase]:
        """All phases that appeared on the timeline."""
        return {phase for _, phase in self.phase_timeline}


class SILRunner:
    """Full-stack closed-loop SIL with real UDP MAVLink and fault injection."""

    def __init__(self, config: SILConfig) -> None:
        self._config = config

        # Vehicle / control.
        self._params = VehicleParams()
        self._controller = CascadedController(
            ControlGains.defaults(), self._params
        )
        self._guidance = NLGLGuidance(NLGLConfig())

        # Mission (None when no waypoints: CRUISE just holds).
        self._mission: WaypointMission | None = (
            WaypointMission(list(config.waypoints)) if config.waypoints else None
        )
        self._rtl_mission: WaypointMission | None = None

        # Truth + actuators. Vehicle starts at rest ON the home point; the
        # ground clamp keeps it there until the first TAKEOFF thrust.
        self._truth = State()
        self._truth.position = np.asarray(
            config.start_position_ned, dtype=np.float64
        ).copy()
        self._home = np.asarray(
            config.start_position_ned, dtype=np.float64
        ).copy()
        self._ground_z = float(self._home[2])
        self._motors = MotorModel()

        # Sensors + estimator.
        self._sensors = SensorSuite(
            IMUConfig(),
            GPSConfig(
                position_noise=config.gps_position_noise,
                position_noise_vertical=config.gps_position_noise_vertical,
                velocity_noise=config.gps_velocity_noise,
                update_rate=1.0 / DT_GPS,
            ),
            seed=config.seed,
        )
        self._ekf = Ekf(EKFConfig())
        self._ekf.initialize_level(self._home, np.zeros(3))

        # Battery + wind.
        self._battery = BatteryModel(initial_soc=config.battery_initial_soc)
        self._battery_state = BatteryState(soc=self._battery.soc, voltage=11.1, current_a=0.0)
        any_turbulence = any(sigma > 0.0 for sigma in config.wind_sigma)
        self._wind: DrydenWind | None = (
            DrydenWind(
                config.wind_sigma[0],
                config.wind_sigma[1],
                config.wind_sigma[2],
                200.0,
                200.0,
                50.0,
                DT,
                seed=config.seed,
            )
            if any_turbulence
            else None
        )
        self._wind_steady = np.asarray(
            config.wind_steady_ned, dtype=np.float64
        )

        # State machine + fault state.
        self._state_machine = FlightPhaseMachine(
            takeoff_altitude=config.takeoff_altitude
        )
        self._codec = MavlinkCodec()
        self._armed = False
        self._emergency_descent_active = False

        # Fault-injection state.
        self._gps_outage_start = -math.inf
        self._gps_outage_end = -math.inf
        self._fault_suppress_updates = False
        self._adversarial_gps_offset: ArrayF | None = None

        # Derived-estimate flags.
        self._gps_accepted = 0
        self._gps_gated = 0
        self._gps_skipped = 0
        self._gated_streak = 0
        self._ekf_converged = False
        self._ekf_diverged = False
        # Rising-edge memory for the automatic critical_fault (module
        # docstring FDIR): the auto fault fires only on False -> True.
        self._ekf_diverged_prev = False

        # Loop state.
        self._time = 0.0
        self._v_prev = np.zeros(3, dtype=np.float64)
        self._last_gyro = np.zeros(3, dtype=np.float64)
        self._next_telemetry_time = 0.0
        # Per-message telemetry dividers (SILConfig docstring), derived
        # from the burst rate so ATTITUDE / LOCAL_POSITION_NED keep it.
        burst_hz = config.telemetry_rate_hz
        self._heartbeat_divider = max(1, round(burst_hz / 1.0))
        self._gps_divider = max(1, round(burst_hz / 10.0))
        self._battery_divider = max(1, round(burst_hz / 1.0))
        self._telemetry_tick = 0
        self._last_gps_time = -math.inf
        self._reference = np.asarray(
            config.start_position_ned, dtype=np.float64
        ).copy()
        self._reference_phase: FlightPhase | None = None
        self._v_cmd = np.zeros(3, dtype=np.float64)
        self._command_thrust = 0.0
        self._command_moment = np.zeros(3, dtype=np.float64)
        self._last_gps: GPSMeasurement | None = None
        self._phase_enter_time = 0.0

        # Queues / scheduling / threading.
        self._event_queue: deque[str] = deque()
        self._scheduled: list[tuple[float, Callable[[], None]]] = []
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._result: SILResult | None = None
        self._thread: threading.Thread | None = None

        # Sockets (opened by run()).
        self._command_sock: socket.socket | None = None
        self._telemetry_sock: socket.socket | None = None
        self._command_parse_errors = 0

        # Histories (SIL thread only, under lock).
        self._history_time: list[float] = []
        self._history_truth_pos: list[ArrayF] = []
        self._history_truth_vel: list[ArrayF] = []
        self._history_ekf_pos: list[ArrayF] = []
        self._history_ekf_vel: list[ArrayF] = []
        self._history_e_ct: list[float] = []
        self._history_gps_times: list[float] = []
        self._phase_timeline: list[tuple[float, FlightPhase]] = [
            (0.0, FlightPhase.IDLE)
        ]
        self._last_phase = FlightPhase.IDLE

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def state_machine(self) -> FlightPhaseMachine:
        """The live flight phase machine (for inspection while running)."""
        return self._state_machine

    @property
    def result(self) -> SILResult | None:
        """The finished SILResult, available after run() returns/joins."""
        with self._lock:
            return self._result

    def request_event(self, event: str, at_time: float | None = None) -> None:
        """Queue a state machine event (arm, takeoff, ...).

        at_time=None applies at the next opportunity; a sim time schedules
        it (evaluated against the sim clock, which starts at 0 after the
        convergence pre-roll).
        """
        if at_time is None:
            self._event_queue.append(event)
        else:
            self.schedule(at_time, lambda: self._event_queue.append(event))

    def schedule(self, at_time: float, action: Callable[[], None]) -> None:
        """Run ``action`` once sim time reaches ``at_time``."""
        self._scheduled.append((float(at_time), action))

    def inject_gps_outage(self, duration_s: float) -> None:
        """Black out GPS for ``duration_s`` sim seconds from NOW.

        During the outage no fix is generated or fed to the EKF (the
        filter dead-reckons on IMU only) and telemetry reports fix_type 0.
        Repeated calls extend the outage.
        """
        if duration_s <= 0.0:
            raise ValueError(f"duration_s must be positive, got {duration_s}")
        if self._time >= self._gps_outage_end:
            self._gps_outage_start = self._time
        self._gps_outage_end = max(
            self._gps_outage_end, self._time + duration_s
        )

    def inject_battery_soc(self, soc: float) -> None:
        """Force the battery state of charge (fault injection)."""
        self._battery.force_soc(soc)
        self._battery_state = BatteryState(
            soc=self._battery.soc, voltage=11.1, current_a=0.0
        )

    def inject_ekf_divergence(self, mode: str = "corrupt_bias") -> None:
        """Force the EKF into a diverged state. Modes:

        corrupt_bias: one-shot +25 m offset on the nominal NORTH position
            via mutable_nominal (a hard corruption of the filter's
            solution; the innovation gate then rejects every subsequent
            fix, so the corruption persists and ekf_diverged trips on the
            position-error clause within one control step).
        suppress_updates: stop feeding GPS to the filter (dead-reckoning
            forever; divergence only if the error eventually exceeds 20 m).
        adversarial_gps: offset every GPS position fed to the filter by
            +50 m north; a converged filter gates every fix out, so the
            gated-streak clause trips divergence after 5 rejections.
        """
        if mode == "corrupt_bias":
            nominal = self._ekf.mutable_nominal()
            position = np.asarray(nominal.position, dtype=np.float64).copy()
            position[0] += 25.0
            nominal.position = position
        elif mode == "suppress_updates":
            self._fault_suppress_updates = True
        elif mode == "adversarial_gps":
            self._adversarial_gps_offset = np.array([50.0, 0.0, 0.0])
        else:
            raise ValueError(
                f"unknown divergence mode {mode!r}; expected corrupt_bias, "
                "suppress_updates, or adversarial_gps"
            )

    def run(self, duration_s: float, realtime: bool = False) -> SILResult:
        """Run the SIL for ``duration_s`` sim seconds of main-loop time.

        Starts with the convergence pre-roll (negative sim time), then the
        main loop. realtime=True paces the loop against the wall clock;
        default False runs as fast as possible (pytest).
        """
        if duration_s <= 0.0:
            raise ValueError(f"duration_s must be positive, got {duration_s}")
        self._stop_event.clear()
        self._next_telemetry_time = 0.0
        self._telemetry_tick = 0
        self._open_sockets()
        try:
            self._preroll()
            steps = round(duration_s / DT)
            steps_per_control = max(1, round(DT_CONTROL / DT))
            steps_per_gps = max(1, round(DT_GPS / DT))
            wall_start = time.monotonic()
            for i in range(steps):
                if self._stop_event.is_set():
                    break
                self._step(i, steps_per_control, steps_per_gps)
                if realtime:
                    delay = wall_start + (i + 1) * DT - time.monotonic()
                    if delay > 0.0:
                        time.sleep(delay)
        finally:
            self._close_sockets()
        return self._finish(duration_s)

    def start_async(self, duration_s: float) -> threading.Thread:
        """Run() in a background thread; returns the started thread."""
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError("SIL thread already running")
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self.run, args=(duration_s,), daemon=True
        )
        self._thread.start()
        return self._thread

    def stop(self) -> None:
        """Ask a running SIL to stop at the next step (non-blocking)."""
        self._stop_event.set()

    # ------------------------------------------------------------------
    # Pre-roll and one simulation step
    # ------------------------------------------------------------------

    def _preroll(self) -> None:
        """Converge the EKF at rest before the main loop (negative time)."""
        self._time = -PREROLL_MAX_S
        total_steps = round(PREROLL_MAX_S / DT)
        steps_per_gps = max(1, round(DT_GPS / DT))
        for i in range(total_steps):
            self._sensor_step(i, steps_per_gps)
            self._update_derived()
            self._plant_step()
            # Advance the negative sim clock per step so accepted-update
            # timestamps count up from -PREROLL_MAX_S instead of sitting
            # at a constant -5.0; the main loop still starts at t = 0.
            self._time += DT
            if self._ekf_converged:
                break
        self._time = 0.0

    def _step(self, i: int, steps_per_control: int, steps_per_gps: int) -> None:
        is_control = i % steps_per_control == 0

        # 1. Due scheduled faults, then the command socket.
        self._fire_scheduled()
        self._drain_commands()

        # 2-3. Sensors + estimator half-step (fresh guards; module docstring).
        self._sensor_step(i, steps_per_gps)

        # 4. Derived estimate flags from the fresh estimate.
        self._update_derived()

        # 5. State machine: critical fault, queued + auto events.
        self._handle_events()

        # 6-7. Phase setpoints + cascaded PD (100 Hz, ZOH).
        if is_control:
            self._control_tick()

        # 8. Motors + wind + truth + ground clamp.
        forces = self._motors.step(
            self._command_thrust, self._command_moment, dt=DT
        )
        thrust_actual, moment_actual = forward_mix(forces)
        self._plant_step(thrust_actual, moment_actual)

        # 9. Battery at the control rate from the ACTUAL mixed thrust.
        if is_control:
            self._battery_state = self._battery.update(
                thrust_actual, DT_CONTROL
            )
            self._record_history()

        # 10. Telemetry broadcast.
        self._broadcast_telemetry()

        self._time += DT

    def _sensor_step(self, i: int, steps_per_gps: int) -> None:
        """IMU generation + EKF predict every step; GPS update when due."""
        truth_velocity = np.asarray(self._truth.velocity, dtype=np.float64)
        rotation = np.asarray(
            self._truth.orientation.to_dcm(), dtype=np.float64
        )
        f_body = specific_force_from_velocity(
            self._v_prev, truth_velocity, rotation, DT
        )
        self._v_prev = truth_velocity.copy()
        imu = self._sensors.generate_imu(self._truth, f_body, DT)
        self._last_gyro = np.asarray(imu.gyro, dtype=np.float64)
        self._ekf.predict(np.asarray(imu.accel), np.asarray(imu.gyro), DT)

        if i % steps_per_gps != 0:
            return
        if self._time < self._gps_outage_end or self._fault_suppress_updates:
            self._gps_skipped += 1
            return
        gps = self._sensors.generate_gps(self._truth)
        self._last_gps = gps
        self._last_gps_time = self._time
        position = gps_to_ned(gps)
        velocity = gps_velocity_ned(gps)
        if self._adversarial_gps_offset is not None:
            position = position + self._adversarial_gps_offset
        if self._ekf.update(position, velocity):
            self._gps_accepted += 1
            self._gated_streak = 0
            self._history_gps_times.append(self._time)
        else:
            self._gps_gated += 1
            self._gated_streak += 1

    def _plant_step(
        self,
        thrust_actual: float | None = None,
        moment_actual: ArrayF | None = None,
    ) -> None:
        """One truth RK4 step + wind + ground clamp."""
        if thrust_actual is None:
            # Idle (pre-roll / hold): motors commanded to zero this step.
            forces = self._motors.step(0.0, np.zeros(3), dt=DT)
            thrust_actual, moment_actual = forward_mix(forces)
        if self._wind is not None:
            airspeed = float(
                np.linalg.norm(np.asarray(self._truth.velocity)[:2])
            )
            wind_ned = np.asarray(self._wind.step(airspeed)) + self._wind_steady
        else:
            wind_ned = self._wind_steady
        self._truth = propagate(
            self._truth,
            thrust_actual,
            moment_actual,
            wind_ned,
            DT,
            self._params,
        )
        self._apply_ground_clamp()

    def _apply_ground_clamp(self) -> None:
        """Simple ground contact: cap z at home, kill downward velocity."""
        position = np.asarray(self._truth.position, dtype=np.float64).copy()
        velocity = np.asarray(self._truth.velocity, dtype=np.float64).copy()
        touched = False
        if position[2] > self._ground_z:
            position[2] = self._ground_z
            if velocity[2] > 0.0:
                velocity[2] = 0.0
            touched = True
        if touched:
            self._truth.position = position
            self._truth.velocity = velocity

    # ------------------------------------------------------------------
    # Derived flags, telemetry, events
    # ------------------------------------------------------------------

    def _ekf_position(self) -> ArrayF:
        return np.asarray(self._ekf.nominal().position, dtype=np.float64)

    def _ekf_velocity(self) -> ArrayF:
        return np.asarray(self._ekf.nominal().velocity, dtype=np.float64)

    def _update_derived(self) -> None:
        """Recompute ekf_converged / ekf_diverged (module docstring)."""
        covariance = np.asarray(self._ekf.covariance(), dtype=np.float64)
        pos_trace = float(covariance[0, 0] + covariance[1, 1] + covariance[2, 2])
        self._ekf_converged = (
            self._gps_accepted >= CONVERGENCE_MIN_GPS_UPDATES
            and pos_trace < CONVERGENCE_POS_TRACE_M2
        )
        position_error = float(
            np.linalg.norm(
                self._ekf_position()
                - np.asarray(self._truth.position, dtype=np.float64)
            )
        )
        self._ekf_diverged = (
            self._time >= DIVERGENCE_MIN_TIME_S
            and (
                position_error > DIVERGENCE_POS_ERROR_M
                or self._gated_streak >= DIVERGENCE_GATED_STREAK
            )
        )

    def _gps_outage_duration(self) -> float:
        if self._time >= self._gps_outage_end:
            return 0.0
        return self._time - self._gps_outage_start

    def _build_telemetry(self) -> Telemetry:
        return Telemetry(
            position_ned=self._ekf_position(),
            velocity_ned=self._ekf_velocity(),
            battery_soc=self._battery.soc,
            ekf_converged=self._ekf_converged,
            ekf_diverged=self._ekf_diverged,
            gps_outage_duration=self._gps_outage_duration(),
            mission_waypoint_count=(
                len(self._config.waypoints) if self._mission else 0
            ),
            mission_complete=(
                self._mission.complete if self._mission is not None else False
            ),
            motors_armed=self._armed,
            home_position_ned=self._home,
            emergency_descent_active=self._emergency_descent_active,
        )

    def _handle_events(self) -> None:
        """Advance the mission bookkeeping, then run the event pipeline."""
        # Mission acceptance uses the fresh EKF position.
        if self._mission is not None and self._state_machine.phase in (
            FlightPhase.CRUISE,
            FlightPhase.WAYPOINT,
        ):
            self._mission.update(self._ekf_position())

        machine = self._state_machine

        # Auto events: pre-check the guard condition first so a guard that
        # is merely not-yet-true does not spam the log with rejections.
        # critical_fault fires on the RISING EDGE of ekf_diverged (module
        # docstring FDIR): one automatic response per fault episode. An
        # estimate that stays diverged (e.g. corrupt_bias, which never
        # recovers) never re-fires, so the emergency sequence owns the
        # vehicle until landed; when the estimate recovers the trigger
        # re-arms for a new episode. An EXTERNAL critical_fault request
        # still works at any time (the machine allows ANY -> EMERGENCY).
        if self._ekf_diverged and not self._ekf_diverged_prev:
            machine.request("critical_fault", self._build_telemetry(), self._time)
        self._ekf_diverged_prev = self._ekf_diverged

        telemetry = self._build_telemetry()

        # Immediate (queued / UDP) events use the fresh snapshot too; each
        # ACCEPTED event refreshes the snapshot so a same-step follow-up
        # (e.g. arm then takeoff) sees the post-transition armed flag.
        while self._event_queue:
            event = self._event_queue.popleft()
            if machine.request(event, telemetry, self._time):
                self._armed = machine.phase is not FlightPhase.IDLE
                telemetry = self._build_telemetry()

        phase = machine.phase
        if phase is FlightPhase.TAKEOFF and (
            telemetry.altitude_above_home
            > self._config.takeoff_altitude - 0.5
        ):
            machine.request("altitude_reached", telemetry, self._time)
        if phase is FlightPhase.CRUISE:
            if (
                self._mission is not None
                and (self._time - self._phase_enter_time)
                >= self._config.cruise_hold_s
            ):
                machine.request("mission_start", telemetry, self._time)
        if phase is FlightPhase.WAYPOINT:
            if telemetry.mission_complete:
                machine.request("mission_complete", telemetry, self._time)
            if telemetry.battery_soc < 0.20:
                machine.request("low_battery", telemetry, self._time)
            if telemetry.gps_outage_duration > 5.0:
                machine.request("gps_lost", telemetry, self._time)
        if phase is FlightPhase.RTL and (
            telemetry.horizontal_dist_to_home < 1.0
        ):
            machine.request("home_reached", telemetry, self._time)
        if phase is FlightPhase.LAND and telemetry.altitude_above_home < 0.1:
            if abs(telemetry.vel_z) < 0.1:
                machine.request("landed", telemetry, self._time)
        if phase is FlightPhase.EMERGENCY:
            # Latch the descent flag on the first step in EMERGENCY so the
            # emergency_landing guard (and LAND entry) can see it.
            if not self._emergency_descent_active:
                self._emergency_descent_active = True
                telemetry = self._build_telemetry()
            if telemetry.altitude_above_home < 0.5:
                machine.request("emergency_landing", telemetry, self._time)

        # Armed flag follows the phase (IDLE disarms).
        self._armed = machine.phase is not FlightPhase.IDLE

        # Phase timeline bookkeeping.
        if machine.phase is not self._last_phase:
            self._phase_timeline.append((self._time, machine.phase))
            self._phase_enter_time = self._time
            self._last_phase = machine.phase

    # ------------------------------------------------------------------
    # Phase setpoints + controller
    # ------------------------------------------------------------------

    def _estimated_state(self, gyro: ArrayF) -> State:
        """Controller-facing State view built from the EKF estimate."""
        nominal = self._ekf.nominal()
        estimated = State()
        estimated.position = np.asarray(nominal.position)
        estimated.velocity = np.asarray(nominal.velocity)
        estimated.orientation = nominal.attitude
        estimated.angular_rate = self._ekf.estimated_body_rates(gyro)
        return estimated

    def _control_tick(self) -> None:
        """Phase-dependent setpoints + cascaded PD (100 Hz, ZOH)."""
        machine = self._state_machine
        phase = machine.phase
        config = self._config
        ekf_position = self._ekf_position()

        if phase is not self._reference_phase:
            # Fresh reference on every phase entry.
            self._reference = ekf_position.copy()
            self._reference_phase = phase
            if phase is FlightPhase.RTL:
                # New single-waypoint mission home from the entry position.
                home_hover = np.array(
                    [
                        self._home[0],
                        self._home[1],
                        self._home[2] - config.takeoff_altitude,
                    ]
                )
                self._rtl_mission = WaypointMission(
                    [Waypoint(home_hover, speed=5.0, radius=1.0)]
                )
                self._rtl_mission.set_start(ekf_position)

        estimated = self._estimated_state(self._last_gyro)

        if phase in (FlightPhase.IDLE, FlightPhase.ARM):
            # Hold on the ground: zero thrust, reset the PD internals.
            self._command_thrust = 0.0
            self._command_moment = np.zeros(3, dtype=np.float64)
            self._v_cmd = np.zeros(3, dtype=np.float64)
            self._controller.reset()
            return

        yaw_desired = 0.0

        if phase is FlightPhase.TAKEOFF:
            z_des = self._home[2] - config.takeoff_altitude
            cfg = self._guidance.config
            rate_z = float(
                np.clip(
                    cfg.kp_alt * (z_des - ekf_position[2]),
                    -cfg.climb_rate_max,
                    cfg.climb_rate_max,
                )
            )
            self._v_cmd = np.array([0.0, 0.0, rate_z])
            self._reference = self._reference + self._v_cmd * DT_CONTROL

        elif phase is FlightPhase.CRUISE:
            # Hold the entry position (altitude via P like NLGL's channel).
            cfg = self._guidance.config
            z_des = self._home[2] - config.takeoff_altitude
            rate_z = float(
                np.clip(
                    cfg.kp_alt * (z_des - ekf_position[2]),
                    -cfg.climb_rate_max,
                    cfg.climb_rate_max,
                )
            )
            self._v_cmd = np.array([0.0, 0.0, rate_z])
            self._reference = np.array(
                [
                    self._reference[0],
                    self._reference[1],
                    ekf_position[2] + rate_z * DT_CONTROL,
                ]
            )

        elif phase is FlightPhase.WAYPOINT:
            assert self._mission is not None
            ekf_velocity = self._ekf_velocity()
            v_des, diagnostics = self._guidance.compute_mission(
                ekf_position, ekf_velocity, self._mission
            )
            self._v_cmd = np.asarray(v_des)
            self._reference = self._reference + self._v_cmd * DT_CONTROL
            yaw_desired = diagnostics.chi_d
            self._history_e_ct.append(abs(diagnostics.e_ct))

        elif phase is FlightPhase.RTL:
            assert self._rtl_mission is not None
            ekf_velocity = self._ekf_velocity()
            self._rtl_mission.update(ekf_position)
            v_des, diagnostics = self._guidance.compute_mission(
                ekf_position, ekf_velocity, self._rtl_mission
            )
            self._v_cmd = np.asarray(v_des)
            self._reference = self._reference + self._v_cmd * DT_CONTROL
            yaw_desired = diagnostics.chi_d

        elif phase is FlightPhase.LAND:
            # Descent profile: 0.5 m/s above 0.5 m, tapering to a 0.05 m/s
            # creep below 0.15 m so |vz| < 0.1 is reachable for the
            # landed guard while the ground clamp absorbs the last bit.
            altitude = max(0.0, self._home[2] - ekf_position[2])
            if altitude > 0.5:
                rate_z = 0.5
            elif altitude > 0.15:
                rate_z = 0.05 + (altitude - 0.15) / 0.35 * 0.45
            else:
                rate_z = 0.05
            # Gentle horizontal re-centering on home during the descent,
            # NEAR-FIELD ONLY: a large horizontal error (for instance one
            # manufactured by a corrupted estimate) is not chased during
            # landing, so a fault cannot command a long lateral translation
            # while the vehicle is trying to touch down.
            ne_error = self._home[:2] - ekf_position[:2]
            if float(np.linalg.norm(ne_error)) < 5.0:
                v_ne = np.clip(0.8 * ne_error, -1.0, 1.0)
            else:
                v_ne = np.zeros(2, dtype=np.float64)
            self._v_cmd = np.array([v_ne[0], v_ne[1], rate_z])
            self._reference = self._reference + self._v_cmd * DT_CONTROL

        elif phase is FlightPhase.EMERGENCY:
            # Descend at 1.0 m/s with zero lateral velocity command; the
            # velocity reference makes the PD produce ~0.7 mg of thrust
            # (never negative). Reference altitude runs down at 1.0 m/s.
            self._v_cmd = np.array([0.0, 0.0, 1.0])
            self._reference = np.array(
                [
                    self._reference[0],
                    self._reference[1],
                    self._reference[2] + 1.0 * DT_CONTROL,
                ]
            )

        command = self._controller.step(
            estimated, self._reference, self._v_cmd, yaw_desired, DT_CONTROL
        )
        self._command_thrust = max(0.0, command.collective_thrust)
        self._command_moment = np.asarray(command.moment, dtype=np.float64)

    # ------------------------------------------------------------------
    # UDP command + telemetry
    # ------------------------------------------------------------------

    def _open_sockets(self) -> None:
        if not self._config.enable_udp:
            return
        self._command_sock = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM
        )
        self._command_sock.setblocking(False)
        self._command_sock.bind(
            (self._config.telemetry_host, self._config.command_port)
        )
        self._telemetry_sock = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM
        )
        self._telemetry_sock.setblocking(False)

    def _close_sockets(self) -> None:
        for sock in (self._command_sock, self._telemetry_sock):
            if sock is not None:
                sock.close()
        self._command_sock = None
        self._telemetry_sock = None

    def _drain_commands(self) -> None:
        """Non-blocking read of the command socket; queue known events."""
        if self._command_sock is None:
            return
        while True:
            try:
                data, _address = self._command_sock.recvfrom(2048)
            except (BlockingIOError, InterruptedError):
                return
            except OSError:
                return
            try:
                fields = MavlinkCodec.parse(data)
            except ValueError:
                self._command_parse_errors += 1
                continue
            if fields.get("msg_type") != "COMMAND_LONG":
                continue
            event = COMMAND_EVENT_MAP.get(int(fields["command"]))
            if event is not None:
                self._event_queue.append(event)

    def _broadcast_telemetry(self) -> None:
        """Send one telemetry burst with per-message rate dividers.

        ATTITUDE and LOCAL_POSITION_NED go out on every burst
        (telemetry_rate_hz). The other three messages are gated by
        counter-based dividers off the burst tick (SILConfig docstring):
        HEARTBEAT and BATTERY_STATUS at 1 Hz, GPS_RAW_INT at 10 Hz and
        only when the last generated fix is fresh (no stale repeat at
        the burst rate during an outage).
        """
        if self._telemetry_sock is None:
            return
        period = 1.0 / self._config.telemetry_rate_hz
        if self._time + 1e-9 < self._next_telemetry_time:
            return
        self._next_telemetry_time = self._time + period
        tick = self._telemetry_tick
        self._telemetry_tick += 1

        time_boot_ms = int(max(0.0, self._time) * 1000.0)
        messages: list[bytes] = []
        if tick % self._heartbeat_divider == 0:
            messages.append(
                self._codec.heartbeat(self._state_machine.phase, self._armed)
            )
        messages.append(self._codec.attitude(self._truth, time_boot_ms))
        messages.append(self._codec.local_position_ned(self._truth, time_boot_ms))
        # A fix is fresh if it was generated within the GPS period; an
        # outage stops fixes, so GPS_RAW_INT drops out rather than
        # repeating a stale fix at the burst rate.
        gps_fresh = self._time - self._last_gps_time < 1.5 * DT_GPS
        if tick % self._gps_divider == 0 and gps_fresh:
            messages.append(
                self._codec.gps_raw_int(
                    self._synthetic_gps(),
                    fix_type=0 if self._time < self._gps_outage_end else 3,
                )
            )
        if tick % self._battery_divider == 0:
            messages.append(
                self._codec.battery_status(
                    self._battery_state, self._battery_state.current_a
                )
            )
        target = (self._config.telemetry_host, self._config.telemetry_port)
        for message in messages:
            try:
                self._telemetry_sock.sendto(message, target)
            except (BlockingIOError, OSError):
                # Never let telemetry backpressure kill the sim loop.
                break

    def _synthetic_gps(self) -> GPSMeasurement:
        """GPS fix for telemetry: last generated fix, or a zero placeholder."""
        if self._last_gps is not None:
            return self._last_gps
        return GPSMeasurement(
            timestamp=self._time, lat=0.0, lon=0.0, alt=0.0, vn=0.0, ve=0.0, vd=0.0
        )

    # ------------------------------------------------------------------
    # Scheduling, history, result
    # ------------------------------------------------------------------

    def _fire_scheduled(self) -> None:
        if not self._scheduled:
            return
        remaining: list[tuple[float, Callable[[], None]]] = []
        for at_time, action in self._scheduled:
            if self._time >= at_time:
                action()
            else:
                remaining.append((at_time, action))
        self._scheduled = remaining

    def _record_history(self) -> None:
        with self._lock:
            self._history_time.append(self._time)
            self._history_truth_pos.append(
                np.asarray(self._truth.position, dtype=np.float64).copy()
            )
            self._history_truth_vel.append(
                np.asarray(self._truth.velocity, dtype=np.float64).copy()
            )
            self._history_ekf_pos.append(self._ekf_position().copy())
            self._history_ekf_vel.append(self._ekf_velocity().copy())

    def _finish(self, duration_s: float) -> SILResult:
        with self._lock:
            result = SILResult(
                time_s=np.asarray(self._history_time, dtype=np.float64),
                truth_position=np.asarray(
                    self._history_truth_pos, dtype=np.float64
                ).reshape(-1, 3),
                truth_velocity=np.asarray(
                    self._history_truth_vel, dtype=np.float64
                ).reshape(-1, 3),
                ekf_position=np.asarray(
                    self._history_ekf_pos, dtype=np.float64
                ).reshape(-1, 3),
                ekf_velocity=np.asarray(
                    self._history_ekf_vel, dtype=np.float64
                ).reshape(-1, 3),
                phase_timeline=list(self._phase_timeline),
                event_log=self._state_machine.event_log,
                e_ct=np.asarray(self._history_e_ct, dtype=np.float64),
                gps_update_times=np.asarray(
                    self._history_gps_times, dtype=np.float64
                ),
                gps_skipped=self._gps_skipped,
                gps_gated=self._gps_gated,
                metrics={
                    "duration_s": duration_s,
                    "gps_accepted": self._gps_accepted,
                    "gps_gated": self._gps_gated,
                    "gps_skipped": self._gps_skipped,
                    "ekf_converged": self._ekf_converged,
                    "ekf_diverged": self._ekf_diverged,
                    "final_phase": self._state_machine.phase.value,
                    "waypoints_reached": (
                        self._mission.active_index + 1
                        if self._mission is not None
                        else 0
                    ),
                    "mission_complete": (
                        1.0 if (self._mission and self._mission.complete) else 0.0
                    ),
                    "final_truth_altitude_m": float(
                        self._ground_z
                        - np.asarray(self._truth.position)[2]
                    ),
                    "final_truth_vz_mps": float(
                        np.asarray(self._truth.velocity)[2]
                    ),
                    "command_parse_errors": self._command_parse_errors,
                },
            )
            self._result = result
        return result


__all__ = [
    "COMMAND_EVENT_MAP",
    "CONVERGENCE_MIN_GPS_UPDATES",
    "CONVERGENCE_POS_TRACE_M2",
    "DIVERGENCE_GATED_STREAK",
    "DIVERGENCE_MIN_TIME_S",
    "DIVERGENCE_POS_ERROR_M",
    "SILConfig",
    "SILResult",
    "SILRunner",
]
