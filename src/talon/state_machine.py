"""Flight-phase state machine (Phase 5).

An 8-state guarded transition machine over :class:`~talon.types.FlightPhase`
(IDLE, ARM, TAKEOFF, CRUISE, WAYPOINT, RTL, LAND, EMERGENCY). Every state
change goes through :meth:`FlightPhaseMachine.request`, which evaluates one
event against the transition table plus its guard on the current
:class:`Telemetry` snapshot. Accepted and rejected attempts are both
appended to ``event_log`` (:class:`PhaseTransition`, sim-time stamped) so a
post-flight review can reconstruct exactly when and why the phase changed.

Transition table (guards read the Telemetry snapshot; "alt" is altitude
above home = home.d - position.d, positive up):

    IDLE      -> ARM       on "arm"               if ekf_converged and soc > 0.30
    ARM       -> TAKEOFF   on "takeoff"           if motors_armed
    TAKEOFF   -> CRUISE    on "altitude_reached"  if alt > takeoff_alt - 0.5
    CRUISE    -> WAYPOINT  on "mission_start"     if mission_waypoint_count > 0
    WAYPOINT  -> RTL       on "mission_complete"  if mission_complete
    WAYPOINT  -> RTL       on "low_battery"       if soc < 0.20
    WAYPOINT  -> EMERGENCY on "gps_lost"          if gps_outage_duration > 5.0 s
    RTL       -> LAND      on "home_reached"      if horizontal dist to home < 1.0 m
    LAND      -> IDLE      on "landed"            if alt < 0.1 m and |vz| < 0.1 m/s
    EMERGENCY -> LAND      on "emergency_landing" if emergency_descent_active
    ANY       -> EMERGENCY on "critical_fault"    if ekf_diverged or motor_fault

``critical_fault`` is checked FIRST in :meth:`request`, regardless of the
current phase, so EMERGENCY is reachable from every state in one call.

Operator-RTL extension (beyond the spec table, documented): the MAVLink
SIL maps ``MAV_CMD_NAV_RETURN_TO_LAUNCH`` to an "rtl" event, so TAKEOFF,
CRUISE, and WAYPOINT also accept "rtl" with an always-true guard (an
operator command needs no health guard; the LAND/IDLE descent logic is
identical regardless of what sent the vehicle home).

``ekf_converged`` / ``ekf_diverged`` are caller-computed (the state machine
does not own the estimator): the SIL defines them from the EKF covariance
trace, accepted-update count, and EKF-vs-truth error; see
talon.mavlink_sil for the exact definitions. Guards like
``emergency_descent_active`` are likewise flags maintained by the caller
(scenario / SIL).
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from talon.types import FlightPhase

ArrayF = NDArray[np.float64]


@dataclass(slots=True)
class Telemetry:
    """Snapshot of everything the guards read, at one instant.

    Position/velocity are the EKF estimates (the autopilot's view); home is
    the recorded launch point (NED, m). altitude_above_home and
    horizontal_dist_to_home are derived (positive altitude = above home).
    """

    position_ned: ArrayF
    velocity_ned: ArrayF
    battery_soc: float
    ekf_converged: bool
    ekf_diverged: bool
    gps_outage_duration: float
    mission_waypoint_count: int
    mission_complete: bool
    motors_armed: bool
    home_position_ned: ArrayF
    emergency_descent_active: bool = False
    motor_fault: bool = False

    def __post_init__(self) -> None:
        self.position_ned = np.asarray(self.position_ned, dtype=np.float64)
        self.velocity_ned = np.asarray(self.velocity_ned, dtype=np.float64)
        self.home_position_ned = np.asarray(
            self.home_position_ned, dtype=np.float64
        )
        if self.position_ned.shape != (3,):
            raise ValueError(
                f"position_ned must have shape (3,), got {self.position_ned.shape}"
            )
        if self.velocity_ned.shape != (3,):
            raise ValueError(
                f"velocity_ned must have shape (3,), got {self.velocity_ned.shape}"
            )
        if self.home_position_ned.shape != (3,):
            raise ValueError(
                "home_position_ned must have shape (3,), "
                f"got {self.home_position_ned.shape}"
            )

    @property
    def altitude_above_home(self) -> float:
        """Altitude above the home point (m, positive up; NED-safe)."""
        return float(self.home_position_ned[2] - self.position_ned[2])

    @property
    def horizontal_dist_to_home(self) -> float:
        """Horizontal (North-East plane) distance to home (m)."""
        ne = self.position_ned[:2] - self.home_position_ned[:2]
        return float(np.linalg.norm(ne))

    @property
    def vel_z(self) -> float:
        """NED down velocity (m/s, positive descending)."""
        return float(self.velocity_ned[2])


@dataclass(slots=True)
class PhaseTransition:
    """One logged event evaluation (accepted or rejected)."""

    time: float
    from_phase: FlightPhase
    to_phase: FlightPhase
    event: str
    guard_ok: bool


# Guards take (telemetry, takeoff_altitude) and return True when the
# transition may fire. takeoff_altitude is only used by altitude_reached.
_GuardFn = Callable[[Telemetry, float], bool]


def _g_arm(t: Telemetry, _takeoff_alt: float) -> bool:
    return t.ekf_converged and t.battery_soc > 0.30


def _g_takeoff(t: Telemetry, _takeoff_alt: float) -> bool:
    return t.motors_armed


def _g_altitude_reached(t: Telemetry, takeoff_alt: float) -> bool:
    return t.altitude_above_home > takeoff_alt - 0.5


def _g_mission_start(t: Telemetry, _takeoff_alt: float) -> bool:
    return t.mission_waypoint_count > 0


def _g_mission_complete(t: Telemetry, _takeoff_alt: float) -> bool:
    return t.mission_complete


def _g_low_battery(t: Telemetry, _takeoff_alt: float) -> bool:
    return t.battery_soc < 0.20


def _g_gps_lost(t: Telemetry, _takeoff_alt: float) -> bool:
    return t.gps_outage_duration > 5.0


def _g_home_reached(t: Telemetry, _takeoff_alt: float) -> bool:
    return t.horizontal_dist_to_home < 1.0


def _g_landed(t: Telemetry, _takeoff_alt: float) -> bool:
    return t.altitude_above_home < 0.1 and abs(t.vel_z) < 0.1


def _g_emergency_landing(t: Telemetry, _takeoff_alt: float) -> bool:
    return t.emergency_descent_active


def _g_operator(_t: Telemetry, _takeoff_alt: float) -> bool:
    """Always-true guard for operator commands (rtl)."""
    return True


# (from_phase, event) -> (to_phase, guard).
_TRANSITIONS: dict[
    tuple[FlightPhase, str], tuple[FlightPhase, _GuardFn]
] = {
    (FlightPhase.IDLE, "arm"): (FlightPhase.ARM, _g_arm),
    (FlightPhase.ARM, "takeoff"): (FlightPhase.TAKEOFF, _g_takeoff),
    (FlightPhase.TAKEOFF, "altitude_reached"): (
        FlightPhase.CRUISE,
        _g_altitude_reached,
    ),
    (FlightPhase.CRUISE, "mission_start"): (
        FlightPhase.WAYPOINT,
        _g_mission_start,
    ),
    (FlightPhase.WAYPOINT, "mission_complete"): (
        FlightPhase.RTL,
        _g_mission_complete,
    ),
    (FlightPhase.WAYPOINT, "low_battery"): (FlightPhase.RTL, _g_low_battery),
    (FlightPhase.WAYPOINT, "gps_lost"): (
        FlightPhase.EMERGENCY,
        _g_gps_lost,
    ),
    (FlightPhase.RTL, "home_reached"): (FlightPhase.LAND, _g_home_reached),
    (FlightPhase.LAND, "landed"): (FlightPhase.IDLE, _g_landed),
    (FlightPhase.EMERGENCY, "emergency_landing"): (
        FlightPhase.LAND,
        _g_emergency_landing,
    ),
    # Operator-RTL extension (module docstring).
    (FlightPhase.TAKEOFF, "rtl"): (FlightPhase.RTL, _g_operator),
    (FlightPhase.CRUISE, "rtl"): (FlightPhase.RTL, _g_operator),
    (FlightPhase.WAYPOINT, "rtl"): (FlightPhase.RTL, _g_operator),
}


class FlightPhaseMachine:
    """Guarded 8-state flight phase machine with a transition event log."""

    def __init__(self, takeoff_altitude: float = 10.0) -> None:
        if takeoff_altitude <= 0.0:
            raise ValueError(
                f"takeoff_altitude must be positive, got {takeoff_altitude}"
            )
        self._takeoff_altitude = takeoff_altitude
        self._phase = FlightPhase.IDLE
        self._event_log: list[PhaseTransition] = []

    @property
    def phase(self) -> FlightPhase:
        """The current flight phase."""
        return self._phase

    @property
    def event_log(self) -> list[PhaseTransition]:
        """All event evaluations, oldest first (accepted and rejected)."""
        return list(self._event_log)

    @property
    def takeoff_altitude(self) -> float:
        """The takeoff altitude the altitude_reached guard uses (m)."""
        return self._takeoff_altitude

    def request(self, event: str, telemetry: Telemetry, time_s: float) -> bool:
        """Evaluate one event against the current phase.

        critical_fault is checked first (ANY -> EMERGENCY) regardless of the
        current phase. Otherwise the (phase, event) table entry and its
        guard decide. On success the phase changes, the transition is
        logged with guard_ok=True, and True is returned. On failure the
        attempt is logged with guard_ok=False (to_phase stays the current
        phase) and False is returned. An already-EMERGENCY critical_fault
        is idempotent: accepted (True), logged as
        "critical_fault:already_emergency" (same-phase entry for the
        audit trail) rather than as a phase transition.
        """
        if not isinstance(telemetry, Telemetry):
            raise TypeError(
                f"telemetry must be a Telemetry, got {type(telemetry).__name__}"
            )
        if event == "critical_fault":
            if telemetry.ekf_diverged or telemetry.motor_fault:
                if self._phase is FlightPhase.EMERGENCY:
                    # Already in EMERGENCY: accept without changing phase,
                    # but leave an audit entry (guard_ok=True, same phase).
                    self._log(
                        time_s,
                        self._phase,
                        self._phase,
                        "critical_fault:already_emergency",
                        True,
                    )
                    return True
                self._log(
                    time_s, self._phase, FlightPhase.EMERGENCY, event, True
                )
                self._phase = FlightPhase.EMERGENCY
                return True
            self._log(time_s, self._phase, self._phase, event, False)
            return False

        entry = _TRANSITIONS.get((self._phase, event))
        if entry is None:
            # Event not applicable from the current phase.
            self._log(time_s, self._phase, self._phase, event, False)
            return False
        target, guard = entry
        if not guard(telemetry, self._takeoff_altitude):
            self._log(time_s, self._phase, self._phase, event, False)
            return False
        self._log(time_s, self._phase, target, event, True)
        self._phase = target
        return True

    def force_phase(
        self, phase: FlightPhase, time_s: float, reason: str
    ) -> None:
        """Test hook: set the phase directly, logging the jump.

        The event field of the log entry is "force:<reason>" so a forced
        jump is never mistaken for a guarded transition in post-analysis.
        """
        if not isinstance(phase, FlightPhase):
            raise TypeError(f"phase must be a FlightPhase, got {type(phase)}")
        if phase is self._phase:
            return
        self._log(time_s, self._phase, phase, f"force:{reason}", True)
        self._phase = phase

    def _log(
        self,
        time_s: float,
        from_phase: FlightPhase,
        to_phase: FlightPhase,
        event: str,
        guard_ok: bool,
    ) -> None:
        self._event_log.append(
            PhaseTransition(
                time=float(time_s),
                from_phase=from_phase,
                to_phase=to_phase,
                event=event,
                guard_ok=guard_ok,
            )
        )


__all__ = ["ArrayF", "FlightPhaseMachine", "PhaseTransition", "Telemetry"]
