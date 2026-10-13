"""Unit tests for the Phase 5 flight-phase state machine."""

from __future__ import annotations

import pytest

from talon.state_machine import (
    FlightPhaseMachine,
    PhaseTransition,
    Telemetry,
)
from talon.types import FlightPhase

HOME = [0.0, 0.0, 0.0]


def telemetry(**overrides) -> Telemetry:
    """Telemetry snapshot with every guard-passing default."""
    fields: dict = {
        "position_ned": [0.0, 0.0, -10.0],  # 10 m above home
        "velocity_ned": [0.0, 0.0, 0.0],
        "battery_soc": 0.90,
        "ekf_converged": True,
        "ekf_diverged": False,
        "gps_outage_duration": 0.0,
        "mission_waypoint_count": 3,
        "mission_complete": False,
        "motors_armed": True,
        "home_position_ned": HOME,
        "emergency_descent_active": False,
        "motor_fault": False,
    }
    fields.update(overrides)
    return Telemetry(**fields)


def landed_telemetry() -> Telemetry:
    return telemetry(position_ned=[0.0, 0.0, -0.05], velocity_ned=[0, 0, -0.05])


def force(machine: FlightPhaseMachine, phase: FlightPhase) -> None:
    machine.force_phase(phase, 0.0, "test setup")


# ---------------------------------------------------------------------------
# Happy path: every table transition with passing guards
# ---------------------------------------------------------------------------


def test_happy_path_full_sequence() -> None:
    machine = FlightPhaseMachine(takeoff_altitude=10.0)
    assert machine.phase is FlightPhase.IDLE

    assert machine.request("arm", telemetry(), 0.0)
    assert machine.phase is FlightPhase.ARM

    assert machine.request("takeoff", telemetry(), 0.1)
    assert machine.phase is FlightPhase.TAKEOFF

    assert machine.request("altitude_reached", telemetry(), 6.0)
    assert machine.phase is FlightPhase.CRUISE

    assert machine.request("mission_start", telemetry(), 8.0)
    assert machine.phase is FlightPhase.WAYPOINT

    assert machine.request("mission_complete", telemetry(mission_complete=True), 20.0)
    assert machine.phase is FlightPhase.RTL

    assert machine.request("home_reached", telemetry(), 30.0)
    assert machine.phase is FlightPhase.LAND

    assert machine.request("landed", landed_telemetry(), 40.0)
    assert machine.phase is FlightPhase.IDLE

    accepted = [entry for entry in machine.event_log if entry.guard_ok]
    assert len(accepted) == 7
    assert [entry.to_phase for entry in accepted] == [
        FlightPhase.ARM,
        FlightPhase.TAKEOFF,
        FlightPhase.CRUISE,
        FlightPhase.WAYPOINT,
        FlightPhase.RTL,
        FlightPhase.LAND,
        FlightPhase.IDLE,
    ]


def test_low_battery_triggers_waypoint_to_rtl() -> None:
    machine = FlightPhaseMachine()
    force(machine, FlightPhase.WAYPOINT)
    assert machine.request(
        "low_battery", telemetry(battery_soc=0.19), 1.0
    )
    assert machine.phase is FlightPhase.RTL


def test_mission_complete_triggers_waypoint_to_rtl() -> None:
    machine = FlightPhaseMachine()
    force(machine, FlightPhase.WAYPOINT)
    assert machine.request(
        "mission_complete", telemetry(mission_complete=True), 1.0
    )
    assert machine.phase is FlightPhase.RTL


def test_gps_lost_triggers_waypoint_to_emergency() -> None:
    machine = FlightPhaseMachine()
    force(machine, FlightPhase.WAYPOINT)
    assert machine.request(
        "gps_lost", telemetry(gps_outage_duration=5.01), 1.0
    )
    assert machine.phase is FlightPhase.EMERGENCY


def test_gps_lost_waypoint_only_documents_phase_dependence() -> None:
    # The transition table has a gps_lost entry from WAYPOINT ONLY: the
    # SIL fires the event only while in WAYPOINT, and from any other
    # phase the (phase, gps_lost) lookup simply misses. Consequence, by
    # design: a mission that COMPLETES during a long outage wins the
    # same-step check (mission_complete is evaluated first) and the
    # vehicle flies RTL home on the dead-reckoned estimate instead of
    # taking an EMERGENCY descent.
    outage = telemetry(gps_outage_duration=30.0)
    non_waypoint_phases = [
        FlightPhase.IDLE,
        FlightPhase.ARM,
        FlightPhase.TAKEOFF,
        FlightPhase.CRUISE,
        FlightPhase.RTL,
        FlightPhase.LAND,
        FlightPhase.EMERGENCY,
    ]
    machine = FlightPhaseMachine()
    for phase in non_waypoint_phases:
        force(machine, phase)
        assert machine.request("gps_lost", outage, 1.0) is False
        assert machine.phase is phase

    # The same 30 s outage from WAYPOINT DOES fire (guard is > 5.0 s).
    machine2 = FlightPhaseMachine()
    force(machine2, FlightPhase.WAYPOINT)
    assert machine2.request("gps_lost", outage, 1.0)
    assert machine2.phase is FlightPhase.EMERGENCY


def test_emergency_landing_guard_requires_descent_flag() -> None:
    machine = FlightPhaseMachine()
    force(machine, FlightPhase.EMERGENCY)
    assert (
        machine.request(
            "emergency_landing",
            telemetry(emergency_descent_active=False),
            1.0,
        )
        is False
    )
    assert machine.phase is FlightPhase.EMERGENCY
    assert machine.request(
        "emergency_landing",
        telemetry(emergency_descent_active=True),
        2.0,
    )
    assert machine.phase is FlightPhase.LAND


def test_operator_rtl_extension_from_cruise_and_waypoint() -> None:
    # Documented extension: MAV_CMD_NAV_RETURN_TO_LAUNCH maps to "rtl".
    machine = FlightPhaseMachine()
    force(machine, FlightPhase.CRUISE)
    assert machine.request("rtl", telemetry(), 1.0)
    assert machine.phase is FlightPhase.RTL

    machine2 = FlightPhaseMachine()
    force(machine2, FlightPhase.WAYPOINT)
    assert machine2.request("rtl", telemetry(), 1.0)
    assert machine2.phase is FlightPhase.RTL


# ---------------------------------------------------------------------------
# Guard rejections: each failing guard keeps the phase, logs the rejection
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("phase", "event", "bad_telemetry"),
    [
        (FlightPhase.IDLE, "arm", telemetry(ekf_converged=False)),
        (FlightPhase.IDLE, "arm", telemetry(battery_soc=0.30)),
        (FlightPhase.ARM, "takeoff", telemetry(motors_armed=False)),
        # altitude above home 9.0 <= 10 - 0.5
        (
            FlightPhase.TAKEOFF,
            "altitude_reached",
            telemetry(position_ned=[0.0, 0.0, -9.0]),
        ),
        (FlightPhase.CRUISE, "mission_start", telemetry(mission_waypoint_count=0)),
        (
            FlightPhase.WAYPOINT,
            "mission_complete",
            telemetry(mission_complete=False),
        ),
        (FlightPhase.WAYPOINT, "low_battery", telemetry(battery_soc=0.20)),
        (FlightPhase.WAYPOINT, "gps_lost", telemetry(gps_outage_duration=5.0)),
        # horizontal distance 1.5 >= 1.0
        (
            FlightPhase.RTL,
            "home_reached",
            telemetry(position_ned=[1.5, 0.0, -10.0]),
        ),
        # altitude 0.2 >= 0.1
        (
            FlightPhase.LAND,
            "landed",
            telemetry(position_ned=[0.0, 0.0, -0.2], velocity_ned=[0, 0, -0.05]),
        ),
        # |vz| 0.15 >= 0.1
        (
            FlightPhase.LAND,
            "landed",
            telemetry(position_ned=[0.0, 0.0, -0.05], velocity_ned=[0, 0, -0.15]),
        ),
    ],
)
def test_guard_rejection_keeps_phase_and_logs(
    phase: FlightPhase, event: str, bad_telemetry: Telemetry
) -> None:
    machine = FlightPhaseMachine()
    force(machine, phase)
    log_before = len(machine.event_log)

    assert machine.request(event, bad_telemetry, 1.0) is False
    assert machine.phase is phase
    assert len(machine.event_log) == log_before + 1
    entry = machine.event_log[-1]
    assert entry.event == event
    assert entry.guard_ok is False
    assert entry.from_phase == phase
    assert entry.to_phase == phase


def test_unknown_event_from_phase_is_rejected() -> None:
    machine = FlightPhaseMachine()
    # "takeoff" is not a valid event from IDLE.
    assert machine.request("takeoff", telemetry(), 0.0) is False
    assert machine.phase is FlightPhase.IDLE
    assert machine.event_log[-1].guard_ok is False


# ---------------------------------------------------------------------------
# critical_fault from EVERY state
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("phase", list(FlightPhase))
def test_critical_fault_diverged_from_every_state(phase: FlightPhase) -> None:
    machine = FlightPhaseMachine()
    force(machine, phase)
    assert machine.request(
        "critical_fault", telemetry(ekf_diverged=True), 1.0
    )
    assert machine.phase is FlightPhase.EMERGENCY
    entry = machine.event_log[-1]
    if phase is FlightPhase.EMERGENCY:
        # Idempotent: accepted, same-phase audit entry, no transition.
        assert entry.event == "critical_fault:already_emergency"
        assert entry.from_phase == FlightPhase.EMERGENCY
        assert entry.to_phase == FlightPhase.EMERGENCY
        assert entry.guard_ok is True
        return
    assert entry.from_phase == phase
    assert entry.to_phase is FlightPhase.EMERGENCY
    assert entry.event == "critical_fault"
    assert entry.guard_ok is True


@pytest.mark.parametrize("phase", list(FlightPhase))
def test_critical_fault_motor_from_every_state(phase: FlightPhase) -> None:
    machine = FlightPhaseMachine()
    force(machine, phase)
    assert machine.request(
        "critical_fault", telemetry(motor_fault=True), 1.0
    )
    assert machine.phase is FlightPhase.EMERGENCY


def test_critical_fault_is_idempotent_in_emergency() -> None:
    machine = FlightPhaseMachine()
    force(machine, FlightPhase.EMERGENCY)
    log_before = len(machine.event_log)
    assert machine.request("critical_fault", telemetry(ekf_diverged=True), 1.0)
    assert machine.phase is FlightPhase.EMERGENCY
    # Same-phase audit entry, not a transition: accepted (guard_ok=True),
    # from == to == EMERGENCY, reason encoded in the event name.
    assert len(machine.event_log) == log_before + 1
    entry = machine.event_log[-1]
    assert entry.event == "critical_fault:already_emergency"
    assert entry.from_phase is FlightPhase.EMERGENCY
    assert entry.to_phase is FlightPhase.EMERGENCY
    assert entry.guard_ok is True


def test_critical_fault_rejected_when_healthy() -> None:
    machine = FlightPhaseMachine()
    force(machine, FlightPhase.CRUISE)
    assert machine.request("critical_fault", telemetry(), 1.0) is False
    assert machine.phase is FlightPhase.CRUISE
    assert machine.event_log[-1].guard_ok is False


def test_arm_rejected_from_emergency() -> None:
    # "arm" is a valid IDLE table event with a passing guard here
    # (converged, healthy battery), but the table has no EMERGENCY ->
    # ARM entry, so the attempt is rejected and the machine stays in
    # EMERGENCY.
    machine = FlightPhaseMachine()
    force(machine, FlightPhase.EMERGENCY)
    assert machine.request("arm", telemetry(ekf_diverged=True), 1.0) is False
    assert machine.phase is FlightPhase.EMERGENCY


# ---------------------------------------------------------------------------
# Event log shape
# ---------------------------------------------------------------------------


def test_event_log_timestamps_monotonic_and_fields() -> None:
    machine = FlightPhaseMachine()
    machine.request("arm", telemetry(), 0.0)
    machine.request("takeoff", telemetry(), 0.5)
    machine.request("takeoff", telemetry(motors_armed=False), 0.7)  # rejected
    machine.request("altitude_reached", telemetry(), 6.0)

    times = [entry.time for entry in machine.event_log]
    assert times == sorted(times)
    assert all(isinstance(entry, PhaseTransition) for entry in machine.event_log)
    assert [entry.event for entry in machine.event_log] == [
        "arm",
        "takeoff",
        "takeoff",
        "altitude_reached",
    ]
    assert [entry.guard_ok for entry in machine.event_log] == [
        True,
        True,
        False,
        True,
    ]


def test_event_log_property_returns_copy() -> None:
    machine = FlightPhaseMachine()
    log = machine.event_log
    log.append(None)  # type: ignore[arg-type]
    assert machine.event_log == []


# ---------------------------------------------------------------------------
# force_phase hook
# ---------------------------------------------------------------------------


def test_force_phase_sets_and_logs() -> None:
    machine = FlightPhaseMachine()
    machine.force_phase(FlightPhase.RTL, 3.0, "unit test")
    assert machine.phase is FlightPhase.RTL
    entry = machine.event_log[-1]
    assert entry.from_phase is FlightPhase.IDLE
    assert entry.to_phase is FlightPhase.RTL
    assert entry.event == "force:unit test"
    assert entry.guard_ok is True


def test_force_phase_same_phase_is_silent() -> None:
    machine = FlightPhaseMachine()
    machine.force_phase(FlightPhase.IDLE, 0.0, "noop")
    assert machine.event_log == []


def test_force_phase_rejects_non_phase() -> None:
    machine = FlightPhaseMachine()
    with pytest.raises(TypeError):
        machine.force_phase("cruise", 0.0, "bad")  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# Telemetry validation and derived fields
# ---------------------------------------------------------------------------


def test_telemetry_derived_fields() -> None:
    tele = telemetry(
        position_ned=[3.0, 4.0, -12.0], home_position_ned=[0.0, 0.0, -2.0]
    )
    assert tele.altitude_above_home == pytest.approx(10.0)
    assert tele.horizontal_dist_to_home == pytest.approx(5.0)
    assert tele.vel_z == 0.0


def test_telemetry_rejects_bad_shapes() -> None:
    with pytest.raises(ValueError):
        telemetry(position_ned=[0.0, 0.0])
    with pytest.raises(ValueError):
        telemetry(velocity_ned=[0.0, 0.0])


def test_machine_rejects_non_telemetry() -> None:
    machine = FlightPhaseMachine()
    with pytest.raises(TypeError):
        machine.request("arm", {"position_ned": [0, 0, 0]}, 0.0)  # type: ignore[arg-type]


def test_takeoff_altitude_guard_uses_constructor_value() -> None:
    machine = FlightPhaseMachine(takeoff_altitude=5.0)
    force(machine, FlightPhase.TAKEOFF)
    # 4.6 m above a 5.0 m takeoff target: 4.6 > 4.5 passes.
    tele = telemetry(position_ned=[0.0, 0.0, -4.6])
    assert machine.request("altitude_reached", tele, 1.0)
    assert machine.phase is FlightPhase.CRUISE


def test_takeoff_altitude_must_be_positive() -> None:
    with pytest.raises(ValueError):
        FlightPhaseMachine(takeoff_altitude=0.0)
