"""Python vs ``talon._core`` parity for the flight-phase state machine.

The transition table, every guard at its exact boundary and one step
inside, the event log compared row for row, the critical-fault path and
the ``force_phase`` hook are exercised on both implementations, plus three
registered divergences: the NaN takeoff altitude Python silently accepts,
the ``isinstance`` checks Python performs that the duck-typed boundary
does not, and the plain label strings the boundary accepts in
``force_phase``. One ``Telemetry`` instance drives both machines, which is
exactly what the duck-typed boundary is for.
"""

from __future__ import annotations

import pytest

from parity_util import assert_exact, assert_same_error, expect_divergence
from talon import _core
from talon.state_machine import FlightPhaseMachine, Telemetry
from talon.types import FlightPhase

HOME = [0.0, 0.0, 0.0]
PHASES = list(FlightPhase)


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


def machines(
    takeoff_altitude: float = 10.0,
) -> tuple[FlightPhaseMachine, "_core.FlightPhaseMachine"]:
    return FlightPhaseMachine(takeoff_altitude), _core.FlightPhaseMachine(
        takeoff_altitude
    )


def force_both(
    py_machine: FlightPhaseMachine,
    rs_machine: "_core.FlightPhaseMachine",
    phase: FlightPhase,
    time_s: float = 0.0,
    reason: str = "test setup",
) -> None:
    py_machine.force_phase(phase, time_s, reason)
    rs_machine.force_phase(phase, time_s, reason)


def py_rows(machine: FlightPhaseMachine) -> list[dict]:
    """The Python log as plain dicts, comparable with the binding rows."""
    return [
        {
            "time": entry.time,
            "event": entry.event,
            "from_phase": entry.from_phase.value,
            "to_phase": entry.to_phase.value,
            "guard_ok": entry.guard_ok,
        }
        for entry in machine.event_log
    ]


def assert_same_log(
    py_machine: FlightPhaseMachine, rs_machine: "_core.FlightPhaseMachine"
) -> None:
    py_log, rs_log = py_rows(py_machine), list(rs_machine.event_log())
    assert len(rs_log) == len(py_log), f"rust {rs_log} vs python {py_log}"
    for index, (py_row, rs_row) in enumerate(zip(py_log, rs_log)):
        for key in ("event", "from_phase", "to_phase", "guard_ok"):
            assert rs_row[key] == py_row[key], f"row {index} {key}: {rs_row} vs {py_row}"
        assert_exact(rs_row["time"], py_row["time"], label=f"row {index} time")


def assert_same_phase(
    py_machine: FlightPhaseMachine, rs_machine: "_core.FlightPhaseMachine"
) -> None:
    assert rs_machine.phase == py_machine.phase.value


# ---------------------------------------------------------------------------
# Defaults and the nominal flight
# ---------------------------------------------------------------------------


def test_defaults_parity() -> None:
    py_machine, rs_machine = machines()
    assert rs_machine.phase == "idle" == py_machine.phase.value
    assert_exact(rs_machine.takeoff_altitude, py_machine.takeoff_altitude)
    assert_exact(rs_machine.takeoff_altitude, 10.0)
    assert py_machine.event_log == []
    assert list(rs_machine.event_log()) == []


def test_nominal_sequence_parity() -> None:
    # The full nominal flight, comparing every return value, the phase
    # after each step, and the complete event log row for row at the end.
    py_machine, rs_machine = machines()
    steps = [
        ("arm", telemetry(), 0.0, FlightPhase.ARM),
        ("takeoff", telemetry(), 1.0, FlightPhase.TAKEOFF),
        ("altitude_reached", telemetry(), 12.0, FlightPhase.CRUISE),
        ("mission_start", telemetry(mission_waypoint_count=4), 13.0, FlightPhase.WAYPOINT),
        ("mission_complete", telemetry(mission_complete=True), 60.0, FlightPhase.RTL),
        ("home_reached", telemetry(), 70.0, FlightPhase.LAND),
        ("landed", telemetry(position_ned=[0.0, 0.0, -0.05], velocity_ned=[0, 0, -0.05]), 75.0, FlightPhase.IDLE),
    ]
    for event, snapshot, time_s, target in steps:
        py_ok = py_machine.request(event, snapshot, time_s)
        rs_ok = rs_machine.request(event, snapshot, time_s)
        assert py_ok is True and rs_ok is True, event
        assert py_machine.phase is target
        assert rs_machine.phase == target.value
        assert_same_phase(py_machine, rs_machine)
    assert len(py_machine.event_log) == 7
    assert all(entry.guard_ok for entry in py_machine.event_log)
    assert_same_log(py_machine, rs_machine)


# ---------------------------------------------------------------------------
# Guard thresholds: the exact boundary and one step inside
# ---------------------------------------------------------------------------


BOUNDARY_CASES = [
    pytest.param(
        FlightPhase.IDLE, "arm", telemetry(battery_soc=0.30),
        telemetry(battery_soc=0.31), id="arm_soc_strict_0.30",
    ),
    pytest.param(
        FlightPhase.IDLE, "arm", telemetry(ekf_converged=False),
        telemetry(), id="arm_requires_ekf_convergence",
    ),
    pytest.param(
        FlightPhase.ARM, "takeoff", telemetry(motors_armed=False),
        telemetry(), id="takeoff_requires_armed_motors",
    ),
    pytest.param(
        FlightPhase.TAKEOFF, "altitude_reached",
        telemetry(position_ned=[0.0, 0.0, -9.5]),
        telemetry(position_ned=[0.0, 0.0, -9.6]),
        id="altitude_reached_margin_0.5",
    ),
    pytest.param(
        FlightPhase.CRUISE, "mission_start", telemetry(mission_waypoint_count=0),
        telemetry(mission_waypoint_count=1), id="mission_start_needs_a_waypoint",
    ),
    pytest.param(
        FlightPhase.WAYPOINT, "mission_complete", telemetry(mission_complete=False),
        telemetry(mission_complete=True), id="mission_complete_flag",
    ),
    pytest.param(
        FlightPhase.WAYPOINT, "low_battery", telemetry(battery_soc=0.20),
        telemetry(battery_soc=0.19), id="low_battery_strict_0.20",
    ),
    pytest.param(
        FlightPhase.WAYPOINT, "gps_lost", telemetry(gps_outage_duration=5.0),
        telemetry(gps_outage_duration=5.01), id="gps_lost_strict_5.0s",
    ),
    pytest.param(
        FlightPhase.RTL, "home_reached", telemetry(position_ned=[1.0, 0.0, -10.0]),
        telemetry(position_ned=[0.9, 0.0, -10.0]), id="home_reached_strict_1.0m",
    ),
    pytest.param(
        FlightPhase.LAND, "landed",
        telemetry(position_ned=[0.0, 0.0, -0.1], velocity_ned=[0, 0, 0.05]),
        telemetry(position_ned=[0.0, 0.0, -0.05], velocity_ned=[0, 0, 0.05]),
        id="landed_alt_strict_0.1m",
    ),
    pytest.param(
        FlightPhase.LAND, "landed",
        telemetry(position_ned=[0.0, 0.0, -0.05], velocity_ned=[0, 0, 0.1]),
        telemetry(position_ned=[0.0, 0.0, -0.05], velocity_ned=[0, 0, -0.05]),
        id="landed_vz_strict_0.1mps",
    ),
    pytest.param(
        FlightPhase.LAND, "landed",
        telemetry(position_ned=[0.0, 0.0, -0.05], velocity_ned=[0, 0, 0.5]),
        telemetry(position_ned=[0.0, 0.0, -0.05], velocity_ned=[0, 0, 0.05]),
        id="landed_requires_low_speed",
    ),
    pytest.param(
        FlightPhase.LAND, "landed",
        telemetry(position_ned=[0.0, 0.0, -0.5], velocity_ned=[0, 0, 0.05]),
        telemetry(position_ned=[0.0, 0.0, -0.05], velocity_ned=[0, 0, 0.05]),
        id="landed_requires_low_altitude",
    ),
    pytest.param(
        FlightPhase.EMERGENCY, "emergency_landing",
        telemetry(emergency_descent_active=False),
        telemetry(emergency_descent_active=True),
        id="emergency_landing_requires_descent_flag",
    ),
]


@pytest.mark.parametrize(("phase", "event", "bad", "good"), BOUNDARY_CASES)
def test_guard_threshold_parity(
    phase: FlightPhase,
    event: str,
    bad: Telemetry,
    good: Telemetry,
) -> None:
    # At the exact threshold the strict inequality fails on both sides,
    # one step inside it passes; the phase and the logged row agree.
    py_machine, rs_machine = machines()
    force_both(py_machine, rs_machine, phase)
    py_ok = py_machine.request(event, bad, 1.0)
    rs_ok = rs_machine.request(event, bad, 1.0)
    assert py_ok is False and rs_ok is False
    assert_same_phase(py_machine, rs_machine)
    assert py_machine.phase is phase
    assert_same_log(py_machine, rs_machine)
    last = rs_machine.event_log()[-1]
    assert last["guard_ok"] is False
    assert last["from_phase"] == last["to_phase"] == phase.value

    py_machine, rs_machine = machines()
    force_both(py_machine, rs_machine, phase)
    py_ok = py_machine.request(event, good, 2.0)
    rs_ok = rs_machine.request(event, good, 2.0)
    assert py_ok is True and rs_ok is True
    assert_same_phase(py_machine, rs_machine)
    assert_same_log(py_machine, rs_machine)
    assert rs_machine.event_log()[-1]["guard_ok"] is True


def test_landed_requires_both_conditions() -> None:
    # Neither condition alone fires the LAND -> IDLE transition on either
    # implementation: low altitude with a fast descent is rejected, and so
    # is a slow descent from too high.
    for snapshot in (
        telemetry(position_ned=[0.0, 0.0, -0.05], velocity_ned=[0, 0, 0.5]),
        telemetry(position_ned=[0.0, 0.0, -0.5], velocity_ned=[0, 0, 0.05]),
    ):
        py_machine, rs_machine = machines()
        force_both(py_machine, rs_machine, FlightPhase.LAND)
        py_ok = py_machine.request("landed", snapshot, 1.0)
        rs_ok = rs_machine.request("landed", snapshot, 1.0)
        assert py_ok is False and rs_ok is False
        assert_same_phase(py_machine, rs_machine)
        assert_same_log(py_machine, rs_machine)


# ---------------------------------------------------------------------------
# critical_fault from every phase, idempotence, and guard-false logging
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("phase", PHASES)
@pytest.mark.parametrize(
    ("flag", "bad"),
    [
        pytest.param("ekf_diverged", True, id="ekf_diverged"),
        pytest.param("motor_fault", True, id="motor_fault"),
    ],
)
def test_critical_fault_from_every_phase(phase: FlightPhase, flag: str, bad: bool) -> None:
    py_machine, rs_machine = machines()
    force_both(py_machine, rs_machine, phase)
    snapshot = telemetry(**{flag: bad})
    py_ok = py_machine.request("critical_fault", snapshot, 1.0)
    rs_ok = rs_machine.request("critical_fault", snapshot, 1.0)
    assert py_ok is True and rs_ok is True
    assert py_machine.phase is FlightPhase.EMERGENCY
    assert rs_machine.phase == "emergency"
    assert_same_log(py_machine, rs_machine)
    last = rs_machine.event_log()[-1]
    if phase is FlightPhase.EMERGENCY:
        # Idempotent: accepted, same-phase audit entry, no transition.
        assert last["event"] == "critical_fault:already_emergency"
        assert last["from_phase"] == last["to_phase"] == "emergency"
        assert last["guard_ok"] is True
    else:
        assert last["event"] == "critical_fault"
        assert last["from_phase"] == phase.value
        assert last["to_phase"] == "emergency"
        assert last["guard_ok"] is True


def test_critical_fault_idempotence_parity() -> None:
    py_machine, rs_machine = machines()
    force_both(py_machine, rs_machine, FlightPhase.EMERGENCY)
    py_ok = py_machine.request("critical_fault", telemetry(ekf_diverged=True), 1.0)
    rs_ok = rs_machine.request("critical_fault", telemetry(ekf_diverged=True), 1.0)
    assert py_ok is True and rs_ok is True
    assert_same_phase(py_machine, rs_machine)
    assert_same_log(py_machine, rs_machine)
    # A second fault while still EMERGENCY logs the same audit entry.
    py_ok = py_machine.request("critical_fault", telemetry(motor_fault=True), 2.0)
    rs_ok = rs_machine.request("critical_fault", telemetry(motor_fault=True), 2.0)
    assert py_ok is True and rs_ok is True
    assert_same_log(py_machine, rs_machine)


def test_critical_fault_guard_false_logging_parity() -> None:
    # Healthy telemetry: both sides reject, keep the phase and log the
    # attempt with guard_ok False and from == to == current phase.
    py_machine, rs_machine = machines()
    force_both(py_machine, rs_machine, FlightPhase.CRUISE)
    py_ok = py_machine.request("critical_fault", telemetry(), 1.0)
    rs_ok = rs_machine.request("critical_fault", telemetry(), 1.0)
    assert py_ok is False and rs_ok is False
    assert_same_phase(py_machine, rs_machine)
    assert_same_log(py_machine, rs_machine)
    assert len(rs_machine.event_log()) == 2  # force + rejection


# ---------------------------------------------------------------------------
# Events not applicable from the current phase, and operator RTL
# ---------------------------------------------------------------------------


NOT_APPLICABLE_CASES = [
    pytest.param(FlightPhase.IDLE, "takeoff", id="takeoff_from_idle"),
    pytest.param(FlightPhase.EMERGENCY, "arm", id="arm_from_emergency"),
    pytest.param(FlightPhase.CRUISE, "landed", id="landed_from_cruise"),
    # Documented table phase dependence: gps_lost only exists from WAYPOINT,
    # so even a 30 s outage does nothing elsewhere.
    pytest.param(FlightPhase.CRUISE, "gps_lost", id="gps_lost_from_cruise"),
]


@pytest.mark.parametrize(("phase", "event"), NOT_APPLICABLE_CASES)
def test_event_not_applicable_parity(phase: FlightPhase, event: str) -> None:
    py_machine, rs_machine = machines()
    force_both(py_machine, rs_machine, phase)
    snapshot = telemetry(gps_outage_duration=30.0)
    py_ok = py_machine.request(event, snapshot, 1.0)
    rs_ok = rs_machine.request(event, snapshot, 1.0)
    assert py_ok is False and rs_ok is False
    assert_same_phase(py_machine, rs_machine)
    assert_same_log(py_machine, rs_machine)
    assert rs_machine.event_log()[-1]["guard_ok"] is False


@pytest.mark.parametrize(
    "phase",
    [
        pytest.param(FlightPhase.TAKEOFF, id="from_takeoff"),
        pytest.param(FlightPhase.CRUISE, id="from_cruise"),
        pytest.param(FlightPhase.WAYPOINT, id="from_waypoint"),
    ],
)
def test_operator_rtl_parity(phase: FlightPhase) -> None:
    # Documented extension: the MAVLink SIL maps
    # MAV_CMD_NAV_RETURN_TO_LAUNCH to an "rtl" event with an always-true
    # guard.
    py_machine, rs_machine = machines()
    force_both(py_machine, rs_machine, phase)
    py_ok = py_machine.request("rtl", telemetry(), 1.0)
    rs_ok = rs_machine.request("rtl", telemetry(), 1.0)
    assert py_ok is True and rs_ok is True
    assert py_machine.phase is FlightPhase.RTL
    assert rs_machine.phase == "rtl"
    assert_same_log(py_machine, rs_machine)


@pytest.mark.parametrize(
    "phase",
    [
        pytest.param(FlightPhase.IDLE, id="from_idle"),
        pytest.param(FlightPhase.ARM, id="from_arm"),
        pytest.param(FlightPhase.RTL, id="from_rtl"),
        pytest.param(FlightPhase.LAND, id="from_land"),
    ],
)
def test_operator_rtl_not_applicable_parity(phase: FlightPhase) -> None:
    py_machine, rs_machine = machines()
    force_both(py_machine, rs_machine, phase)
    py_ok = py_machine.request("rtl", telemetry(), 1.0)
    rs_ok = rs_machine.request("rtl", telemetry(), 1.0)
    assert py_ok is False and rs_ok is False
    assert_same_phase(py_machine, rs_machine)
    assert_same_log(py_machine, rs_machine)


# ---------------------------------------------------------------------------
# force_phase hook
# ---------------------------------------------------------------------------


def test_force_phase_and_same_phase_noop_parity() -> None:
    py_machine, rs_machine = machines()
    force_both(py_machine, rs_machine, FlightPhase.CRUISE, time_s=2.0, reason="scenario")
    assert py_machine.phase is FlightPhase.CRUISE
    assert rs_machine.phase == "cruise"
    assert_same_log(py_machine, rs_machine)
    assert rs_machine.event_log()[-1]["event"] == "force:scenario"
    assert rs_machine.event_log()[-1]["guard_ok"] is True

    # Forcing the current phase is a silent no-op on both sides.
    force_both(py_machine, rs_machine, FlightPhase.CRUISE, time_s=3.0, reason="again")
    assert_same_log(py_machine, rs_machine)
    assert len(rs_machine.event_log()) == 1


def test_force_phase_logs_the_jump_parity() -> None:
    py_machine, rs_machine = machines()
    force_both(py_machine, rs_machine, FlightPhase.RTL, time_s=3.0, reason="unit test")
    assert_same_phase(py_machine, rs_machine)
    assert_same_log(py_machine, rs_machine)
    last = rs_machine.event_log()[-1]
    assert last == {
        "time": 3.0,
        "event": "force:unit test",
        "from_phase": "idle",
        "to_phase": "rtl",
        "guard_ok": True,
    }


# ---------------------------------------------------------------------------
# Constructor validation and telemetry-derived quantities
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("bad", "message"),
    [
        pytest.param(0.0, "takeoff_altitude must be positive, got 0.0", id="zero"),
        pytest.param(-1.0, "takeoff_altitude must be positive, got -1.0", id="negative"),
        pytest.param(-0.0, "takeoff_altitude must be positive, got -0.0", id="negative_zero"),
    ],
)
def test_bad_takeoff_altitude_error_parity(bad: float, message: str) -> None:
    with pytest.raises(ValueError) as py_error:
        FlightPhaseMachine(bad)
    with pytest.raises(ValueError) as rust_error:
        _core.FlightPhaseMachine(bad)
    assert_same_error(py_error.value, rust_error.value, label=f"alt {bad!r}")
    assert str(rust_error.value) == message


def test_altitude_above_home_is_ned_safe_on_both_sides() -> None:
    # The guard reads home.d - position.d (positive up): 5 m above home
    # with a 10 m takeoff target must NOT fire, while a naive -position.d
    # reading (35 m) would have fired it.
    snapshot = telemetry(
        position_ned=[0.0, 0.0, -35.0], home_position_ned=[0.0, 0.0, -30.0]
    )
    py_machine, rs_machine = machines()
    force_both(py_machine, rs_machine, FlightPhase.TAKEOFF)
    py_ok = py_machine.request("altitude_reached", snapshot, 1.0)
    rs_ok = rs_machine.request("altitude_reached", snapshot, 1.0)
    assert py_ok is False and rs_ok is False
    assert_same_log(py_machine, rs_machine)

    # One metre higher (6 m above home) still fails; 9.6 m above home fires.
    py_machine, rs_machine = machines()
    force_both(py_machine, rs_machine, FlightPhase.TAKEOFF)
    snapshot = telemetry(
        position_ned=[0.0, 0.0, -39.6], home_position_ned=[0.0, 0.0, -30.0]
    )
    py_ok = py_machine.request("altitude_reached", snapshot, 1.0)
    rs_ok = rs_machine.request("altitude_reached", snapshot, 1.0)
    assert py_ok is True and rs_ok is True
    assert_same_log(py_machine, rs_machine)


@pytest.mark.parametrize(
    ("position", "reaches_home"),
    [
        pytest.param([1.0, 0.0, -10.0], False, id="exactly_1.0m_holds"),
        pytest.param([3.0, 4.0, -10.0], False, id="5.0m_hypotenuse_holds"),
        pytest.param([0.6, 0.0, -10.0], True, id="0.6m_reaches"),
        pytest.param([0.0, 0.0, -10.0], True, id="at_home_reaches"),
    ],
)
def test_horizontal_distance_to_home_parity(position: list[float], reaches_home: bool) -> None:
    # The guard reads the horizontal (north-east) distance only; the
    # 3-4-5 case pins the Pythagorean sum itself.
    snapshot = telemetry(position_ned=position)
    py_machine, rs_machine = machines()
    force_both(py_machine, rs_machine, FlightPhase.RTL)
    py_ok = py_machine.request("home_reached", snapshot, 1.0)
    rs_ok = rs_machine.request("home_reached", snapshot, 1.0)
    assert py_ok is reaches_home and rs_ok is reaches_home
    assert_same_phase(py_machine, rs_machine)
    assert_same_log(py_machine, rs_machine)


@pytest.mark.parametrize(
    "vel_z",
    [
        pytest.param(0.05, id="descending_slowly"),
        pytest.param(-0.05, id="ascending_slowly"),
        pytest.param(0.1, id="descending_at_threshold"),
        pytest.param(-0.1, id="ascending_at_threshold"),
    ],
)
def test_vel_z_symmetry_parity(vel_z: float) -> None:
    # vel_z is velocity_ned[2] (positive descending) and the landed guard
    # tests its absolute value, so both signs behave identically at the
    # threshold on both sides.
    snapshot = telemetry(
        position_ned=[0.0, 0.0, -0.05], velocity_ned=[0.0, 0.0, vel_z]
    )
    py_machine, rs_machine = machines()
    force_both(py_machine, rs_machine, FlightPhase.LAND)
    py_ok = py_machine.request("landed", snapshot, 1.0)
    rs_ok = rs_machine.request("landed", snapshot, 1.0)
    expected = abs(vel_z) < 0.1
    assert py_ok is expected and rs_ok is expected
    assert_same_log(py_machine, rs_machine)


def test_custom_takeoff_altitude_guard_parity() -> None:
    # The altitude_reached guard uses the constructor value on both sides.
    snapshot = telemetry(position_ned=[0.0, 0.0, -4.6])
    py_machine, rs_machine = machines(5.0)
    assert_exact(rs_machine.takeoff_altitude, py_machine.takeoff_altitude)
    force_both(py_machine, rs_machine, FlightPhase.TAKEOFF)
    py_ok = py_machine.request("altitude_reached", snapshot, 1.0)
    rs_ok = rs_machine.request("altitude_reached", snapshot, 1.0)
    assert py_ok is True and rs_ok is True
    assert_same_log(py_machine, rs_machine)


# ---------------------------------------------------------------------------
# Registered divergences
# ---------------------------------------------------------------------------


def test_nan_takeoff_altitude_is_a_registered_divergence() -> None:
    reason = expect_divergence("state_machine.nan_takeoff_altitude")
    # Python's guard is `takeoff_altitude <= 0.0`, and NaN comparisons are
    # all False, so the machine is silently constructed with a NaN guard
    # operand.
    py_machine = FlightPhaseMachine(float("nan"))
    assert py_machine.phase is FlightPhase.IDLE
    # The Rust core rejects NaN explicitly (core/src/state_machine.rs).
    with pytest.raises(ValueError, match="takeoff_altitude must be positive"):
        _core.FlightPhaseMachine(float("nan"))
    assert reason.startswith("Python FlightPhaseMachine(nan) is accepted")


def test_request_type_check_is_a_registered_divergence() -> None:
    reason = expect_divergence("state_machine.request_duck_types_telemetry")
    py_machine, rs_machine = machines()
    with pytest.raises(TypeError, match="telemetry must be a Telemetry"):
        py_machine.request("arm", {"position_ned": [0, 0, 0]}, 1.0)  # type: ignore[arg-type]
    # The boundary duck-types: a dict has no guard fields, so the read
    # fails with AttributeError and the machine is untouched.
    with pytest.raises(AttributeError):
        rs_machine.request("arm", {"position_ned": [0, 0, 0]}, 1.0)
    assert_same_phase(py_machine, rs_machine)
    assert rs_machine.event_log() == []
    assert reason.startswith("Python FlightPhaseMachine.request raises TypeError")


def test_force_phase_label_is_a_registered_divergence() -> None:
    reason = expect_divergence("state_machine.force_phase_accepts_labels")
    py_machine, rs_machine = machines()
    with pytest.raises(TypeError, match="phase must be a FlightPhase"):
        py_machine.force_phase("cruise", 1.0, "bad")  # type: ignore[arg-type]
    # The boundary accepts the plain label the enum's .value carries.
    rs_machine.force_phase("cruise", 1.0, "bad")
    assert rs_machine.phase == FlightPhase.CRUISE.value
    assert rs_machine.event_log()[-1]["event"] == "force:bad"
    # An unknown label is still an error on the boundary.
    with pytest.raises(ValueError, match="unknown flight phase label"):
        rs_machine.force_phase("warp_drive", 2.0, "bad")
    assert reason.startswith("Python FlightPhaseMachine.force_phase raises TypeError")


# ---------------------------------------------------------------------------
# Duck-typed telemetry reader: absent defaults, broken properties propagate
# ---------------------------------------------------------------------------


def test_broken_guard_flag_property_is_propagated() -> None:
    # Security review close-out: the duck-typed reader defaults the
    # optional flags only on AttributeError. An optional flag that exists
    # but raises on access must propagate, not silently read as False and
    # suppress a critical fault. (The Python side cannot drive these
    # objects: it raises TypeError for a non-Telemetry, the registered
    # state_machine.request_duck_types_telemetry divergence, so only the
    # binding side is pinned here.)
    class BrokenTelemetry:
        position_ned = [0.0, 0.0, -10.0]
        velocity_ned = [0.0, 0.0, 0.0]
        battery_soc = 0.9
        ekf_converged = True
        ekf_diverged = False
        gps_outage_duration = 0.0
        mission_waypoint_count = 3
        mission_complete = False
        motors_armed = True
        home_position_ned = [0.0, 0.0, 0.0]
        emergency_descent_active = False

        @property
        def motor_fault(self) -> bool:
            raise RuntimeError("motor_fault read failed")

    rs_machine = _core.FlightPhaseMachine()
    with pytest.raises(RuntimeError, match="motor_fault read failed"):
        rs_machine.request("critical_fault", BrokenTelemetry(), 1.0)
    # The failure left the machine untouched: no phase change, no log row.
    assert rs_machine.phase == "idle"
    assert rs_machine.event_log() == []

    class BrokenDescent(BrokenTelemetry):
        motor_fault = False

        @property
        def emergency_descent_active(self) -> bool:
            raise RuntimeError("emergency_descent_active read failed")

    rs_machine = _core.FlightPhaseMachine()
    rs_machine.force_phase("emergency", 0.0, "test setup")
    with pytest.raises(RuntimeError, match="emergency_descent_active read failed"):
        rs_machine.request("emergency_landing", BrokenDescent(), 1.0)
    assert rs_machine.phase == "emergency"
    assert [row["event"] for row in rs_machine.event_log()] == ["force:test setup"]


def test_absent_optional_guard_flags_still_default() -> None:
    # A genuinely absent optional flag defaults to False, and a property
    # raising AttributeError is indistinguishable from absence in Python's
    # own getattr protocol, so it defaults too.
    class MinimalTelemetry:
        position_ned = [0.0, 0.0, -10.0]
        velocity_ned = [0.0, 0.0, 0.0]
        battery_soc = 0.9
        ekf_converged = True
        ekf_diverged = False
        gps_outage_duration = 0.0
        mission_waypoint_count = 3
        mission_complete = False
        motors_armed = True
        home_position_ned = [0.0, 0.0, 0.0]

    rs_machine = _core.FlightPhaseMachine()
    assert rs_machine.request("arm", MinimalTelemetry(), 1.0) is True
    assert rs_machine.phase == "arm"

    class AttributeGoneTelemetry(MinimalTelemetry):
        @property
        def motor_fault(self) -> bool:
            raise AttributeError("motor_fault is gone")

    rs_machine = _core.FlightPhaseMachine()
    assert rs_machine.request("arm", AttributeGoneTelemetry(), 1.0) is True

    class RealTelemetry:
        position_ned = [0.0, 0.0, -10.0]
        velocity_ned = [0.0, 0.0, 0.0]
        battery_soc = 0.9
        ekf_converged = True
        ekf_diverged = False
        gps_outage_duration = 0.0
        mission_waypoint_count = 3
        mission_complete = False
        motors_armed = True
        home_position_ned = [0.0, 0.0, 0.0]
        emergency_descent_active = False
        motor_fault = True

    rs_machine = _core.FlightPhaseMachine()
    assert rs_machine.request("critical_fault", RealTelemetry(), 1.0) is True
    assert rs_machine.phase == "emergency"
