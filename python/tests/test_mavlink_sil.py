"""Tests for the Phase 5 MAVLink codec and the UDP SIL runner.

UDP ports are unique per test (15550/15551, 15560/15561, 15570/15571) so
re-runs and the three-reliability check never collide with a lingering
socket. All sims run realtime=False and are short (2-10 s of sim time,
which is well under a few seconds of wall time on this stack); the
COMMAND_LONG test sends repeatedly while the SIL thread is alive because
the non-realtime loop can finish faster than a single sleep().
"""

from __future__ import annotations

import math
import socket
import time

import numpy as np
import pytest
from pymavlink.dialects.v20 import common as mavlink2

from talon import State
from talon.battery import BatteryState
from talon.mavlink_codec import MavlinkCodec
from talon.mavlink_sil import SILConfig, SILRunner
from talon.types import FlightPhase, GPSMeasurement, Waypoint


# ---------------------------------------------------------------------------
# Codec round-trips (parse proves the pymavlink CRC too)
# ---------------------------------------------------------------------------


def _roundtrip(data: bytes) -> dict:
    return MavlinkCodec.parse(data)


def test_heartbeat_roundtrip_and_phase_mapping() -> None:
    codec = MavlinkCodec(system_id=7, component_id=3)
    fields = _roundtrip(codec.heartbeat(FlightPhase.RTL, armed=True))
    assert fields["msg_type"] == "HEARTBEAT"
    assert fields["type"] == mavlink2.MAV_TYPE_QUADROTOR
    assert fields["custom_mode"] == list(FlightPhase).index(FlightPhase.RTL)
    assert fields["base_mode"] & mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    assert fields["base_mode"] & mavlink2.MAV_MODE_FLAG_SAFETY_ARMED
    assert fields["system_status"] == mavlink2.MAV_STATE_ACTIVE


def test_heartbeat_idle_disarmed_is_standby() -> None:
    codec = MavlinkCodec()
    fields = _roundtrip(codec.heartbeat(FlightPhase.IDLE, armed=False))
    assert fields["base_mode"] == mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    assert fields["system_status"] == mavlink2.MAV_STATE_STANDBY


def test_attitude_roundtrip() -> None:
    codec = MavlinkCodec()
    state = State()
    # 90 deg yaw, level: R = Rot_z(90).
    half = math.pi / 4.0
    state.orientation = type(state.orientation)(
        math.cos(half), 0.0, 0.0, math.sin(half)
    )
    state.angular_rate = np.array([0.1, -0.2, 0.3])

    fields = _roundtrip(codec.attitude(state, time_boot_ms=1234))
    assert fields["msg_type"] == "ATTITUDE"
    assert fields["time_boot_ms"] == 1234
    assert fields["roll"] == pytest.approx(0.0, abs=1e-6)
    assert fields["pitch"] == pytest.approx(0.0, abs=1e-6)
    assert fields["yaw"] == pytest.approx(math.pi / 2.0, rel=1e-6)
    assert fields["rollspeed"] == pytest.approx(0.1, rel=1e-6)
    assert fields["pitchspeed"] == pytest.approx(-0.2, rel=1e-6)
    assert fields["yawspeed"] == pytest.approx(0.3, rel=1e-6)


def test_local_position_ned_roundtrip() -> None:
    codec = MavlinkCodec()
    state = State()
    state.position = np.array([1.5, -2.25, -10.0])
    state.velocity = np.array([0.5, 0.0, -0.125])

    fields = _roundtrip(codec.local_position_ned(state, time_boot_ms=99))
    assert fields["msg_type"] == "LOCAL_POSITION_NED"
    assert fields["time_boot_ms"] == 99
    assert fields["x"] == pytest.approx(1.5, rel=1e-6)
    assert fields["y"] == pytest.approx(-2.25, rel=1e-6)
    assert fields["z"] == pytest.approx(-10.0, rel=1e-6)
    assert fields["vx"] == pytest.approx(0.5, rel=1e-6)
    assert fields["vz"] == pytest.approx(-0.125, rel=1e-6)
    assert all(
        math.isfinite(fields[name]) for name in ("x", "y", "z", "vx", "vy", "vz")
    )


def test_gps_raw_int_roundtrip() -> None:
    codec = MavlinkCodec()
    gps = GPSMeasurement(
        timestamp=1.5,
        lat=1.0e-5,
        lon=-2.0e-5,
        alt=12.34,
        vn=3.0,
        ve=4.0,
        vd=0.1,
    )
    fields = _roundtrip(codec.gps_raw_int(gps))
    assert fields["msg_type"] == "GPS_RAW_INT"
    assert fields["fix_type"] == 3
    assert fields["lat"] == 100
    assert fields["lon"] == -200
    assert fields["alt"] == 12340
    assert fields["vel"] == 500  # hypot(3, 4) * 100 cm/s
    assert fields["satellites_visible"] == 12


def test_battery_status_roundtrip() -> None:
    codec = MavlinkCodec()
    battery = BatteryState(soc=0.87, voltage=11.0, current_a=4.2)
    fields = _roundtrip(codec.battery_status(battery, current=4.2))
    assert fields["msg_type"] == "BATTERY_STATUS"
    assert fields["voltages"][0] == 11000
    assert fields["voltages"][1] == 65535
    assert fields["current_battery"] == 420
    assert fields["battery_remaining"] == 87


def test_command_long_roundtrip() -> None:
    fields = _roundtrip(MavlinkCodec.command_long(40001, param1=0.5))
    assert fields["msg_type"] == "COMMAND_LONG"
    assert fields["command"] == 40001
    assert fields["param1"] == pytest.approx(0.5)


def test_parse_rejects_garbage_and_truncation() -> None:
    with pytest.raises(ValueError):
        MavlinkCodec.parse(b"\x00\x01\x02")
    with pytest.raises(ValueError):
        MavlinkCodec.parse(b"")
    # A truncated VALID frame must also fail (CRC can never complete).
    good = MavlinkCodec().heartbeat(FlightPhase.ARM, armed=False)
    with pytest.raises(ValueError):
        MavlinkCodec.parse(good[:-2])


def test_codec_ids_stamped_in_frame() -> None:
    codec = MavlinkCodec(system_id=42, component_id=11)
    frame = codec.heartbeat(FlightPhase.CRUISE, armed=True)
    # MAVLink v2 frame: magic, len, incompat, compat, seq, sysid, compid.
    assert frame[0] == 0xFD
    assert frame[5] == 42
    assert frame[6] == 11


def test_codec_rejects_bad_ids() -> None:
    with pytest.raises(ValueError):
        MavlinkCodec(system_id=0)
    with pytest.raises(ValueError):
        MavlinkCodec(component_id=300)


# ---------------------------------------------------------------------------
# UDP telemetry integration
# ---------------------------------------------------------------------------


def test_udp_telemetry_stream() -> None:
    telemetry_port, command_port = 15550, 15551
    config = SILConfig(
        telemetry_port=telemetry_port, command_port=command_port
    )
    runner = SILRunner(config)
    # Bind the receiver BEFORE the SIL starts sending so no burst is lost.
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", telemetry_port))
    receiver.settimeout(0.5)

    # 10 s of sim time: HEARTBEAT is divided to 1 Hz, so a 2 s run would
    # only produce ~2 heartbeats (fewer than the 5 the test drains).
    thread = runner.start_async(10.0)
    heartbeats: list[dict] = []
    positions: list[dict] = []
    end = time.monotonic() + 15.0
    drain_timeouts = 0
    try:
        while time.monotonic() < end and (
            len(heartbeats) < 5 or len(positions) < 5
        ):
            try:
                data, _address = receiver.recvfrom(4096)
            except socket.timeout:
                drain_timeouts += 1
                # After the SIL thread ends, allow a few extra timeouts to
                # drain anything still sitting in the socket buffer.
                if not thread.is_alive() and drain_timeouts > 3:
                    break
                continue
            drain_timeouts = 0
            fields = MavlinkCodec.parse(data)
            if fields["msg_type"] == "HEARTBEAT":
                heartbeats.append(fields)
            elif fields["msg_type"] == "LOCAL_POSITION_NED":
                positions.append(fields)
    finally:
        receiver.close()
    thread.join(timeout=15.0)
    assert not thread.is_alive()

    assert len(heartbeats) >= 5
    assert len(positions) >= 5
    for fields in heartbeats:
        assert fields["type"] == mavlink2.MAV_TYPE_QUADROTOR
        assert fields["custom_mode"] in range(len(FlightPhase))
    for fields in positions:
        for name in ("x", "y", "z", "vx", "vy", "vz"):
            assert math.isfinite(fields[name])


def test_telemetry_rate_dividers() -> None:
    # Per-message dividers off the 20 Hz burst (SILConfig docstring):
    # HEARTBEAT and BATTERY_STATUS at 1 Hz, ATTITUDE and
    # LOCAL_POSITION_NED at 20 Hz, GPS_RAW_INT at 10 Hz. Over 6 s of sim
    # time that is ~6 / 6 / 120 / 120 / 60 messages; the bands below
    # allow a couple of datagrams of slack. The drain loop runs until
    # the thread ends and the socket goes quiet, so counts are exact up
    # to UDP buffer overflow (not expected at this volume).
    telemetry_port, command_port = 15580, 15581
    config = SILConfig(
        telemetry_port=telemetry_port, command_port=command_port
    )
    runner = SILRunner(config)
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", telemetry_port))
    receiver.settimeout(0.5)

    thread = runner.start_async(6.0)
    counts: dict[str, int] = {}
    drain_timeouts = 0
    try:
        while True:
            try:
                data, _address = receiver.recvfrom(4096)
            except socket.timeout:
                drain_timeouts += 1
                if not thread.is_alive() and drain_timeouts > 3:
                    break
                continue
            drain_timeouts = 0
            fields = MavlinkCodec.parse(data)
            msg_type = fields["msg_type"]
            counts[msg_type] = counts.get(msg_type, 0) + 1
    finally:
        receiver.close()
    thread.join(timeout=15.0)
    assert not thread.is_alive()

    assert 4 <= counts.get("HEARTBEAT", 0) <= 8
    assert 4 <= counts.get("BATTERY_STATUS", 0) <= 8
    assert 50 <= counts.get("GPS_RAW_INT", 0) <= 68
    assert 110 <= counts.get("ATTITUDE", 0) <= 130
    assert 110 <= counts.get("LOCAL_POSITION_NED", 0) <= 130


def test_command_long_arm_roundtrip() -> None:
    telemetry_port, command_port = 15560, 15561
    config = SILConfig(
        telemetry_port=telemetry_port, command_port=command_port
    )
    runner = SILRunner(config)
    thread = runner.start_async(8.0)

    # The non-realtime loop is fast: keep sending until the thread ends so
    # at least one datagram lands on the bound command socket.
    payload = MavlinkCodec.command_long(40001)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sends = 0
    while thread.is_alive():
        sender.sendto(payload, ("127.0.0.1", command_port))
        sends += 1
        time.sleep(0.005)
    thread.join(timeout=15.0)
    sender.close()
    assert sends >= 1

    result = runner.result
    assert result is not None
    arm_events = [e for e in result.event_log if e.event == "arm"]
    assert arm_events, "COMMAND_LONG arm never reached the state machine"
    assert any(e.guard_ok for e in arm_events)
    # The pre-roll converged the EKF, so the arm guard must PASS and the
    # machine must leave IDLE.
    assert runner.state_machine.phase is not FlightPhase.IDLE


def test_gps_outage_injection_freezes_updates() -> None:
    config = SILConfig(enable_udp=False)
    runner = SILRunner(config)
    runner.schedule(0.5, lambda: runner.inject_gps_outage(10.0))
    result = runner.run(3.0)

    accepted_times = result.gps_update_times
    assert accepted_times.size > 0, "no GPS updates before the outage"
    assert float(np.max(accepted_times)) <= 0.5 + 1.0e-9
    # Every GPS opportunity inside the 2.5 s outage window was skipped.
    assert result.gps_skipped >= 20
    # Nothing gated: the outage SKIPS fixes rather than feeding bad ones.
    assert result.gps_gated == 0


def test_stop_interrupts_async_run() -> None:
    config = SILConfig(enable_udp=False)
    runner = SILRunner(config)
    thread = runner.start_async(200.0)
    time.sleep(0.05)
    runner.stop()
    thread.join(timeout=15.0)
    assert not thread.is_alive()
    assert runner.result is not None


def test_sil_config_validation() -> None:
    with pytest.raises(ValueError):
        SILConfig(takeoff_altitude=0.0)
    with pytest.raises(ValueError):
        SILConfig(start_position_ned=[0.0, 0.0])
    with pytest.raises(ValueError):
        SILConfig(telemetry_rate_hz=0.0)


def test_inject_ekf_divergence_rejects_unknown_mode() -> None:
    runner = SILRunner(SILConfig(enable_udp=False))
    with pytest.raises(ValueError):
        runner.inject_ekf_divergence("sparkle")


def test_inject_gps_outage_rejects_nonpositive() -> None:
    runner = SILRunner(SILConfig(enable_udp=False))
    with pytest.raises(ValueError):
        runner.inject_gps_outage(0.0)


def test_inject_gps_outage_rejects_a_non_finite_duration() -> None:
    # `inf <= 0.0` and `nan <= 0.0` are both False, so the positivity guard
    # alone accepted them (the hardened Rust twin rejects them too).
    runner = SILRunner(SILConfig(enable_udp=False))
    for bad in (float("inf"), float("nan")):
        with pytest.raises(
            ValueError, match="gps outage duration must be finite"
        ):
            runner.inject_gps_outage(bad)
    # Shaped-but-invalid input keeps the original message.
    with pytest.raises(ValueError, match="duration_s must be positive"):
        runner.inject_gps_outage(0.0)
    with pytest.raises(ValueError, match="duration_s must be positive"):
        runner.inject_gps_outage(-1.0)


def test_request_event_at_time_is_scheduled() -> None:
    config = SILConfig(
        enable_udp=False,
        waypoints=[Waypoint(np.array([10.0, 0.0, -10.0]))],
    )
    runner = SILRunner(config)
    runner.request_event("arm", at_time=0.0)
    runner.request_event("takeoff", at_time=0.5)
    result = runner.run(8.0)
    arm = [e for e in result.event_log if e.event == "arm" and e.guard_ok]
    takeoff = [
        e for e in result.event_log if e.event == "takeoff" and e.guard_ok
    ]
    assert arm and takeoff
    assert takeoff[0].time >= arm[0].time
    assert FlightPhase.TAKEOFF in result.phases_visited()


def test_parse_error_counter_counts_garbage() -> None:
    # A garbage datagram on the command port must not kill the loop; it
    # increments the parse-error metric instead.
    config = SILConfig(telemetry_port=15570, command_port=15571)
    runner = SILRunner(config)
    thread = runner.start_async(8.0)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while thread.is_alive():
        sender.sendto(b"not mavlink at all", ("127.0.0.1", 15571))
        time.sleep(0.005)
    thread.join(timeout=15.0)
    sender.close()
    result = runner.result
    assert result is not None
    assert result.metrics["command_parse_errors"] >= 1
    # The sim still ran its full course on the ground.
    assert result.time_s.size > 0


def test_command_event_mapping_documented_commands() -> None:
    from talon.mavlink_sil import COMMAND_EVENT_MAP

    assert COMMAND_EVENT_MAP[mavlink2.MAV_CMD_NAV_TAKEOFF] == "takeoff"
    assert (
        COMMAND_EVENT_MAP[mavlink2.MAV_CMD_NAV_RETURN_TO_LAUNCH] == "rtl"
    )
    assert COMMAND_EVENT_MAP[40001] == "arm"
    assert COMMAND_EVENT_MAP[40002] == "mission_start"


def test_command_long_without_a_command_field_is_counted_not_raised(
    monkeypatch,
) -> None:
    # A COMMAND_LONG that parses but carries no command id must be counted
    # as a parse error: `int(fields["command"])` raised KeyError out of the
    # drain loop and killed the sim thread.
    config = SILConfig(enable_udp=False)
    runner = SILRunner(config)
    # enable_udp=False keeps run() from binding the configured ports, so the
    # command socket is attached by hand on an ephemeral port. Non-blocking,
    # like _open_sockets makes it, so the drain loop can return when the
    # queue is empty.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.setblocking(False)
    runner._command_sock = sock
    monkeypatch.setattr(
        MavlinkCodec, "parse", lambda data: {"msg_type": "COMMAND_LONG"}
    )
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sender.sendto(b"\x00", sock.getsockname())
        time.sleep(0.01)
        runner._drain_commands()  # must not raise KeyError
        assert list(runner._event_queue) == []
        assert runner._command_parse_errors >= 1
    finally:
        sender.close()
        sock.close()
        runner._command_sock = None


def test_run_rejects_a_non_finite_duration() -> None:
    # JSON `1e999` reaches Python as math.inf; `inf <= 0.0` is False, so the
    # positivity guard alone accepted it and `round(inf / DT)` then blew up
    # with an opaque OverflowError deep inside the loop.
    runner = SILRunner(SILConfig(enable_udp=False))
    for bad in (float("inf"), float("nan")):
        with pytest.raises(ValueError, match="finite"):
            runner.run(bad)
    # The existing rejection message is unchanged for valid-shaped input.
    with pytest.raises(ValueError, match="positive"):
        runner.run(0.0)
    with pytest.raises(ValueError, match="positive"):
        runner.run(-1.0)
