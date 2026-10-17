"""MAVLink v2 message codec (Phase 5), built on pymavlink.

MAVLink v2 framing and CRC are handled entirely by pymavlink (the CRC is
computed inside ``Message.pack`` and verified inside ``MAVLink.parse_char``);
this module only chooses field values and packs/parses bytes. Two usage
patterns are available in pymavlink; Talon uses the LOW-LEVEL one:

    pack:   mav = common.MAVLink(io.BytesIO(), srcSystem=..., ...)
            data = common.MAVLink_attitude_message(...).pack(mav)
    parse:  mav = common.MAVLink(io.BytesIO())
            msg = mav.parse_char(byte)  # accumulate per byte

``mavutil.mavlink_connection`` would work too but binds sockets / opens
files, which the codec does not need: the SIL owns its UDP sockets itself
(see talon.mavlink_sil) and tests want a pure function of bytes.

Heartbeat mapping: base_mode always carries MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
and additionally MAV_MODE_FLAG_SAFETY_ARMED while armed; custom_mode is the
FlightPhase ORDINAL (IDLE=0 ... EMERGENCY=7, the declaration order of
talon.types.FlightPhase; FlightPhase itself is a str-valued Enum so
``int(phase)`` is undefined and the ordinal is the stable int encoding).
system_status is STANDBY in IDLE/ARM and ACTIVE in every flying phase.

The attitude / local_position_ned time_boot_ms fields default to the codec
instance's monotonic wall clock (milliseconds, wrapped to uint32) so a bare
``codec.attitude(state)`` still produces a plausible stream; the SIL passes
explicit simulation time instead.
"""

from __future__ import annotations

import io
import math
import time
import typing

import numpy as np
from pymavlink.dialects.v20 import common as mavlink2

from talon._core import State
from talon.battery import BatteryState
from talon.types import FlightPhase, GPSMeasurement

# FlightPhase ordinal -> heartbeat custom_mode. Declaration order is the
# public contract (IDLE=0, ARM=1, ..., EMERGENCY=7).
_PHASE_TO_CUSTOM_MODE: dict[FlightPhase, int] = {
    phase: index for index, phase in enumerate(FlightPhase)
}

# MAVLink "invalid / unknown" fill values used by this codec.
_UNKNOWN_INT16 = -1
_UNKNOWN_UINT16 = 65535
_UNKNOWN_TEMP_C = 32767  # degC * 100, MAVLink "unknown" for temperature


class MavlinkCodec:
    """Builds and parses MAVLink v2 frames for the Talon SIL link."""

    def __init__(self, system_id: int = 1, component_id: int = 1) -> None:
        if not (0 < system_id < 256):
            raise ValueError(f"system_id must be in [1, 255], got {system_id}")
        if not (0 < component_id < 256):
            raise ValueError(
                f"component_id must be in [1, 255], got {component_id}"
            )
        self._system_id = system_id
        self._component_id = component_id
        # Writer used only for pack(); its .write sink is never consulted
        # because pack() returns the framed bytes directly.
        self._writer = mavlink2.MAVLink(
            io.BytesIO(), srcSystem=system_id, srcComponent=component_id
        )

    @property
    def system_id(self) -> int:
        """The source system id stamped on outgoing frames."""
        return self._system_id

    @property
    def component_id(self) -> int:
        """The source component id stamped on outgoing frames."""
        return self._component_id

    def _pack(self, message: typing.Any) -> bytes:
        return bytes(message.pack(self._writer))

    def _time_boot_ms(self, override: int | None) -> int:
        if override is not None:
            return int(override) & 0xFFFFFFFF
        return int(time.monotonic() * 1000.0) & 0xFFFFFFFF

    def heartbeat(
        self, phase: FlightPhase, armed: bool, time_boot_ms: int | None = None
    ) -> bytes:
        """HEARTBEAT: quadrotor, custom_mode = phase ordinal.

        base_mode = CUSTOM_MODE_ENABLED [| SAFETY_ARMED]; system_status is
        STANDBY in IDLE/ARM, ACTIVE otherwise. time_boot_ms is accepted for
        symmetry but MAVLink HEARTBEAT carries no time field.
        """
        del time_boot_ms  # HEARTBEAT has no time field
        base_mode = mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if armed:
            base_mode |= mavlink2.MAV_MODE_FLAG_SAFETY_ARMED
        if phase in (FlightPhase.IDLE, FlightPhase.ARM):
            system_status = mavlink2.MAV_STATE_STANDBY
        else:
            system_status = mavlink2.MAV_STATE_ACTIVE
        message = mavlink2.MAVLink_heartbeat_message(
            type=mavlink2.MAV_TYPE_QUADROTOR,
            autopilot=mavlink2.MAV_AUTOPILOT_GENERIC,
            base_mode=base_mode,
            custom_mode=_PHASE_TO_CUSTOM_MODE[phase],
            system_status=system_status,
            mavlink_version=3,
        )
        return self._pack(message)

    def attitude(
        self, state: State, time_boot_ms: int | None = None
    ) -> bytes:
        """ATTITUDE: Euler angles / body rates from a truth State."""
        dcm = np.asarray(state.orientation.to_dcm(), dtype=np.float64)
        roll = math.atan2(dcm[2, 1], dcm[2, 2])
        pitch = math.asin(min(1.0, max(-1.0, -dcm[2, 0])))
        yaw = math.atan2(dcm[1, 0], dcm[0, 0])
        rate = np.asarray(state.angular_rate, dtype=np.float64)
        message = mavlink2.MAVLink_attitude_message(
            time_boot_ms=self._time_boot_ms(time_boot_ms),
            roll=float(roll),
            pitch=float(pitch),
            yaw=float(yaw),
            rollspeed=float(rate[0]),
            pitchspeed=float(rate[1]),
            yawspeed=float(rate[2]),
        )
        return self._pack(message)

    def local_position_ned(
        self, state: State, time_boot_ms: int | None = None
    ) -> bytes:
        """LOCAL_POSITION_NED: position / velocity from a truth State.

        time_boot_ms defaults to the codec wall clock, symmetric with
        attitude(); the SIL passes explicit simulation time instead.
        """
        position = np.asarray(state.position, dtype=np.float64)
        velocity = np.asarray(state.velocity, dtype=np.float64)
        message = mavlink2.MAVLink_local_position_ned_message(
            time_boot_ms=self._time_boot_ms(time_boot_ms),
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2]),
            vx=float(velocity[0]),
            vy=float(velocity[1]),
            vz=float(velocity[2]),
        )
        return self._pack(message)

    def gps_raw_int(
        self,
        gps: GPSMeasurement,
        fix_type: int = 3,
        time_usec: int | None = None,
    ) -> bytes:
        """GPS_RAW_INT from a Talon GPSMeasurement.

        lat/lon are degrees * 1e7, alt is MSL mm (the flat-Earth sim altitude
        passes straight through), vel is the horizontal ground speed in cm/s.
        eph/epv are unitless DOP placeholders scaled x100 per the MAVLink
        GPS_RAW_INT contract (eph 80 = HDOP 0.80, epv 120 = VDOP 1.20; the
        sim has no real DOP model); satellites_visible is a fixed 12.
        """
        if time_usec is None:
            time_usec = int(round(gps.timestamp * 1e6))
        speed = math.hypot(gps.vn, gps.ve)
        message = mavlink2.MAVLink_gps_raw_int_message(
            time_usec=int(time_usec),
            fix_type=int(fix_type),
            lat=int(round(gps.lat * 1e7)),
            lon=int(round(gps.lon * 1e7)),
            alt=int(round(gps.alt * 1000.0)),
            eph=80,
            epv=120,
            vel=int(round(speed * 100.0)),
            cog=_UNKNOWN_UINT16,
            satellites_visible=12,
        )
        return self._pack(message)

    def battery_status(
        self, battery: BatteryState, current: float
    ) -> bytes:
        """BATTERY_STATUS: single pack, voltages in mV, current in cA.

        ``current`` (A) is the caller's measured pack current and overrides
        ``battery.current_a`` (kept in the signature per the SIL spec so a
        measured shunt value can differ from the model's estimate).
        """
        voltage_mv = int(round(max(0.0, battery.voltage) * 1000.0))
        voltages = [voltage_mv] + [_UNKNOWN_UINT16] * 9
        message = mavlink2.MAVLink_battery_status_message(
            id=0,
            battery_function=mavlink2.MAV_BATTERY_FUNCTION_ALL,
            type=mavlink2.MAV_BATTERY_TYPE_LIPO,
            temperature=_UNKNOWN_TEMP_C,
            voltages=voltages,
            current_battery=int(round(current * 100.0)),
            current_consumed=_UNKNOWN_INT16,
            energy_consumed=_UNKNOWN_INT16,
            battery_remaining=int(round(min(1.0, max(0.0, battery.soc)) * 100.0)),
        )
        return self._pack(message)

    @staticmethod
    def command_long(
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
        target_system: int = 1,
        target_component: int = 1,
    ) -> bytes:
        """COMMAND_LONG frame (GCS/test side helper; needs no codec state)."""
        writer = mavlink2.MAVLink(io.BytesIO())
        message = mavlink2.MAVLink_command_long_message(
            target_system=target_system,
            target_component=target_component,
            command=int(command),
            confirmation=0,
            param1=float(param1),
            param2=float(param2),
            param3=float(param3),
            param4=float(param4),
            param5=float(param5),
            param6=float(param6),
            param7=float(param7),
        )
        return bytes(message.pack(writer))

    @staticmethod
    def parse(data: bytes) -> dict:
        """Decode the FIRST complete MAVLink frame in ``data``.

        Returns {'msg_type': str, <all message fields>}. Raises ValueError
        when no complete frame is present (garbage, truncated frame, or a
        frame whose CRC pymavlink rejects; pymavlink's MAVError is
        normalized to ValueError so callers have one failure contract).
        Frames after the first are ignored: one call decodes one message,
        matching the SIL's one-datagram-one-frame UDP usage.
        """
        parser = mavlink2.MAVLink(io.BytesIO())
        message = None
        for index in range(len(data)):
            try:
                parsed = parser.parse_char(data[index : index + 1])
            except mavlink2.MAVError as exc:
                raise ValueError(f"invalid MAVLink data: {exc}") from exc
            if parsed is not None:
                message = parsed
                break
        if message is None:
            raise ValueError(
                f"no complete MAVLink frame in {len(data)} bytes of data"
            )
        fields = message.to_dict()
        fields.pop("mavpackettype", None)
        return {"msg_type": message.get_type(), **fields}


__all__ = ["MavlinkCodec", "_PHASE_TO_CUSTOM_MODE"]
