"""SILResult JSON exporter (Sprint 2.3).

Serializes one :class:`~talon.mavlink_sil.SILResult` to a versioned JSON
document (schema_version "1.0") for post-flight analysis and archival:

- every numpy history becomes a nested list (1-D or (N, 3)), floats
  rounded to ``round_decimals`` (default 6; None keeps full precision);
- the auto-filled ``meta`` block carries schema_version, generated_utc
  (ISO 8601 Z), sample_count, duration_s, the 0.01 s control-rate dt,
  and the units of every data field; caller-supplied meta entries are
  merged in but never overwrite the auto keys;
- ``phase_timeline`` is emitted as ``[[t, "phase_name"], ...]`` sorted by
  time, and ``event_log`` as a list of ``{time, event, from_phase,
  to_phase, guard_ok}`` dicts (PhaseTransition dataclasses or plain
  dicts are both accepted);
- any NaN/Inf in a numeric array raises ValueError before anything is
  written (JSON has no NaN; silently emitting null would corrupt the
  round trip).

:func:`validate_flight_json` re-checks a loaded document against the
same schema (keys, shapes, phase names) and raises ValueError with a
clear message on the first violation.
"""

from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from talon.mavlink_sil import SILResult
from talon.state_machine import PhaseTransition
from talon.types import FlightPhase

SCHEMA_VERSION: str = "1.0"

# Histories are sampled at the CONTROL rate (100 Hz; SILResult docstring).
SAMPLE_DT_S: float = 0.01

# Units of every exported data field (meta.units, 9 keys).
UNITS: dict[str, str] = {
    "time": "s",
    "position": "m NED",
    "velocity": "m/s NED",
    "acceleration": "m/s^2",
    "angular_rate": "rad/s",
    "thrust": "N",
    "moment": "N*m",
    "wind": "m/s",
    "covariance": "m^2",
}

VALID_PHASE_NAMES: frozenset[str] = frozenset(
    phase.value for phase in FlightPhase
)

# Top-level keys of the schema, in emission order (meta first).
REQUIRED_KEYS: tuple[str, ...] = (
    "meta",
    "time_s",
    "truth_position",
    "truth_velocity",
    "ekf_position",
    "ekf_velocity",
    "position_var",
    "gyro_bias",
    "accel_bias",
    "wind_ned",
    "command_thrust",
    "command_moment",
    "e_ct",
    "e_ct_times",
    "e_ct_leg",
    "waypoints_ned",
    "home_ned",
    "phase_timeline",
    "event_log",
)

# (N,) histories that must match len(time_s).
_SERIES_1D: tuple[str, ...] = ("time_s", "command_thrust")
# (N, 3) histories that must match len(time_s).
_SERIES_3D: tuple[str, ...] = (
    "truth_position",
    "truth_velocity",
    "ekf_position",
    "ekf_velocity",
    "position_var",
    "gyro_bias",
    "accel_bias",
    "wind_ned",
    "command_moment",
)
# (M,) cross-track series that must share one length M.
_ECT_1D: tuple[str, ...] = ("e_ct", "e_ct_times", "e_ct_leg")


def _phase_name(phase: FlightPhase | str) -> str:
    """Canonical lowercase phase name for the JSON document.

    FlightPhase.name is the UPPERCASE member name ("IDLE"); the schema
    and validate_flight_json use the lowercase values ("idle"), so enum
    members go through .value and plain strings pass through unchanged.
    """
    if isinstance(phase, FlightPhase):
        return phase.value
    return str(phase)


def _check_finite(name: str, array: np.ndarray) -> None:
    if not np.all(np.isfinite(array)):
        raise ValueError(
            f"{name} contains non-finite values (NaN/Inf); "
            "JSON cannot represent them"
        )


def _to_json_value(value: Any, round_decimals: int | None) -> Any:
    """Recursively convert numpy containers to plain JSON-ready types."""
    if isinstance(value, np.ndarray):
        return _to_json_value(value.tolist(), round_decimals)
    if isinstance(value, (list, tuple)):
        return [_to_json_value(item, round_decimals) for item in value]
    if isinstance(value, (np.floating, np.integer)):
        value = value.item()
    if isinstance(value, bool):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(
                "non-finite value (NaN/Inf) encountered during serialization"
            )
        if round_decimals is not None:
            return round(value, round_decimals)
        return value
    return value


def _event_to_dict(event: PhaseTransition | dict[str, Any]) -> dict[str, Any]:
    """Normalize one event_log entry (dataclass or dict) to a dict."""
    if isinstance(event, PhaseTransition):
        return {
            "time": float(event.time),
            "event": str(event.event),
            "from_phase": _phase_name(event.from_phase),
            "to_phase": _phase_name(event.to_phase),
            "guard_ok": bool(event.guard_ok),
        }
    return {
        "time": float(event["time"]),
        "event": str(event["event"]),
        "from_phase": _phase_name(event["from_phase"]),
        "to_phase": _phase_name(event["to_phase"]),
        "guard_ok": bool(event["guard_ok"]),
    }


def _build_meta(
    result: SILResult,
    meta: dict[str, Any] | None,
    round_decimals: int | None,
) -> dict[str, Any]:
    time_s = np.asarray(result.time_s, dtype=np.float64)
    duration_s = float(time_s[-1]) if time_s.shape[0] else 0.0
    if round_decimals is not None:
        duration_s = round(duration_s, round_decimals)
    merged: dict[str, Any] = dict(meta) if meta else {}
    merged.update(
        {
            "schema_version": SCHEMA_VERSION,
            "generated_utc": datetime.now(timezone.utc).strftime(
                "%Y-%m-%dT%H:%M:%SZ"
            ),
            "sample_count": int(time_s.shape[0]),
            "duration_s": duration_s,
            "dt": SAMPLE_DT_S,
            "units": dict(UNITS),
        }
    )
    return merged


def export_sil_result_json(
    result: SILResult,
    path: str | Path,
    *,
    meta: dict[str, Any] | None = None,
    round_decimals: int | None = 6,
) -> str:
    """Write ``result`` to ``path`` as schema 1.0 JSON; return abs path.

    Parent directories are created as needed. ``meta`` entries are merged
    into the auto-filled meta block, but the auto keys (schema_version,
    generated_utc, sample_count, duration_s, dt, units) always win.
    ``round_decimals=None`` keeps full float precision.
    """
    time_s = np.asarray(result.time_s, dtype=np.float64)
    _check_finite("time_s", time_s)
    arrays: dict[str, np.ndarray] = {}
    for name in _SERIES_1D[1:] + _SERIES_3D + _ECT_1D:
        array = np.asarray(getattr(result, name), dtype=np.float64)
        _check_finite(name, array)
        arrays[name] = array
    home_ned = np.asarray(result.home_ned, dtype=np.float64)
    _check_finite("home_ned", home_ned)
    waypoints_ned = np.asarray(result.waypoints_ned, dtype=np.float64)
    _check_finite("waypoints_ned", waypoints_ned)

    document: dict[str, Any] = {
        "meta": _build_meta(result, meta, round_decimals),
        "time_s": _to_json_value(time_s, round_decimals),
        "phase_timeline": [
            [float(t), _phase_name(phase)]
            for t, phase in sorted(
                result.phase_timeline, key=lambda entry: float(entry[0])
            )
        ],
        "event_log": [
            _event_to_dict(event) for event in result.event_log
        ],
    }
    for name in _SERIES_3D:
        document[name] = _to_json_value(arrays[name], round_decimals)
    for name in _SERIES_1D[1:]:
        document[name] = _to_json_value(arrays[name], round_decimals)
    for name in _ECT_1D:
        document[name] = _to_json_value(arrays[name], round_decimals)
    document["waypoints_ned"] = _to_json_value(waypoints_ned, round_decimals)
    document["home_ned"] = _to_json_value(home_ned, round_decimals)
    # Re-emit in schema order (document grew past insertion order above).
    document = {key: document[key] for key in REQUIRED_KEYS}

    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        json.dumps(document, indent=2) + "\n", encoding="utf-8"
    )
    return str(out.resolve())


def _shape_of(value: Any) -> tuple[int, ...]:
    """Shape of a nested list (uniform rows assumed)."""
    if not isinstance(value, list):
        return ()
    if not value:
        return (0,)
    return (len(value),) + _shape_of(value[0])


def _expect_shape(name: str, value: Any, expected: tuple[int, ...]) -> None:
    shape = _shape_of(value)
    if shape != expected:
        raise ValueError(
            f"{name} has shape {shape}, expected {expected}"
        )


def validate_flight_json(data: dict[str, Any]) -> None:
    """Validate a loaded schema 1.0 document; raise ValueError on the first violation."""
    missing = [key for key in REQUIRED_KEYS if key not in data]
    if missing:
        raise ValueError(f"missing top-level keys: {missing}")

    meta = data["meta"]
    if not isinstance(meta, dict):
        raise ValueError(f"meta must be a dict, got {type(meta).__name__}")
    version = meta.get("schema_version")
    if version != SCHEMA_VERSION:
        raise ValueError(
            f"meta.schema_version is {version!r}, expected {SCHEMA_VERSION!r}"
        )
    units = meta.get("units")
    if not isinstance(units, dict) or len(units) != len(UNITS):
        raise ValueError(
            f"meta.units must be a dict with {len(UNITS)} keys, "
            f"got {units!r}"
        )

    time_series = data["time_s"]
    if not isinstance(time_series, list):
        raise ValueError(
            f"time_s must be a list, got {type(time_series).__name__}"
        )
    sample_count = len(time_series)
    _expect_shape("time_s", time_series, (sample_count,))
    for name in _SERIES_1D[1:]:
        _expect_shape(name, data[name], (sample_count,))
    for name in _SERIES_3D:
        _expect_shape(name, data[name], (sample_count, 3))
    ect_series = data["e_ct"]
    if not isinstance(ect_series, list):
        raise ValueError(
            f"e_ct must be a list, got {type(ect_series).__name__}"
        )
    ect_count = len(ect_series)
    for name in _ECT_1D:
        _expect_shape(name, data[name], (ect_count,))
    waypoints = data["waypoints_ned"]
    if waypoints != []:
        _expect_shape("waypoints_ned", waypoints, (len(waypoints), 3))
    _expect_shape("home_ned", data["home_ned"], (3,))

    for index, entry in enumerate(data["phase_timeline"]):
        if (
            not isinstance(entry, list)
            or len(entry) != 2
            or isinstance(entry[0], bool)
            or not isinstance(entry[0], (int, float))
            or not isinstance(entry[1], str)
        ):
            raise ValueError(
                f"phase_timeline[{index}] must be [float, str], got {entry!r}"
            )
        if entry[1] not in VALID_PHASE_NAMES:
            raise ValueError(
                f"phase_timeline[{index}] phase {entry[1]!r} is not a "
                f"valid FlightPhase name"
            )

    for index, entry in enumerate(data["event_log"]):
        if not isinstance(entry, dict):
            raise ValueError(
                f"event_log[{index}] must be a dict, got {type(entry).__name__}"
            )
        if not isinstance(entry.get("time"), (int, float)) or isinstance(
            entry.get("time"), bool
        ):
            raise ValueError(f"event_log[{index}].time must be a float")
        if not isinstance(entry.get("event"), str):
            raise ValueError(f"event_log[{index}].event must be a string")
        for phase_key in ("from_phase", "to_phase"):
            name = entry.get(phase_key)
            if name not in VALID_PHASE_NAMES:
                raise ValueError(
                    f"event_log[{index}].{phase_key} {name!r} is not a "
                    f"valid FlightPhase name"
                )
        if not isinstance(entry.get("guard_ok"), bool):
            raise ValueError(f"event_log[{index}].guard_ok must be a bool")


__all__ = [
    "REQUIRED_KEYS",
    "SCHEMA_VERSION",
    "UNITS",
    "VALID_PHASE_NAMES",
    "export_sil_result_json",
    "validate_flight_json",
]
