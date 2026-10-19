"""SILResult JSON exporter (Sprint 2.3).

Serializes one :class:`~talon.mavlink_sil.SILResult` to a versioned JSON
document (schema_version "1.1") for post-flight analysis and archival:

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
accepted schema versions 1.0 and 1.1 (keys, shapes, finite numeric
leaves, meta consistency, phase names) and raises ValueError with a
clear message on the first violation.
"""

from __future__ import annotations

import json
import math
from collections.abc import Iterator
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from talon.mavlink_sil import SILResult
from talon.state_machine import PhaseTransition
from talon.types import FlightPhase, SphericalObstacle

SCHEMA_VERSION: str = "1.1"

# Schema versions `validate_flight_json` accepts. The writer always emits
# 1.1; 1.0 predates the `obstacles` key, which the validator now reads as an
# empty zone list (decision D4), so committed 1.0 documents stay loadable.
ACCEPTED_SCHEMA_VERSIONS: tuple[str, ...] = ("1.0", "1.1")

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
    "obstacles",
)


def _obstacle_to_dict(obstacle: SphericalObstacle, round_decimals: int | None) -> dict[str, Any]:
    """Serialize one SphericalObstacle to a JSON-ready dict."""
    return {
        "id": obstacle.id,
        "center_ned": _to_json_value(obstacle.center_ned, round_decimals),
        "radius_m": _to_json_value(obstacle.radius_m, round_decimals),
    }

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


def _iter_leaves(value: Any) -> Iterator[Any]:
    """Yield every scalar leaf of an ndarray, nested list/tuple, or scalar.

    Nothing is coerced on the way, unlike ``np.asarray(..., dtype=float)``
    which reads ``"1.5"`` and ``True`` as the numbers 1.5 and 1.0.
    """
    if isinstance(value, np.ndarray):
        if value.dtype.kind == "O":
            # Object arrays can hold anything, including nested lists.
            for item in value:
                yield from _iter_leaves(item)
        else:
            yield from value.flat
    elif isinstance(value, (list, tuple)):
        for item in value:
            yield from _iter_leaves(item)
    else:
        yield value


def _non_finite_error(name: str) -> ValueError:
    return ValueError(
        f"{name} contains non-finite or non-numeric values "
        "(NaN/Inf/None); JSON cannot represent them"
    )


def _check_finite(name: str, array: Any) -> None:
    """Raise ValueError naming ``name`` unless every numeric leaf is a finite number.

    Accepts an ndarray (the writer's path), nested JSON lists, or a bare
    scalar (the validator's path), so both share one check. Every flattened
    leaf must be a real ``int`` or ``float`` with bools excluded: numeric
    strings (``"1.5"``), bools, ``None`` and anything else that is not a
    number are rejected *before* the finiteness test, and NaN/Inf fail that
    test. A JSON document must carry real finite numbers in these fields.
    """
    if isinstance(array, np.ndarray) and array.dtype.kind in "fiu":
        # A real-number ndarray (the writer's path): the dtype already
        # proves every leaf is numeric, so only finiteness can fail here.
        # Bool arrays (kind "b") and string arrays fall through to the
        # strict leaf loop below.
        if not bool(np.all(np.isfinite(array))):
            raise _non_finite_error(name)
        return
    for leaf in _iter_leaves(array):
        if isinstance(leaf, bool) or not isinstance(leaf, (int, float)):
            raise _non_finite_error(name)
        try:
            finite = math.isfinite(leaf)
        except OverflowError:
            # An int too large for f64 (10**400) is out of range, which
            # for a document field is as unusable as infinity.
            finite = False
        if not finite:
            raise _non_finite_error(name)


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
    obstacles: list[SphericalObstacle] | None = None,
) -> str:
    """Write ``result`` to ``path`` as schema 1.1 JSON; return abs path.

    Parent directories are created as needed. ``meta`` entries are merged
    into the auto-filled meta block, but the auto keys (schema_version,
    generated_utc, sample_count, duration_s, dt, units) always win.
    ``round_decimals=None`` keeps full float precision. ``obstacles`` is
    the list of spherical keep-out zones the mission was planned around;
    when None an empty list is written so the schema stays uniform.
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
    document["obstacles"] = [
        _obstacle_to_dict(obs, round_decimals)
        for obs in (obstacles if obstacles is not None else [])
    ]
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
    """Validate a loaded schema 1.0 or 1.1 document; raise ValueError on the first violation.

    Beyond the structure (keys, shapes, phase names) this rejects the first
    non-finite numeric leaf (NaN, Inf, or None) in every series, in
    ``home_ned``, ``waypoints_ned``, each obstacle, the ``phase_timeline``
    times, the ``event_log`` times, and ``meta.duration_s`` / ``meta.dt``:
    JSON cannot carry those values, and one that slipped through would defeat
    every comparison downstream. ``meta`` must also agree with the data:
    ``sample_count`` equals ``len(time_s)``, ``dt`` is positive, and ``seed``
    fits a u64 when present. ``obstacles`` alone may be absent, reading as an
    empty zone list, so schema 1.0 documents validate too (decision D4,
    mirroring the reader's ``#[serde(default)]``); the writer still always
    emits the key, and always as schema 1.1.
    """
    missing = [
        key for key in REQUIRED_KEYS if key not in data and key != "obstacles"
    ]
    if missing:
        raise ValueError(f"missing top-level keys: {missing}")

    meta = data["meta"]
    if not isinstance(meta, dict):
        raise ValueError(f"meta must be a dict, got {type(meta).__name__}")
    version = meta.get("schema_version")
    if version not in ACCEPTED_SCHEMA_VERSIONS:
        raise ValueError(
            f"meta.schema_version is {version!r}, expected one of "
            f"{list(ACCEPTED_SCHEMA_VERSIONS)!r}"
        )
    units = meta.get("units")
    if not isinstance(units, dict) or len(units) != len(UNITS):
        raise ValueError(
            f"meta.units must be a dict with {len(UNITS)} keys, "
            f"got {units!r}"
        )
    for key in ("duration_s", "dt"):
        if key in meta:
            value = meta[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"meta.{key} must be a number, got {value!r}")
            # `_check_finite` after the type check so a list-valued leaf
            # cannot slip through as a finite one-element array.
            _check_finite(f"meta.{key}", value)
    if "dt" in meta and meta["dt"] <= 0.0:
        raise ValueError(f"meta.dt must be positive, got {meta['dt']}")
    if "seed" in meta:
        seed = meta["seed"]
        if isinstance(seed, bool) or not isinstance(seed, int):
            raise ValueError(f"meta.seed must be an int, got {seed!r}")
        if not 0 <= seed <= 2**64 - 1:
            raise ValueError(
                f"meta.seed must be in [0, {2**64 - 1}], got {seed}"
            )

    time_series = data["time_s"]
    if not isinstance(time_series, list):
        raise ValueError(
            f"time_s must be a list, got {type(time_series).__name__}"
        )
    sample_count = len(time_series)
    _expect_shape("time_s", time_series, (sample_count,))

    # meta.sample_count must describe the series it accompanies: an int
    # (bools are not counts), non-negative, and equal to len(time_s).
    declared = meta.get("sample_count")
    if isinstance(declared, bool) or not isinstance(declared, int):
        raise ValueError(f"meta.sample_count must be an int, got {declared!r}")
    if declared < 0:
        raise ValueError(
            f"meta.sample_count must be non-negative, got {declared}"
        )
    if declared != sample_count:
        raise ValueError(
            f"meta.sample_count is {declared}, but time_s carries "
            f"{sample_count} samples"
        )
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
    # isinstance before len(): a non-list (42, None, a string) must be a
    # field-named ValueError, not a TypeError escaping from len().
    waypoints = data["waypoints_ned"]
    if not isinstance(waypoints, list):
        raise ValueError(
            f"waypoints_ned must be a list, got {type(waypoints).__name__}"
        )
    if waypoints != []:
        _expect_shape("waypoints_ned", waypoints, (len(waypoints), 3))
    # home_ned goes through _shape_of only, which returns () for a non-list
    # and fails the shape check, so no len() can raise there.
    _expect_shape("home_ned", data["home_ned"], (3,))

    # Finiteness over every numeric leaf, after the shapes are known good:
    # NaN defeats comparisons (a NaN time sails through `<=` checks) and
    # None is not a number at all, so both fail here with the field named.
    for name in _SERIES_1D + _SERIES_3D + _ECT_1D:
        _check_finite(name, data[name])
    _check_finite("home_ned", data["home_ned"])
    _check_finite("waypoints_ned", waypoints)

    # Missing obstacles reads as an empty zone list (schema 1.0 compatibility,
    # decision D4); the writer always emits the key itself.
    obstacles = data.get("obstacles", [])
    if not isinstance(obstacles, list):
        raise ValueError(
            f"obstacles must be a list, got {type(obstacles).__name__}"
        )
    for index, entry in enumerate(obstacles):
        if not isinstance(entry, dict):
            raise ValueError(
                f"obstacles[{index}] must be a dict, got {type(entry).__name__}"
            )
        if not isinstance(entry.get("id"), str):
            raise ValueError(f"obstacles[{index}].id must be a string")
        _expect_shape(
            f"obstacles[{index}].center_ned", entry.get("center_ned"), (3,)
        )
        _check_finite(
            f"obstacles[{index}].center_ned", entry.get("center_ned")
        )
        radius = entry.get("radius_m")
        if not isinstance(radius, (int, float)) or isinstance(radius, bool):
            raise ValueError(f"obstacles[{index}].radius_m must be a number")
        # Finite before positive: `nan <= 0` is False, so without this a NaN
        # radius would pass the positivity test below.
        _check_finite(f"obstacles[{index}].radius_m", radius)
        if radius <= 0:
            raise ValueError(f"obstacles[{index}].radius_m must be positive")

    # Same class of guard as waypoints_ned: enumerate() on a non-list would
    # escape as TypeError instead of naming the field.
    phase_timeline = data["phase_timeline"]
    if not isinstance(phase_timeline, list):
        raise ValueError(
            f"phase_timeline must be a list, got {type(phase_timeline).__name__}"
        )
    for index, entry in enumerate(phase_timeline):
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
        # A NaN/Inf time passes every isinstance above and would corrupt the
        # timeline ordering downstream; the leaf itself must be finite.
        _check_finite(f"phase_timeline[{index}][0]", entry[0])
        if entry[1] not in VALID_PHASE_NAMES:
            raise ValueError(
                f"phase_timeline[{index}] phase {entry[1]!r} is not a "
                f"valid FlightPhase name"
            )

    event_log = data["event_log"]
    if not isinstance(event_log, list):
        raise ValueError(
            f"event_log must be a list, got {type(event_log).__name__}"
        )
    for index, entry in enumerate(event_log):
        if not isinstance(entry, dict):
            raise ValueError(
                f"event_log[{index}] must be a dict, got {type(entry).__name__}"
            )
        if not isinstance(entry.get("time"), (int, float)) or isinstance(
            entry.get("time"), bool
        ):
            raise ValueError(f"event_log[{index}].time must be a float")
        _check_finite(f"event_log[{index}].time", entry.get("time"))
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
