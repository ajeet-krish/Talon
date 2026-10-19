"""Tests for the Sprint 2.3 SILResult JSON exporter (talon.export_json).

A minimal synthetic SILResult (10 control-rate samples) is constructed
inline; no simulation is run, so these tests are pure serialization.
"""

from __future__ import annotations

import json

import numpy as np
import pytest

from talon.export_json import (
    REQUIRED_KEYS,
    SCHEMA_VERSION,
    UNITS,
    export_sil_result_json,
    validate_flight_json,
)
from talon.mavlink_sil import SILResult
from talon.state_machine import PhaseTransition
from talon.types import FlightPhase

N_SAMPLES = 10


def _make_result(*, nan: bool = False, with_waypoints: bool = True) -> SILResult:
    """Minimal 10-sample SILResult with every exported field populated."""
    time_s = np.arange(N_SAMPLES, dtype=np.float64) * 0.01
    shape_pos = (N_SAMPLES, 3)
    truth_position = np.column_stack(
        [time_s, np.zeros(N_SAMPLES), -np.full(N_SAMPLES, 5.0)]
    )
    if nan:
        truth_position[0, 0] = np.nan
    return SILResult(
        time_s=time_s,
        truth_position=truth_position,
        truth_velocity=np.full(shape_pos, 0.1),
        ekf_position=truth_position + 0.01,
        ekf_velocity=np.full(shape_pos, 0.1),
        phase_timeline=[
            (0.0, FlightPhase.IDLE),
            (0.02, FlightPhase.ARM),
            (0.04, FlightPhase.TAKEOFF),
        ],
        event_log=[
            PhaseTransition(0.02, FlightPhase.IDLE, FlightPhase.ARM, "arm", True),
            PhaseTransition(0.03, FlightPhase.ARM, FlightPhase.ARM, "takeoff", False),
        ],
        e_ct=np.linspace(0.0, 1.0, 4),
        gps_update_times=np.array([0.01, 0.02]),
        gps_skipped=0,
        gps_gated=0,
        metrics={},
        home_ned=np.zeros(3),
        waypoints_ned=(
            np.array([[10.0, 0.0, -5.0], [10.0, 10.0, -5.0]])
            if with_waypoints
            else np.zeros((0, 3))
        ),
        command_thrust=np.full(N_SAMPLES, 12.0),
        command_moment=np.full(shape_pos, 0.01),
        wind_ned=np.full(shape_pos, 0.5),
        position_var=np.full(shape_pos, 0.25),
        gyro_bias=np.full(shape_pos, 1e-4),
        accel_bias=np.full(shape_pos, 1e-3),
        e_ct_times=np.array([0.04, 0.05, 0.06, 0.07]),
        e_ct_leg=np.zeros(4),
    )


def _export_load(result: SILResult, tmp_path, **kwargs) -> dict:
    path = export_sil_result_json(result, tmp_path / "out.json", **kwargs)
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def test_round_trip_arrays(tmp_path) -> None:
    result = _make_result()
    data = _export_load(result, tmp_path)
    assert np.array(data["time_s"]).shape == (N_SAMPLES,)
    for name in (
        "truth_position", "truth_velocity", "ekf_position", "ekf_velocity",
        "position_var", "gyro_bias", "accel_bias", "wind_ned", "command_moment",
    ):
        assert np.array(data[name]).shape == (N_SAMPLES, 3), name
    assert np.array(data["command_thrust"]).shape == (N_SAMPLES,)
    for name in ("e_ct", "e_ct_times", "e_ct_leg"):
        assert np.array(data[name]).shape == (4,), name
    assert np.allclose(data["truth_position"], result.truth_position, atol=1e-6)


def test_required_keys(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    assert set(data) == set(REQUIRED_KEYS)


def test_meta_schema_version(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    assert data["meta"]["schema_version"] == SCHEMA_VERSION == "1.1"


def test_meta_units_block(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    units = data["meta"]["units"]
    assert len(units) == 9
    assert units == UNITS


def test_phase_timeline_strings(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    valid = {phase.value for phase in FlightPhase}
    for entry in data["phase_timeline"]:
        assert isinstance(entry[0], float)
        assert entry[1] in valid


def test_phase_timeline_sorted(tmp_path) -> None:
    result = _make_result()
    result.phase_timeline = [
        (0.04, FlightPhase.TAKEOFF),
        (0.0, FlightPhase.IDLE),
        (0.02, FlightPhase.ARM),
    ]
    data = _export_load(result, tmp_path)
    times = [entry[0] for entry in data["phase_timeline"]]
    assert times == sorted(times)


def test_event_log_structure(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    assert len(data["event_log"]) == 2
    entry = data["event_log"][0]
    assert isinstance(entry["time"], float)
    assert isinstance(entry["event"], str)
    assert isinstance(entry["from_phase"], str)
    assert isinstance(entry["to_phase"], str)
    assert isinstance(entry["guard_ok"], bool)


def test_event_log_phases_valid(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    valid = {phase.value for phase in FlightPhase}
    for entry in data["event_log"]:
        assert entry["from_phase"] in valid
        assert entry["to_phase"] in valid


def test_no_nan_inf(tmp_path) -> None:
    with pytest.raises(ValueError, match="non-finite"):
        export_sil_result_json(_make_result(nan=True), tmp_path / "bad.json")


def test_validate_flight_json_passes(tmp_path) -> None:
    validate_flight_json(_export_load(_make_result(), tmp_path))


def test_validate_flight_json_missing_key(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    del data["wind_ned"]
    with pytest.raises(ValueError, match="missing top-level keys"):
        validate_flight_json(data)


def test_validate_flight_json_wrong_shape(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    data["truth_position"] = [[1.0, 2.0]]  # (1, 3) != (10, 3)
    with pytest.raises(ValueError, match="truth_position has shape"):
        validate_flight_json(data)


def test_round_decimals_none(tmp_path) -> None:
    result = _make_result()
    result.truth_position[0, 0] = 1.0 / 3.0
    full = _export_load(result, tmp_path / "full", round_decimals=None)
    rounded = _export_load(result, tmp_path / "rounded")
    assert full["truth_position"][0][0] == pytest.approx(1.0 / 3.0)
    assert len(str(full["truth_position"][0][0]).split(".")[-1]) > 6
    assert rounded["truth_position"][0][0] == 0.333333


def test_empty_waypoints(tmp_path) -> None:
    data = _export_load(_make_result(with_waypoints=False), tmp_path)
    assert data["waypoints_ned"] == []
    validate_flight_json(data)


def test_home_ned_shape(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    assert np.array(data["home_ned"]).shape == (3,)
    validate_flight_json(data)


def test_export_returns_absolute_path_and_creates_dirs(tmp_path) -> None:
    target = tmp_path / "nested" / "json" / "flight.json"
    path = export_sil_result_json(_make_result(), target)
    assert target.exists()
    assert path == str(target.resolve())


# ---------------------------------------------------------------------------
# Validator finiteness (Phase 4 input hardening)
#
# `json.loads` happily parses a literal NaN or 1e999 (which becomes inf),
# and neither survives a `<=` comparison, so the validator has to reject
# them explicitly with the field named.
# ---------------------------------------------------------------------------


def test_validate_flight_json_rejects_nan_in_a_1d_series(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    data["time_s"][2] = float("nan")
    with pytest.raises(ValueError, match="time_s contains non-finite"):
        validate_flight_json(data)


def test_validate_flight_json_rejects_inf_in_a_3d_series(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    data["truth_position"][0][1] = float("inf")
    with pytest.raises(
        ValueError, match="truth_position contains non-finite"
    ):
        validate_flight_json(data)


def test_validate_flight_json_rejects_nan_in_e_ct(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    data["e_ct"][1] = float("nan")
    with pytest.raises(ValueError, match="e_ct contains non-finite"):
        validate_flight_json(data)


def test_validate_flight_json_rejects_a_non_finite_waypoint(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    data["waypoints_ned"][0][1] = float("nan")
    with pytest.raises(ValueError, match="waypoints_ned contains non-finite"):
        validate_flight_json(data)


def test_validate_flight_json_rejects_a_non_finite_home(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    data["home_ned"][2] = float("inf")
    with pytest.raises(ValueError, match="home_ned contains non-finite"):
        validate_flight_json(data)


def test_validate_flight_json_rejects_a_null_numeric_leaf(tmp_path) -> None:
    # None is not a number: numpy reads it as NaN when casting to float.
    data = _export_load(_make_result(), tmp_path)
    data["home_ned"][0] = None
    with pytest.raises(ValueError, match="home_ned contains non-finite"):
        validate_flight_json(data)


def test_validate_flight_json_rejects_a_non_finite_obstacle(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    # `nan <= 0` is False, so the positivity check alone accepts a NaN radius.
    data["obstacles"] = [
        {"id": "zone", "center_ned": [0.0, 0.0, -5.0], "radius_m": float("nan")}
    ]
    with pytest.raises(
        ValueError, match=r"obstacles\[0\]\.radius_m contains non-finite"
    ):
        validate_flight_json(data)
    data["obstacles"] = [
        {"id": "zone", "center_ned": [0.0, float("nan"), -5.0], "radius_m": 2.0}
    ]
    with pytest.raises(
        ValueError, match=r"obstacles\[0\]\.center_ned contains non-finite"
    ):
        validate_flight_json(data)


def test_validate_flight_json_rejects_non_finite_meta(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    data["meta"]["duration_s"] = float("nan")
    with pytest.raises(
        ValueError, match="meta.duration_s contains non-finite"
    ):
        validate_flight_json(data)
    data = _export_load(_make_result(), tmp_path)
    data["meta"]["dt"] = float("inf")
    with pytest.raises(ValueError, match="meta.dt contains non-finite"):
        validate_flight_json(data)


def test_validate_flight_json_accepts_a_missing_obstacles_key(tmp_path) -> None:
    # Schema 1.0 documents carry no obstacles key; the Rust reader defaults
    # it (serde(default), decision D4) and the validator must too. The writer
    # still always emits the key, pinned by test_required_keys above.
    data = _export_load(_make_result(), tmp_path)
    assert "obstacles" in data
    del data["obstacles"]
    validate_flight_json(data)


def test_validate_flight_json_still_rejects_a_missing_required_key(tmp_path) -> None:
    # The obstacles exception is exactly one key wide.
    data = _export_load(_make_result(), tmp_path)
    del data["obstacles"]
    del data["home_ned"]
    with pytest.raises(ValueError, match="missing top-level keys"):
        validate_flight_json(data)


# ---------------------------------------------------------------------------
# Validator completeness: log times, meta consistency, schema versions
# ---------------------------------------------------------------------------


def test_validate_flight_json_rejects_a_nan_timeline_time(tmp_path) -> None:
    # A NaN time passes every isinstance in the timeline check and would
    # corrupt the ordering downstream; the leaf must be finite.
    data = _export_load(_make_result(), tmp_path)
    data["phase_timeline"][1][0] = float("nan")
    with pytest.raises(
        ValueError, match=r"phase_timeline\[1\]\[0\] contains non-finite"
    ):
        validate_flight_json(data)


def test_validate_flight_json_rejects_an_infinite_event_time(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    data["event_log"][0]["time"] = float("inf")
    with pytest.raises(
        ValueError, match=r"event_log\[0\]\.time contains non-finite"
    ):
        validate_flight_json(data)


def test_validate_flight_json_rejects_a_sample_count_mismatch(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    assert data["meta"]["sample_count"] == N_SAMPLES
    data["meta"]["sample_count"] = N_SAMPLES - 1
    with pytest.raises(ValueError, match="meta.sample_count is 9"):
        validate_flight_json(data)


def test_validate_flight_json_rejects_a_non_int_sample_count(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    data["meta"]["sample_count"] = 10.5
    with pytest.raises(ValueError, match="meta.sample_count must be an int"):
        validate_flight_json(data)
    # bool is an int subclass; counts are not bools.
    data = _export_load(_make_result(), tmp_path)
    data["meta"]["sample_count"] = True
    with pytest.raises(ValueError, match="meta.sample_count must be an int"):
        validate_flight_json(data)
    data = _export_load(_make_result(), tmp_path)
    data["meta"]["sample_count"] = -1
    with pytest.raises(
        ValueError, match="meta.sample_count must be non-negative"
    ):
        validate_flight_json(data)


def test_validate_flight_json_checks_the_seed_against_u64(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    data["meta"]["seed"] = 2**64
    with pytest.raises(ValueError, match="meta.seed must be in"):
        validate_flight_json(data)
    data["meta"]["seed"] = -1
    with pytest.raises(ValueError, match="meta.seed must be in"):
        validate_flight_json(data)
    data = _export_load(_make_result(), tmp_path)
    data["meta"]["seed"] = 1.5
    with pytest.raises(ValueError, match="meta.seed must be an int"):
        validate_flight_json(data)
    data = _export_load(_make_result(), tmp_path)
    data["meta"]["seed"] = True
    with pytest.raises(ValueError, match="meta.seed must be an int"):
        validate_flight_json(data)
    # The u64 endpoints themselves are accepted.
    data = _export_load(_make_result(), tmp_path)
    data["meta"]["seed"] = 2**64 - 1
    validate_flight_json(data)


def test_validate_flight_json_accepts_schema_1_0(tmp_path) -> None:
    # D4: a 1.0 document predates the obstacles key, and both differences
    # together are exactly what the committed 1.0 artifacts look like.
    data = _export_load(_make_result(), tmp_path)
    data["meta"]["schema_version"] = "1.0"
    del data["obstacles"]
    validate_flight_json(data)


def test_validate_flight_json_rejects_an_unknown_schema_version(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    data["meta"]["schema_version"] = "1.2"
    with pytest.raises(ValueError, match="meta.schema_version is"):
        validate_flight_json(data)


def test_validate_flight_json_rejects_a_non_positive_dt(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    data["meta"]["dt"] = 0.0
    with pytest.raises(ValueError, match="meta.dt must be positive"):
        validate_flight_json(data)
    data = _export_load(_make_result(), tmp_path)
    data["meta"]["dt"] = -0.01
    with pytest.raises(ValueError, match="meta.dt must be positive"):
        validate_flight_json(data)


# ---------------------------------------------------------------------------
# Type guards: non-list containers and non-numeric leaves
# ---------------------------------------------------------------------------


def test_validate_flight_json_rejects_a_string_waypoints_ned(tmp_path) -> None:
    data = _export_load(_make_result(), tmp_path)
    data["waypoints_ned"] = "[[10.0, 0.0, -5.0]]"
    with pytest.raises(ValueError, match="waypoints_ned must be a list"):
        validate_flight_json(data)
    # An int (or None) is what used to escape as TypeError from len().
    data = _export_load(_make_result(), tmp_path)
    data["waypoints_ned"] = 42
    with pytest.raises(ValueError, match="waypoints_ned must be a list"):
        validate_flight_json(data)


def test_validate_flight_json_rejects_a_non_list_timeline_or_event_log(
    tmp_path,
) -> None:
    # enumerate() on a non-list raised TypeError instead of naming the field.
    data = _export_load(_make_result(), tmp_path)
    data["phase_timeline"] = 42
    with pytest.raises(ValueError, match="phase_timeline must be a list"):
        validate_flight_json(data)
    data = _export_load(_make_result(), tmp_path)
    data["event_log"] = None
    with pytest.raises(ValueError, match="event_log must be a list"):
        validate_flight_json(data)


def test_validate_flight_json_rejects_a_nan_string_or_bool_series_leaf(
    tmp_path,
) -> None:
    # NaN is the numeric villain; the string and the bool are the leaves
    # numpy coercion used to wave through as 1.5 and 1.0.
    data = _export_load(_make_result(), tmp_path)
    data["command_thrust"][0] = float("nan")
    with pytest.raises(
        ValueError, match="command_thrust contains non-finite"
    ):
        validate_flight_json(data)
    data = _export_load(_make_result(), tmp_path)
    data["command_thrust"][0] = "1.5"
    with pytest.raises(
        ValueError, match="command_thrust contains non-finite"
    ):
        validate_flight_json(data)
    data = _export_load(_make_result(), tmp_path)
    data["truth_position"][1][2] = True
    with pytest.raises(
        ValueError, match="truth_position contains non-finite"
    ):
        validate_flight_json(data)
    data = _export_load(_make_result(), tmp_path)
    data["time_s"][0] = True
    with pytest.raises(ValueError, match="time_s contains non-finite"):
        validate_flight_json(data)
