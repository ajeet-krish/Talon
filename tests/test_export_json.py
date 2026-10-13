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
    assert data["meta"]["schema_version"] == SCHEMA_VERSION == "1.0"


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
