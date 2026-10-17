"""Tests for the Sprint 2.4 cinematic flight video export (talon.animation).

A minimal synthetic 2 s hover SILResult (200 control-rate samples) is
constructed inline; no simulation is run, so both exports stay fast
(under 15 s total).
"""

from __future__ import annotations

import shutil
from pathlib import Path

import numpy as np
import pytest

from talon.animation import export_flight_video
from talon.mavlink_sil import SILResult
from talon.types import FlightPhase

N_SAMPLES = 200  # 2 s at the 100 Hz control rate
DT_CONTROL_S = 0.01
HOVER_ALTITUDE_M = 10.0


def _tiny_result() -> SILResult:
    """Synthetic 2 s hover at 10 m with a takeoff -> cruise timeline."""
    time_s = np.arange(N_SAMPLES, dtype=np.float64) * DT_CONTROL_S
    wobble = 0.02 * np.sin(3.0 * time_s)
    truth_position = np.column_stack(
        [
            wobble,
            0.5 * wobble,
            -np.full(N_SAMPLES, HOVER_ALTITUDE_M),
        ]
    )
    truth_velocity = np.column_stack(
        [
            0.06 * np.cos(3.0 * time_s),
            0.03 * np.cos(3.0 * time_s),
            np.zeros(N_SAMPLES),
        ]
    )
    return SILResult(
        time_s=time_s,
        truth_position=truth_position,
        truth_velocity=truth_velocity,
        ekf_position=truth_position + 0.05,
        ekf_velocity=truth_velocity,
        phase_timeline=[
            (0.0, FlightPhase.ARM),
            (0.05, FlightPhase.TAKEOFF),
            (1.0, FlightPhase.CRUISE),
        ],
        event_log=[],
        e_ct=np.zeros(0, dtype=np.float64),
        gps_update_times=np.zeros(0, dtype=np.float64),
        gps_skipped=0,
        gps_gated=0,
        metrics={},
        home_ned=np.zeros(3, dtype=np.float64),
        waypoints_ned=np.array(
            [[5.0, 3.0, -HOVER_ALTITUDE_M], [-4.0, 6.0, -HOVER_ALTITUDE_M]]
        ),
    )


def test_export_flight_video_smoke(tmp_path: Path) -> None:
    if shutil.which("ffmpeg") is None:
        pytest.skip("ffmpeg not available")
    out = tmp_path / "hover.mp4"
    path = Path(export_flight_video(_tiny_result(), out, dpi=96))
    assert path.is_absolute()
    assert path.suffix == ".mp4"
    assert path.exists()
    assert path.stat().st_size > 10 * 1024


def test_export_gif_fallback(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(
        "talon.animation.shutil.which", lambda *_args, **_kwargs: None
    )
    out = tmp_path / "hover.gif"
    path = Path(export_flight_video(_tiny_result(), out))
    assert path.suffix == ".gif"
    assert path.exists()
    assert path.stat().st_size > 5 * 1024
