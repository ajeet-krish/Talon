"""Tests for the Sprint 2.5 data-art plots (talon.plotting).

Synthetic metric dicts and a tiny synthetic hover SILResult; no
simulation is run, so all four exports stay fast. Pillow (bundled with
matplotlib) verifies the exact PNG pixel dimensions the deliverables
promise: fingerprint landscape 1200 x 627, square 1080 x 1080.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image

from talon.mavlink_sil import SILResult
from talon.plotting import plot_flight_fingerprint, plot_trajectory_poster
from talon.types import FlightPhase

N_SAMPLES = 200  # 2 s at the 100 Hz control rate
DT_CONTROL_S = 0.01
HOVER_ALTITUDE_M = 10.0


def _synthetic_metrics() -> dict[str, dict[str, float | None]]:
    """Three scenario metric dicts with a few None spokes."""
    return {
        "gps_outage": {
            "hover_drift_m": 0.03,
            "rise_time_s": 1.4,
            "ekf_rmse_m": 0.8,
            "mean_e_ct_m": 0.5,
            "max_e_ct_m": 1.9,
            "wind_rms_m": None,
            "outage_growth_m": 2.1,
            "emergency_delay_s": None,
        },
        "wind": {
            "hover_drift_m": 0.04,
            "rise_time_s": 1.4,
            "ekf_rmse_m": 0.4,
            "mean_e_ct_m": None,
            "max_e_ct_m": None,
            "wind_rms_m": 0.18,
            "outage_growth_m": None,
            "emergency_delay_s": None,
        },
        "emergency": {
            "hover_drift_m": 0.02,
            "rise_time_s": 1.4,
            "ekf_rmse_m": 1.1,
            "mean_e_ct_m": None,
            "max_e_ct_m": None,
            "wind_rms_m": None,
            "outage_growth_m": None,
            "emergency_delay_s": 0.4,
        },
    }


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


def test_plot_flight_fingerprint_creates_file(tmp_path: Path) -> None:
    out = tmp_path / "fingerprint.png"
    path = plot_flight_fingerprint(_synthetic_metrics(), out)
    assert Path(path).exists()
    assert Path(path).stat().st_size > 10 * 1024
    with Image.open(path) as img:
        assert img.size == (1200, 627)


def test_plot_flight_fingerprint_square(tmp_path: Path) -> None:
    out = tmp_path / "fingerprint_square.png"
    path = plot_flight_fingerprint(
        _synthetic_metrics(), out, aspect="square"
    )
    with Image.open(path) as img:
        assert img.size == (1080, 1080)


def test_plot_trajectory_poster_creates_file(tmp_path: Path) -> None:
    out = tmp_path / "trajectory_poster.png"
    path = plot_trajectory_poster(_tiny_result(), out)
    assert Path(path).exists()
    assert Path(path).stat().st_size > 10 * 1024
    with Image.open(path) as img:
        assert img.size == (1080, 1080)


def test_plot_trajectory_poster_svg(tmp_path: Path) -> None:
    out = tmp_path / "trajectory_poster.svg"
    path = plot_trajectory_poster(_tiny_result(), out)
    assert Path(path).suffix == ".svg"
    assert Path(path).stat().st_size > 0
    head = Path(path).read_text(encoding="utf-8")[:64]
    assert head.startswith("<")
