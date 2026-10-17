"""Phase 6 validation driver: run all six Talon scenarios, aggregate
metrics, generate the publication plots, and write the validation report.

Usage (from the repository root):

    uv run python sims/run_all.py
    uv run python sims/run_all.py --monte-carlo   # + Monte Carlo sweep
    uv run python sims/run_all.py --skip-video    # skip docs/videos/ export
    uv run python sims/run_all.py --skip-fingerprint  # skip Sprint 2.5 art

What it does, in order:

1. Runs each of the six scenarios through its own main() with stdout
   captured (that is where each scenario's printed metrics and PASS/FAIL
   asserts live). The combined status is the AND of the six exit codes.
2. Re-runs the SILRunner-backed scenarios (wind, gps_outage, emergency,
   obstacle_field) through their run_scenario() entry points to obtain
   full SILResult histories for metric computation and plotting. The
   three Phase 4-style scripts (hover, waypoints, rtl) are standalone
   loops without a SILResult, so their headline numbers are parsed from
   the captured stdout (stable print formats, regex-matched).
3. Runs a quick 1 m position step through the cascaded PD (the Phase 2
   acceptance shape) to measure rise time, overshoot, and steady-state
   error for the report.
4. Counts the test suites at runtime: pytest by collection (subprocess,
   no test execution), GTest by running ./build/talon_tests (fast).
5. Generates seven publication plots into plots/ (gitignored). Plot
   scenario assignment (documented choice):

       trajectory_3d, position_error, ekf_convergence,
       cross_track_error  <- gps_outage run_scenario()
           the 2-waypoint circuit gives a full climb / leg / corner /
           RTL descent trajectory, real e_ct samples on two legs, and a
           visible GPS-outage bump in the position error;
       control_effort, wind_response  <- wind run_scenario()
           60 s cruise hold under Dryden + steady wind: the most
           interesting thrust/moment modulation and logged wind;
       state_machine  <- emergency run_scenario()
           the fullest failure narrative (idle -> arm -> takeoff ->
           cruise -> emergency -> land -> idle) with fault markers.

6. Exports the three SILRunner-backed results (gps_outage, wind,
   emergency) as schema 1.0 JSON documents into plots/json/ via
   talon.export_json (Sprint 2.3). The export is wrapped so a failure
   only logs a warning and never fails the validation.
7. Renders five cinematic 3D flight videos into docs/videos/ via
   talon.animation (Sprint 2.4): gps_outage, wind, emergency from the
   SIL re-runs, waypoints from the Phase 4 circuit re-captured with
   histories (_run_waypoints_with_capture), and obstacle_field from
   the FlightLab Phase 1 scenario. Skipped with --skip-video or
   TALON_SKIP_VIDEO=1; each video is wrapped so a failure only logs a
   warning and never fails the validation.
8. Exports the Sprint 2.5 data-art flight fingerprints via
   talon.plotting: a radar-polygon fingerprint (plots/fingerprint.png
   landscape + fingerprint_square.png) built from the SIL histories and
   the parsed scenario metrics, plus a 3D trajectory poster
   (plots/trajectory_poster.png from the gps_outage circuit). The
   portfolio copies land in docs/portfolio/ (fingerprint.png and
   trajectory_poster.svg). Skipped with --skip-fingerprint or
   TALON_SKIP_FINGERPRINT=1; each file is wrapped so a failure only
   logs a warning and never fails the validation.
9. Writes plots/validation_report.md (markdown table: Test, Target,
   Achieved, Pass) plus plots/json/validation.json (the same rows in
   machine-readable form, consumed by the talon-replay Metrics view), and
   prints the same summary to stdout.
10. With --monte-carlo (default off): after the main validation, runs the
    Sprint 2.2 Monte Carlo sweep over the waypoint circuit
    (sims/monte_carlo.py) with n_runs from the TALON_MC_RUNS environment
    variable (default 10) and writes plots/monte_carlo_report.md plus
    plots/monte_carlo_summary.png.

Exit status: 0 when every scenario main() returned 0 (their internal
acceptance asserts already ran), 1 otherwise.
"""

from __future__ import annotations

import argparse
import io
import json
import os
import re
import subprocess
import sys
import time
from contextlib import redirect_stdout
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import numpy as np

from sims import (  # noqa: E402  (path setup above)
    scenario_emergency,
    scenario_gps_outage,
    scenario_hover,
    scenario_obstacle_field,
    scenario_rtl,
    scenario_waypoints,
    scenario_wind,
)
from sims.monte_carlo import (  # noqa: E402
    MonteCarloConfig,
    plot_monte_carlo,
    run_monte_carlo,
    write_report as write_monte_carlo_report,
)
from talon import (  # noqa: E402
    DT,
    DT_CONTROL,
    CascadedController,
    ControlGains,
    State,
    VehicleParams,
    propagate,
)
from talon.animation import (  # noqa: E402
    _run_waypoints_with_capture,
    export_flight_video,
)
from talon.export_json import export_sil_result_json  # noqa: E402
from talon.mavlink_sil import SILConfig, SILResult  # noqa: E402
from talon.motors import MotorModel, forward_mix  # noqa: E402
from talon.plotting import (  # noqa: E402
    plot_control_effort,
    plot_cross_track_error,
    plot_ekf_convergence,
    plot_flight_fingerprint,
    plot_position_error,
    plot_state_machine,
    plot_trajectory_3d,
    plot_trajectory_poster,
    plot_wind_response,
)
from talon.types import FlightPhase  # noqa: E402

PLOTS_DIR = REPO_ROOT / "plots"
JSON_DIR = PLOTS_DIR / "json"
VIDEOS_DIR = REPO_ROOT / "docs" / "videos"
PORTFOLIO_DIR = REPO_ROOT / "docs" / "portfolio"
REPORT_PATH = PLOTS_DIR / "validation_report.md"
VALIDATION_JSON_PATH = JSON_DIR / "validation.json"
MC_REPORT_PATH = PLOTS_DIR / "monte_carlo_report.md"
MC_PLOT_PATH = PLOTS_DIR / "monte_carlo_summary.png"

# Sprint 2.5 fingerprint windows/points (mirrors the scenario hover
# definitions: home at the origin, hover altitude 10 m).
FINGERPRINT_HOVER_POINT = np.array([0.0, 0.0, -10.0])
FINGERPRINT_HOVER_WINDOW_S = 20.0
FINGERPRINT_WIND_WINDOW_S = 30.0

# Transient excluded from cross-track statistics (matches
# scenario_waypoints: the initial capture transient).
ECT_TRANSIENT_S = 15.0

# Acceptance targets mirrored from the scenario constants and tests.
HOVER_TOLERANCE_M = 0.05
STEP_RISE_MAX_S = 2.5
STEP_OVERSHOOT_MAX_PCT = 10.0
STEP_STEADY_MAX_M = 0.05
EKF_RMSE_MAX_M = 2.0
GYRO_BIAS_MAX_RPS = 0.01
MEAN_E_CT_MAX_M = 1.0
MAX_E_CT_MAX_M = 3.0
RTL_TRIGGER_MAX_S = 1.0
RTL_HORIZONTAL_MAX_M = 3.0
RTL_TOUCHDOWN_VZ_MAX = 0.5
WIND_ERROR_MAX_M = 3.0
OUTAGE_GROWTH_MAX_M = 5.0
OUTAGE_RECONVERGE_MAX_S = 5.0
EMERGENCY_DETECT_MAX_S = 1.0
EMERGENCY_FINAL_ALT_MAX_M = 0.5
PHASES_TOTAL = len(FlightPhase)
# 13 table entries in FlightPhaseMachine._TRANSITIONS plus the ANY ->
# EMERGENCY critical_fault path.
GUARD_TRANSITIONS_TOTAL = 14
GUARD_TRANSITIONS_MIN = 9

# Expected suite sizes when the binaries/collection are unavailable; the
# report prefers the runtime counts and shows "n/a" when unavailable.
# The pytest row passes when the runtime count is >= the expected size,
# so growing the suite does not fail the report.
PYTEST_EXPECTED = 139
GTEST_EXPECTED = 34  # was 31; Sprint 2.1 added 3 controller tests


@dataclass(slots=True)
class ReportRow:
    """One validation_report.md table row."""

    test: str
    target: str
    achieved: str
    passed: bool


def _run_main(module) -> tuple[int, str]:
    """Run a scenario main() with stdout captured; return (code, output)."""
    buf = io.StringIO()
    with redirect_stdout(buf):
        code = int(module.main())
    return code, buf.getvalue()


def _search(pattern: str, text: str, cast=float):
    """First regex group of ``pattern`` in ``text``, cast, else None."""
    match = re.search(pattern, text)
    if match is None:
        return None
    return cast(match.group(1))


def _fmt(value: float | None, unit: str = "", digits: int = 3) -> str:
    if value is None:
        return "n/a"
    return f"{value:.{digits}f}{(' ' + unit) if unit else ''}"


def _quick_position_step(duration_s: float = 10.0) -> dict[str, float]:
    """1 m North position step through truth + motor lag + cascaded PD.

    Mirrors tests/test_controller.test_position_step_metrics: samples the
    commanded-axis position at the control rate, then reports rise time
    (0 to 90 %), overshoot (%), and final steady-state error.
    """
    params = VehicleParams()
    controller = CascadedController(ControlGains.defaults(), params)
    motors = MotorModel()
    state = State()
    pos_desired = np.array([1.0, 0.0, 0.0])
    steps_per_control = max(1, round(DT_CONTROL / DT))
    command_thrust = params.mass * params.gravity
    command_moment = np.zeros(3, dtype=np.float64)
    xs: list[float] = []
    for i in range(round(duration_s / DT)):
        if i % steps_per_control == 0:
            cmd = controller.step(
                state, pos_desired, np.zeros(3), 0.0, DT_CONTROL
            )
            command_thrust = cmd.collective_thrust
            command_moment = np.asarray(cmd.moment)
            xs.append(float(np.asarray(state.position)[0]))
        forces = motors.step(command_thrust, command_moment, dt=DT)
        thrust_actual, moment_actual = forward_mix(forces)
        state = propagate(
            state, thrust_actual, moment_actual, np.zeros(3), DT, params
        )
    xs_arr = np.asarray(xs)
    above = np.flatnonzero(xs_arr >= 0.9)
    rise_time = float(above[0]) * DT_CONTROL if above.size else float("inf")
    overshoot_pct = max(0.0, float(np.max(xs_arr)) - 1.0) * 100.0
    steady_error = abs(float(xs_arr[-1]) - 1.0)
    return {
        "rise_time_s": rise_time,
        "overshoot_pct": overshoot_pct,
        "steady_error_m": steady_error,
    }


def _count_pytest() -> int | None:
    """Number of pytest tests via collection (no execution)."""
    try:
        proc = subprocess.run(
            [sys.executable, "-m", "pytest", "tests/", "-q", "--collect-only"],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            timeout=120,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    match = re.search(r"(\d+) tests? collected", proc.stdout + proc.stderr)
    return int(match.group(1)) if match else None


def _count_gtest() -> int | None:
    """Number of GTest tests: run the binary (fast) and parse PASSED."""
    binary = REPO_ROOT / "build" / "talon_tests"
    if not binary.exists():
        return None
    try:
        proc = subprocess.run(
            [str(binary), "--gtest_brief=1"],
            capture_output=True,
            text=True,
            timeout=120,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    match = re.search(r"\[\s*PASSED\s*\]\s*(\d+) tests?", proc.stdout)
    return int(match.group(1)) if match else None


def _phases_and_transitions(
    results: list[SILResult],
) -> tuple[set[FlightPhase], set[tuple[str, str]]]:
    """Union of phases visited and accepted (from, to) transition pairs."""
    phases: set[FlightPhase] = set()
    transitions: set[tuple[str, str]] = set()
    for result in results:
        phases |= result.phases_visited()
        for event in result.event_log:
            if event.guard_ok and event.from_phase is not event.to_phase:
                transitions.add(
                    (event.from_phase.value, event.to_phase.value)
                )
    return phases, transitions


def _generate_plots(
    gps_result: SILResult, wind_result: SILResult, emergency_result: SILResult
) -> list[str]:
    """Write the seven report plots (scenario assignment in docstring)."""
    paths = [
        plot_trajectory_3d(gps_result, PLOTS_DIR / "trajectory_3d.png"),
        plot_position_error(gps_result, PLOTS_DIR / "position_error.png"),
        plot_ekf_convergence(gps_result, PLOTS_DIR / "ekf_convergence.png"),
        plot_cross_track_error(
            gps_result, PLOTS_DIR / "cross_track_error.png"
        ),
        plot_control_effort(wind_result, PLOTS_DIR / "control_effort.png"),
        plot_wind_response(wind_result, PLOTS_DIR / "wind_response.png"),
        plot_state_machine(
            emergency_result, PLOTS_DIR / "state_machine.png"
        ),
    ]
    return paths


def _export_sil_json(
    gps_result: SILResult, wind_result: SILResult, emergency_result: SILResult
) -> list[str]:
    """Export the three SIL histories as schema 1.0 JSON (Sprint 2.3).

    Writes plots/json/{gps_outage,wind,emergency}.json with meta
    scenario + seed filled in. An export failure only logs a warning:
    JSON export must never fail the validation driver.
    """
    seed = SILConfig().seed  # the scenarios use the default config
    paths: list[str] = []
    obstacle_result = scenario_obstacle_field.run_scenario()[0]
    for name, result, obstacles in (
        ("gps_outage", gps_result, None),
        ("wind", wind_result, None),
        ("emergency", emergency_result, None),
        (
            "obstacle_field",
            obstacle_result,
            scenario_obstacle_field.OBSTACLES,
        ),
    ):
        try:
            path = export_sil_result_json(
                result,
                JSON_DIR / f"{name}.json",
                meta={"scenario": name, "seed": seed},
                obstacles=obstacles,
            )
            size_kb = Path(path).stat().st_size / 1024.0
            print(f"  {name}.json written ({size_kb:.1f} KB)")
            paths.append(path)
        except Exception as exc:  # noqa: BLE001 - warn, never fail validation
            print(f"WARNING: JSON export failed for {name}: {exc}")
    return paths


def _export_videos(
    gps_result: SILResult, wind_result: SILResult, emergency_result: SILResult
) -> list[str]:
    """Render the five cinematic flight videos (Sprint 2.4).

    Writes docs/videos/{gps_outage,wind,emergency,waypoints,obstacle_field}.mp4:
    the SILRunner-backed scenarios plus the Phase 4 waypoint circuit
    (re-captured with histories via the animation adapter). A failure
    only logs a warning: video export must never fail the validation
    driver.
    """
    print("capturing waypoint-circuit history for the video ...", flush=True)
    waypoints_result = _run_waypoints_with_capture()
    obstacle_result_video = scenario_obstacle_field.run_scenario()[0]
    paths: list[str] = []
    for name, result in (
        ("gps_outage", gps_result),
        ("wind", wind_result),
        ("emergency", emergency_result),
        ("waypoints", waypoints_result),
        ("obstacle_field", obstacle_result_video),
    ):
        t0 = time.perf_counter()
        try:
            path = export_flight_video(result, VIDEOS_DIR / f"{name}.mp4")
            elapsed = time.perf_counter() - t0
            size_mb = Path(path).stat().st_size / (1024.0 * 1024.0)
            print(
                f"  {Path(path).name} written "
                f"({size_mb:.2f} MB, {elapsed:.1f} s wall)"
            )
            paths.append(path)
        except Exception as exc:  # noqa: BLE001 - warn, never fail validation
            print(f"WARNING: video export failed for {name}: {exc}")
    return paths


def _rms(values: np.ndarray) -> float | None:
    """Root-mean-square of a sample vector; None for an empty vector."""
    if values.size == 0:
        return None
    return float(np.sqrt(np.mean(np.square(values))))


def _last_window(result: SILResult, seconds: float) -> np.ndarray:
    """Boolean mask of the samples in the last ``seconds`` of the run."""
    if result.time_s.size == 0:
        return np.zeros(0, dtype=bool)
    t_end = float(result.time_s[-1])
    return result.time_s >= t_end - seconds


def _build_fingerprint_metrics(
    gps_result: SILResult,
    wind_result: SILResult,
    emergency_result: SILResult,
    parsed: dict[str, dict[str, float | None]],
    step_metrics: dict[str, float],
) -> dict[str, dict[str, float | None]]:
    """Sprint 2.5 fingerprint metrics, one dict per scenario.

    Every scenario dict carries all eight fingerprint keys (the ones
    plot_flight_fingerprint specs); a metric that does not apply to the
    scenario or has no samples is None and its spoke is skipped:

    - hover_drift: RMS of ||truth - hover_point|| over the last 20 s
      (hover_point = [0, 0, -10] m, the takeoff/hover altitude);
    - ekf_rmse: sqrt(mean(||ekf - truth||^2)) over the whole run;
    - mean/max e_ct: from result.e_ct when the run flew a WAYPOINT leg;
    - wind_rms: RMS of truth-vs-hover error over the last 30 s, wind
      scenario only (the scenario's own acceptance window);
    - outage_growth: growth of |ekf - truth| over the 4 s GPS blackout,
      i.e. error at the outage end minus error at the outage start
      (matches the scenario_gps_outage growth definition; can be
      negative when the error shrinks), gps_outage only;
    - emergency_delay: detection delay, i.e. the time of the first
      EMERGENCY phase entry minus the fault-injection time, emergency
      scenario only (matches the printed detect_delay and the 1 s bound);
    - rise_time: the shared 1 m position-step rise time (vehicle-level,
      identical on every polygon).
    The waypoints scenario has no SILResult (standalone loop), so its
    polygon is built from the parsed stdout metrics and skipped when
    none of them could be parsed.
    """

    def _base(result: SILResult) -> dict[str, float | None]:
        metrics: dict[str, float | None] = {
            "hover_drift_m": None,
            "rise_time_s": float(step_metrics["rise_time_s"]),
            "ekf_rmse_m": None,
            "mean_e_ct_m": None,
            "max_e_ct_m": None,
            "wind_rms_m": None,
            "outage_growth_m": None,
            "emergency_delay_s": None,
        }
        if result.time_s.size:
            drift = np.linalg.norm(
                result.truth_position - FINGERPRINT_HOVER_POINT, axis=1
            )
            metrics["hover_drift_m"] = _rms(
                drift[_last_window(result, FINGERPRINT_HOVER_WINDOW_S)]
            )
            err = np.linalg.norm(
                result.ekf_position - result.truth_position, axis=1
            )
            metrics["ekf_rmse_m"] = float(np.sqrt(np.mean(np.square(err))))
        if result.e_ct.size:
            metrics["mean_e_ct_m"] = float(np.mean(result.e_ct))
            metrics["max_e_ct_m"] = float(np.max(result.e_ct))
        return metrics

    gps_metrics = _base(gps_result)
    outage_start = scenario_gps_outage.OUTAGE_START_S
    outage_end = outage_start + scenario_gps_outage.OUTAGE_DURATION_S
    if gps_result.time_s.size:
        # Growth delta, matching scenario_gps_outage.main(): the error at
        # the outage end minus the error at the outage start (NOT the
        # absolute end-of-outage error). Indexing mirrors the scenario:
        # last sample at or before each boundary time. A negative growth
        # (error shrank across the outage) is kept as-is; the fingerprint
        # normalizer treats any value <= 0 as beating the 5.0 m spec
        # (ratio capped at _FINGERPRINT_R_MAX).
        i_start = int(
            np.searchsorted(gps_result.time_s, outage_start, side="right")
            - 1
        )
        i_end = int(
            np.searchsorted(gps_result.time_s, outage_end, side="right") - 1
        )
        i_start = int(np.clip(i_start, 0, gps_result.time_s.size - 1))
        i_end = int(np.clip(i_end, 0, gps_result.time_s.size - 1))
        err_start = float(
            np.linalg.norm(
                gps_result.ekf_position[i_start]
                - gps_result.truth_position[i_start]
            )
        )
        err_end = float(
            np.linalg.norm(
                gps_result.ekf_position[i_end]
                - gps_result.truth_position[i_end]
            )
        )
        gps_metrics["outage_growth_m"] = err_end - err_start

    wind_metrics = _base(wind_result)
    if wind_result.time_s.size:
        hover_err = np.linalg.norm(
            wind_result.truth_position - FINGERPRINT_HOVER_POINT, axis=1
        )
        wind_metrics["wind_rms_m"] = _rms(
            hover_err[_last_window(wind_result, FINGERPRINT_WIND_WINDOW_S)]
        )

    emergency_metrics = _base(emergency_result)
    emergency_times = [
        t
        for t, phase in emergency_result.phase_timeline
        if phase is FlightPhase.EMERGENCY
    ]
    if emergency_times:
        emergency_metrics["emergency_delay_s"] = (
            float(emergency_times[0]) - scenario_emergency.INJECT_S
        )

    metrics: dict[str, dict[str, float | None]] = {
        "gps_outage": gps_metrics,
        "wind": wind_metrics,
        "emergency": emergency_metrics,
    }

    waypoints = parsed.get("waypoints", {})
    wp_values = {
        key: waypoints.get(src)
        for key, src in (
            ("ekf_rmse_m", "ekf_rmse_m"),
            ("mean_e_ct_m", "mean_e_ct_m"),
            ("max_e_ct_m", "max_e_ct_m"),
        )
    }
    if any(value is not None for value in wp_values.values()):
        metrics["waypoints"] = {
            "hover_drift_m": None,
            "rise_time_s": float(step_metrics["rise_time_s"]),
            **wp_values,
            "wind_rms_m": None,
            "outage_growth_m": None,
            "emergency_delay_s": None,
        }
    return metrics


def _export_fingerprints(
    metrics: dict[str, dict[str, float | None]],
    gps_result: SILResult,
    _emergency_result: SILResult,
) -> list[str]:
    """Write the Sprint 2.5 fingerprint deliverables (Sprint 2.5).

    plots/fingerprint.png (landscape) + fingerprint_square.png (square),
    plots/trajectory_poster.png (square, gps_outage circuit), and the
    docs/portfolio/ copies (fingerprint.png landscape +
    trajectory_poster.svg). ``_emergency_result`` is accepted for
    signature symmetry with the other exporters and reserved for a
    second poster (the leading underscore marks it intentionally
    unused). A failure only logs a warning: fingerprint export
    must never fail the validation driver.
    """
    jobs: list[tuple[str, Callable[[], str]]] = [
        (
            "fingerprint.png",
            lambda: plot_flight_fingerprint(
                metrics, PLOTS_DIR / "fingerprint.png", aspect="landscape"
            ),
        ),
        (
            "fingerprint_square.png",
            lambda: plot_flight_fingerprint(
                metrics, PLOTS_DIR / "fingerprint_square.png", aspect="square"
            ),
        ),
        (
            "trajectory_poster.png",
            lambda: plot_trajectory_poster(
                gps_result, PLOTS_DIR / "trajectory_poster.png",
                aspect="square",
            ),
        ),
        (
            "portfolio/fingerprint.png",
            lambda: plot_flight_fingerprint(
                metrics, PORTFOLIO_DIR / "fingerprint.png", aspect="landscape"
            ),
        ),
        (
            "portfolio/trajectory_poster.svg",
            lambda: plot_trajectory_poster(
                gps_result, PORTFOLIO_DIR / "trajectory_poster.svg",
                aspect="square",
            ),
        ),
    ]
    paths: list[str] = []
    for name, job in jobs:
        try:
            path = job()
            size_kb = Path(path).stat().st_size / 1024.0
            print(f"  {name} written ({size_kb:.1f} KB)")
            paths.append(path)
        except Exception as exc:  # noqa: BLE001 - warn, never fail validation
            print(f"WARNING: fingerprint export failed for {name}: {exc}")
    return paths


def _build_report(
    codes: dict[str, int],
    parsed: dict[str, dict[str, float | None]],
    step: dict[str, float],
    gps_result: SILResult,
    wind_result: SILResult,
    emergency_result: SILResult,
    pytest_count: int | None,
    gtest_count: int | None,
) -> list[ReportRow]:
    """Assemble every ReportRow from parsed scenario outputs + SIL data."""
    rows: list[ReportRow] = []

    passed_count = sum(1 for code in codes.values() if code == 0)
    rows.append(
        ReportRow(
            "Scenario asserts (7 scripts)",
            "7 of 7 pass",
            f"{passed_count} of {len(codes)} pass",
            passed_count == len(codes),
        )
    )

    # Hover (parsed).
    hover = parsed["hover"]
    rows.append(
        ReportRow(
            "Hover steady-state drift (final |err|)",
            f"< {HOVER_TOLERANCE_M} m",
            _fmt(hover.get("final_err_m"), "m"),
            hover.get("final_err_m") is not None
            and hover["final_err_m"] < HOVER_TOLERANCE_M,
        )
    )
    rows.append(
        ReportRow(
            "Hover window deviation (max, last 20 s)",
            f"< {HOVER_TOLERANCE_M} m",
            _fmt(hover.get("window_max_m"), "m"),
            hover.get("window_max_m") is not None
            and hover["window_max_m"] < HOVER_TOLERANCE_M,
        )
    )

    # Position step (re-run quick test).
    rows.append(
        ReportRow(
            "Position step rise time (0 to 90%)",
            f"< {STEP_RISE_MAX_S} s",
            _fmt(step["rise_time_s"], "s"),
            step["rise_time_s"] < STEP_RISE_MAX_S,
        )
    )
    rows.append(
        ReportRow(
            "Position step overshoot",
            f"< {STEP_OVERSHOOT_MAX_PCT:.0f} %",
            _fmt(step["overshoot_pct"], "%", digits=2),
            step["overshoot_pct"] < STEP_OVERSHOOT_MAX_PCT,
        )
    )
    rows.append(
        ReportRow(
            "Position step steady-state error",
            f"< {STEP_STEADY_MAX_M} m",
            _fmt(step["steady_error_m"], "m"),
            step["steady_error_m"] < STEP_STEADY_MAX_M,
        )
    )

    # EKF RMSE (waypoints stdout + SIL cross-check).
    waypoints = parsed["waypoints"]
    rows.append(
        ReportRow(
            "EKF position RMSE (waypoint circuit)",
            f"< {EKF_RMSE_MAX_M} m",
            _fmt(waypoints.get("ekf_rmse_m"), "m"),
            waypoints.get("ekf_rmse_m") is not None
            and waypoints["ekf_rmse_m"] < EKF_RMSE_MAX_M,
        )
    )

    # Gyro bias estimate (SIL, final sample, max |axis|).
    gyro_bias_max = (
        float(np.max(np.abs(gps_result.gyro_bias[-1])))
        if gps_result.gyro_bias.size
        else None
    )
    rows.append(
        ReportRow(
            "Gyro bias estimate (max |axis|, final)",
            f"< {GYRO_BIAS_MAX_RPS} rad/s",
            _fmt(gyro_bias_max, "rad/s", digits=5),
            gyro_bias_max is not None
            and gyro_bias_max < GYRO_BIAS_MAX_RPS,
        )
    )

    # NLGL cross-track (waypoints stdout).
    mean_e_ct = waypoints.get("mean_e_ct_m")
    max_e_ct = waypoints.get("max_e_ct_m")
    rows.append(
        ReportRow(
            "NLGL mean cross-track (after 15 s)",
            f"< {MEAN_E_CT_MAX_M} m",
            _fmt(mean_e_ct, "m"),
            mean_e_ct is not None and mean_e_ct < MEAN_E_CT_MAX_M,
        )
    )
    rows.append(
        ReportRow(
            "NLGL max cross-track (after 15 s)",
            f"< {MAX_E_CT_MAX_M} m",
            _fmt(max_e_ct, "m"),
            max_e_ct is not None and max_e_ct < MAX_E_CT_MAX_M,
        )
    )

    # Waypoints reached (waypoints stdout).
    reached = waypoints.get("reached")
    total = waypoints.get("total")
    rows.append(
        ReportRow(
            "Waypoints reached (40x30 m circuit)",
            "5 of 5",
            (
                f"{int(reached)} of {int(total)}"
                if reached is not None and total is not None
                else "n/a"
            ),
            reached is not None and total is not None and reached == total,
        )
    )

    # RTL (parsed).
    rtl = parsed["rtl"]
    rows.append(
        ReportRow(
            "RTL trigger delay after low battery",
            f"< {RTL_TRIGGER_MAX_S} s",
            _fmt(rtl.get("trigger_delay_s"), "s"),
            rtl.get("trigger_delay_s") is not None
            and rtl["trigger_delay_s"] < RTL_TRIGGER_MAX_S,
        )
    )
    rows.append(
        ReportRow(
            "RTL final horizontal distance to home",
            f"< {RTL_HORIZONTAL_MAX_M} m",
            _fmt(rtl.get("final_horizontal_m"), "m"),
            rtl.get("final_horizontal_m") is not None
            and rtl["final_horizontal_m"] < RTL_HORIZONTAL_MAX_M,
        )
    )
    rows.append(
        ReportRow(
            "RTL touchdown vertical speed at 0.5 m",
            f"< {RTL_TOUCHDOWN_VZ_MAX} m/s",
            _fmt(rtl.get("touchdown_vz_mps"), "m/s"),
            rtl.get("touchdown_vz_mps") is not None
            and rtl["touchdown_vz_mps"] < RTL_TOUCHDOWN_VZ_MAX,
        )
    )

    # Wind (parsed from the windy + baseline pair).
    wind = parsed["wind"]
    wind_rms = wind.get("rms_m")
    baseline_rms = wind.get("baseline_rms_m")
    wind_bound = (
        max(0.25, 2.0 * baseline_rms + 0.05)
        if baseline_rms is not None
        else None
    )
    achieved = (
        f"{wind_rms:.3f} m (bound {wind_bound:.3f} m)"
        if wind_rms is not None and wind_bound is not None
        else "n/a"
    )
    rows.append(
        ReportRow(
            "Wind hover RMS (last 30 s, 4-6 m/s airflow)",
            "below bound max(0.25, 2x baseline + 0.05) m",
            achieved,
            wind_rms is not None
            and wind_bound is not None
            and wind_rms < wind_bound,
        )
    )
    rows.append(
        ReportRow(
            "Wind hover max error",
            f"< {WIND_ERROR_MAX_M} m",
            _fmt(wind.get("max_m"), "m"),
            wind.get("max_m") is not None
            and wind["max_m"] < WIND_ERROR_MAX_M,
        )
    )

    # GPS outage (parsed).
    outage = parsed["gps_outage"]
    rows.append(
        ReportRow(
            "GPS outage error growth (4 s blackout)",
            f"< {OUTAGE_GROWTH_MAX_M} m",
            _fmt(outage.get("growth_m"), "m"),
            outage.get("growth_m") is not None
            and outage["growth_m"] < OUTAGE_GROWTH_MAX_M,
        )
    )
    rows.append(
        ReportRow(
            "GPS reconvergence after fixes resume",
            f"< {OUTAGE_RECONVERGE_MAX_S} s",
            _fmt(outage.get("reconverge_s"), "s", digits=2),
            outage.get("reconverge_s") is not None
            and outage["reconverge_s"] < OUTAGE_RECONVERGE_MAX_S,
        )
    )

    # Obstacle field (parsed).
    obstacle = parsed["obstacle_field"]
    rows.append(
        ReportRow(
            "Obstacle field min 3-D surface clearance",
            "> 0 m",
            _fmt(obstacle.get("min_clearance_m"), "m"),
            obstacle.get("min_clearance_m") is not None
            and obstacle["min_clearance_m"] > 0.0,
        )
    )

    # Emergency (parsed).
    emergency = parsed["emergency"]
    rows.append(
        ReportRow(
            "Emergency detection delay (EKF corrupt_bias)",
            f"< {EMERGENCY_DETECT_MAX_S} s",
            _fmt(emergency.get("detect_delay_s"), "s"),
            emergency.get("detect_delay_s") is not None
            and emergency["detect_delay_s"] < EMERGENCY_DETECT_MAX_S,
        )
    )
    rows.append(
        ReportRow(
            "Emergency final truth altitude",
            f"< {EMERGENCY_FINAL_ALT_MAX_M} m",
            _fmt(emergency.get("final_altitude_m"), "m"),
            emergency.get("final_altitude_m") is not None
            and emergency["final_altitude_m"] < EMERGENCY_FINAL_ALT_MAX_M,
        )
    )

    # State machine coverage (union over the three SIL scenarios).
    phases, transitions = _phases_and_transitions(
        [gps_result, wind_result, emergency_result]
    )
    rows.append(
        ReportRow(
            "State machine phases covered (SIL union)",
            f"{PHASES_TOTAL} of {PHASES_TOTAL}",
            f"{len(phases)} of {PHASES_TOTAL}",
            len(phases) == PHASES_TOTAL,
        )
    )
    rows.append(
        ReportRow(
            "State machine guard transitions covered",
            f">= {GUARD_TRANSITIONS_MIN} of {GUARD_TRANSITIONS_TOTAL}",
            f"{len(transitions)} of {GUARD_TRANSITIONS_TOTAL}",
            len(transitions) >= GUARD_TRANSITIONS_MIN,
        )
    )

    # Test suites (runtime counts).
    rows.append(
        ReportRow(
            "Python tests (pytest)",
            str(PYTEST_EXPECTED),
            str(pytest_count) if pytest_count is not None else "n/a",
            pytest_count is not None and pytest_count >= PYTEST_EXPECTED,
        )
    )
    rows.append(
        ReportRow(
            "C++ tests (GTest)",
            str(GTEST_EXPECTED),
            str(gtest_count) if gtest_count is not None else "n/a",
            gtest_count is not None and gtest_count >= GTEST_EXPECTED,
        )
    )
    return rows


def _write_report(rows: list[ReportRow], plot_paths: list[str]) -> Path:
    """Write plots/validation_report.md from the assembled rows."""
    now = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    lines = [
        "# Talon Validation Report",
        "",
        f"Generated by `uv run python sims/run_all.py` on {now}.",
        "",
        "Every row is checked by the validation driver or parsed from a",
        "scenario's own acceptance output; regenerate this file by re-running",
        "the driver (the plots/ directory is gitignored).",
        "",
        "| Test | Target | Achieved | Pass |",
        "|------|--------|----------|------|",
    ]
    for row in rows:
        mark = "yes" if row.passed else "NO"
        # Escape '|' so metric names like "final |err|" keep the table
        # four columns wide in rendered markdown.
        cells = [
            cell.replace("|", "\\|")
            for cell in (row.test, row.target, row.achieved)
        ]
        lines.append(f"| {cells[0]} | {cells[1]} | {cells[2]} | {mark} |")
    passed = sum(1 for row in rows if row.passed)
    lines += [
        "",
        f"Overall: {passed}/{len(rows)} checks passed.",
        "",
        "## Plots",
        "",
        "Generated into `plots/` (gitignored; regenerate with the driver):",
        "",
    ]
    for path in plot_paths:
        # Repo-relative when possible so plots/fingerprint.png and the
        # docs/portfolio/ copy read as distinct entries.
        try:
            display: str | Path = Path(path).resolve().relative_to(REPO_ROOT)
        except ValueError:
            display = Path(path).name
        lines.append(f"- `{display}`")
    lines += [
        "",
        "Plot scenario assignment: trajectory / position error / EKF",
        "convergence / cross-track from the GPS-outage circuit, control",
        "effort and wind response from the wind scenario, state machine",
        "Gantt from the emergency scenario (see sims/run_all.py docstring).",
        "",
    ]
    REPORT_PATH.write_text("\n".join(lines), encoding="utf-8")
    return REPORT_PATH


def _write_report_json(rows: list[ReportRow]) -> Path:
    """Write plots/json/validation.json, the machine-readable report.

    The talon-replay Metrics view fetches this file (copied into the app by
    talon-replay/scripts/copy_data.sh), so the web report tracks the driver's
    output instead of a hand-maintained table.
    """
    JSON_DIR.mkdir(parents=True, exist_ok=True)
    passed = sum(1 for row in rows if row.passed)
    document = {
        "generated_utc": datetime.now(timezone.utc).strftime(
            "%Y-%m-%d %H:%M UTC"
        ),
        "passed": passed,
        "total": len(rows),
        "rows": [
            {
                "test": row.test,
                "target": row.target,
                "achieved": row.achieved,
                "passed": row.passed,
            }
            for row in rows
        ],
    }
    VALIDATION_JSON_PATH.write_text(
        json.dumps(document, indent=2), encoding="utf-8"
    )
    return VALIDATION_JSON_PATH


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Command-line flags for the validation driver."""
    parser = argparse.ArgumentParser(
        description="Talon Phase 6 validation driver."
    )
    parser.add_argument(
        "--skip-video",
        action="store_true",
        help=(
            "skip the docs/videos/ cinematic export (also honored via "
            "the TALON_SKIP_VIDEO environment variable)"
        ),
    )
    parser.add_argument(
        "--skip-fingerprint",
        action="store_true",
        help=(
            "skip the Sprint 2.5 fingerprint/poster export (also honored "
            "via the TALON_SKIP_FINGERPRINT environment variable)"
        ),
    )
    parser.add_argument(
        "--monte-carlo",
        action="store_true",
        help=(
            "after the main validation, run the waypoint-circuit Monte "
            "Carlo sweep (n_runs from TALON_MC_RUNS, default 10) and "
            "write plots/monte_carlo_report.md + "
            "plots/monte_carlo_summary.png"
        ),
    )
    return parser.parse_args(argv)


def _run_monte_carlo_sweep() -> None:
    """Run the Sprint 2.2 Monte Carlo sweep and write report + plot."""
    n_runs = int(os.environ.get("TALON_MC_RUNS", "10"))
    config = MonteCarloConfig(n_runs=n_runs)
    print(
        f"Monte Carlo: running {n_runs} waypoint-circuit runs ...",
        flush=True,
    )
    t0 = time.perf_counter()
    results = run_monte_carlo(config)
    elapsed = time.perf_counter() - t0
    report_path = write_monte_carlo_report(results, config, MC_REPORT_PATH)
    plot_path = plot_monte_carlo(results, config, MC_PLOT_PATH)
    passed = sum(1 for r in results if r.success)
    print(
        f"Monte Carlo: {passed}/{len(results)} runs passed "
        f"({elapsed:.1f} s wall)"
    )
    print(f"Monte Carlo report written to {report_path}")
    print(f"Monte Carlo plot written to {plot_path}")


def main() -> int:
    """Run every scenario, build the report, print the summary."""
    args = _parse_args()
    t0 = time.perf_counter()
    print("=== Talon Phase 6 validation driver ===")

    # 1. Scenario scripts (their own asserts + printed metrics).
    scenario_outputs: dict[str, tuple[int, str]] = {}
    for name, module in (
        ("hover", scenario_hover),
        ("waypoints", scenario_waypoints),
        ("rtl", scenario_rtl),
        ("wind", scenario_wind),
        ("gps_outage", scenario_gps_outage),
        ("emergency", scenario_emergency),
        ("obstacle_field", scenario_obstacle_field),
    ):
        print(f"running scenario: {name} ...", flush=True)
        scenario_outputs[name] = _run_main(module)
    codes = {name: code for name, (code, _) in scenario_outputs.items()}
    for name, code in codes.items():
        status = "PASS" if code == 0 else "FAIL"
        print(f"  {name}: {status}")

    # 2. Parse the stable printed metrics.
    parsed: dict[str, dict[str, float | None]] = {
        "hover": {
            "final_err_m": _search(
                r"final position error norm \(m\): ([0-9.eE+-]+)",
                scenario_outputs["hover"][1],
            ),
            "window_max_m": _search(
                r"max deviation over last \d+ s \(m\): ([0-9.eE+-]+)",
                scenario_outputs["hover"][1],
            ),
        },
        "waypoints": {
            # reached/total and e_ct mean/max share one printed line each;
            # parsed with dedicated multi-group matches below.
            "reached": None,
            "total": None,
            "mean_e_ct_m": None,
            "max_e_ct_m": None,
            "ekf_rmse_m": _search(
                r"EKF-vs-truth position RMSE: ([0-9.eE+-]+) m",
                scenario_outputs["waypoints"][1],
            ),
        },
        "rtl": {
            "trigger_delay_s": _search(
                r"delay ([0-9.eE+-]+) s\)",
                scenario_outputs["rtl"][1],
            ),
            "final_horizontal_m": _search(
                r"final horizontal distance to home: ([0-9.eE+-]+) m",
                scenario_outputs["rtl"][1],
            ),
            "touchdown_vz_mps": _search(
                r"vertical speed at 0\.5 m crossing: ([0-9.eE+-]+) m/s",
                scenario_outputs["rtl"][1],
            ),
        },
        "wind": {
            "rms_m": _search(
                r"truth-vs-hover error over last \d+ s: "
                r"RMS ([0-9.eE+-]+) m",
                scenario_outputs["wind"][1],
            ),
            "max_m": _search(
                r"truth-vs-hover error over last \d+ s: "
                r"RMS [0-9.eE+-]+ m, max ([0-9.eE+-]+) m",
                scenario_outputs["wind"][1],
            ),
            "baseline_rms_m": _search(
                r"wind disabled\): RMS ([0-9.eE+-]+) m",
                scenario_outputs["wind"][1],
            ),
        },
        "gps_outage": {
            "growth_m": _search(
                r"at outage end \(growth \+?([0-9.eE+-]+) m\)",
                scenario_outputs["gps_outage"][1],
            ),
            "reconverge_s": _search(
                r"reconvergence after resume: ([0-9.eE+-]+) s",
                scenario_outputs["gps_outage"][1],
            ),
        },
        "emergency": {
            "detect_delay_s": _search(
                r"emergency detected ([0-9.eE+-]+) s after injection",
                scenario_outputs["emergency"][1],
            ),
            "final_altitude_m": _search(
                r"final truth: altitude ([0-9.eE+-]+) m",
                scenario_outputs["emergency"][1],
            ),
        },
        "obstacle_field": {
            "min_clearance_m": _search(
                r"min 3-D surface clearance: ([0-9.eE+-]+) m",
                scenario_outputs["obstacle_field"][1],
            ),
        },
    }
    # Waypoints: reached/total and the e_ct mean/max share one line.
    wp_text = scenario_outputs["waypoints"][1]
    match = re.search(r"waypoints reached: (\d+)/(\d+)", wp_text)
    if match is not None:
        parsed["waypoints"]["reached"] = int(match.group(1))
        parsed["waypoints"]["total"] = int(match.group(2))
    match = re.search(
        r"cross-track \|e_ct\| after \d+ s: mean ([0-9.eE+-]+) m, "
        r"max ([0-9.eE+-]+) m",
        wp_text,
    )
    if match is not None:
        parsed["waypoints"]["mean_e_ct_m"] = float(match.group(1))
        parsed["waypoints"]["max_e_ct_m"] = float(match.group(2))

    # 3. Quick position step through the cascaded PD.
    step_metrics = _quick_position_step()
    print(
        "position step: rise "
        f"{step_metrics['rise_time_s']:.2f} s, overshoot "
        f"{step_metrics['overshoot_pct']:.1f} %"
    )

    # 4. Test suite counts (runtime).
    print("counting test suites ...", flush=True)
    pytest_count = _count_pytest()
    gtest_count = _count_gtest()
    print(f"  pytest: {pytest_count}, gtest: {gtest_count}")

    # 5. SIL re-runs for histories (plots + derived metrics).
    print("SIL re-run: gps_outage (plots: trajectory, e_ct, EKF) ...",
          flush=True)
    gps_result = scenario_gps_outage.run_scenario()
    print("SIL re-run: wind (plots: control effort, wind response) ...",
          flush=True)
    wind_result = scenario_wind.run_scenario()
    print("SIL re-run: emergency (plot: state machine) ...", flush=True)
    emergency_result = scenario_emergency.run_scenario()

    # 6. Plots, JSON export (Sprint 2.3), videos (Sprint 2.4), + report.
    plot_paths = _generate_plots(gps_result, wind_result, emergency_result)
    print("exporting SIL JSON to plots/json/ ...", flush=True)
    _export_sil_json(gps_result, wind_result, emergency_result)
    skip_video = args.skip_video or os.environ.get(
        "TALON_SKIP_VIDEO", ""
    ) not in ("", "0")
    if skip_video:
        print("skipping video export (--skip-video / TALON_SKIP_VIDEO)")
    else:
        print("exporting flight videos to docs/videos/ ...", flush=True)
        _export_videos(gps_result, wind_result, emergency_result)

    # Sprint 2.5 fingerprints + poster, after videos and before the
    # report so their paths can be listed in the Plots section. Wrapped
    # inside _export_fingerprints: never fails validation.
    skip_fingerprint = args.skip_fingerprint or os.environ.get(
        "TALON_SKIP_FINGERPRINT", ""
    ) not in ("", "0")
    if skip_fingerprint:
        print(
            "skipping fingerprint export "
            "(--skip-fingerprint / TALON_SKIP_FINGERPRINT)"
        )
    else:
        print("exporting flight fingerprints to plots/ + docs/portfolio/"
              " ...", flush=True)
        fingerprint_metrics = _build_fingerprint_metrics(
            gps_result, wind_result, emergency_result, parsed, step_metrics
        )
        growth = fingerprint_metrics["gps_outage"]["outage_growth_m"]
        if growth is not None:
            print(
                "fingerprint outage_growth_m: "
                f"{growth:.3f} m "
                "(err at outage end minus err at outage start)"
            )
        plot_paths += _export_fingerprints(
            fingerprint_metrics, gps_result, emergency_result
        )
    rows = _build_report(
        codes,
        parsed,
        step_metrics,
        gps_result,
        wind_result,
        emergency_result,
        pytest_count,
        gtest_count,
    )
    report_path = _write_report(rows, plot_paths)
    validation_json_path = _write_report_json(rows)

    # 7. Stdout summary.
    print()
    for row in rows:
        mark = "PASS" if row.passed else "FAIL"
        print(f"[{mark}] {row.test}: achieved {row.achieved} "
              f"(target {row.target})")
    passed = sum(1 for row in rows if row.passed)
    print()
    print(f"plots written to {PLOTS_DIR}/:")
    for path in plot_paths:
        try:
            display = Path(path).resolve().relative_to(REPO_ROOT)
        except ValueError:
            display = Path(path).name
        print(f"  {display}")
    print(f"report written to {report_path}")
    print(f"validation json written to {validation_json_path}")
    print(
        f"summary: {passed}/{len(rows)} checks passed, "
        f"scenario asserts {sum(1 for c in codes.values() if c == 0)}"
        f"/{len(codes)}, {time.perf_counter() - t0:.1f} s wall"
    )

    # 8. Optional Monte Carlo sweep (Sprint 2.2; --monte-carlo, default
    # off). Runs after the main validation so a failing scenario still
    # leaves the validation report on disk.
    if args.monte_carlo:
        _run_monte_carlo_sweep()

    return 0 if all(code == 0 for code in codes.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
