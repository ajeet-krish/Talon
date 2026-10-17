"""Cinematic MP4 flight video export (Sprint 2.4).

export_flight_video renders a SILResult history to a 3D flight video in
the house style of talon.plotting: altitude-above-home framing (map-like
North / East / altitude-up axes), the Wong palette, and a static camera
(elev 25 deg, azim -60 deg) with an equal box aspect so slopes stay
physically honest. The scene is:

- a phase-colored trail (LineCollection of consecutive samples) whose
  per-segment alpha ages from 0.1 (oldest) to 0.8 (most recent), plus a
  bright 2.5 pt line over the last ``trail_length`` points;
- a quadrotor marker (scatter) with four short cross arms rotated by the
  velocity-heading yaw;
- a gray ground shadow directly below the vehicle;
- numbered vermillion waypoint pins on the ground plane;
- a monospace HUD line (time, phase, altitude, speed);
- a title card: the first 30 frames are blank axes with the scenario
  title fading in.

Time compression: the video plays at ``speed`` x real time, i.e. each
frame advances ``speed / fps`` sim seconds through the control-rate
history (2x at 30 fps samples every 1/15 s of sim time, one sample every
6 to 7 control ticks).

Writers: FFMpegWriter (libx264, yuv420p) when ffmpeg is on PATH and the
target suffix is a video container; otherwise PillowWriter GIF with the
path suffix adjusted to .gif.

Matplotlib 3D alpha workaround: translucent per-segment colors in a 3D
collection are drawn in painters order, so faded trail segments can pop
in front of newer ones. The trail colors are therefore pre-blended
against the white axes background and drawn fully opaque; on a white
background the result is visually identical with no sorting artifacts.

_run_waypoints_with_capture is a private helper (used by
sims/run_all.py) that re-runs the Phase 4 waypoint-circuit loop from
sims/scenario_waypoints.py while capturing the control-rate histories,
because that scenario is a standalone metrics loop without a SILResult.
"""

from __future__ import annotations

import math
import shutil
from pathlib import Path
from typing import TYPE_CHECKING

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import (
    AbstractMovieWriter,
    FFMpegWriter,
    PillowWriter,
)
from matplotlib.colors import to_rgba
from mpl_toolkits.mplot3d import art3d

from talon.plotting import PHASE_COLORS, WONG_PALETTE, apply_wong_theme

if TYPE_CHECKING:  # pragma: no cover - typing only
    from talon.mavlink_sil import SILResult

# Video framing constants.
_TITLE_FRAMES = 30
_FIGSIZE = (8.0, 6.0)
_TRAIL_ALPHA_OLD = 0.1
_TRAIL_ALPHA_NEW = 0.8
_TRAIL_LINEWIDTH = 0.8
_BRIGHT_LINEWIDTH = 2.5
_QUAD_MARKER_SIZE = 40
_QUAD_ARM_FRACTION = 0.025  # cross-arm length as a fraction of scene span
_YAW_STOP_SPEED_MPS = 0.05  # below this ground speed the yaw is held
_SHADOW_COLOR = "#888888"
_SHADOW_ALPHA = 0.4
_CAM_ELEV_DEG = 25.0
_CAM_AZIM_DEG = -60.0
_LIMIT_PADDING = 0.05
_HUD_BBOX = {
    "facecolor": "black",
    "alpha": 0.55,
    "edgecolor": "none",
    "pad": 3.0,
}
_VIDEO_SUFFIXES = {".mp4", ".mov", ".mkv"}

# Constants mirrored from sims/scenario_waypoints.py for the capture
# adapter (the installed talon package cannot import the sims tree).
_WP_MAX_DURATION_S = 120.0
_WP_SETTLE_S = 5.0
_WP_ALTITUDE_M = 10.0
_WP_CRUISE_SPEED = 5.0
_WP_RADIUS_M = 2.0
_WP_POSITIONS = (
    (0.0, 0.0),
    (40.0, 0.0),
    (40.0, 30.0),
    (0.0, 30.0),
    (0.0, 0.0),
)
_WP_GPS_POS_NOISE = 0.3
_WP_GPS_POS_NOISE_V = 0.5
_WP_GPS_VEL_NOISE = 0.05
_WP_WIND_SIGMA_H = 2.0
_WP_WIND_SIGMA_V = 1.0
_WP_MIN_GPS_NOISE_SCALE = 0.01


def _altitude_above_home(result: SILResult) -> np.ndarray:
    """Altitude above the home point (m, positive up) per sample."""
    home_d = float(result.home_ned[2]) if result.home_ned.size else 0.0
    return home_d - result.truth_position[:, 2]


def _sample_phase_colors(
    times: np.ndarray, timeline: list[tuple[float, object]]
) -> np.ndarray:
    """Per-sample trail base RGB, colored by the phase active at the sample.

    The phase at any control sample is the last timeline entry at or
    before that sample's time (SILResult contract); unknown phase names
    fall back to gray.
    """
    if not timeline:
        return np.tile(np.array([0.6, 0.6, 0.6]), (times.size, 1))
    tl_times = np.array([t for t, _ in timeline], dtype=np.float64)
    idx = np.clip(np.searchsorted(tl_times, times, side="right") - 1, 0, len(timeline) - 1)
    rgb = np.empty((times.size, 3), dtype=np.float64)
    for i, entry in enumerate(idx):
        color = PHASE_COLORS.get(timeline[entry][1].value, "#999999")
        rgb[i] = to_rgba(color)[:3]
    return rgb


def _blend(rgb: np.ndarray, alpha: np.ndarray) -> np.ndarray:
    """Pre-multiply translucent RGB against the white axes background.

    Returns opaque RGBA (alpha 1) so matplotlib's 3D painters sort never
    misorders faded trail segments (module docstring workaround).
    """
    blended = rgb * alpha[:, None] + (1.0 - alpha[:, None])
    return np.concatenate([blended, np.ones((alpha.size, 1))], axis=1)


def _scenario_title(result: SILResult, out: Path) -> str:
    """Human-readable scenario title for the title card."""
    stem = out.stem.replace("_", " ").strip()
    if stem and stem.lower() not in {"out", "tmp", "video", "hover"}:
        return f"Talon flight: {stem}"
    phases = " -> ".join(
        dict.fromkeys(phase.value for _, phase in result.phase_timeline)
    )
    return f"Talon flight: {phases}"


def _make_writer(
    out: Path, fps: int, codec: str
) -> tuple[AbstractMovieWriter, Path]:
    """Pick the movie writer; adjust the suffix for the GIF fallback."""
    if shutil.which("ffmpeg") is not None and out.suffix.lower() in _VIDEO_SUFFIXES:
        writer: AbstractMovieWriter = FFMpegWriter(
            fps=fps,
            codec=codec,
            extra_args=["-pix_fmt", "yuv420p", "-crf", "20"],
        )
        return writer, out
    return PillowWriter(fps=fps), out.with_suffix(".gif")


def _even_dpi(dpi: int) -> int:
    """Bump dpi by one when the frame would be odd-sized (yuv420p)."""
    width = round(_FIGSIZE[0] * dpi)
    height = round(_FIGSIZE[1] * dpi)
    return dpi + 1 if (width % 2 or height % 2) else dpi


def export_flight_video(
    result: SILResult,
    path: str | Path,
    *,
    fps: int = 30,
    speed: float = 2.0,
    trail_length: int = 200,
    dpi: int = 120,
    codec: str = "libx264",
) -> str:
    """Render a SILResult flight history to an MP4 (or GIF fallback).

    All keyword arguments default to the cinematic house settings: 30 fps
    at 2x time compression, a 200-point bright trail head, 120 dpi, and
    libx264. ``speed`` is the time-compression factor: each video frame
    advances ``speed / fps`` sim seconds through the control-rate
    history. Creates parent directories and returns the absolute output
    path; when ffmpeg is unavailable the export degrades to a PillowWriter
    GIF and the path suffix becomes .gif.
    """
    if result.time_s.size == 0:
        raise ValueError("cannot export a flight video from an empty SILResult")
    if fps <= 0:
        raise ValueError(f"fps must be positive, got {fps}")
    if speed <= 0.0:
        raise ValueError(f"speed must be positive, got {speed}")
    if trail_length < 2:
        raise ValueError(f"trail_length must be >= 2, got {trail_length}")

    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    writer, out = _make_writer(target, fps, codec)
    out = out.resolve()
    if isinstance(writer, FFMpegWriter):
        dpi = _even_dpi(dpi)

    apply_wong_theme()
    fig = plt.figure(figsize=_FIGSIZE, dpi=dpi)
    ax = fig.add_subplot(111, projection="3d")

    # ---- Scene geometry (altitude-above-home framing) ----------------
    north = result.truth_position[:, 0]
    east = result.truth_position[:, 1]
    alt = _altitude_above_home(result)
    points = np.column_stack([north, east, alt])
    ground = np.column_stack([north, east, np.zeros_like(north)])
    if result.waypoints_ned.size:
        wp_ne = result.waypoints_ned[:, :2]
        ground = np.vstack(
            [ground, np.column_stack([wp_ne, np.zeros(wp_ne.shape[0])])]
        )
    bounds = np.vstack([points, ground])
    mins = bounds.min(axis=0)
    maxs = bounds.max(axis=0)
    span = np.maximum(maxs - mins, 1.0)
    pad = _LIMIT_PADDING * span
    ax.set_xlim(mins[0] - pad[0], maxs[0] + pad[0])
    ax.set_ylim(mins[1] - pad[1], maxs[1] + pad[1])
    ax.set_zlim(mins[2] - pad[2], maxs[2] + pad[2])
    ax.set_box_aspect(tuple(span))
    ax.set_xlabel("North (m)")
    ax.set_ylabel("East (m)")
    ax.set_zlabel("Altitude (m)")
    ax.view_init(elev=_CAM_ELEV_DEG, azim=_CAM_AZIM_DEG)
    arm = _QUAD_ARM_FRACTION * float(np.max(span))

    # ---- Static scene artists ----------------------------------------
    waypoint_artists: list[object] = []
    if result.waypoints_ned.size:
        wp_n = result.waypoints_ned[:, 0]
        wp_e = result.waypoints_ned[:, 1]
        wp_z = np.zeros_like(wp_n)
        waypoint_artists.append(
            ax.scatter(
                wp_n,
                wp_e,
                wp_z,
                marker="^",
                s=60,
                color=WONG_PALETTE[6],
                depthshade=False,
            )
        )
        for k in range(wp_n.size):
            waypoint_artists.append(
                ax.text(
                    float(wp_n[k]),
                    float(wp_e[k]),
                    0.0,
                    f" {k + 1}",
                    color=WONG_PALETTE[6],
                    fontsize=8,
                )
            )

    # ---- Dynamic artists (updated per frame) -------------------------
    # 3D trail: Line3DCollection is the Axes3D-projectable LineCollection
    # (a plain 2D LineCollection cannot be depth-sorted in 3D).
    trail = art3d.Line3DCollection(
        np.zeros((0, 2, 3)), linewidths=_TRAIL_LINEWIDTH
    )
    ax.add_collection(trail)
    bright = ax.plot(
        [], [], [], color=WONG_PALETTE[5], linewidth=_BRIGHT_LINEWIDTH
    )[0]
    quad = ax.scatter(
        [],
        [],
        [],
        marker="o",
        s=_QUAD_MARKER_SIZE,
        color=WONG_PALETTE[5],
        edgecolors="white",
        linewidths=0.8,
        depthshade=False,
    )
    shadow = ax.scatter(
        [],
        [],
        [],
        marker=".",
        s=18,
        color=_SHADOW_COLOR,
        alpha=_SHADOW_ALPHA,
        depthshade=False,
    )
    cross = [
        ax.plot([], [], [], color=WONG_PALETTE[5], linewidth=1.1)[0]
        for _ in range(4)
    ]
    hud = ax.text2D(
        0.015,
        0.98,
        "",
        transform=ax.transAxes,
        va="top",
        ha="left",
        family="monospace",
        fontsize=9,
        color="white",
        bbox=dict(_HUD_BBOX),
    )
    title_text = ax.text2D(
        0.5,
        0.55,
        _scenario_title(result, out),
        transform=ax.transAxes,
        ha="center",
        va="center",
        fontsize=14,
        alpha=0.0,
        color="#222222",
    )

    # Pre-compute per-sample trail base colors (phase at each sample).
    sample_rgb = _sample_phase_colors(result.time_s, result.phase_timeline)
    velocity = result.truth_velocity
    duration = float(result.time_s[-1])
    n_flight = int(math.ceil(duration * fps / speed)) + 1
    n_frames = _TITLE_FRAMES + n_flight
    yaw = 0.0

    def _hide_scene(hidden: bool) -> None:
        for artist in (trail, bright, quad, shadow, hud, *cross):
            artist.set_visible(not hidden)
        for artist in waypoint_artists:
            artist.set_visible(not hidden)

    with writer.saving(fig, str(out), dpi=dpi):
        for frame in range(n_frames):
            if frame < _TITLE_FRAMES:
                # Title card: blank axes, scenario title fading in.
                _hide_scene(True)
                title_text.set_alpha((frame + 1) / _TITLE_FRAMES)
                writer.grab_frame()
                continue

            _hide_scene(False)
            title_text.set_alpha(0.0)
            t = (frame - _TITLE_FRAMES) * speed / fps
            cur = int(np.searchsorted(result.time_s, t, side="right") - 1)
            cur = min(max(cur, 0), result.time_s.size - 1)

            # Trail: consecutive pairs, phase color, aging alpha.
            pts = points[: cur + 1]
            if cur >= 1:
                segments = np.stack([pts[:-1], pts[1:]], axis=1)
                trail.set_segments(segments)
                alphas = np.linspace(
                    _TRAIL_ALPHA_OLD, _TRAIL_ALPHA_NEW, cur
                )
                trail.set_colors(_blend(sample_rgb[1 : cur + 1], alphas))
            # Bright head of the trail: the last trail_length points.
            head_lo = max(0, cur + 1 - trail_length)
            head = pts[head_lo:]
            bright.set_data_3d(head[:, 0], head[:, 1], head[:, 2])

            # Quadrotor marker + velocity-heading cross + ground shadow.
            n0 = float(north[cur])
            e0 = float(east[cur])
            a0 = float(alt[cur])
            quad._offsets3d = ([n0], [e0], [a0])
            shadow._offsets3d = ([n0], [e0], [0.0])
            ground_speed = float(
                np.hypot(velocity[cur, 0], velocity[cur, 1])
            )
            if ground_speed > _YAW_STOP_SPEED_MPS:
                yaw = math.atan2(velocity[cur, 1], velocity[cur, 0])
            for k, line in enumerate(cross):
                angle = yaw + k * math.pi / 2.0
                line.set_data_3d(
                    [n0, n0 + arm * math.cos(angle)],
                    [e0, e0 + arm * math.sin(angle)],
                    [a0, a0],
                )

            # HUD (time, phase, altitude, speed). The phase active at t
            # is the last timeline entry at or before t.
            tl_times = np.array(
                [tp for tp, _ in result.phase_timeline], dtype=np.float64
            )
            entry = int(np.searchsorted(tl_times, t, side="right") - 1)
            phase = result.phase_timeline[max(entry, 0)][1]
            speed_now = float(np.linalg.norm(velocity[cur]))
            hud.set_text(
                f"t = {t:.1f} s | phase: {phase.value} | "
                f"alt: {a0:.1f} m | speed: {speed_now:.1f} m/s"
            )
            writer.grab_frame()

    plt.close(fig)
    return str(out)


def _run_waypoints_with_capture(
    *,
    gps_noise_scale: float = 1.0,
    wind_scale: float = 1.0,
    sensor_seed: int = 20261012,
    wind_seed: int = 42,
) -> SILResult:
    """Re-run the Phase 4 waypoint circuit, capturing flight histories.

    Private helper for the Sprint 2.4 video pipeline: the Phase 4
    scenario (sims/scenario_waypoints.py) is a standalone metrics loop
    without a SILResult, so this adapter re-runs the same fixed-seed loop
    (same stack, constants, and noise defaults) and records time_s,
    truth_position, truth_velocity, ekf_position, and ekf_velocity at the
    control rate. The returned minimal SILResult has empty e_ct, zeroed
    commands, phase_timeline [(0.0, WAYPOINT)], and home/waypoints from
    the scenario config (home at the ground origin, start hover 10 m
    above wp0).
    """
    # Local imports: keep module import light and avoid a cycle through
    # talon.__init__ (same pattern as talon.plotting).
    from talon import (
        DT,
        DT_CONTROL,
        DT_GPS,
        CascadedController,
        ControlGains,
        DrydenWind,
        EKFConfig,
        EKFNominalState,
        Ekf,
        GPSConfig,
        IMUConfig,
        NLGLConfig,
        NLGLGuidance,
        SensorSuite,
        State,
        VehicleParams,
        Waypoint,
        WaypointMission,
        gps_to_ned,
        gps_velocity_ned,
        hover_thrust,
        propagate,
        specific_force_from_velocity,
    )
    from talon.mavlink_sil import SILResult
    from talon.motors import MotorModel, forward_mix
    from talon.types import FlightPhase

    gps_noise_scale = max(gps_noise_scale, _WP_MIN_GPS_NOISE_SCALE)
    params = VehicleParams()
    controller = CascadedController(ControlGains.defaults(), params)
    guidance = NLGLGuidance(NLGLConfig())

    waypoints = [
        Waypoint(
            np.array([north, east, -_WP_ALTITUDE_M]),
            speed=_WP_CRUISE_SPEED,
            radius=_WP_RADIUS_M,
        )
        for north, east in _WP_POSITIONS
    ]
    mission = WaypointMission(list(waypoints))

    start_position = np.array([0.0, 0.0, -_WP_ALTITUDE_M])
    truth = State()
    truth.position = start_position.copy()

    # Pre-spin the motor lag to hover thrust (Phase 3 convention).
    hover = hover_thrust(params)
    motors = MotorModel()
    for _ in range(round(1.0 / DT)):
        motors.step(hover, np.zeros(3), dt=DT)

    sensors = SensorSuite(
        IMUConfig(),
        GPSConfig(
            position_noise=_WP_GPS_POS_NOISE * gps_noise_scale,
            position_noise_vertical=_WP_GPS_POS_NOISE_V * gps_noise_scale,
            velocity_noise=_WP_GPS_VEL_NOISE * gps_noise_scale,
            update_rate=1.0 / DT_GPS,
        ),
        seed=sensor_seed,
    )
    ekf = Ekf(EKFConfig())
    ekf.initialize_level(start_position, np.zeros(3))
    wind = DrydenWind(
        _WP_WIND_SIGMA_H * wind_scale,
        _WP_WIND_SIGMA_H * wind_scale,
        _WP_WIND_SIGMA_V * wind_scale,
        200.0,
        200.0,
        50.0,
        DT,
        seed=wind_seed,
    )

    reference = start_position.copy()
    command_thrust = hover
    command_moment = np.zeros(3)
    steps = round(_WP_MAX_DURATION_S / DT)
    steps_per_control = round(DT_CONTROL / DT)
    steps_per_gps = round(DT_GPS / DT)
    v_prev = np.asarray(truth.velocity).copy()
    mission_complete_time: float | None = None
    time_s = 0.0

    times: list[float] = []
    truth_pos: list[np.ndarray] = []
    truth_vel: list[np.ndarray] = []
    ekf_pos: list[np.ndarray] = []
    ekf_vel: list[np.ndarray] = []

    for i in range(steps):
        # IMU + EKF predict (200 Hz), GPS update (10 Hz).
        rotation = np.asarray(truth.orientation.to_dcm())
        f_body = specific_force_from_velocity(
            v_prev, np.asarray(truth.velocity), rotation, DT
        )
        v_prev = np.asarray(truth.velocity).copy()
        imu = sensors.generate_imu(truth, f_body, DT)
        ekf.predict(np.asarray(imu.accel), np.asarray(imu.gyro), DT)
        if i % steps_per_gps == 0:
            gps = sensors.generate_gps(truth)
            ekf.update(gps_to_ned(gps), gps_velocity_ned(gps))

        # Mission + guidance + controller (100 Hz, ZOH) with capture.
        if i % steps_per_control == 0:
            nominal: EKFNominalState = ekf.nominal()
            ekf_position = np.asarray(nominal.position)
            ekf_velocity = np.asarray(nominal.velocity)
            diag = None
            if not mission.complete:
                mission.update(ekf_position)
                if mission.complete:
                    mission_complete_time = time_s
                v_des, diag = guidance.compute_mission(
                    ekf_position, ekf_velocity, mission
                )
            else:
                cfg = NLGLConfig()
                v_des = np.array(
                    [
                        0.0,
                        0.0,
                        float(
                            np.clip(
                                cfg.kp_alt
                                * (-_WP_ALTITUDE_M - ekf_position[2]),
                                -cfg.climb_rate_max,
                                cfg.climb_rate_max,
                            )
                        ),
                    ]
                )
            v_cmd = np.asarray(v_des)
            if not mission.complete:
                reference = reference + v_cmd * DT_CONTROL

            estimated = State()
            estimated.position = ekf_position.copy()
            estimated.velocity = ekf_velocity.copy()
            estimated.orientation = nominal.attitude
            estimated.angular_rate = ekf.estimated_body_rates(
                np.asarray(imu.gyro)
            )
            yaw_desired = diag.chi_d if diag is not None else 0.0
            command = controller.step(
                estimated, reference, v_cmd, yaw_desired, DT_CONTROL
            )
            command_thrust = command.collective_thrust
            command_moment = np.asarray(command.moment)

            times.append(time_s)
            truth_pos.append(np.asarray(truth.position).copy())
            truth_vel.append(np.asarray(truth.velocity).copy())
            ekf_pos.append(ekf_position.copy())
            ekf_vel.append(ekf_velocity.copy())

        # Motors + wind + truth propagation.
        forces = motors.step(command_thrust, command_moment, dt=DT)
        thrust_actual, moment_actual = forward_mix(forces)
        wind_ned = wind.step(float(np.linalg.norm(np.asarray(truth.velocity))))
        truth = propagate(
            truth, thrust_actual, moment_actual, wind_ned, DT, params
        )
        time_s += DT

        if mission_complete_time is not None and time_s >= (
            mission_complete_time + _WP_SETTLE_S
        ):
            break

    n = len(times)
    return SILResult(
        time_s=np.asarray(times, dtype=np.float64),
        truth_position=np.asarray(truth_pos, dtype=np.float64).reshape(-1, 3),
        truth_velocity=np.asarray(truth_vel, dtype=np.float64).reshape(-1, 3),
        ekf_position=np.asarray(ekf_pos, dtype=np.float64).reshape(-1, 3),
        ekf_velocity=np.asarray(ekf_vel, dtype=np.float64).reshape(-1, 3),
        phase_timeline=[(0.0, FlightPhase.WAYPOINT)],
        event_log=[],
        e_ct=np.zeros(0, dtype=np.float64),
        gps_update_times=np.zeros(0, dtype=np.float64),
        gps_skipped=0,
        gps_gated=0,
        metrics={},
        home_ned=np.zeros(3, dtype=np.float64),
        waypoints_ned=np.asarray(
            [wp.position for wp in waypoints], dtype=np.float64
        ).reshape(-1, 3),
        command_thrust=np.zeros(n, dtype=np.float64),
        command_moment=np.zeros((n, 3), dtype=np.float64),
    )


__all__ = ["export_flight_video"]
