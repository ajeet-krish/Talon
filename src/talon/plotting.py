"""Publication-quality validation plots for Talon (Phase 6).

Colorblind safety: the default color cycle is the Wong 8 palette (the
Okabe-Ito extension recommended by Wong, Nature Methods 2011): black,
orange, sky blue, bluish green, yellow, blue, vermillion, reddish
purple. Every plot is readable in grayscale printouts too (line styles
differ where it matters).

Portfolio style: clean Matplotlib, light grid, SI units on every axis
label, tight_layout, 150 dpi PNG output. Call apply_wong_theme() once
per figure (each plot_* function does it for you) to keep every plot in
the report visually consistent.

Contract: every plot_* function takes a SILResult (from
talon.mavlink_sil) plus an output path, writes exactly one PNG, and
returns the path as a str. Empty or missing histories degrade to an
annotated near-empty figure instead of raising, so a caller can plot any
scenario result unconditionally. The functions only read the result;
they never mutate it.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.figure import Figure
from matplotlib.patches import Patch

if TYPE_CHECKING:  # pragma: no cover - typing only
    from talon.mavlink_sil import SILResult

# Wong 8 colorblind-safe palette (Okabe-Ito extension, Wong 2011).
WONG_PALETTE: tuple[str, ...] = (
    "#000000",  # black
    "#E69F00",  # orange
    "#56B4E9",  # sky blue
    "#009E73",  # bluish green
    "#F0E442",  # yellow
    "#0072B2",  # blue
    "#D55E00",  # vermillion
    "#CC79A7",  # reddish purple
)

# Flight phases in mission order with dedicated palette colors (the
# state-machine Gantt and trajectory annotations use these).
PHASE_ORDER: tuple[str, ...] = (
    "idle",
    "arm",
    "takeoff",
    "cruise",
    "waypoint",
    "rtl",
    "land",
    "emergency",
)
PHASE_COLORS: dict[str, str] = {
    "idle": "#999999",
    "arm": WONG_PALETTE[1],
    "takeoff": WONG_PALETTE[2],
    "cruise": WONG_PALETTE[3],
    "waypoint": WONG_PALETTE[5],
    "rtl": WONG_PALETTE[4],
    "land": WONG_PALETTE[6],
    "emergency": WONG_PALETTE[7],
}

_AXIS_NAMES = ("North", "East", "Down")
_COMPONENT_NAMES = ("North", "East", "Down")


def apply_wong_theme() -> None:
    """Set the shared publication rcParams (Wong palette + clean axes).

    Idempotent; called by every plot_* function so a figure made outside
    the report pipeline still matches the house style.
    """
    plt.rcParams.update(
        {
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "axes.grid": True,
            "grid.alpha": 0.3,
            "grid.linestyle": "-",
            "grid.linewidth": 0.6,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.titlesize": 11,
            "axes.labelsize": 10,
            "axes.xmargin": 0.02,
            "axes.ymargin": 0.05,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "legend.fontsize": 8,
            "legend.frameon": False,
            "lines.linewidth": 1.6,
            "lines.markersize": 4,
            "savefig.dpi": 150,
            "savefig.bbox": "tight",
            "font.family": "sans-serif",
        }
    )
    plt.rcParams["axes.prop_cycle"] = plt.cycler(color=list(WONG_PALETTE))


def _save(fig: Figure, path: str | Path) -> str:
    """Create parent directories, save at 150 dpi, close, return path."""
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    plt.close(fig)
    return str(out)


def _empty(fig: Figure, message: str, path: str | Path) -> str:
    """Save an annotated empty figure (degraded-data path)."""
    ax = fig.gca()
    ax.text(
        0.5,
        0.5,
        message,
        ha="center",
        va="center",
        transform=ax.transAxes,
        color="#666666",
    )
    ax.set_xticks([])
    ax.set_yticks([])
    return _save(fig, path)


def _altitude_above_home(result: SILResult) -> np.ndarray:
    """Altitude above the home point (m, positive up) per sample."""
    home_d = float(result.home_ned[2]) if result.home_ned.size else 0.0
    return home_d - result.truth_position[:, 2]


def plot_trajectory_3d(result: SILResult, path: str | Path) -> str:
    """3D NED truth trajectory with waypoint and home markers.

    Plotted as North, East, and ALTITUDE above home (up-positive) so the
    figure reads like a map rather than a NED-z-down plot; the equal-ish
    box aspect keeps slopes physically honest. Waypoints come from
    SILResult.waypoints_ned (empty for hover-only runs).
    """
    apply_wong_theme()
    fig = plt.figure(figsize=(7.5, 5.5))
    if result.time_s.size == 0:
        return _empty(fig, "no trajectory samples", path)
    ax = fig.add_subplot(111, projection="3d")

    north = result.truth_position[:, 0]
    east = result.truth_position[:, 1]
    alt = _altitude_above_home(result)
    ax.plot(north, east, alt, color=WONG_PALETTE[5], label="Truth trajectory")
    ax.scatter(
        north[:1],
        east[:1],
        alt[:1],
        color=WONG_PALETTE[3],
        marker="o",
        s=50,
        label="Start",
        depthshade=False,
    )
    if result.waypoints_ned.size:
        home_d = float(result.home_ned[2]) if result.home_ned.size else 0.0
        ax.scatter(
            result.waypoints_ned[:, 0],
            result.waypoints_ned[:, 1],
            home_d - result.waypoints_ned[:, 2],
            color=WONG_PALETTE[6],
            marker="^",
            s=55,
            label="Waypoints",
            depthshade=False,
        )
    if result.home_ned.size:
        ax.scatter(
            [result.home_ned[0]],
            [result.home_ned[1]],
            [0.0],
            color=WONG_PALETTE[1],
            marker="*",
            s=110,
            label="Home",
            depthshade=False,
        )

    points = np.column_stack([north, east, alt])
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    span = np.maximum(maxs - mins, 1.0)
    pad = 0.05 * span
    ax.set_xlim(mins[0] - pad[0], maxs[0] + pad[0])
    ax.set_ylim(mins[1] - pad[1], maxs[1] + pad[1])
    ax.set_zlim(mins[2] - pad[2], maxs[2] + pad[2])
    ax.set_box_aspect(tuple(span))
    ax.set_xlabel("North (m)")
    ax.set_ylabel("East (m)")
    ax.set_zlabel("Altitude (m)")
    ax.set_title("Truth trajectory (NED, altitude above home)")
    ax.legend(loc="upper left")
    return _save(fig, path)


def plot_position_error(result: SILResult, path: str | Path) -> str:
    """EKF-vs-truth position error per axis plus norm, with 3-sigma.

    Four stacked axes sharing time: signed error on North, East, Down,
    then the 3D norm. When SILResult.position_var is populated the
    per-axis +/-3-sigma envelopes (3*sqrt(P_ii)) are overlaid, so a
    healthy filter keeps its error inside the envelope by construction.
    """
    apply_wong_theme()
    if result.time_s.size == 0:
        fig = plt.figure(figsize=(7.5, 7))
        return _empty(fig, "no samples", path)
    err = result.ekf_position - result.truth_position
    norm = np.linalg.norm(err, axis=1)
    has_var = result.position_var.shape == err.shape

    fig, axes = plt.subplots(
        4, 1, figsize=(7.5, 7), sharex=True, height_ratios=[1, 1, 1, 1.2]
    )
    t = result.time_s
    for i, ax in enumerate(axes[:3]):
        ax.plot(t, err[:, i], color=WONG_PALETTE[5], label="Error")
        if has_var:
            sigma = 3.0 * np.sqrt(result.position_var[:, i])
            ax.fill_between(
                t,
                -sigma,
                sigma,
                color=WONG_PALETTE[2],
                alpha=0.25,
                label="3-sigma",
                linewidth=0,
            )
        ax.axhline(0.0, color="#444444", linewidth=0.7)
        ax.set_ylabel(f"{_AXIS_NAMES[i]} (m)")
        if i == 0:
            ax.set_title("EKF-vs-truth position error")
            ax.legend(loc="upper right")
    axes[3].plot(t, norm, color=WONG_PALETTE[6], label="|error|")
    if has_var:
        sigma_norm = 3.0 * np.sqrt(result.position_var.sum(axis=1))
        axes[3].fill_between(
            t,
            0.0,
            sigma_norm,
            color=WONG_PALETTE[2],
            alpha=0.25,
            label="3-sigma (trace)",
            linewidth=0,
        )
    axes[3].set_ylabel("Norm (m)")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(loc="upper right")
    fig.tight_layout()
    return _save(fig, path)


def plot_ekf_convergence(result: SILResult, path: str | Path) -> str:
    """EKF convergence: position/velocity error norms and bias estimates.

    2x2 grid over time: position error norm (with 3-sigma trace envelope
    when available), velocity error norm, gyro bias estimates, and accel
    bias estimates. Bias panels show the filter's random-walk states
    tracking the (simulated) true sensor biases.
    """
    apply_wong_theme()
    if result.time_s.size == 0:
        fig = plt.figure(figsize=(8.5, 5.5))
        return _empty(fig, "no samples", path)
    t = result.time_s
    pos_err = np.linalg.norm(result.ekf_position - result.truth_position, axis=1)
    vel_err = np.linalg.norm(result.ekf_velocity - result.truth_velocity, axis=1)

    fig, axes = plt.subplots(2, 2, figsize=(8.5, 5.5), sharex=True)
    ax = axes[0, 0]
    ax.plot(t, pos_err, color=WONG_PALETTE[5], label="|pos error|")
    if result.position_var.shape[0] == t.size:
        ax.fill_between(
            t,
            0.0,
            3.0 * np.sqrt(result.position_var.sum(axis=1)),
            color=WONG_PALETTE[2],
            alpha=0.25,
            label="3-sigma (trace)",
            linewidth=0,
        )
    ax.set_ylabel("Position (m)")
    ax.set_title("EKF convergence")
    ax.legend(loc="upper right")

    axes[0, 1].plot(t, vel_err, color=WONG_PALETTE[6], label="|vel error|")
    axes[0, 1].set_ylabel("Velocity (m/s)")
    axes[0, 1].legend(loc="upper right")

    for i, name in enumerate(_COMPONENT_NAMES):
        if result.gyro_bias.shape[0] == t.size:
            axes[1, 0].plot(t, result.gyro_bias[:, i], label=name)
        if result.accel_bias.shape[0] == t.size:
            axes[1, 1].plot(t, result.accel_bias[:, i], label=name)
    axes[1, 0].axhline(0.0, color="#444444", linewidth=0.7)
    axes[1, 1].axhline(0.0, color="#444444", linewidth=0.7)
    axes[1, 0].set_ylabel("Gyro bias (rad/s)")
    axes[1, 1].set_ylabel("Accel bias (m/s$^2$")
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 0].legend(loc="upper right", title="Body")
    axes[1, 1].legend(loc="upper right", title="Body")
    fig.tight_layout()
    return _save(fig, path)


def plot_control_effort(result: SILResult, path: str | Path) -> str:
    """Control effort: actual collective thrust and three body moments.

    Four stacked axes sharing time. Thrust is the lagged, mixed value
    applied to the plant (SILResult.command_thrust), with the hover
    thrust m*g as a dashed reference; moments are the applied body
    moments Mx, My, Mz.
    """
    apply_wong_theme()
    if result.time_s.size == 0 or result.command_thrust.size == 0:
        fig = plt.figure(figsize=(7.5, 7))
        return _empty(fig, "no actuator history", path)
    # Local import: keeps plotting importable without the constants pull
    # at module load (and avoids a cycle through talon.__init__).
    from talon.constants import GRAVITY, MASS

    t = result.time_s
    fig, axes = plt.subplots(4, 1, figsize=(7.5, 7), sharex=True)
    axes[0].plot(t, result.command_thrust, color=WONG_PALETTE[5])
    axes[0].axhline(
        MASS * GRAVITY,
        color=WONG_PALETTE[6],
        linestyle="--",
        linewidth=1.0,
        label="Hover $m g$",
    )
    axes[0].set_ylabel("Thrust (N)")
    axes[0].set_title("Control effort (applied to plant)")
    axes[0].legend(loc="upper right")
    moment_labels = ("$M_x$", "$M_y$", "$M_z$")
    for i, ax in enumerate(axes[1:]):
        ax.plot(t, result.command_moment[:, i], color=WONG_PALETTE[(i + 1) % 8])
        ax.axhline(0.0, color="#444444", linewidth=0.7)
        ax.set_ylabel(f"{moment_labels[i]} (N m)")
    axes[3].set_xlabel("Time (s)")
    fig.tight_layout()
    return _save(fig, path)


def plot_cross_track_error(result: SILResult, path: str | Path) -> str:
    """NLGL cross-track error over time and over path length, per leg.

    Top axis: |e_ct| vs sim time, colored by mission leg index (samples
    only exist during WAYPOINT phase, SILResult.e_ct_times). Bottom
    axis: |e_ct| vs cumulative horizontal distance flown (an
    along-track-style progress coordinate for the whole run). Runs with
    no WAYPOINT phase (hover, emergency) degrade to an empty note.
    """
    apply_wong_theme()
    fig, (ax_t, ax_s) = plt.subplots(
        2, 1, figsize=(7.5, 6), sharex=False
    )
    if result.e_ct.size == 0 or result.e_ct_times.size == 0:
        for ax in (ax_t, ax_s):
            ax.text(
                0.5,
                0.5,
                "no cross-track samples (no WAYPOINT phase)",
                ha="center",
                va="center",
                transform=ax.transAxes,
                color="#666666",
            )
            ax.set_xticks([])
            ax.set_yticks([])
        ax_t.set_title("NLGL cross-track error")
        fig.tight_layout()
        return _save(fig, path)

    legs = np.unique(result.e_ct_leg)
    for k, leg in enumerate(legs):
        mask = result.e_ct_leg == leg
        label = f"leg {int(leg)}" if leg > 0 else "leg 0 (start)"
        ax_t.plot(
            result.e_ct_times[mask],
            result.e_ct[mask],
            color=WONG_PALETTE[int(k) % 8],
            label=label,
        )
    ax_t.set_ylabel("|$e_{ct}$| (m)")
    ax_t.set_xlabel("Time (s)")
    ax_t.set_title("NLGL cross-track error")
    ax_t.legend(loc="upper right")

    # Cumulative horizontal path length as the along-track coordinate.
    ne_speed = np.linalg.norm(
        np.diff(result.truth_position[:, :2], axis=0), axis=1
    )
    path_len = np.concatenate([[0.0], np.cumsum(ne_speed)])
    for k, leg in enumerate(legs):
        mask = result.e_ct_leg == leg
        # e_ct samples sit on control ticks; map them to the nearest
        # path-length sample via searchsorted on time.
        idx = np.searchsorted(result.time_s, result.e_ct_times[mask])
        idx = np.clip(idx, 0, path_len.size - 1)
        ax_s.plot(
            path_len[idx],
            result.e_ct[mask],
            color=WONG_PALETTE[int(k) % 8],
            label=f"leg {int(leg)}",
        )
    ax_s.set_ylabel("|$e_{ct}$| (m)")
    ax_s.set_xlabel("Distance flown (m)")
    fig.tight_layout()
    return _save(fig, path)


def plot_state_machine(result: SILResult, path: str | Path) -> str:
    """Gantt-style flight-phase timeline from the phase history.

    One horizontal band per phase occupancy (SILResult.phase_timeline);
    accepted state-machine events (result.event_log, guard_ok) are
    vertical markers labeled with the event name, rejected attempts are
    small ticks along the bottom edge for the audit trail.
    """
    apply_wong_theme()
    fig, ax = plt.subplots(figsize=(8.5, 3.8))
    if not result.phase_timeline:
        return _empty(fig, "empty phase timeline", path)
    t_end = float(result.time_s[-1]) if result.time_s.size else 0.0
    timeline = list(result.phase_timeline)
    if timeline[-1][0] < t_end:
        timeline.append((t_end, timeline[-1][1]))

    y_of = {name: i for i, name in enumerate(PHASE_ORDER)}
    for (t0, phase_a), (t1, _phase_b) in zip(timeline[:-1], timeline[1:]):
        name = phase_a.value
        ax.axhspan(
            y_of[name] - 0.4,
            y_of[name] + 0.4,
            xmin=t0 / max(t_end, 1e-9),
            xmax=t1 / max(t_end, 1e-9),
            facecolor=PHASE_COLORS.get(name, "#999999"),
            alpha=0.8,
            edgecolor="white",
            linewidth=0.5,
        )

    accepted = [e for e in result.event_log if e.guard_ok]
    rejected = [e for e in result.event_log if not e.guard_ok]
    for k, event in enumerate(accepted):
        ax.axvline(
            event.time, color="#333333", linestyle=":", linewidth=1.0, alpha=0.8
        )
        ax.text(
            event.time,
            len(PHASE_ORDER) - 0.25 - 0.55 * (k % 2),
            f" {event.event}",
            rotation=90,
            va="top",
            ha="left",
            fontsize=7.5,
            color="#333333",
        )
    if rejected:
        times = [e.time for e in rejected]
        ax.plot(
            times,
            np.full(len(times), -0.85),
            linestyle="none",
            marker=3,  # tri-down tick
            markersize=4,
            color=WONG_PALETTE[6],
            label="rejected attempt",
            clip_on=False,
        )

    ax.set_yticks(range(len(PHASE_ORDER)))
    ax.set_yticklabels(PHASE_ORDER)
    ax.set_ylim(-1.1, len(PHASE_ORDER) + 0.6)
    ax.set_xlim(0.0, max(t_end, 1e-9))
    ax.set_xlabel("Time (s)")
    ax.set_title("Flight phase machine (bands) and accepted events (dotted)")
    handles = [
        Patch(facecolor=PHASE_COLORS[name], alpha=0.8, label=name)
        for name in PHASE_ORDER
    ]
    if rejected:
        handles.append(
            plt.Line2D(
                [],
                [],
                linestyle="none",
                marker=3,
                color=WONG_PALETTE[6],
                label="rejected attempt",
            )
        )
    ax.legend(
        handles=handles, loc="upper center", ncol=4, fontsize=7.5
    )
    fig.tight_layout()
    return _save(fig, path)


def plot_wind_response(result: SILResult, path: str | Path) -> str:
    """Wind components and vehicle response on stacked time axes.

    Top: total wind (turbulence + steady) NED components, from
    SILResult.wind_ned. Bottom: the EKF-vs-truth position error norm,
    the same estimator-health metric the wind scenario gates on. Calm-air
    runs show a flat zero wind (the steady component is still logged).
    """
    apply_wong_theme()
    if result.time_s.size == 0:
        fig = plt.figure(figsize=(7.5, 5))
        return _empty(fig, "no samples", path)
    t = result.time_s
    fig, (ax_w, ax_e) = plt.subplots(
        2, 1, figsize=(7.5, 5), sharex=True, height_ratios=[1.3, 1]
    )
    if result.wind_ned.shape[0] == t.size:
        for i, name in enumerate(_COMPONENT_NAMES):
            ax_w.plot(t, result.wind_ned[:, i], label=name)
    ax_w.axhline(0.0, color="#444444", linewidth=0.7)
    ax_w.set_ylabel("Wind (m/s)")
    ax_w.set_title("Wind and vehicle response")
    ax_w.legend(loc="upper right", ncol=3)

    err = np.linalg.norm(result.ekf_position - result.truth_position, axis=1)
    ax_e.plot(t, err, color=WONG_PALETTE[6], label="|pos error|")
    ax_e.set_ylabel("EKF error (m)")
    ax_e.set_xlabel("Time (s)")
    ax_e.legend(loc="upper right")
    fig.tight_layout()
    return _save(fig, path)


__all__ = [
    "PHASE_COLORS",
    "PHASE_ORDER",
    "WONG_PALETTE",
    "apply_wong_theme",
    "plot_control_effort",
    "plot_cross_track_error",
    "plot_ekf_convergence",
    "plot_position_error",
    "plot_state_machine",
    "plot_trajectory_3d",
    "plot_wind_response",
]
