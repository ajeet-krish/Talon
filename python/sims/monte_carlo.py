"""Monte Carlo robustness sweep for the Phase 4 waypoint circuit (Sprint 2.2).

Wraps sims.scenario_waypoints.run_scenario() in a sequential sweep: each
run i perturbs the GPS noise scale, Dryden wind scale, and start-position
offset by drawing from Gaussian distributions seeded per run, so the
sweep is fully deterministic given the MonteCarloConfig.

Documented choices:
- Perturbation draws use their own RNG stream (base_seed + 200_000 + i)
  so they are independent of the per-run sensor/wind seeds
  (base_seed + i and base_seed + 100_000 + i); changing n_runs never
  shifts another run's seeds.
- Sequential loop, no multiprocessing: one scenario run is a couple of
  seconds at most, and determinism plus simple stack traces matter more
  than wall time for a 50-run sweep.
- The acceptance thresholds mirrored in MonteCarloConfig default to the
  scenario's own constants (1.0 m mean |e_ct|, 3.0 m max |e_ct|,
  2.0 m EKF RMSE); the plot/report thresholds come from the config.

Honest framing (also stated in the generated report): this sweep samples
an empirical distribution of perturbed scenarios. A high pass rate is
evidence of robustness over that distribution, NOT a formal robustness
proof (no worst-case guarantee, no coverage of out-of-distribution
faults).
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.figure import Figure

from sims.scenario_waypoints import (
    WAYPOINT_POSITIONS,
    WaypointMetrics,
    run_scenario,
)
from talon.plotting import apply_wong_theme

# Wong orange (Okabe-Ito); matches talon.plotting.WONG_PALETTE[1].
WONG_ORANGE = "#E69F00"

# Fixed plot geometry (2 stacked panels).
FIGURE_SIZE_IN = (8.0, 10.0)


@dataclass(slots=True)
class MonteCarloConfig:
    """Parameters of the waypoint-circuit Monte Carlo sweep.

    The acceptance_* thresholds gate the per-run success flag computed
    by run_scenario() (mirrored here for the report and plot lines);
    changing them does not re-evaluate past runs' success flags, so
    keep them equal to the scenario constants unless you also re-run.
    """

    n_runs: int = 50
    base_seed: int = 2026
    gps_noise_sigma: float = 0.15
    wind_sigma: float = 0.20
    init_offset_sigma_m: float = 0.5
    acceptance_mean_e_ct_m: float = 1.0
    acceptance_max_e_ct_m: float = 3.0
    acceptance_ekf_rmse_m: float = 2.0


def run_monte_carlo(
    config: MonteCarloConfig | None = None,
) -> list[WaypointMetrics]:
    """Run the sweep sequentially; return one WaypointMetrics per run.

    Run i uses perturbation RNG seed ``base_seed + 200_000 + i`` for the
    GPS/wind/offset draws, sensor seed ``base_seed + i``, and wind seed
    ``base_seed + 100_000 + i``. Identical config => identical results.
    """
    cfg = config if config is not None else MonteCarloConfig()
    results: list[WaypointMetrics] = []
    for i in range(cfg.n_runs):
        perturb_rng = np.random.default_rng(cfg.base_seed + 200_000 + i)
        gps_scale = float(perturb_rng.normal(1.0, cfg.gps_noise_sigma))
        wind_scale = float(perturb_rng.normal(1.0, cfg.wind_sigma))
        offset_n, offset_e = perturb_rng.normal(
            0.0, cfg.init_offset_sigma_m, 2
        )
        results.append(
            run_scenario(
                gps_noise_scale=gps_scale,
                wind_scale=wind_scale,
                init_offset_ne=(float(offset_n), float(offset_e)),
                sensor_seed=cfg.base_seed + i,
                wind_seed=cfg.base_seed + 100_000 + i,
            )
        )
    return results


def _metric_panel(
    ax,
    values: np.ndarray,
    threshold: float,
    ylabel: str,
    title: str,
) -> None:
    """One violin + box panel with a red dashed acceptance threshold."""
    violin = ax.violinplot(
        [values], showmeans=False, showmedians=False, showextrema=False
    )
    for body in violin["bodies"]:
        body.set_facecolor(WONG_ORANGE)
        body.set_alpha(0.5)
        body.set_edgecolor(WONG_ORANGE)
    boxes = ax.boxplot(
        [values],
        widths=0.12,
        patch_artist=True,
        showfliers=True,
    )
    for box in boxes["boxes"]:
        box.set_facecolor(WONG_ORANGE)
        box.set_alpha(0.9)
    for median in boxes["medians"]:
        median.set_color("#000000")
    ax.axhline(
        threshold,
        color="red",
        linestyle="--",
        linewidth=1.2,
        label=f"acceptance {threshold:g} m",
    )
    ax.set_xticks([1])
    ax.set_xticklabels([f"n = {values.size}"])
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend(loc="upper right")


def plot_monte_carlo(
    results: list[WaypointMetrics],
    config: MonteCarloConfig,
    path: str | Path,
) -> str:
    """Write the 2-panel Monte Carlo summary PNG; return its path.

    Top panel: per-run mean |e_ct| with the acceptance threshold;
    bottom panel: per-run EKF position RMSE with its threshold. Wong
    orange violin + box per panel, 8x10 inches, 150 dpi (house theme).
    """
    apply_wong_theme()
    fig: Figure = plt.figure(figsize=FIGURE_SIZE_IN)
    gs = fig.add_gridspec(2, 1, hspace=0.35)
    mean_e_ct = np.array([r.mean_e_ct for r in results], dtype=float)
    ekf_rmse = np.array([r.ekf_rmse for r in results], dtype=float)

    ax_top = fig.add_subplot(gs[0])
    ax_bottom = fig.add_subplot(gs[1])
    if results:
        _metric_panel(
            ax_top,
            mean_e_ct,
            config.acceptance_mean_e_ct_m,
            "Mean cross-track |e_ct| (m)",
            "Mean cross-track error per run",
        )
        _metric_panel(
            ax_bottom,
            ekf_rmse,
            config.acceptance_ekf_rmse_m,
            "EKF position RMSE (m)",
            "EKF-vs-truth position RMSE per run",
        )
    else:  # degraded path: keep the figure, annotate, never raise
        for ax, label in ((ax_top, "mean |e_ct|"), (ax_bottom, "EKF RMSE")):
            ax.text(
                0.5,
                0.5,
                "no runs to plot",
                ha="center",
                va="center",
                transform=ax.transAxes,
                color="#666666",
            )
            ax.set_xticks([])
            ax.set_ylabel(label)

    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    plt.close(fig)
    return str(out)


def _stats_row(
    label: str, values: np.ndarray, acceptance: str, passes: int
) -> str:
    """One markdown table row: mean/std/p5/p50/p95/acceptance/pass rate."""
    n = values.size
    p5, p50, p95 = (float(x) for x in np.percentile(values, [5, 50, 95]))
    rate = 100.0 * passes / n if n else 0.0
    return (
        f"| {label} | {float(np.mean(values)):.3f} "
        f"| {float(np.std(values, ddof=1)) if n > 1 else 0.0:.3f} "
        f"| {p5:.3f} | {p50:.3f} | {p95:.3f} "
        f"| {acceptance} | {passes}/{n} ({rate:.1f} %) |"
    )


def write_report(
    results: list[WaypointMetrics],
    config: MonteCarloConfig,
    path: str | Path,
) -> str:
    """Write the markdown Monte Carlo report; return its path.

    The report contains the sweep parameters, a summary table
    (mean/std/p5/p50/p95/acceptance/pass rate per metric), the overall
    pass rate, and a caveats paragraph stating that this is an empirical
    distribution sample, NOT a formal robustness proof.
    """
    n = len(results)
    mean_e_ct = np.array([r.mean_e_ct for r in results], dtype=float)
    max_e_ct = np.array([r.max_e_ct for r in results], dtype=float)
    ekf_rmse = np.array([r.ekf_rmse for r in results], dtype=float)
    reached = np.array([r.waypoints_reached for r in results], dtype=float)
    n_wp = len(WAYPOINT_POSITIONS)
    passed = sum(1 for r in results if r.success)

    now = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    lines = [
        "# Talon Monte Carlo Report: Waypoint Circuit",
        "",
        f"Generated by `sims/monte_carlo.py` on {now}.",
        "",
        "## Sweep parameters",
        "",
        f"- runs: {config.n_runs}",
        f"- base seed: {config.base_seed}",
        f"- GPS noise scale sigma: {config.gps_noise_sigma}",
        f"- wind scale sigma: {config.wind_sigma}",
        f"- initial offset sigma: {config.init_offset_sigma_m} m",
        "",
        "Per run i the perturbation RNG (seed base_seed + 200_000 + i)",
        "draws the GPS noise scale and wind scale around 1.0 and the",
        "north/east start offset around 0; the sensor seed is",
        "base_seed + i and the wind seed is base_seed + 100_000 + i.",
        "",
        "## Results",
        "",
        "| Metric | Mean | Std | p5 | p50 | p95 | Acceptance | Pass rate |",
        "|--------|------|-----|----|-----|-----|------------|-----------|",
    ]
    lines.append(
        _stats_row(
            "mean \\|e_ct\\| (m)",
            mean_e_ct,
            f"< {config.acceptance_mean_e_ct_m:g} m",
            int(np.sum(mean_e_ct < config.acceptance_mean_e_ct_m)),
        )
    )
    lines.append(
        _stats_row(
            "max \\|e_ct\\| (m)",
            max_e_ct,
            f"< {config.acceptance_max_e_ct_m:g} m",
            int(np.sum(max_e_ct < config.acceptance_max_e_ct_m)),
        )
    )
    lines.append(
        _stats_row(
            "EKF position RMSE (m)",
            ekf_rmse,
            f"< {config.acceptance_ekf_rmse_m:g} m",
            int(np.sum(ekf_rmse < config.acceptance_ekf_rmse_m)),
        )
    )
    lines.append(
        _stats_row(
            "waypoints reached",
            reached,
            f"{n_wp} of {n_wp}",
            int(np.sum(reached >= n_wp)),
        )
    )
    rate = 100.0 * passed / n if n else 0.0
    lines += [
        "",
        f"Overall pass rate (all four criteria): {passed}/{n} ({rate:.1f} %)",
        "",
        "## Caveats",
        "",
        "This sweep samples an empirical distribution of perturbed",
        "scenarios: Gaussian draws on the GPS noise scale, wind scale,",
        "and start offset around the nominal configuration, with fixed",
        "per-run seeds. A high pass rate is evidence of robustness over",
        "THAT sampled distribution only. It is not a formal robustness",
        "proof: there is no worst-case guarantee, no statistical",
        "confidence bound on the true failure rate, and no coverage of",
        "out-of-distribution faults (sensor dropouts, actuator failures,",
        "wind outside the sampled range). Treat the pass rate as a",
        "regression signal across code changes, not as a certification",
        "artifact.",
        "",
    ]
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines), encoding="utf-8")
    return str(out)


if __name__ == "__main__":  # pragma: no cover - convenience entry point
    results = run_monte_carlo()
    report = write_report(
        results, MonteCarloConfig(), Path("plots/monte_carlo_report.md")
    )
    plot = plot_monte_carlo(
        results, MonteCarloConfig(), Path("plots/monte_carlo_summary.png")
    )
    passed = sum(1 for r in results if r.success)
    print(f"pass rate: {passed}/{len(results)}")
    print(f"report: {report}")
    print(f"plot: {plot}")
    sys.exit(0 if passed == len(results) else 1)
