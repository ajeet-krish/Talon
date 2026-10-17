"""Sprint 2.2 Monte Carlo wrapper tests (sims/monte_carlo.py).

Small smoke + determinism coverage only: the sweep is exercised with
tiny run counts so the whole file stays well under 10 s. Correctness of
the underlying scenario is covered by the scenario tests and the
Sprint 2.1 regression check.
"""

from __future__ import annotations

from dataclasses import fields

import pytest

from sims.monte_carlo import MonteCarloConfig, run_monte_carlo
from sims.scenario_waypoints import WaypointMetrics

EXPECTED_FIELDS = {
    "mean_e_ct",
    "max_e_ct",
    "ekf_rmse",
    "waypoints_reached",
    "mean_speed",
    "duration_s",
    "success",
}


def test_smoke_3_runs() -> None:
    """3 runs with a fixed seed all complete with well-formed metrics."""
    config = MonteCarloConfig(n_runs=3, base_seed=2026)
    results = run_monte_carlo(config)

    assert len(results) == 3
    for metrics in results:
        assert isinstance(metrics, WaypointMetrics)
        assert {f.name for f in fields(WaypointMetrics)} == EXPECTED_FIELDS
        assert isinstance(metrics.mean_e_ct, float)
        assert isinstance(metrics.max_e_ct, float)
        assert isinstance(metrics.ekf_rmse, float)
        assert isinstance(metrics.waypoints_reached, int)
        assert isinstance(metrics.mean_speed, float)
        assert isinstance(metrics.duration_s, float)
        assert metrics.success in (True, False)
        assert metrics.duration_s > 0.0


def test_deterministic_same_seed() -> None:
    """Two sweeps with an identical config produce identical metrics."""
    config = MonteCarloConfig(n_runs=2, base_seed=4242)
    first = run_monte_carlo(config)
    second = run_monte_carlo(config)

    assert len(first) == len(second) == 2
    for a, b in zip(first, second):
        assert a == b, f"non-deterministic run: {a} != {b}"


if __name__ == "__main__":  # pragma: no cover - manual convenience
    pytest.main([__file__, "-q"])
