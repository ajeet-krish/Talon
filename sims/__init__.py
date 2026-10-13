"""Talon scenario scripts (Phases 2-5) plus the Phase 6 validation driver
and the Sprint 2.2 Monte Carlo sweep.

Importable package: run_all.py inserts the repository root on sys.path
before importing these modules, and each scenario file also remains
directly executable (``uv run python sims/scenario_hover.py``).
"""

from sims.monte_carlo import MonteCarloConfig, run_monte_carlo

__all__ = ["MonteCarloConfig", "run_monte_carlo"]
