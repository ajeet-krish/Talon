"""MEKF state estimator (single source of truth: core/include/talon/ekf.hpp).

Thin re-export of the C++ 15-state multiplicative error-state Kalman filter
exposed through the pybind11 extension `talon._core`. There is intentionally
no Python reimplementation; all simulation and flight code must use this
class so frame and sign conventions stay in one place (see
docs/conventions.md and the header's error-convention documentation).
"""

from __future__ import annotations

from talon._core import EKFConfig, EKFNominalState, Ekf

__all__ = ["EKFConfig", "EKFNominalState", "Ekf"]
