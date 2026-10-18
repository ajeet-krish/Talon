"""Cascaded PD autopilot (single source of truth: core/src/controller.rs).

This module is a thin re-export of the Rust controller exposed through the
PyO3 extension `talon._core`. There is intentionally no Python
reimplementation; all simulation and (later) flight code must use this class so
sign and frame conventions stay in one place (see docs/conventions.md).
"""

from __future__ import annotations

from talon._core import CascadedController, ControlCommand, ControlGains

__all__ = ["CascadedController", "ControlCommand", "ControlGains"]
