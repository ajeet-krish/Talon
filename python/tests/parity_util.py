"""Shared helpers for the Python vs ``talon._core`` parity suites (Phase 3a).

Every parity test runs the same inputs through the pure-Python module and
through the Rust extension (the same ``talon-core`` the browser runs) and
compares the outputs. The two sides evaluate transcendentals in different
libm builds (CPython uses the platform libm, the Rust core binds the
pure-Rust ``libm`` crate), so individual ``sin`` / ``cos`` / ``atan2`` /
``exp`` results may legitimately differ by one or two ULPs (about 1e-16
relative). ``assert_near`` therefore defaults to a 1e-12 relative AND
absolute tolerance: orders of magnitude tighter than any ULP difference and
orders of magnitude looser than any real algorithmic difference. Values that
are pure copies or exact IEEE arithmetic (configuration defaults, waypoint
fields, clamped altitude commands) are pinned at bit level by
``assert_exact``, which compares ``float.hex()`` for floats.

``DIVERGENCES`` registers the API-edge behaviours that genuinely differ
between the two implementations. Each entry is pinned by a test that calls
``expect_divergence`` first, so an unregistered divergence cannot be pinned
silently. The registered set, with the evidence for each:

* ``motors.step_negative_dt``: Python ``MotorModel.step`` raises
  ``ValueError("dt must be non-negative, got {dt}")`` for a negative ``dt``
  (python/talon/motors.py:100); the Rust core clamps ``dt`` to zero and
  returns the forces unchanged, because a flight loop must not be able to
  take the vehicle down by arithmetic (core/src/motors.rs:152).
* ``motors.time_const_nan``: Python ``MotorModel(float("nan"))`` is accepted,
  because its guard is ``time_const <= 0.0`` and NaN comparisons are all
  False (python/talon/motors.py:85); the first step then produces NaN
  forces. The Rust core rejects a NaN lag explicitly (core/src/motors.rs:129)
  and the binding raises ``ValueError``.
* ``motors.step_nan_dt``: Python ``MotorModel.step`` guards only
  ``dt < 0.0``, and NaN comparisons are all False, so a NaN ``dt``
  silently yields NaN forces that poison the model's stored state; the
  Rust core clamps NaN ``dt`` to zero (``NaN > 0.0`` is False) and returns
  the forces unchanged, and the binding mirrors the core.
* ``mission.update_non_finite_position``: Python ``WaypointMission.update``
  raises ``ValueError("position_ned must be finite, got <numpy repr>")``
  (python/talon/mission.py:99); the Rust core returns early without
  advancing instead, keeping the control loop panic-free
  (core/src/mission.rs:112), and the binding mirrors the core, returning
  the latched ``complete`` flag (True on a completed mission). Both sides
  leave the mission state untouched; only the reporting differs.
* ``guidance.config_is_a_snapshot``: Python ``NLGLGuidance.config`` is the
  live config dataclass, so writing ``g.config.zeta = ...`` retunes the next
  call; the binding's ``config`` getter returns a snapshot (the same policy
  as ``Ekf.nominal`` in docs/decisions.md), so the write is lost. Configure
  through the constructor argument when the value must reach the law.
* ``battery.config_is_a_snapshot``: Python ``BatteryModel`` keeps the passed
  ``BatteryConfig`` object and re-reads it every ``update``; the binding
  copies the config at construction, so mutating it afterwards does not
  retune the Rust model. Same snapshot policy as ``guidance.config``.
* ``battery.config_nan_rejected``: Python ``BatteryConfig(capacity_mah=
  nan)`` (or NaN ``hover_thrust_n``) is accepted (its guard is ``<= 0.0``)
  and silently corrupts the model (``max(0.0, nan)`` floors the state of
  charge to 0.0); the ``_core`` model rejects a NaN config with
  ``ValueError`` at ``BatteryModel`` construction, like the Rust core.
* ``battery.soc_nan_clamp``: Python ``BatteryModel(initial_soc=nan)`` and
  ``force_soc(nan)`` floor the state of charge to 0.0, because
  ``min(1.0, max(0.0, nan))`` runs ``max(0.0, nan)`` first and Python's
  ``max`` returns its first argument when the comparison is False; the
  ``_core`` stores NaN, because Rust's ``f64::clamp`` returns NaN for a
  NaN ``self``, so the corruption is visible instead of hidden.
* ``battery.update_nan_dt``: Python ``BatteryModel.update`` guards only
  ``dt < 0.0``, so a NaN ``dt`` integrates a NaN charge step and floors
  the state of charge to 0.0 (``max(0.0, nan)`` swallows the NaN); the
  Rust core clamps NaN ``dt`` to zero and leaves the state unchanged, and
  the binding mirrors the core.
* ``battery.update_negative_dt``: Python ``BatteryModel.update`` raises
  ``ValueError("dt must be non-negative, got {dt}")`` for a negative ``dt``
  (python/talon/battery.py:98); the Rust core clamps ``dt`` to zero and
  computes the current and voltage with an unchanged state of charge
  (core/src/battery.rs:167), and the binding mirrors the core.
* ``sensors.config_is_a_snapshot``: Python ``SensorSuite`` keeps the passed
  ``IMUConfig`` / ``GPSConfig`` objects and re-reads them every call; the
  binding copies both configs at construction, so mutating them afterwards
  does not retune the Rust suite. Same snapshot policy as
  ``guidance.config``.
* ``sensors.generate_imu_non_positive_dt``: Python ``SensorSuite.
  generate_imu`` raises ``ValueError("dt must be positive, got {dt}")`` for
  a non-positive ``dt`` (python/talon/sensors.py line 177); the Rust core
  returns a zero sample and does not advance the sensor clock
  (core/src/sensors.rs line 212), and the binding mirrors the core.
* ``sensors.specific_force_non_positive_dt``: Python
  ``specific_force_from_velocity`` raises ``ValueError("dt must be
  positive, got {dt}")`` for a non-positive ``dt`` (python/talon/sensors.py
  line 90); the Rust core returns the zero vector (core/src/sensors.rs line
  126), and the binding mirrors the core.
* ``state_machine.force_phase_accepts_labels``: Python
  ``FlightPhaseMachine.force_phase`` type-checks and raises ``TypeError``
  for anything but a ``FlightPhase`` enum; the binding duck-types and also
  accepts a plain lowercase label string (``"cruise"``), which is the
  ``FlightPhase.value`` the enum carries.
* ``state_machine.nan_takeoff_altitude``: Python ``FlightPhaseMachine(nan)``
  is accepted (its guard is ``takeoff_altitude <= 0.0`` and NaN comparisons
  are all False); the ``_core`` constructor rejects a NaN altitude with
  ``ValueError("takeoff_altitude must be positive, got NaN")``, like the
  Rust core (which spells NaN ``NaN`` where Python's ``str`` prints
  ``nan``).
* ``state_machine.request_duck_types_telemetry``: Python
  ``FlightPhaseMachine.request`` type-checks and raises ``TypeError`` for a
  non-``Telemetry``; the binding duck-types on the guard fields, so a dict
  (or any object missing the fields) raises ``AttributeError`` instead.
"""

from __future__ import annotations

import math
from typing import Any

REL_TOLERANCE = 1e-12
ABS_TOLERANCE = 1e-12

DIVERGENCES: dict[str, str] = {
    "battery.config_is_a_snapshot": (
        "Python BatteryModel keeps the passed BatteryConfig object and "
        "re-reads it on every update; the _core copies the config at "
        "construction, so mutating it afterwards does not retune the "
        "Rust model."
    ),
    "battery.config_nan_rejected": (
        "Python BatteryConfig(nan) is accepted (its guard is <= 0.0) and "
        "silently corrupts the model (max(0.0, nan) floors soc to 0.0); "
        "the _core BatteryModel rejects a NaN capacity or hover thrust "
        "with ValueError, like the Rust core."
    ),
    "battery.soc_nan_clamp": (
        "Python BatteryModel clamps a NaN initial_soc or force_soc to "
        "0.0 (min(1.0, max(0.0, nan)) returns 0.0); the _core stores NaN "
        "(Rust f64::clamp returns NaN for a NaN self), so the corrupted "
        "state is visible rather than hidden."
    ),
    "battery.update_nan_dt": (
        "Python BatteryModel.update guards only dt < 0.0, so a NaN dt "
        "integrates a NaN charge step and floors soc to 0.0; the _core "
        "clamps NaN dt to zero and leaves the state unchanged."
    ),
    "battery.update_negative_dt": (
        "Python BatteryModel.update raises ValueError on dt < 0; the _core "
        "clamps dt to zero, keeps the state of charge unchanged and still "
        "reports the load current and voltage."
    ),
    "guidance.config_is_a_snapshot": (
        "Python NLGLGuidance.config is the live dataclass (writes retune the "
        "law); the _core getter returns a snapshot, so g.config.zeta = x "
        "affects only the copy. Same policy as Ekf.nominal."
    ),
    "mission.update_non_finite_position": (
        "Python WaypointMission.update raises ValueError on a non-finite "
        "position; the _core mirrors the Rust core and returns the latched "
        "complete flag (True on a completed mission) with the mission "
        "state untouched."
    ),
    "motors.step_nan_dt": (
        "Python MotorModel.step guards only dt < 0.0, so a NaN dt yields "
        "NaN forces and poisons the model's stored state; the _core "
        "clamps NaN dt to zero and returns the forces unchanged."
    ),
    "motors.step_negative_dt": (
        "Python MotorModel.step raises ValueError on dt < 0; the _core "
        "clamps dt to zero and returns the forces unchanged."
    ),
    "motors.time_const_nan": (
        "Python MotorModel(float('nan')) is accepted (its guard is "
        "time_const <= 0.0) and yields NaN forces; the _core rejects a NaN "
        "lag with ValueError, like the Rust core."
    ),
    "sensors.config_is_a_snapshot": (
        "Python SensorSuite keeps the passed IMUConfig / GPSConfig objects "
        "and re-reads them every call; the _core copies both configs at "
        "construction, so mutating them afterwards does not retune the "
        "Rust suite."
    ),
    "sensors.generate_imu_non_positive_dt": (
        "Python SensorSuite.generate_imu raises ValueError on a "
        "non-positive dt; the _core returns a zero sample and does not "
        "advance the sensor clock, like the Rust core."
    ),
    "sensors.specific_force_non_positive_dt": (
        "Python specific_force_from_velocity raises ValueError on a "
        "non-positive dt; the _core returns the zero vector, like the Rust "
        "core."
    ),
    "state_machine.force_phase_accepts_labels": (
        "Python FlightPhaseMachine.force_phase raises TypeError for a "
        "non-FlightPhase; the _core duck-types and also accepts a plain "
        "lowercase label string (the FlightPhase.value text)."
    ),
    "state_machine.nan_takeoff_altitude": (
        "Python FlightPhaseMachine(nan) is accepted (its guard is "
        "takeoff_altitude <= 0.0); the _core constructor rejects a NaN "
        "altitude with ValueError, like the Rust core."
    ),
    "state_machine.request_duck_types_telemetry": (
        "Python FlightPhaseMachine.request raises TypeError for a "
        "non-Telemetry; the _core duck-types on the guard fields, so a "
        "missing attribute raises AttributeError instead."
    ),
}


def expect_divergence(key: str) -> str:
    """Mark the enclosing test as pinning a registered divergence.

    Call at the top of any parity test whose Python and Rust results are
    ALLOWED to differ, with one of the documented ``DIVERGENCES`` keys.
    Returns the registered explanation for reuse in assertion messages.
    Raises ``AssertionError`` for an unknown key, so a new divergence cannot
    be pinned without being registered here first.
    """
    try:
        return DIVERGENCES[key]
    except KeyError:
        raise AssertionError(
            f"undocumented divergence {key!r}; register it in "
            "python/tests/parity_util.py DIVERGENCES with evidence first"
        ) from None


def _is_sequence(value: Any) -> bool:
    """True for anything that behaves like a vector or matrix, not a scalar."""
    return hasattr(value, "__iter__") and not isinstance(value, (str, bytes, float))


def assert_exact(actual: Any, expected: Any, *, label: str = "") -> None:
    """Assert bit-level equality.

    Floats are compared through ``float.hex()``, so the failure message can
    show the two representations and ``0.1`` cannot pass as ``0.1 + 1e-17``.
    Sequences (lists, tuples, numpy arrays, nested rows) are compared
    element-wise, recursively.
    """
    if _is_sequence(actual) or _is_sequence(expected):
        actual_list, expected_list = list(actual), list(expected)
        assert len(actual_list) == len(expected_list), (
            f"{label}: length {len(actual_list)} != {len(expected_list)}"
        )
        for index, (item, wanted) in enumerate(zip(actual_list, expected_list)):
            assert_exact(item, wanted, label=f"{label}[{index}]")
        return
    if isinstance(actual, float) or isinstance(expected, float):
        actual_hex, expected_hex = float(actual).hex(), float(expected).hex()
        assert actual_hex == expected_hex, (
            f"{label}: {actual!r} ({actual_hex}) != {expected!r} ({expected_hex})"
        )
        return
    assert actual == expected, f"{label}: {actual!r} != {expected!r}"


def assert_near(
    actual: Any,
    expected: Any,
    *,
    rel_tol: float = REL_TOLERANCE,
    abs_tol: float = ABS_TOLERANCE,
    label: str = "",
) -> None:
    """Assert closeness within the parity tolerance (rel/abs 1e-12).

    Scalars compare with ``math.isclose``; sequences delegate to
    ``assert_vec_near`` so vectors and matrices work directly.
    """
    if _is_sequence(actual) or _is_sequence(expected):
        assert_vec_near(
            actual, expected, rel_tol=rel_tol, abs_tol=abs_tol, label=label
        )
        return
    actual_f, expected_f = float(actual), float(expected)
    assert math.isclose(actual_f, expected_f, rel_tol=rel_tol, abs_tol=abs_tol), (
        f"{label}: {actual_f!r} != {expected_f!r} "
        f"(rel_tol={rel_tol}, abs_tol={abs_tol})"
    )


def assert_vec_near(
    actual: Any,
    expected: Any,
    *,
    rel_tol: float = REL_TOLERANCE,
    abs_tol: float = ABS_TOLERANCE,
    label: str = "",
) -> None:
    """Assert two vectors (or matrices) match element-wise within tolerance."""
    actual_list, expected_list = list(actual), list(expected)
    assert len(actual_list) == len(expected_list), (
        f"{label}: length {len(actual_list)} != {len(expected_list)}"
    )
    for index, (item, wanted) in enumerate(zip(actual_list, expected_list)):
        if _is_sequence(item) or _is_sequence(wanted):
            assert_vec_near(
                item, wanted, rel_tol=rel_tol, abs_tol=abs_tol, label=f"{label}[{index}]"
            )
        else:
            assert_near(
                item, wanted, rel_tol=rel_tol, abs_tol=abs_tol, label=f"{label}[{index}]"
            )


def assert_same_error(
    python_error: BaseException,
    rust_error: BaseException,
    *,
    label: str = "",
) -> None:
    """Assert both implementations raised the same type with the same text."""
    assert type(python_error) is type(rust_error), (  # noqa: E721
        f"{label}: exception types differ: python "
        f"{type(python_error).__name__} vs rust {type(rust_error).__name__}"
    )
    assert str(python_error) == str(rust_error), (
        f"{label}: messages differ: python {str(python_error)!r} vs "
        f"rust {str(rust_error)!r}"
    )
