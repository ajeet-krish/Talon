from __future__ import annotations

import pytest


def _core_import_error() -> str | None:
    try:
        import talon  # noqa: F401
    except Exception as exc:  # pragma: no cover - depends on build state
        return str(exc)
    return None


_CORE_ERROR = _core_import_error()


def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    if _CORE_ERROR is None:
        return
    skip_core = pytest.mark.skip(
        reason=(
            "talon._core is not built; run "
            "`cmake -B build && cmake --build build -j` (from `uv run` or with "
            f"the virtualenv active) first. Import error: {_CORE_ERROR}"
        )
    )
    for item in items:
        item.add_marker(skip_core)


@pytest.fixture()
def default_state():
    """State at the origin, at rest, level (identity orientation)."""
    from talon import State

    return State()


@pytest.fixture()
def default_params():
    from talon import VehicleParams

    return VehicleParams()
