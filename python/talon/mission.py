"""Waypoint mission manager (Phase 4).

Owns the waypoint list, the active-index bookkeeping, and the acceptance
radius logic. Guidance (talon.guidance) is a pure function of (position,
velocity, leg); it never advances waypoints itself.

Frames: positions are NED (m); acceptance and distance computations are
horizontal-plane only (North-East), per docs/conventions.md. The mission
start position is recorded on the first ``update()`` call or explicitly via
``set_start()``; it serves as the start point of the first leg so a mission
begun off the first waypoint still has a well-defined path (the NLGL
behind-A recovery handles the off-leg case).
"""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

from talon.types import Waypoint


def _horizontal_distance(a: NDArray[np.float64], b: NDArray[np.float64]) -> float:
    """North-East plane distance between two NED positions (m)."""
    return float(np.linalg.norm(a[:2] - b[:2]))


class WaypointMission:
    """Sequential waypoint list with an active index and acceptance radius.

    ``update()`` advances the active index while the vehicle is within the
    ACTIVE waypoint's horizontal ``radius``. Advancing past the last
    waypoint latches ``complete``; the active index clamps to the last
    waypoint so guidance can still hold station on it.
    """

    def __init__(self, waypoints: list[Waypoint]) -> None:
        if not waypoints:
            raise ValueError("waypoints must be non-empty")
        self._waypoints = list(waypoints)
        self._active_index = 0
        self._complete = False
        self._start_position: NDArray[np.float64] | None = None

    @property
    def active_index(self) -> int:
        """Index of the waypoint currently being flown toward."""
        return self._active_index

    @property
    def complete(self) -> bool:
        """True once the vehicle has been within radius of the last waypoint."""
        return self._complete

    @property
    def waypoints(self) -> list[Waypoint]:
        """The mission waypoint list (read-only copy of the container)."""
        return list(self._waypoints)

    def active_waypoint(self) -> Waypoint:
        """The waypoint currently being flown toward."""
        return self._waypoints[self._active_index]

    def set_start(self, position_ned: ArrayLike) -> None:
        """Record the mission start position (leg origin of the first leg)."""
        position = np.asarray(position_ned, dtype=np.float64)
        if position.shape != (3,):
            raise ValueError(f"position_ned must have shape (3,), got {position.shape}")
        self._start_position = position.copy()

    def previous_point(self, current_position: ArrayLike) -> NDArray[np.float64]:
        """Start point of the active leg (NED, m).

        Returns waypoint[i-1] once past the first waypoint; otherwise the
        recorded mission start position; if neither exists yet, falls back
        to ``current_position`` so guidance always has a valid leg origin.
        """
        if self._active_index > 0:
            return self._waypoints[self._active_index - 1].position.copy()
        if self._start_position is not None:
            return self._start_position.copy()
        return np.asarray(current_position, dtype=np.float64).copy()

    def update(self, position_ned: ArrayLike) -> bool:
        """Advance the active waypoint if within its radius.

        Records the mission start position on the first call (if not set
        explicitly). Returns ``True`` once the mission is complete (the
        vehicle has been within radius of the LAST waypoint); complete
        stays latched True afterwards.
        """
        position = np.asarray(position_ned, dtype=np.float64)
        if position.shape != (3,):
            raise ValueError(f"position_ned must have shape (3,), got {position.shape}")
        # NaN comparisons are all False, so a non-finite position would
        # silently "advance through" every waypoint and complete the
        # mission; reject it loudly instead.
        if not np.all(np.isfinite(position)):
            raise ValueError(f"position_ned must be finite, got {position}")
        if self._start_position is None:
            self._start_position = position.copy()

        while not self._complete:
            active = self._waypoints[self._active_index]
            if _horizontal_distance(position, active.position) >= active.radius:
                break
            if self._active_index + 1 >= len(self._waypoints):
                self._complete = True
            else:
                self._active_index += 1
        return self._complete

    def distance_remaining(self, position_ned: ArrayLike) -> float:
        """Horizontal distance still to fly (m).

        Distance from ``position_ned`` to the active waypoint plus the
        horizontal lengths of all remaining legs. Once complete this is
        simply the horizontal distance to the final waypoint (how far the
        vehicle has drifted from the mission end point).
        """
        position = np.asarray(position_ned, dtype=np.float64)
        if position.shape != (3,):
            raise ValueError(f"position_ned must have shape (3,), got {position.shape}")
        total = _horizontal_distance(position, self.active_waypoint().position)
        for i in range(self._active_index, len(self._waypoints) - 1):
            total += _horizontal_distance(
                self._waypoints[i].position, self._waypoints[i + 1].position
            )
        return total


__all__ = ["WaypointMission"]
