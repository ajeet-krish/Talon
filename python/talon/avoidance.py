"""Deterministic leg-detour obstacle avoidance and keep-out-zone geometry.

Single source of truth: ``core/src/avoidance.rs``. This module is a thin
re-export over the Rust core through the PyO3 extension ``talon._core``, with
no Python reimplementation: the leg geometry, the detour point, the offline
planner, and the clearance report are the same functions the autopilot flies in
the loop through its reactive law.

The public surface is unchanged from the pure-Python v1:

- :func:`leg_obstacle_distance` minimum horizontal distance from an
  obstacle centre to a leg segment;
- :func:`leg_intersects_obstacle` boolean penetration test with margin;
- :func:`compute_detour_point` one detour waypoint for one leg/obstacle
  pair;
- :func:`plan_avoidance_mission` insert detour waypoints into a circuit;
- :func:`min_clearance` minimum 3-D surface clearance over a trajectory.

Geometry is horizontal-plane (2-D) for the detour decision because the
flyable scenarios cruise at a constant altitude through a field of
obstacles that overlap that altitude band. Clearance reporting uses the
full 3-D Euclidean distance so a near-miss in altitude is also caught.

One behaviour is slightly stricter than the retired Python version: a negative
clearance margin is rejected up front by every function that takes one, rather
than only when a leg and an obstacle happen to reach the geometry.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray

from talon import _core
from talon.types import SphericalObstacle, Waypoint

def _leg(
    p0: NDArray[np.float64] | Sequence[float],
    p1: NDArray[np.float64] | Sequence[float],
) -> tuple[Sequence[float], Sequence[float]]:
    """Present two leg endpoints as plain sequences for the boundary."""
    return _vec(p0), _vec(p1)


def _vec(value: NDArray[np.float64] | Sequence[float]) -> Sequence[float]:
    """Convert a numpy vector to a plain list so the boundary reads it."""
    return [float(value[0]), float(value[1]), float(value[2])]


def leg_obstacle_distance(
    p0: NDArray[np.float64],
    p1: NDArray[np.float64],
    obstacle: SphericalObstacle,
) -> float:
    """Minimum horizontal distance from obstacle centre to segment p0 to p1.

    Projects the obstacle centre onto the 2-D leg segment (clamped to the
    segment endpoints) and returns the 2-D distance to that closest point.
    """
    a, b = _leg(p0, p1)
    return float(_core.leg_obstacle_distance(a, b, obstacle))


def leg_intersects_obstacle(
    p0: NDArray[np.float64],
    p1: NDArray[np.float64],
    obstacle: SphericalObstacle,
    clearance_margin: float = 0.0,
) -> bool:
    """True when the horizontal leg passes within radius + margin of centre."""
    a, b = _leg(p0, p1)
    return bool(_core.leg_intersects_obstacle(a, b, obstacle, clearance_margin))


def compute_detour_point(
    p0: NDArray[np.float64],
    p1: NDArray[np.float64],
    obstacle: SphericalObstacle,
    clearance_margin: float,
) -> NDArray[np.float64]:
    """One detour waypoint that routes the leg around the obstacle.

    Finds the closest-approach point of the obstacle centre on the 2-D
    leg, then steps radially outward from the centre by
    ``radius + clearance_margin + buffer`` so the detour point clears the
    keep-out zone. Altitude is held at the leg-start altitude.
    """
    a, b = _leg(p0, p1)
    detour = _core.compute_detour_point(a, b, obstacle, clearance_margin)
    return np.asarray(detour, dtype=np.float64)


def plan_avoidance_mission(
    waypoints: list[Waypoint],
    obstacles: list[SphericalObstacle],
    clearance_margin: float,
) -> list[Waypoint]:
    """Return a new waypoint list with detour waypoints inserted.

    For each consecutive pair (leg), the first obstacle whose keep-out
    zone (radius + clearance_margin) intersects the horizontal leg gets a
    single detour waypoint inserted before the leg end. The detour
    inherits the leg's cruise speed and acceptance radius. At most one
    detour per leg is inserted (v1); multiple obstacles on one leg would
    need a richer planner (v2), and the reactive law in the core is the path
    that handles several.
    """
    planned = _core.plan_avoidance_mission(waypoints, obstacles, clearance_margin)
    return [
        Waypoint(
            np.asarray(position, dtype=np.float64),
            speed=speed,
            heading=heading,
            radius=radius,
        )
        for position, speed, heading, radius in planned
    ]


def min_clearance(
    truth_positions: NDArray[np.float64],
    obstacles: list[SphericalObstacle],
) -> float:
    """Minimum 3-D surface clearance over a trajectory (m).

    For every sample and every obstacle, the 3-D Euclidean distance from
    the sample to the obstacle centre minus the radius. Returns +inf when
    there are no obstacles or no samples.
    """
    positions = np.asarray(truth_positions, dtype=np.float64).reshape(-1, 3)
    return float(_core.min_clearance(positions.tolist(), obstacles))


__all__ = [
    "compute_detour_point",
    "leg_intersects_obstacle",
    "leg_obstacle_distance",
    "min_clearance",
    "plan_avoidance_mission",
]
