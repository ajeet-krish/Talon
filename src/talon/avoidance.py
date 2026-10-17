"""Deterministic leg-detour obstacle avoidance (Phase 1, v1).

Pure-Python geometric planner that inserts detour waypoints into a
waypoint circuit so the resulting path steers around spherical keep-out
zones. No change to ``guidance.py`` or the C++ kernel: the augmented
waypoint list is consumed by the ordinary WaypointMission + NLGL path.

Geometry is horizontal-plane (2-D) for the detour decision because the
flyable scenarios cruise at a constant altitude through a field of
obstacles that overlap that altitude band. Clearance reporting uses the
full 3-D Euclidean distance so a near-miss in altitude is also caught.

The public surface is intentionally small and pure:

- :func:`leg_obstacle_distance` minimum horizontal distance from an
  obstacle centre to a leg segment;
- :func:`leg_intersects_obstacle` boolean penetration test with margin;
- :func:`compute_detour_point` one detour waypoint for one leg/obstacle
  pair;
- :func:`plan_avoidance_mission` insert detour waypoints into a circuit;
- :func:`min_clearance` minimum 3-D surface clearance over a trajectory.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from talon.types import SphericalObstacle, Waypoint

# Extra standoff (m) added beyond obstacle radius + clearance margin so
# the detour point itself stays outside the keep-out zone with a small
# buffer for the acceptance radius and guidance tracking error.
_DETOUR_BUFFER_M = 0.5


def _horizontal_distance(a: NDArray[np.float64], b: NDArray[np.float64]) -> float:
    """North-East plane distance between two NED positions (m)."""
    return float(np.linalg.norm(a[:2] - b[:2]))


def _3d_distance(a: NDArray[np.float64], b: NDArray[np.float64]) -> float:
    """Full 3-D Euclidean distance between two NED positions (m)."""
    return float(np.linalg.norm(a - b))


def leg_obstacle_distance(
    p0: NDArray[np.float64],
    p1: NDArray[np.float64],
    obstacle: SphericalObstacle,
) -> float:
    """Minimum horizontal distance from obstacle centre to segment p0 to p1.

    Projects the obstacle centre onto the 2-D leg segment (clamped to the
    segment endpoints) and returns the 2-D distance to that closest point.
    """
    centre = np.asarray(obstacle.center_ned, dtype=np.float64)[:2]
    a = np.asarray(p0, dtype=np.float64)[:2]
    b = np.asarray(p1, dtype=np.float64)[:2]
    d = b - a
    d_norm_sq = float(np.dot(d, d))
    if d_norm_sq < 1e-12:
        return _horizontal_distance(a, centre)
    t = float(np.clip(np.dot(centre - a, d) / d_norm_sq, 0.0, 1.0))
    closest = a + t * d
    return _horizontal_distance(centre, closest)


def leg_intersects_obstacle(
    p0: NDArray[np.float64],
    p1: NDArray[np.float64],
    obstacle: SphericalObstacle,
    clearance_margin: float = 0.0,
) -> bool:
    """True when the horizontal leg passes within radius + margin of centre."""
    if clearance_margin < 0.0:
        raise ValueError(
            f"clearance_margin must be non-negative, got {clearance_margin}"
        )
    return (
        leg_obstacle_distance(p0, p1, obstacle)
        < obstacle.radius_m + clearance_margin
    )


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
    if clearance_margin < 0.0:
        raise ValueError(
            f"clearance_margin must be non-negative, got {clearance_margin}"
        )
    a = np.asarray(p0, dtype=np.float64)[:2]
    b = np.asarray(p1, dtype=np.float64)[:2]
    centre = np.asarray(obstacle.center_ned, dtype=np.float64)[:2]
    d = b - a
    d_norm_sq = float(np.dot(d, d))
    if d_norm_sq < 1e-12:
        # Degenerate leg: offset east of the obstacle as a safe fallback.
        offset = obstacle.radius_m + clearance_margin + _DETOUR_BUFFER_M
        return np.array(
            [centre[0] + offset, centre[1], float(p0[2])], dtype=np.float64
        )
    t = float(np.clip(np.dot(centre - a, d) / d_norm_sq, 0.0, 1.0))
    closest = a + t * d
    away = closest - centre
    away_norm = float(np.linalg.norm(away))
    if away_norm < 1e-9:
        # Centre lies on the leg: use the leg perpendicular (left of travel).
        perp = np.array([-d[1], d[0]], dtype=np.float64)
        away = perp / np.linalg.norm(perp)
    else:
        away = away / away_norm
    reach = obstacle.radius_m + clearance_margin + _DETOUR_BUFFER_M
    detour_2d = closest + away * reach
    return np.array([detour_2d[0], detour_2d[1], float(p0[2])], dtype=np.float64)


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
    need a richer planner (v2).
    """
    if len(waypoints) < 2:
        return list(waypoints)
    augmented: list[Waypoint] = [waypoints[0]]
    for i in range(len(waypoints) - 1):
        p0 = waypoints[i].position
        p1 = waypoints[i + 1].position
        inserted = False
        for obs in obstacles:
            if leg_intersects_obstacle(p0, p1, obs, clearance_margin):
                detour = compute_detour_point(p0, p1, obs, clearance_margin)
                augmented.append(
                    Waypoint(
                        detour,
                        speed=waypoints[i].speed,
                        heading=waypoints[i].heading,
                        radius=waypoints[i].radius,
                    )
                )
                inserted = True
                break
        _ = inserted
        augmented.append(waypoints[i + 1])
    return augmented


def min_clearance(
    truth_positions: NDArray[np.float64],
    obstacles: list[SphericalObstacle],
) -> float:
    """Minimum 3-D surface clearance over a trajectory (m).

    For every sample and every obstacle, the 3-D Euclidean distance from
    the sample to the obstacle centre minus the radius. Returns +inf when
    there are no obstacles or no samples.
    """
    if len(obstacles) == 0 or len(truth_positions) == 0:
        return float("inf")
    positions = np.asarray(truth_positions, dtype=np.float64)
    worst = float("inf")
    for obs in obstacles:
        centre = np.asarray(obs.center_ned, dtype=np.float64)
        deltas = positions - centre
        distances = np.sqrt(np.sum(deltas * deltas, axis=1)) - obs.radius_m
        sample_min = float(np.min(distances))
        if sample_min < worst:
            worst = sample_min
    return worst


__all__ = [
    "compute_detour_point",
    "leg_intersects_obstacle",
    "leg_obstacle_distance",
    "min_clearance",
    "plan_avoidance_mission",
]
