"""Obstacle-field circuit scenario (FlightLab Phase 1).

A 40 m x 30 m waypoint rectangle at 10 m altitude passes through a field
of spherical keep-out zones. The deterministic leg-detour planner
(talon.avoidance) inserts detour waypoints into the mission so the
resulting NLGL path steers around every obstacle with a stated clearance
margin. The full EKF-in-the-loop SIL stack runs the augmented mission,
and the minimum 3-D surface clearance of the flown trajectory is
asserted in CI.

Architecture (documented choices):
- Circuit: the Phase 4 rectangle (40 m north, 30 m east) at 10 m, cruise
  speed 5 m/s, acceptance radius 2.0 m. Start and finish at the origin.
- Obstacles: three spheres deliberately placed near the straight legs
  so the planner must insert at least one detour per intersected leg.
  Centres and radii are chosen so the detour is visible in the 3-D
  replay and the clearance margin is meaningfully exercised.
- Avoidance: talon.avoidance.plan_avoidance_mission runs BEFORE the
  simulation and returns the augmented waypoint list. No change to
  guidance.py or the C++ kernel.
- SIL: SILRunner with enable_udp=False (headless, no loopback sockets)
  and the augmented waypoints. The full closed loop (EKF, state machine,
  NLGL, cascaded PD, motor lag, Dryden wind) runs exactly as in the
  other SIL-backed scenarios.
- Acceptance: every augmented waypoint is reached (mission_complete),
  the minimum 3-D surface clearance of the truth trajectory is positive
  (no penetration), and the clearance meets the planned margin within a
  tracking tolerance.

Public API: run_scenario() returns (SILResult, ObstacleFieldMetrics);
main() runs the nominal scenario and prints the acceptance report.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass

import numpy as np

from talon.avoidance import (
    min_clearance,
    plan_avoidance_mission,
)
from talon.export_json import export_sil_result_json
from talon.mavlink_sil import SILConfig, SILRunner, SILResult
from talon.types import SphericalObstacle, Waypoint

ALTITUDE_M = 10.0
CRUISE_SPEED = 5.0
WP_RADIUS_M = 2.0
CLEARANCE_MARGIN_M = 2.0
CLEARANCE_TOLERANCE_M = 1.0  # tracking may cost up to 1 m of the margin
MAX_DURATION_S = 180.0

# Rectangle at 10 m: start/finish at the origin, 40 m north, 30 m east.
CIRCUIT_WAYPOINTS: list[Waypoint] = [
    Waypoint(np.array([0.0, 0.0, -ALTITUDE_M]), speed=CRUISE_SPEED, radius=WP_RADIUS_M),
    Waypoint(np.array([40.0, 0.0, -ALTITUDE_M]), speed=CRUISE_SPEED, radius=WP_RADIUS_M),
    Waypoint(np.array([40.0, 30.0, -ALTITUDE_M]), speed=CRUISE_SPEED, radius=WP_RADIUS_M),
    Waypoint(np.array([0.0, 30.0, -ALTITUDE_M]), speed=CRUISE_SPEED, radius=WP_RADIUS_M),
    Waypoint(np.array([0.0, 0.0, -ALTITUDE_M]), speed=CRUISE_SPEED, radius=WP_RADIUS_M),
]

# Spherical keep-out zones placed near the straight legs so the detour
# planner must route around them. Centres are NED (m); radii in m.
OBSTACLES: list[SphericalObstacle] = [
    SphericalObstacle("obs_north", np.array([20.0, 3.0, -ALTITUDE_M]), 4.0),
    SphericalObstacle("obs_east", np.array([37.0, 15.0, -ALTITUDE_M]), 3.5),
    SphericalObstacle("obs_west", np.array([3.0, 22.0, -ALTITUDE_M]), 4.5),
]


@dataclass(slots=True)
class ObstacleFieldMetrics:
    """Headline metrics of one obstacle-field run.

    success is the AND of the acceptance criteria: the augmented mission
    completed, the minimum 3-D surface clearance is positive, and the
    clearance meets the planned margin within the tracking tolerance.
    """

    min_clearance_m: float
    planned_margin_m: float
    waypoints_planned: int
    waypoints_reached: int
    mission_complete: bool
    duration_s: float
    success: bool


def run_scenario() -> tuple[SILResult, ObstacleFieldMetrics]:
    """Run one obstacle-field simulation and return (result, metrics)."""
    augmented = plan_avoidance_mission(
        CIRCUIT_WAYPOINTS, OBSTACLES, CLEARANCE_MARGIN_M
    )
    config = SILConfig(
        waypoints=augmented,
        enable_udp=False,
        seed=20261012,
        obstacles=list(OBSTACLES),
    )
    runner = SILRunner(config)
    runner.request_event("arm")
    runner.request_event("takeoff")
    result = runner.run(MAX_DURATION_S)

    clearance = min_clearance(result.truth_position, OBSTACLES)
    reached = result.metrics.get("waypoints_reached", 0)
    complete = bool(result.metrics.get("mission_complete", 0.0))

    success = (
        complete
        and clearance > 0.0
        and clearance >= CLEARANCE_MARGIN_M - CLEARANCE_TOLERANCE_M
    )
    metrics = ObstacleFieldMetrics(
        min_clearance_m=clearance,
        planned_margin_m=CLEARANCE_MARGIN_M,
        waypoints_planned=len(augmented),
        waypoints_reached=int(reached),
        mission_complete=complete,
        duration_s=float(result.time_s[-1]) if result.time_s.size else 0.0,
        success=success,
    )
    return result, metrics


def main() -> int:
    result, metrics = run_scenario()

    print("=== Obstacle-field circuit (40x30 m @ 10 m, keep-out zones) ===")
    print(f"waypoints planned: {metrics.waypoints_planned}")
    print(
        f"waypoints reached: {metrics.waypoints_reached}"
        f"/{metrics.waypoints_planned}"
    )
    print(f"mission complete: {metrics.mission_complete}")
    print(f"duration: {metrics.duration_s:.1f} s")
    print(
        f"min 3-D surface clearance: {metrics.min_clearance_m:.3f} m "
        f"(planned margin {metrics.planned_margin_m:.1f} m, "
        f"tolerance {CLEARANCE_TOLERANCE_M:.1f} m)"
    )

    ok = True
    if not metrics.mission_complete:
        print("FAIL: augmented mission did not complete")
        ok = False
    if metrics.min_clearance_m <= 0.0:
        print(
            f"FAIL: drone penetrated an obstacle "
            f"(clearance {metrics.min_clearance_m:.3f} m)"
        )
        ok = False
    if metrics.min_clearance_m < CLEARANCE_MARGIN_M - CLEARANCE_TOLERANCE_M:
        print(
            f"FAIL: clearance {metrics.min_clearance_m:.3f} m below margin "
            f"{CLEARANCE_MARGIN_M:.1f} m minus tolerance "
            f"{CLEARANCE_TOLERANCE_M:.1f} m"
        )
        ok = False
    if not ok:
        return 1
    print("PASS: obstacle field cleared within margin")
    return 0


if __name__ == "__main__":
    sys.exit(main())
