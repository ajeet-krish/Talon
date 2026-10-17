"""Tests for the Phase 1 leg-detour obstacle avoidance (talon.avoidance).

Pure-geometry tests: no simulation is run. The leg/sphere intersection,
detour construction, mission augmentation, and clearance reporting are
exercised against hand-computable cases.
"""

from __future__ import annotations

import numpy as np
import pytest

from talon.avoidance import (
    compute_detour_point,
    leg_intersects_obstacle,
    leg_obstacle_distance,
    min_clearance,
    plan_avoidance_mission,
)
from talon.types import SphericalObstacle, Waypoint


def _obstacle(center, radius, id="obs"):
    return SphericalObstacle(id, np.array(center, dtype=np.float64), radius)


class TestLegObstacleDistance:
    def test_center_on_leg(self):
        obs = _obstacle([5.0, 0.0, 0.0], 1.0)
        assert leg_obstacle_distance(np.zeros(3), np.array([10.0, 0.0, 0.0]), obs) == pytest.approx(0.0)

    def test_center_off_leg(self):
        obs = _obstacle([5.0, 3.0, 0.0], 1.0)
        assert leg_obstacle_distance(np.zeros(3), np.array([10.0, 0.0, 0.0]), obs) == pytest.approx(3.0)

    def test_closest_point_clamped_to_start(self):
        obs = _obstacle([-5.0, 0.0, 0.0], 1.0)
        assert leg_obstacle_distance(np.zeros(3), np.array([10.0, 0.0, 0.0]), obs) == pytest.approx(5.0)

    def test_closest_point_clamped_to_end(self):
        obs = _obstacle([15.0, 0.0, 0.0], 1.0)
        assert leg_obstacle_distance(np.zeros(3), np.array([10.0, 0.0, 0.0]), obs) == pytest.approx(5.0)

    def test_degenerate_leg(self):
        obs = _obstacle([3.0, 4.0, 0.0], 1.0)
        assert leg_obstacle_distance(np.zeros(3), np.zeros(3), obs) == pytest.approx(5.0)


class TestLegIntersectsObstacle:
    def test_clear(self):
        obs = _obstacle([5.0, 3.0, 0.0], 1.0)
        assert not leg_intersects_obstacle(np.zeros(3), np.array([10.0, 0.0, 0.0]), obs)

    def test_penetrates(self):
        obs = _obstacle([5.0, 0.5, 0.0], 1.0)
        assert leg_intersects_obstacle(np.zeros(3), np.array([10.0, 0.0, 0.0]), obs)

    def test_margin_catches_near_miss(self):
        obs = _obstacle([5.0, 1.5, 0.0], 1.0)
        assert not leg_intersects_obstacle(np.zeros(3), np.array([10.0, 0.0, 0.0]), obs, clearance_margin=0.0)
        assert leg_intersects_obstacle(np.zeros(3), np.array([10.0, 0.0, 0.0]), obs, clearance_margin=1.0)

    def test_negative_margin_rejected(self):
        obs = _obstacle([5.0, 0.0, 0.0], 1.0)
        with pytest.raises(ValueError, match="non-negative"):
            leg_intersects_obstacle(np.zeros(3), np.array([10.0, 0.0, 0.0]), obs, clearance_margin=-1.0)


class TestComputeDetourPoint:
    def test_detour_clears_obstacle(self):
        obs = _obstacle([5.0, 0.0, 0.0], 2.0)
        detour = compute_detour_point(np.zeros(3), np.array([10.0, 0.0, 0.0]), obs, 1.0)
        dist = float(np.linalg.norm(detour[:2] - obs.center_ned[:2]))
        assert dist >= obs.radius_m + 1.0

    def test_detour_at_leg_altitude(self):
        obs = _obstacle([5.0, 0.0, -10.0], 2.0)
        detour = compute_detour_point(
            np.array([0.0, 0.0, -10.0]),
            np.array([10.0, 0.0, -10.0]),
            obs,
            1.0,
        )
        assert detour[2] == pytest.approx(-10.0)

    def test_center_on_leg_uses_perpendicular(self):
        obs = _obstacle([5.0, 0.0, 0.0], 2.0)
        detour = compute_detour_point(np.zeros(3), np.array([10.0, 0.0, 0.0]), obs, 0.5)
        assert detour[1] != pytest.approx(0.0)


class TestPlanAvoidanceMission:
    def test_no_obstacles_returns_copy(self):
        wps = [Waypoint(np.array([0.0, 0.0, -10.0])), Waypoint(np.array([10.0, 0.0, -10.0]))]
        assert len(plan_avoidance_mission(wps, [], 2.0)) == 2

    def test_intersected_leg_gets_detour(self):
        wps = [Waypoint(np.array([0.0, 0.0, -10.0])), Waypoint(np.array([10.0, 0.0, -10.0]))]
        obs = [_obstacle([5.0, 0.0, -10.0], 2.0)]
        augmented = plan_avoidance_mission(wps, obs, 1.0)
        assert len(augmented) == 3

    def test_clear_leg_unchanged(self):
        wps = [Waypoint(np.array([0.0, 0.0, -10.0])), Waypoint(np.array([10.0, 0.0, -10.0]))]
        obs = [_obstacle([5.0, 5.0, -10.0], 1.0)]
        augmented = plan_avoidance_mission(wps, obs, 1.0)
        assert len(augmented) == 2

    def test_single_waypoint_passthrough(self):
        wps = [Waypoint(np.array([0.0, 0.0, -10.0]))]
        assert len(plan_avoidance_mission(wps, [_obstacle([5.0, 0.0, -10.0], 2.0)], 1.0)) == 1


class TestMinClearance:
    def test_no_obstacles_returns_inf(self):
        assert min_clearance(np.zeros((5, 3)), []) == float("inf")

    def test_no_samples_returns_inf(self):
        assert min_clearance(np.zeros((0, 3)), [_obstacle([0.0, 0.0, 0.0], 1.0)]) == float("inf")

    def test_clearance_positive_outside(self):
        positions = np.array([[10.0, 0.0, 0.0], [12.0, 0.0, 0.0]])
        obs = [_obstacle([0.0, 0.0, 0.0], 2.0)]
        assert min_clearance(positions, obs) == pytest.approx(8.0)

    def test_clearance_negative_inside(self):
        positions = np.array([[1.0, 0.0, 0.0]])
        obs = [_obstacle([0.0, 0.0, 0.0], 2.0)]
        assert min_clearance(positions, obs) == pytest.approx(-1.0)
