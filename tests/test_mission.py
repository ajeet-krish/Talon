from __future__ import annotations

import numpy as np
import pytest

from talon import Waypoint, WaypointMission


def _wp(north: float, east: float, radius: float = 2.0) -> Waypoint:
    return Waypoint(np.array([north, east, -10.0]), radius=radius)


def test_empty_mission_rejected() -> None:
    with pytest.raises(ValueError):
        WaypointMission([])


def test_waypoint_advancement_and_completion() -> None:
    mission = WaypointMission([_wp(0.0, 0.0), _wp(20.0, 0.0), _wp(20.0, 20.0)])
    assert mission.active_index == 0
    assert not mission.complete

    # 2.5 m from wp0 (> radius 2.0): no advance.
    assert mission.update(np.array([2.5, 0.0, -10.0])) is False
    assert mission.active_index == 0

    # 1.5 m from wp0: advance once (the while-loop only advances past
    # waypoints actually within radius).
    assert mission.update(np.array([1.5, 0.0, -10.0])) is False
    assert mission.active_index == 1

    # Fly leg 2; nothing near wp1.
    assert mission.update(np.array([10.0, 0.0, -10.0])) is False
    assert mission.active_index == 1

    # Within radius of wp1: advance to wp2.
    assert mission.update(np.array([20.0, 1.0, -10.0])) is False
    assert mission.active_index == 2

    # Within radius of wp2 (last): mission completes, index clamps to last.
    assert mission.update(np.array([20.5, 19.0, -10.0])) is True
    assert mission.complete
    assert mission.active_index == 2

    # Complete latches: leaving the radius does not un-complete it.
    assert mission.update(np.array([0.0, 0.0, -10.0])) is True
    assert mission.active_index == 2


def test_start_within_first_radius_skips_immediately() -> None:
    # Parked on wp0: the first update() skips straight to wp1.
    mission = WaypointMission([_wp(0.0, 0.0), _wp(20.0, 0.0)])
    assert mission.update(np.array([0.0, 0.0, -10.0])) is False
    assert mission.active_index == 1


def test_previous_point_uses_start_position_then_predecessor() -> None:
    mission = WaypointMission([_wp(0.0, 0.0), _wp(20.0, 0.0), _wp(20.0, 20.0)])
    current = np.array([5.0, 1.0, -10.0])

    # Before any start is recorded: falls back to the current position.
    np.testing.assert_allclose(mission.previous_point(current), current)

    # Explicit start recording wins while still on the first waypoint.
    mission.set_start(np.array([-1.0, -2.0, -10.0]))
    np.testing.assert_allclose(
        mission.previous_point(current), np.array([-1.0, -2.0, -10.0])
    )

    # Once past the first waypoint the predecessor waypoint is the origin.
    mission.update(np.array([0.5, 0.0, -10.0]))  # -> index 1
    np.testing.assert_allclose(mission.previous_point(current), _wp(0.0, 0.0).position)
    mission.update(np.array([20.0, 0.5, -10.0]))  # -> index 2
    np.testing.assert_allclose(mission.previous_point(current), _wp(20.0, 0.0).position)


def test_update_records_start_position_automatically() -> None:
    mission = WaypointMission([_wp(50.0, 50.0)])
    mission.update(np.array([1.0, 2.0, -10.0]))
    np.testing.assert_allclose(
        mission.previous_point(np.zeros(3)), np.array([1.0, 2.0, -10.0])
    )


def test_distance_remaining_sanity() -> None:
    mission = WaypointMission([_wp(0.0, 0.0), _wp(20.0, 0.0), _wp(20.0, 20.0)])

    # At the start: dist to wp0 + both leg lengths = 0 + 20 + 20.
    assert mission.distance_remaining(np.array([0.0, 0.0, -10.0])) == pytest.approx(
        40.0
    )

    # Halfway on leg 1: 10 m to wp0 plus 40 m of remaining legs.
    assert mission.distance_remaining(np.array([-10.0, 0.0, -10.0])) == pytest.approx(
        50.0
    )

    # On wp2 (mission flown): distance to the final waypoint only.
    mission.update(np.array([0.5, 0.0, -10.0]))
    mission.update(np.array([20.0, 0.5, -10.0]))
    mission.update(np.array([20.0, 20.0, -10.0]))
    assert mission.complete
    assert mission.distance_remaining(np.array([20.0, 22.0, -10.0])) == pytest.approx(
        2.0
    )


def test_bad_shapes_rejected() -> None:
    mission = WaypointMission([_wp(0.0, 0.0)])
    with pytest.raises(ValueError):
        mission.update(np.zeros(2))
    with pytest.raises(ValueError):
        mission.distance_remaining(np.zeros(2))
    with pytest.raises(ValueError):
        mission.set_start(np.zeros(4))
