"""Python vs ``talon._core`` parity for the waypoint mission manager.

The acceptance-radius logic, the sequential chaining, the completion latch
and the leg-origin fallbacks are exercised against hand-computable cases on
both implementations, plus one registered divergence: Python raises on a
non-finite ``update`` position where the Rust core (and therefore the
binding) returns early without advancing.
"""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from parity_util import (
    assert_near,
    assert_same_error,
    assert_vec_near,
    expect_divergence,
)
from talon import _core
from talon.mission import WaypointMission
from talon.types import Waypoint


def _waypoint(north: float, east: float, down: float = -10.0, **kwargs: float) -> Waypoint:
    return Waypoint(np.array([north, east, down], dtype=np.float64), **kwargs)


def _missions(waypoints: list[Waypoint]) -> tuple[WaypointMission, "_core.WaypointMission"]:
    return WaypointMission(waypoints), _core.WaypointMission(waypoints)


def _three_waypoints() -> list[Waypoint]:
    return [_waypoint(0.0, 0.0), _waypoint(20.0, 0.0), _waypoint(20.0, 20.0)]


def test_empty_waypoint_list_error_is_identical() -> None:
    with pytest.raises(ValueError) as py_error:
        WaypointMission([])
    with pytest.raises(ValueError) as rust_error:
        _core.WaypointMission([])
    assert_same_error(py_error.value, rust_error.value, label="WaypointMission([])")
    assert str(rust_error.value) == "waypoints must be non-empty"


def test_initial_surface_parity() -> None:
    waypoints = [
        _waypoint(0.0, 0.0),
        _waypoint(20.0, 5.0, down=-12.0, speed=3.0, heading=0.5, radius=1.5),
    ]
    py_mission, rs_mission = _missions(waypoints)
    assert py_mission.active_index == rs_mission.active_index == 0
    assert py_mission.complete is False
    assert rs_mission.complete is False

    # The python side exposes the list as a property of dataclasses, the
    # binding as a method of tuples; the values must be identical, down to
    # the duck-typed defaults (speed 5, heading 0, radius 2).
    py_list = py_mission.waypoints
    rs_list = rs_mission.waypoints()
    assert len(rs_list) == len(py_list) == 2
    for py_wp, rs_wp in zip(py_list, rs_list):
        assert_vec_near(rs_wp[0], py_wp.position, label="waypoint position")
        assert_near(rs_wp[1], py_wp.speed, label="waypoint speed")
        assert_near(rs_wp[2], py_wp.heading, label="waypoint heading")
        assert_near(rs_wp[3], py_wp.radius, label="waypoint radius")

    py_active, rs_active = py_mission.active_waypoint(), rs_mission.active_waypoint()
    assert_vec_near(rs_active[0], py_active.position, label="active position")
    assert_near(rs_active[1], py_active.speed, label="active speed")
    assert_near(rs_active[2], py_active.heading, label="active heading")
    assert_near(rs_active[3], py_active.radius, label="active radius")


@pytest.mark.parametrize(
    ("position", "advances"),
    [
        pytest.param([2.5, 0.0, -10.0], False, id="outside_radius_holds"),
        pytest.param([2.0, 0.0, -10.0], False, id="exactly_at_radius_holds"),
        pytest.param([1.5, 0.0, -10.0], True, id="inside_radius_advances"),
    ],
)
def test_acceptance_radius_boundary(position: list[float], advances: bool) -> None:
    # Acceptance is strict: distance >= radius holds the waypoint, so a
    # vehicle exactly on the radius circle has not been accepted yet. The
    # comparison is horizontal-plane only, so altitude never matters.
    py_mission, rs_mission = _missions([_waypoint(0.0, 0.0), _waypoint(20.0, 0.0)])
    py_done = py_mission.update(np.asarray(position, dtype=np.float64))
    rs_done = rs_mission.update(position)
    expected_index = 1 if advances else 0
    assert py_mission.active_index == rs_mission.active_index == expected_index
    assert py_done is False and rs_done is False


def test_altitude_does_not_affect_acceptance() -> None:
    py_mission, rs_mission = _missions([_waypoint(0.0, 0.0), _waypoint(20.0, 0.0)])
    py_mission.update(np.array([0.0, 0.0, 500.0]))
    rs_mission.update([0.0, 0.0, 500.0])
    assert py_mission.active_index == rs_mission.active_index == 1


def test_chaining_and_completion_parity() -> None:
    py_mission, rs_mission = _missions(_three_waypoints())
    # (position, expected active index, expected complete) steps, walked
    # identically by both missions.
    steps = [
        ([2.5, 0.0, -10.0], 0, False),  # outside wp0: hold
        ([1.5, 0.0, -10.0], 1, False),  # within wp0: advance
        ([10.0, 0.0, -10.0], 1, False),  # mid leg 1: nothing near wp1
        ([20.0, 1.0, -10.0], 2, False),  # within wp1: advance
        ([20.5, 19.0, -10.0], 2, True),  # within wp2: complete
    ]
    for position, index, done in steps:
        py_done = py_mission.update(np.asarray(position, dtype=np.float64))
        rs_done = rs_mission.update(position)
        assert py_done is done, f"python at {position}"
        assert rs_done is done, f"rust at {position}"
        assert py_mission.active_index == rs_mission.active_index == index
        assert py_mission.complete is done
        assert rs_mission.complete is done

    # The completion latch holds: leaving the radius does not un-complete,
    # and the active index clamps to the last waypoint.
    py_latched = py_mission.update(np.array([0.0, 0.0, -10.0]))
    rs_latched = rs_mission.update([0.0, 0.0, -10.0])
    assert py_latched is True and rs_latched is True
    assert py_mission.active_index == rs_mission.active_index == 2


def test_previous_point_fallbacks_parity() -> None:
    py_mission, rs_mission = _missions(_three_waypoints())
    current = [7.0, 1.0, -3.0]

    # (a) No start recorded: the current position is the leg origin.
    assert_vec_near(
        rs_mission.previous_point(current),
        py_mission.previous_point(np.asarray(current)),
        label="fallback to current position",
    )
    assert_vec_near(
        rs_mission.previous_point(current), current, label="current position values"
    )

    # (b) An explicit set_start wins while still on the first waypoint.
    py_mission.set_start([-1.0, -2.0, -10.0])
    rs_mission.set_start([-1.0, -2.0, -10.0])
    assert_vec_near(
        rs_mission.previous_point(current),
        py_mission.previous_point(np.asarray(current)),
        label="explicit start",
    )
    assert_vec_near(
        rs_mission.previous_point(current),
        [-1.0, -2.0, -10.0],
        label="explicit start values",
    )

    # (c) Past the first waypoint the predecessor waypoint is the origin.
    py_mission.update([0.0, 0.0, -10.0])
    rs_mission.update([0.0, 0.0, -10.0])
    assert rs_mission.active_index == py_mission.active_index == 1
    assert_vec_near(
        rs_mission.previous_point(current),
        py_mission.previous_point(np.asarray(current)),
        label="predecessor waypoint",
    )
    assert_vec_near(
        rs_mission.previous_point(current),
        [0.0, 0.0, -10.0],
        label="predecessor waypoint values",
    )


def test_first_update_records_the_start_position() -> None:
    py_mission, rs_mission = _missions([_waypoint(50.0, 50.0)])
    py_mission.update(np.array([1.0, 2.0, -10.0]))
    rs_mission.update([1.0, 2.0, -10.0])
    assert_vec_near(
        rs_mission.previous_point([0.0, 0.0, 0.0]),
        py_mission.previous_point(np.zeros(3)),
        label="recorded start",
    )
    assert_vec_near(
        rs_mission.previous_point([0.0, 0.0, 0.0]),
        [1.0, 2.0, -10.0],
        label="recorded start values",
    )


def test_distance_remaining_parity() -> None:
    py_mission, rs_mission = _missions(_three_waypoints())
    for position in ([0.0, 0.0, -10.0], [-10.0, 0.0, -10.0], [25.0, 17.0, -10.0]):
        py_distance = py_mission.distance_remaining(np.asarray(position, dtype=np.float64))
        rs_distance = rs_mission.distance_remaining(position)
        assert_near(rs_distance, py_distance, label=f"distance at {position}")
    # At the start: 0 to wp0 plus the two 20 m legs.
    assert_near(py_mission.distance_remaining(np.array([0.0, 0.0, -10.0])), 40.0)
    assert_near(rs_mission.distance_remaining([0.0, 0.0, -10.0]), 40.0)

    # Walk the mission; once complete only the drift from the end remains.
    for position in ([0.5, 0.0, -10.0], [20.0, 0.5, -10.0], [20.0, 20.0, -10.0]):
        py_mission.update(np.asarray(position, dtype=np.float64))
        rs_mission.update(position)
    assert py_mission.complete and rs_mission.complete
    py_drift = py_mission.distance_remaining(np.array([20.0, 22.0, -10.0]))
    rs_drift = rs_mission.distance_remaining([20.0, 22.0, -10.0])
    assert_near(py_drift, 2.0, label="python drift from the end")
    assert_near(rs_drift, py_drift, label="drift parity")


def test_position_only_waypoint_fallback_defaults_parity() -> None:
    # The Python side builds a position-only Waypoint (the dataclass
    # fills speed 5.0, heading 0.0, radius 2.0); the binding reads the
    # same fallback defaults from a duck-typed object that carries only
    # `position`. The surfaced waypoints and the acceptance behaviour must
    # then be identical.
    py_mission = WaypointMission([Waypoint(np.array([20.0, 5.0, -10.0]))])
    duck_waypoint = SimpleNamespace(position=[20.0, 5.0, -10.0])
    rs_mission = _core.WaypointMission([duck_waypoint])
    py_waypoints, rs_waypoints = py_mission.waypoints, rs_mission.waypoints()
    assert len(py_waypoints) == len(rs_waypoints) == 1
    py_wp, rs_wp = py_waypoints[0], rs_waypoints[0]
    assert_vec_near(rs_wp[0], py_wp.position, label="position")
    assert_near(rs_wp[1], py_wp.speed, label="speed fallback")
    assert_near(rs_wp[2], py_wp.heading, label="heading fallback")
    assert_near(rs_wp[3], py_wp.radius, label="radius fallback")
    # The fallback defaults themselves: speed 5.0, heading 0.0, radius 2.0.
    assert_near(py_wp.speed, 5.0, label="python default speed")
    assert_near(py_wp.heading, 0.0, label="python default heading")
    assert_near(py_wp.radius, 2.0, label="python default radius")
    # Exactly at the defaulted 2.0 m radius both hold the waypoint.
    py_done = py_mission.update(np.array([22.0, 5.0, -10.0]))
    rs_done = rs_mission.update([22.0, 5.0, -10.0])
    assert py_done is False and rs_done is False
    assert py_mission.active_index == rs_mission.active_index == 0
    # Inside it both accept and complete the single-leg mission.
    py_done = py_mission.update(np.array([19.0, 5.0, -10.0]))
    rs_done = rs_mission.update([19.0, 5.0, -10.0])
    assert py_done is True and rs_done is True
    assert py_mission.complete and rs_mission.complete


def test_broken_optional_waypoint_property_is_propagated() -> None:
    # Security review close-out: the duck-typed waypoint reader defaults
    # speed, heading and radius only on AttributeError. An optional field
    # that exists but raises on access must propagate, not silently read
    # as the default (the Python side stores the object as-is and only
    # reads position/radius on update, so only the binding side is
    # pinned here).
    class BrokenWaypoint:
        position = [20.0, 0.0, -10.0]

        @property
        def radius(self) -> float:
            raise RuntimeError("radius read failed")

    with pytest.raises(RuntimeError, match="radius read failed"):
        _core.WaypointMission([BrokenWaypoint()])

    class BrokenSpeed(BrokenWaypoint):
        radius = 2.0

        @property
        def speed(self) -> float:
            raise RuntimeError("speed read failed")

    with pytest.raises(RuntimeError, match="speed read failed"):
        _core.WaypointMission([BrokenSpeed()])


def test_update_non_finite_position_is_a_registered_divergence() -> None:
    reason = expect_divergence("mission.update_non_finite_position")
    py_mission, rs_mission = _missions(_three_waypoints())
    with pytest.raises(ValueError, match="position_ned must be finite"):
        py_mission.update(np.array([float("nan"), 0.0, -10.0]))
    # The Rust core returns early to keep the control loop panic-free: no
    # exception, no advance, no completion, no recorded start.
    assert rs_mission.update([float("nan"), 0.0, -10.0]) is False
    assert rs_mission.active_index == 0
    assert rs_mission.complete is False
    # Both sides leave the state untouched; only the reporting differs.
    assert py_mission.active_index == rs_mission.active_index, reason
    assert py_mission.complete is rs_mission.complete is False
