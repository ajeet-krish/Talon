from __future__ import annotations

import math

import numpy as np
import pytest

from talon import NLGLConfig, NLGLGuidance, Waypoint, WaypointMission
from talon.guidance import cross2, wrap_angle

# Northbound leg A -> B used by most tests: due north at 10 m altitude.
_A_NORTH = np.array([0.0, 0.0, -10.0])
_B_NORTH = np.array([100.0, 0.0, -10.0])


def test_cross2_sign_and_wrap_angle() -> None:
    assert cross2(np.array([1.0, 0.0]), np.array([0.0, 1.0])) == 1.0
    assert cross2(np.array([0.0, 1.0]), np.array([1.0, 0.0])) == -1.0
    assert wrap_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    assert wrap_angle(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)
    assert wrap_angle(0.3) == pytest.approx(0.3)
    assert wrap_angle(-2.0 * math.pi + 0.3) == pytest.approx(0.3)


def test_straight_line_offset_steers_toward_track() -> None:
    # Vehicle 5 m LEFT (west) of a northbound track, ground velocity along
    # the track: e_ct > 0 (+left), Nu1 clipped at 45 deg (5 m > L1*sin45),
    # Nu2 = 0, chi_d steers right (east) toward the path.
    guidance = NLGLGuidance()
    position = np.array([0.0, -5.0, -10.0])
    velocity = np.array([5.0, 0.0, 0.0])
    v_des, diag = guidance.compute_leg(position, velocity, _A_NORTH, _B_NORTH)

    cfg = NLGLConfig()
    expected_l1 = cfg.zeta * cfg.period_t * 5.0 / math.pi
    assert diag.e_ct == pytest.approx(5.0)
    assert diag.l1 == pytest.approx(expected_l1)
    assert diag.nu1 == pytest.approx(math.radians(45.0))
    assert diag.nu2 == pytest.approx(0.0, abs=1e-9)
    assert diag.chi_d == pytest.approx(math.radians(45.0))
    assert not diag.behind_a_recovery

    # v_des points mostly along the track (north) and steers right (east).
    assert v_des[0] > 0.0
    assert v_des[1] > 0.0
    assert v_des[0] == pytest.approx(5.0 * math.cos(math.radians(45.0)))
    assert v_des[1] == pytest.approx(5.0 * math.sin(math.radians(45.0)))


def test_cross_track_convergence_kinematic_point_mass() -> None:
    # Kinematic closed loop (no rigid-body dynamics): a point mass whose
    # ground velocity first-order tracks v_des (tau approximates the
    # cascaded-PD velocity loop; a mass with INSTANT course tracking would
    # be the sampled-data degenerate case chi_k = Nu1 - chi_{k-1}, which
    # is marginally stable at best, not representative of any real inner
    # loop). Start 5 m RIGHT of the track; the cross-track error must fall
    # below 1 m within 50 m along-track.
    guidance = NLGLGuidance()
    position = np.array([0.0, 5.0, -10.0])
    velocity = np.array([5.0, 0.0, 0.0])

    dt = 0.01
    tau = 0.3
    while position[0] < 50.0:
        v_des, _ = guidance.compute_leg(position, velocity, _A_NORTH, _B_NORTH)
        velocity = velocity + (v_des - velocity) * (dt / tau)
        position = position + velocity * dt

    r_ne = position[:2] - _A_NORTH[:2]
    u_ab = (_B_NORTH[:2] - _A_NORTH[:2]) / 100.0
    assert abs(cross2(r_ne, u_ab)) < 1.0


def test_nu2_ground_velocity_holds_crab_in_wind() -> None:
    # Constant east wind: the vehicle must CRAB (air-relative course
    # differs from ground course) and, because Nu2 uses GROUND velocity,
    # converge to the track with zero steady-state cross-track error.
    # Point mass with first-order ground-velocity tracking (tau); the wind
    # is only visible through the air-relative velocity.
    guidance = NLGLGuidance()
    wind = np.array([0.0, 2.0, 0.0])
    position = np.array([0.0, -5.0, -10.0])  # 5 m left of track
    velocity = np.array([5.0, 0.0, 0.0])

    dt = 0.01
    tau = 0.3
    for _ in range(3000):  # 30 s
        v_des, _ = guidance.compute_leg(position, velocity, _A_NORTH, _B_NORTH)
        velocity = velocity + (v_des - velocity) * (dt / tau)
        position = position + velocity * dt

    _, diag = guidance.compute_leg(position, velocity, _A_NORTH, _B_NORTH)
    assert abs(diag.e_ct) < 0.5

    # Ground course is (nearly) along the track; the AIR-relative course
    # carries the crab: v_air = v_g - wind is pushed downwind (east).
    ground_course = math.atan2(velocity[1], velocity[0])
    v_air = velocity - wind
    air_course = math.atan2(v_air[1], v_air[0])
    assert abs(ground_course) < 0.05
    expected_crab = math.atan2(-2.0, 5.0)  # east wind -> crab west of track
    assert abs(air_course - expected_crab) < 0.1


def test_behind_a_recovery_steers_at_a() -> None:
    # Start 20 m BEFORE the leg start, 3 m off-leg: s < -0.7071 and
    # ||p - A|| > L1 (v = 0 -> L1 = l1_min = 3 m), so recovery triggers
    # and chi_d points straight at A.
    guidance = NLGLGuidance()
    a = np.array([0.0, 0.0, -10.0])
    b = np.array([20.0, 0.0, -10.0])
    position = np.array([-20.0, 3.0, -10.0])
    velocity = np.zeros(3)

    _, diag = guidance.compute_leg(position, velocity, a, b)
    assert diag.behind_a_recovery is True
    assert diag.chi_d == pytest.approx(math.atan2(-3.0, 20.0))

    # 10 m before A on a 20 m leg: s = -0.5 > -0.7071 -> no recovery.
    position_near = np.array([-10.0, 3.0, -10.0])
    _, diag_near = guidance.compute_leg(position_near, velocity, a, b)
    assert diag_near.behind_a_recovery is False


def test_l1_floor_at_zero_velocity() -> None:
    guidance = NLGLGuidance()
    _, diag = guidance.compute_leg(
        np.array([10.0, 2.0, -10.0]), np.zeros(3), _A_NORTH, _B_NORTH
    )
    assert diag.l1 == NLGLConfig().l1_min
    # Near-zero speed: Nu2 falls back to 0 (course undefined), Nu = Nu1.
    assert diag.nu2 == 0.0
    assert diag.nu == pytest.approx(diag.nu1)


def test_altitude_channel_ned_sign_and_clamp() -> None:
    # NED: climb is NEGATIVE v_des_z (z down). wp_b above the vehicle ->
    # climb command clamped to -climb_rate_max; below -> +climb_rate_max.
    guidance = NLGLGuidance()
    position = np.array([0.0, 0.0, -10.0])
    velocity = np.zeros(3)

    v_des, _ = guidance.compute_leg(
        position, velocity, _A_NORTH, np.array([10.0, 0.0, -20.0])
    )
    assert v_des[2] == -NLGLConfig().climb_rate_max

    v_des, _ = guidance.compute_leg(
        position, velocity, _A_NORTH, np.array([10.0, 0.0, -5.0])
    )
    assert v_des[2] == NLGLConfig().climb_rate_max

    # Small error: proportional, unclamped: z_des = -11, z = -10, v_z = 0
    # -> v_des_z = kp * (-1) = -1 (NED: negative = climb).
    v_des, _ = guidance.compute_leg(
        position, velocity, _A_NORTH, np.array([10.0, 0.0, -11.0])
    )
    assert v_des[2] == pytest.approx(-1.0)


def test_degenerate_leg_steers_at_b() -> None:
    # Zero-length leg (A == B): no track direction; steer at wp_b and
    # hold its altitude.
    guidance = NLGLGuidance()
    position = np.array([3.0, 4.0, -10.0])
    velocity = np.zeros(3)
    b = np.array([0.0, 0.0, -15.0])
    v_des, diag = guidance.compute_leg(position, velocity, b, b)
    assert diag.e_ct == 0.0
    assert diag.chi_d == pytest.approx(math.atan2(-4.0, -3.0))
    assert v_des[0] == pytest.approx(5.0 * (-3.0 / 5.0))
    assert v_des[1] == pytest.approx(5.0 * (-4.0 / 5.0))
    assert v_des[2] == -NLGLConfig().climb_rate_max  # climb to z_des -15 (NED)


def test_compute_mission_first_leg_and_hold() -> None:
    # Mission: start at wp0, fly to wp1. First update() advances past wp0
    # (parked on it); compute_mission then flies the wp0 -> wp1 leg.
    mission = WaypointMission(
        [
            Waypoint(np.array([0.0, 0.0, -10.0])),
            Waypoint(np.array([20.0, 0.0, -10.0])),
        ]
    )
    guidance = NLGLGuidance()
    position = np.array([0.0, 0.0, -10.0])
    velocity = np.zeros(3)

    assert mission.update(position) is False
    assert mission.active_index == 1

    v_des, diag = guidance.compute_mission(position, velocity, mission)
    assert diag.e_ct == pytest.approx(0.0, abs=1e-9)
    assert diag.chi_d == pytest.approx(0.0, abs=1e-9)  # due north, at A
    assert v_des[0] > 0.0
    assert v_des[1] == pytest.approx(0.0, abs=1e-9)

    # Fly to within radius of wp1: mission completes; guidance still
    # aims at the (clamped) active waypoint so the vehicle can hold it.
    position = np.array([19.0, 0.5, -10.0])
    assert mission.update(position) is True
    assert mission.complete
    assert mission.active_index == 1
    _, diag = guidance.compute_mission(position, velocity, mission)
    assert not diag.behind_a_recovery
    assert abs(diag.e_ct) < 1.0
