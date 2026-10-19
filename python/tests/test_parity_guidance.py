"""Python vs ``talon._core`` parity for the NLGL guidance law (Phase 3a).

Every case runs the same inputs through ``talon.guidance`` (the pure-Python
module) and through ``talon._core`` (the Rust core behind the PyO3 boundary)
and compares the velocity command and every diagnostic. Hand-computed
expectations pin the corner the two implementations historically disagreed
on: ``chi_d`` wrapping below ``-pi`` (the ``wrap_to_pi`` regression fixed in
``core/src/math.rs``).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from parity_util import (
    assert_exact,
    assert_near,
    assert_vec_near,
    expect_divergence,
)
from talon import _core
from talon.guidance import NLGLConfig, NLGLGuidance
from talon.mission import WaypointMission
from talon.types import Waypoint

CONFIG_FIELDS = (
    "zeta",
    "period_t",
    "l1_min",
    "nu1_clip_deg",
    "nu_clip_deg",
    "cruise_speed",
    "kp_alt",
    "kd_alt",
    "climb_rate_max",
    "min_ground_speed_for_nu2",
)

DIAGNOSTIC_FIELDS = ("e_ct", "nu1", "nu2", "nu", "l1", "chi_d", "a_lat")

# A northbound leg at 10 m altitude (NED z = -10) unless a case says
# otherwise.
LEG_A = [0.0, 0.0, -10.0]
LEG_B = [100.0, 0.0, -10.0]

# (position, velocity, wp_a, wp_b) cases with stable pytest ids.
LEG_CASES = [
    pytest.param([0.0, 0.0, -10.0], [0.0, 0.0, 0.0], LEG_A, LEG_B, id="on_track"),
    pytest.param(
        [0.0, -5.0, -10.0],
        [5.0, 0.0, 0.0],
        LEG_A,
        [-10.0, -10.0, -10.0],
        id="chi_d_wraps_below_minus_pi",
    ),
    pytest.param(
        [-40.0, 10.0, -10.0], [0.0, 0.0, 0.0], LEG_A, [50.0, 0.0, -10.0], id="behind_a_recovery"
    ),
    pytest.param(
        [-70.0, 3.0, -10.0], [0.0, 0.0, 0.0], LEG_A, LEG_B, id="behind_a_s_above_threshold"
    ),
    pytest.param(
        [-70.72, 3.0, -10.0],
        [0.0, 0.0, 0.0],
        LEG_A,
        LEG_B,
        id="behind_a_s_below_threshold",
    ),
    pytest.param(
        [3.0, 4.0, -10.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, -15.0],
        [0.0, 0.0, -15.0],
        id="degenerate_leg",
    ),
    pytest.param(
        [25.0, -200.0, -10.0], [0.0, 0.0, 0.0], LEG_A, LEG_B, id="nu1_clip_saturates"
    ),
    pytest.param(
        [25.0, 0.0, -10.0], [-6.0, 0.0, 0.0], LEG_A, LEG_B, id="nu_clip_saturates"
    ),
    pytest.param(
        [25.0, 0.0, -10.0],
        [0.2, -0.1, 0.0],
        LEG_A,
        LEG_B,
        id="nu2_below_speed_floor",
    ),
    pytest.param(
        [25.0, 0.0, -10.0], [6.0, -1.2, 0.0], LEG_A, LEG_B, id="nu2_compensates_crab"
    ),
    pytest.param(
        [0.0, 0.0, -10.0], [0.0, 0.0, 0.0], LEG_A, [10.0, 0.0, -20.0], id="altitude_climb"
    ),
    pytest.param(
        [0.0, 0.0, -10.0], [0.0, 0.0, 0.0], LEG_A, [10.0, 0.0, 5.0], id="altitude_descent"
    ),
]


def _waypoint(position: list[float], **kwargs: float) -> Waypoint:
    return Waypoint(np.asarray(position, dtype=np.float64), **kwargs)


@pytest.mark.parametrize("field", CONFIG_FIELDS)
def test_config_defaults_match_bit_for_bit(field: str) -> None:
    # The Rust defaults come from NlglConfig::default(); the Python ones are
    # the dataclass defaults. assert_exact compares float.hex(), so this is
    # a bit-level pin, not a tolerance match.
    py_config, rs_config = NLGLConfig(), _core.NLGLConfig()
    assert_exact(
        getattr(rs_config, field),
        getattr(py_config, field),
        label=f"NLGLConfig.{field}",
    )


def test_guidance_exposes_the_same_config_defaults() -> None:
    py_guidance, rs_guidance = NLGLGuidance(), _core.NLGLGuidance()
    for field in CONFIG_FIELDS:
        assert_exact(
            getattr(rs_guidance.config, field),
            getattr(py_guidance.config, field),
            label=f"NLGLGuidance.config.{field}",
        )


def test_config_setters_round_trip_and_reach_the_law() -> None:
    # Set every field to a distinct value on both sides, read it back at bit
    # level, then feed the config into the guidance constructor: the tuning
    # must actually reach the law (L1 scales with zeta, v_des with
    # cruise_speed).
    py_config, rs_config = NLGLConfig(), _core.NLGLConfig()
    for config in (py_config, rs_config):
        config.zeta = 1.5
        config.period_t = 4.0
        config.l1_min = 6.0
        config.nu1_clip_deg = 30.0
        config.nu_clip_deg = 60.0
        config.cruise_speed = 7.0
        config.kp_alt = 2.0
        config.kd_alt = 0.25
        config.climb_rate_max = 1.0
        config.min_ground_speed_for_nu2 = 0.5
    for field in CONFIG_FIELDS:
        assert_exact(
            getattr(rs_config, field), getattr(py_config, field), label=f"set {field}"
        )
    py_guidance, rs_guidance = NLGLGuidance(py_config), _core.NLGLGuidance(rs_config)
    position, velocity = [0.0, -5.0, -10.0], [5.0, 0.0, 0.0]
    py_v, py_d = py_guidance.compute_leg(position, velocity, LEG_A, LEG_B)
    rs_v, rs_d = rs_guidance.compute_leg(position, velocity, LEG_A, LEG_B)
    # L1 = zeta * period_t * V / pi, above the (raised) l1_min floor.
    assert_near(py_d.l1, 1.5 * 4.0 * 5.0 / math.pi, label="python l1")
    assert_near(rs_d.l1, py_d.l1, label="l1 parity")
    # 5 m cross-track at L1 ~ 9.55 m clips at the new 30 deg capture angle.
    assert_near(py_d.nu1, math.radians(30.0), label="python nu1 at the new clip")
    assert_near(rs_d.nu1, py_d.nu1, label="nu1 parity")
    # v_des magnitude is the new cruise speed.
    assert_near(math.hypot(py_v[0], py_v[1]), 7.0, label="python cruise speed")
    assert_vec_near(rs_v, py_v, label="v_des with tuned config")


def test_guidance_config_property_is_a_registered_divergence() -> None:
    reason = expect_divergence("guidance.config_is_a_snapshot")
    py_guidance, rs_guidance = NLGLGuidance(), _core.NLGLGuidance()
    py_guidance.config.zeta = 1.5  # live dataclass: retunes the next call
    rs_guidance.config.zeta = 1.5  # snapshot: the write lands on the copy
    position, velocity = [25.0, 0.0, -10.0], [5.0, 0.0, 0.0]
    py_l1 = py_guidance.compute_leg(position, velocity, LEG_A, LEG_B)[1].l1
    rs_l1 = rs_guidance.compute_leg(position, velocity, LEG_A, LEG_B)[1].l1
    assert_near(py_l1, 1.5 * 5.0 * 5.0 / math.pi, label="python uses the write")
    assert_near(
        rs_l1,
        0.75 * 5.0 * 5.0 / math.pi,
        label=f"rust ignores the write: {reason}",
    )


@pytest.mark.parametrize(("position", "velocity", "wp_a", "wp_b"), LEG_CASES)
def test_compute_leg_matches_on_every_diagnostic(
    position: list[float],
    velocity: list[float],
    wp_a: list[float],
    wp_b: list[float],
) -> None:
    py_v, py_d = NLGLGuidance().compute_leg(position, velocity, wp_a, wp_b)
    rs_v, rs_d = _core.NLGLGuidance().compute_leg(position, velocity, wp_a, wp_b)
    assert_vec_near(rs_v, py_v, label="v_des")
    for field in DIAGNOSTIC_FIELDS:
        assert_near(getattr(rs_d, field), getattr(py_d, field), label=field)
    assert rs_d.behind_a_recovery is py_d.behind_a_recovery


def test_chi_d_wraps_below_minus_pi_to_three_quarter_pi() -> None:
    # Track course -3pi/4 with the steering clip bound at -pi/2 sums to
    # -3.927 rad; the wrap must fold it into [-pi, pi) at +3pi/4, not hand
    # back the unfurled negative value. Both sides are bit-exact here.
    position, velocity = [0.0, -5.0, -10.0], [5.0, 0.0, 0.0]
    wp_a, wp_b = LEG_A, [-10.0, -10.0, -10.0]
    py_v, py_d = NLGLGuidance().compute_leg(position, velocity, wp_a, wp_b)
    rs_v, rs_d = _core.NLGLGuidance().compute_leg(position, velocity, wp_a, wp_b)
    expected = 3.0 * math.pi / 4.0
    assert_exact(py_d.chi_d, expected, label="python chi_d")
    assert_exact(rs_d.chi_d, expected, label="rust chi_d")
    # The commanded velocity steers east of south, magnitude cruise_speed.
    assert_near(py_v[0], -5.0 * math.sqrt(2.0) / 2.0, label="python v_des north")
    assert_near(py_v[1], 5.0 * math.sqrt(2.0) / 2.0, label="python v_des east")
    assert_vec_near(rs_v, py_v, label="v_des")


def test_on_track_leg_is_bit_exact() -> None:
    # Sitting on the leg start with zero velocity: every diagnostic is an
    # exact zero (or the l1_min floor) and v_des is cruise speed due north.
    position, velocity = [0.0, 0.0, -10.0], [0.0, 0.0, 0.0]
    py_v, py_d = NLGLGuidance().compute_leg(position, velocity, LEG_A, LEG_B)
    rs_v, rs_d = _core.NLGLGuidance().compute_leg(position, velocity, LEG_A, LEG_B)
    for field in DIAGNOSTIC_FIELDS:
        expected = 3.0 if field == "l1" else 0.0
        assert_exact(getattr(py_d, field), expected, label=f"python {field}")
        assert_exact(getattr(rs_d, field), expected, label=f"rust {field}")
    assert_exact(py_v, [5.0, 0.0, 0.0], label="python v_des")
    assert_exact(rs_v, py_v, label="rust v_des")


def test_behind_a_recovery_steers_straight_at_a() -> None:
    # 40 m behind A and 10 m east of a 50 m track: s = -40/50 = -0.8 is
    # well below the -0.7071 trigger and ||p - A|| exceeds L1, so recovery
    # fires and the course points at A rather than the clipped track angle.
    position = [-40.0, 10.0, -10.0]
    wp_b = [50.0, 0.0, -10.0]
    _, py_d = NLGLGuidance().compute_leg(position, [0.0, 0.0, 0.0], LEG_A, wp_b)
    _, rs_d = _core.NLGLGuidance().compute_leg(position, [0.0, 0.0, 0.0], LEG_A, wp_b)
    assert py_d.behind_a_recovery is True
    assert rs_d.behind_a_recovery is True
    expected = math.atan2(-10.0, 40.0)
    assert_exact(py_d.chi_d, expected, label="python chi_d points at A")
    assert_exact(rs_d.chi_d, expected, label="rust chi_d points at A")


@pytest.mark.parametrize(
    ("position", "recovers"),
    [
        pytest.param([-0.7071, -10.0, -10.0], False, id="exactly_at_threshold"),
        pytest.param([-0.71, -10.0, -10.0], True, id="strictly_below_threshold"),
        pytest.param([-0.707, -10.0, -10.0], False, id="strictly_above_threshold"),
    ],
)
def test_behind_a_recovery_is_strict_at_the_threshold(
    position: list[float], recovers: bool
) -> None:
    # A leg of horizontal length 1.0 makes the along-track fraction s
    # exactly the x offset (s = x / 1), so the first case sits precisely on
    # the -0.7071 trigger: the comparison is strict (s < threshold), so
    # being exactly on it must NOT recover.
    wp_a, wp_b = [0.0, 0.0, -10.0], [1.0, 0.0, -10.0]
    _, py_d = NLGLGuidance().compute_leg(position, [0.0, 0.0, 0.0], wp_a, wp_b)
    _, rs_d = _core.NLGLGuidance().compute_leg(position, [0.0, 0.0, 0.0], wp_a, wp_b)
    assert py_d.behind_a_recovery is recovers
    assert rs_d.behind_a_recovery is recovers
    assert_near(rs_d.chi_d, py_d.chi_d, label="chi_d parity")


def test_degenerate_leg_steers_straight_at_the_end() -> None:
    # A == B: no track direction exists, so the law steers at wp_b and
    # holds its altitude. e_ct and nu are exact zeros on both sides.
    position, wp = [3.0, 4.0, -10.0], [0.0, 0.0, -15.0]
    py_v, py_d = NLGLGuidance().compute_leg(position, [0.0, 0.0, 0.0], wp, wp)
    rs_v, rs_d = _core.NLGLGuidance().compute_leg(position, [0.0, 0.0, 0.0], wp, wp)
    for guide_d in (py_d, rs_d):
        assert_exact(guide_d.e_ct, 0.0, label="e_ct")
        assert_exact(guide_d.nu, 0.0, label="nu")
        assert guide_d.behind_a_recovery is False
    expected = math.atan2(-4.0, -3.0)
    assert_near(py_d.chi_d, expected, label="python chi_d at B")
    assert_near(rs_d.chi_d, expected, label="rust chi_d at B")
    # NED: climbing to z = -15 from z = -10 saturates the climb rate, so
    # v_des_z is exactly -climb_rate_max.
    assert_exact(py_v[2], -2.0, label="python v_des_z")
    assert_exact(rs_v[2], -2.0, label="rust v_des_z")
    assert_vec_near(rs_v, py_v, label="v_des")


@pytest.mark.parametrize(
    ("position", "saturates"),
    [
        pytest.param([25.0, -200.0, -10.0], True, id="nu1_clip_saturates"),
        pytest.param([25.0, -5.0, -10.0], False, id="nu1_unsaturated"),
    ],
)
def test_nu1_clip_saturation_parity(position: list[float], saturates: bool) -> None:
    _, py_d = NLGLGuidance().compute_leg(position, [0.0, 0.0, 0.0], LEG_A, LEG_B)
    _, rs_d = _core.NLGLGuidance().compute_leg(position, [0.0, 0.0, 0.0], LEG_A, LEG_B)
    assert_near(rs_d.nu1, py_d.nu1, label="nu1 parity")
    if saturates:
        # 200 m cross-track at L1 = 3 m would demand asin(66.7): the clip
        # binds and nu1 sits at the 45 deg capture angle on both sides.
        assert_near(py_d.nu1, math.radians(45.0), label="python nu1 at the clip")
        assert_near(rs_d.nu1, math.radians(45.0), label="rust nu1 at the clip")
    else:
        assert py_d.nu1 < math.radians(45.0)
        assert rs_d.nu1 < math.radians(45.0)


def test_nu_clip_saturation_parity() -> None:
    # Ground velocity due south on a northbound track: the course error is
    # near -pi, so the +/-90 deg steering clip binds and both sides must
    # report |nu| exactly at the limit.
    position, velocity = [25.0, 0.0, -10.0], [-6.0, 0.0, 0.0]
    _, py_d = NLGLGuidance().compute_leg(position, velocity, LEG_A, LEG_B)
    _, rs_d = _core.NLGLGuidance().compute_leg(position, velocity, LEG_A, LEG_B)
    limit = math.radians(90.0)
    assert_near(abs(py_d.nu), limit, label="python |nu| at the clip")
    assert_near(abs(rs_d.nu), limit, label="rust |nu| at the clip")
    assert_near(rs_d.nu, py_d.nu, label="nu parity")
    assert_near(py_d.nu2, -math.pi, label="python nu2 near -pi")
    assert_near(rs_d.nu2, py_d.nu2, label="nu2 parity")


@pytest.mark.parametrize(
    ("wp_b", "expected_z"),
    [
        pytest.param([10.0, 0.0, -20.0], -2.0, id="climb_saturates_negative"),
        pytest.param([10.0, 0.0, 5.0], 2.0, id="descent_saturates_positive"),
        pytest.param([10.0, 0.0, -11.0], -1.0, id="proportional_unclamped"),
    ],
)
def test_altitude_channel_sign_and_clamp(wp_b: list[float], expected_z: float) -> None:
    # NED: a climb is a NEGATIVE v_des_z, clamped at climb_rate_max. The
    # clamped and proportional results are exact IEEE arithmetic, so both
    # sides must be bit-exact against the hand-computed value.
    position = [0.0, 0.0, -10.0]
    py_v, _ = NLGLGuidance().compute_leg(position, [0.0, 0.0, 0.0], LEG_A, wp_b)
    rs_v, _ = _core.NLGLGuidance().compute_leg(position, [0.0, 0.0, 0.0], LEG_A, wp_b)
    assert_exact(py_v[2], expected_z, label="python v_des_z")
    assert_exact(rs_v[2], expected_z, label="rust v_des_z")
    assert_vec_near(rs_v, py_v, label="v_des")


def test_compute_mission_matches_compute_leg_on_the_active_leg() -> None:
    # Fly a three-waypoint mission to the second leg on both sides, then
    # check compute_mission equals compute_leg(previous_point -> active)
    # on the SAME side, and that both sides agree with each other.
    waypoints = [
        [0.0, 0.0, -10.0],
        [20.0, 0.0, -10.0],
        [20.0, 20.0, -10.0],
    ]
    py_mission = WaypointMission([_waypoint(w) for w in waypoints])
    rs_mission = _core.WaypointMission([_waypoint(w) for w in waypoints])
    for mission in (py_mission, rs_mission):
        mission.set_start([-5.0, -5.0, -10.0])
        mission.update([0.5, 0.0, -10.0])  # satisfy wp0, advance to wp1
    assert py_mission.active_index == rs_mission.active_index == 1

    position, velocity = [10.0, 1.0, -10.0], [5.0, 0.0, 0.0]
    py_v, py_d = NLGLGuidance().compute_mission(position, velocity, py_mission)
    rs_v, rs_d = _core.NLGLGuidance().compute_mission(position, velocity, rs_mission)
    # Same-side identity: the mission call is the leg call with the
    # mission's own leg endpoints.
    py_leg_v, py_leg_d = NLGLGuidance().compute_leg(
        position,
        velocity,
        py_mission.previous_point(position),
        py_mission.active_waypoint().position,
    )
    rs_leg_v, rs_leg_d = _core.NLGLGuidance().compute_leg(
        position,
        velocity,
        rs_mission.previous_point(position),
        list(rs_mission.active_waypoint()[0]),
    )
    assert_vec_near(py_v, py_leg_v, label="python mission == leg")
    assert_near(py_d.chi_d, py_leg_d.chi_d, label="python mission == leg chi_d")
    assert_vec_near(rs_v, rs_leg_v, label="rust mission == leg")
    assert_near(rs_d.chi_d, rs_leg_d.chi_d, label="rust mission == leg chi_d")
    # Cross-language parity of the mission call itself.
    assert_vec_near(rs_v, py_v, label="compute_mission v_des")
    for field in DIAGNOSTIC_FIELDS:
        assert_near(getattr(rs_d, field), getattr(py_d, field), label=field)
    assert rs_d.behind_a_recovery is py_d.behind_a_recovery
