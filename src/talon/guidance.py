"""NLGL guidance law (Phase 4): Park, Deyst & How 2004/2007 in the
ArduPilot AP_L1_Control scalar form, specialized to quadrotor velocity
commands.

Formulation (see cpp-style documentation below; signs follow the
specification's cross2 convention and docs/conventions.md NED frames):

    u_ab   = (wp_b - wp_a)_NE / ||.|                       (track unit vector)
    r_ne   = (p - wp_a)_NE
    e_ct   = cross2(r_ne, u_ab) = r_ne.n * u_ab.e - r_ne.e * u_ab.n
                                                     (+ = left of path)
    s      = dot(r_ne, u_ab) / leg_length                (along-track fraction)
    L1     = max(l1_min, zeta * period_t * ground_speed / pi)
    Nu1    = asin(clip(e_ct / L1, -sin(nu1_clip), +sin(nu1_clip)))
    Nu2    = atan2(cross2(v_g, u_ab), dot(v_g, u_ab))
    Nu     = clip(Nu1 + Nu2, -nu_clip, +nu_clip)
    chi_q  = atan2(u_ab.e, u_ab.n)                       (track course, NE)
    chi_d  = wrap(chi_q + Nu)                            (commanded course)
    v_des  = cruise_speed * [cos(chi_d), sin(chi_d), 0] + altitude channel

Documented implementation choices (per the Phase 4 brief, "YOUR CALL"):

1. GROUND VELOCITY in Nu2 (MANDATORY): v_g is the EKF NED ground velocity,
   NE plane only. Using heading + airspeed would reintroduce a cross-track
   bias under wind: the crab angle would be invisible to Nu2, so the loop
   could not hold the crab and the vehicle would drift downwind of the
   track. With ground velocity, a steady wind simply appears as a crab in
   v_g that Nu2 cancels exactly (see tests/test_guidance.py crab test).

2. chi_d = chi_q + Nu with Nu CLIPPED to +/- nu_clip (90 deg), the
   ArduPilot form. The brief also offered the unclipped Nu1 + Nu2 sum;
   closed-loop experience in scenario_waypoints.py shows that form has a
   wrong-way equilibrium: at every waypoint corner the vehicle overflies
   with the old course ~90 deg off the new track, and once the course
   error passes 90 deg (easy during a decelerating turn, and guaranteed
   at low ground speed) the unclipped sum steers AWAY from the track and
   the vehicle hooks 10+ m off-leg before recovering. The +/- 90 deg
   clip is what makes L1 globally convergent in ArduPilot: the commanded
   turn direction never reverses until |Nu| < 90 deg.

3. Nu2 is forced to 0 when the ground speed is below
   min_ground_speed_for_nu2 (0.3 m/s). The brief specifies flooring the
   speed magnitude; that preserves the DIRECTION of a near-zero
   velocity, which at rest is estimator/GPS noise and produces a
   random-looking Nu2 up to +/- 180 deg (observed at scenario start and
   in corner decelerations: a 0.07 m/s noise direction commanded the
   vehicle backward along the track). A quadrotor at rest has no
   course, so the course error is undefined and only Nu1 should steer.

4. a_lat diagnostic uses the ArduPilot K_L1 form:
       a_lat = K_L1 * V^2 / L1 * sin(Nu),  K_L1 = 4 * zeta^2
   (not the (2/L1) V^2 sin(Nu) form), so the number matches what an
   ArduPilot-tuned vehicle would measure for the same zeta.

5. First mission leg originates at the recorded mission START position
   (WaypointMission.set_start / first update), not at wp[0]. Starting a
   mission parked on wp[0] therefore produces a degenerate zero-length
   first leg, which the guard below resolves by steering straight at
   wp_b, and the mission manager skips it immediately (distance < radius).

6. Behind-A recovery (ArduPilot): if s < -0.7071 AND ||p - A||_NE > L1,
   steer directly at A and ignore the path Nu. Prevents fly-away when the
   mission starts off-leg or after a large disturbance.

7. Altitude channel is a separate P/PD on the active waypoint's NED z:
       v_des_z = clamp(kp_alt * (z_des - z) + kd_alt * (0 - v_z),
                        -climb_rate_max, +climb_rate_max)
   NED sign care (docs/conventions.md): z is DOWN, so climbing means
   NEGATIVE v_des_z; z_des above the vehicle (z_des < z) yields a negative
   (climbing) command. (The Phase 4 brief's test sketch says "v_des_z > 0"
   for a climb, which is an ENU reading; NED is the single source of
   truth per docs/conventions.md, so the code and tests are NED-correct:
   climb => v_des_z < 0, magnitude <= climb_rate_max.)

Guidance is a pure function of its inputs plus config: no internal state,
no waypoint advancement (WaypointMission owns that).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from talon.mission import WaypointMission

# Legs shorter than this (m, horizontal) are treated as degenerate: no
# track direction exists, so the law steers straight at wp_b.
_LEG_LENGTH_EPS = 1.0e-6

# Behind-A recovery trigger: |s| corner (ArduPilot uses the same 0.7071).
_BEHIND_S_THRESHOLD = -0.7071


def cross2(a: ArrayLike, b: ArrayLike) -> float:
    """2D cross product (z component): a.x * b.y - a.y * b.x (scalar).

    Sign convention used throughout guidance: positive when b is 90 deg
    counter-clockwise from a in the (North, East) plane.
    """
    aa = np.asarray(a, dtype=np.float64)
    bb = np.asarray(b, dtype=np.float64)
    if aa.shape != (2,) or bb.shape != (2,):
        raise ValueError("cross2 operands must have shape (2,)")
    return float(aa[0] * bb[1] - aa[1] * bb[0])


def wrap_angle(theta: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (theta + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(slots=True)
class NLGLConfig:
    """NLGL tuning parameters.

    zeta / period_t set the look-ahead distance L1 = zeta * period_t * V
    / pi (floor l1_min). period_t = 5.0 s is deliberately much shorter
    than ArduPilot's 17 s fixed-wing default: a quadrotor with a cascaded
    PD velocity inner loop is far more agile than a fixed wing rolling
    into bank.
    """

    zeta: float = 0.75
    period_t: float = 5.0
    l1_min: float = 3.0
    nu1_clip_deg: float = 45.0
    nu_clip_deg: float = 90.0
    cruise_speed: float = 5.0
    kp_alt: float = 1.0
    kd_alt: float = 0.5
    climb_rate_max: float = 2.0
    min_ground_speed_for_nu2: float = 0.3


@dataclass(slots=True)
class GuidanceDiagnostics:
    """Per-call NLGL diagnostics (SI units; angles in rad)."""

    e_ct: float  # signed cross-track error (m, + = left of path)
    nu1: float  # cross-track angle (rad, clipped at nu1_clip)
    nu2: float  # course (ground-velocity) angle relative to track (rad)
    nu: float  # nu1 + nu2, clipped at nu_clip (rad)
    l1: float  # look-ahead distance used (m)
    chi_d: float  # commanded course (rad, wrapped to [-pi, pi])
    behind_a_recovery: bool  # behind-A steering active this call
    a_lat: float  # diagnostic lateral accel, K_L1 form (m/s^2)


class NLGLGuidance:
    """Stateless NLGL line-of-sight guidance for one leg or a whole mission."""

    def __init__(self, config: NLGLConfig | None = None) -> None:
        self._config = config if config is not None else NLGLConfig()

    @property
    def config(self) -> NLGLConfig:
        """The active configuration."""
        return self._config

    def compute_leg(
        self,
        position_ned: ArrayLike,
        velocity_ned: ArrayLike,
        wp_a: ArrayLike,
        wp_b: ArrayLike,
    ) -> tuple[NDArray[np.float64], GuidanceDiagnostics]:
        """NLGL velocity command for the leg wp_a -> wp_b.

        position_ned / velocity_ned: current NED position (m) and EKF NED
        GROUND velocity (m/s); wp_a / wp_b: leg start / end NED (m).
        Returns (v_des NED (3,), GuidanceDiagnostics). Pure function of
        inputs plus config.
        """
        cfg = self._config
        p = np.asarray(position_ned, dtype=np.float64)
        v = np.asarray(velocity_ned, dtype=np.float64)
        a = np.asarray(wp_a, dtype=np.float64)
        b = np.asarray(wp_b, dtype=np.float64)
        for name, arr in (
            ("position_ned", p),
            ("velocity_ned", v),
            ("wp_a", a),
            ("wp_b", b),
        ):
            if arr.shape != (3,):
                raise ValueError(f"{name} must have shape (3,), got {arr.shape}")

        nu1_clip = math.radians(cfg.nu1_clip_deg)
        nu_clip = math.radians(cfg.nu_clip_deg)

        ab_ne = b[:2] - a[:2]
        leg_length = float(np.linalg.norm(ab_ne))
        r_ne = p[:2] - a[:2]
        v_g = v[:2]
        ground_speed = float(np.linalg.norm(v_g))

        # Look-ahead distance from the true ground speed (Park: L1 = zeta
        # * T * V / pi), floored at l1_min for low-speed numerics.
        l1 = max(cfg.l1_min, cfg.zeta * cfg.period_t * ground_speed / math.pi)

        if leg_length < _LEG_LENGTH_EPS:
            # Degenerate leg (zero horizontal length): no track direction,
            # steer straight at wp_b instead.
            e_ct = 0.0
            nu1 = 0.0
            nu2 = 0.0
            nu = 0.0
            to_b = b[:2] - p[:2]
            if float(np.linalg.norm(to_b)) < 1e-12:
                chi_d = 0.0
            else:
                chi_d = math.atan2(to_b[1], to_b[0])
            behind_a_recovery = False
        else:
            u_ab = ab_ne / leg_length
            e_ct = cross2(r_ne, u_ab)
            s = float(np.dot(r_ne, u_ab)) / leg_length

            # Nu1: cross-track angle, clipped so a large offset cannot
            # demand more than nu1_clip of capture angle.
            sin_clip = math.sin(nu1_clip)
            nu1 = math.asin(
                min(sin_clip, max(-sin_clip, e_ct / l1))
            )

            # Nu2 from the EKF GROUND velocity (NE plane). Below the speed
            # floor the course is undefined (a quadrotor at rest has no
            # heading; the residual velocity is estimator noise), so
            # Nu2 = 0 and only Nu1 steers (module docstring, choice 3).
            if ground_speed >= cfg.min_ground_speed_for_nu2:
                nu2 = math.atan2(cross2(v_g, u_ab), float(np.dot(v_g, u_ab)))
            else:
                nu2 = 0.0
            nu = min(nu_clip, max(-nu_clip, nu1 + nu2))

            chi_q = math.atan2(u_ab[1], u_ab[0])

            # Behind-A recovery (ArduPilot): well behind the leg start and
            # far from A -> steer directly at A, ignore path steering.
            behind_a_recovery = (
                s < _BEHIND_S_THRESHOLD and float(np.linalg.norm(r_ne)) > l1
            )
            if behind_a_recovery:
                to_a = a[:2] - p[:2]
                if float(np.linalg.norm(to_a)) < 1e-12:
                    chi_d = chi_q
                else:
                    chi_d = math.atan2(to_a[1], to_a[0])
            else:
                # chi_d = chi_q + Nu with Nu CLIPPED (module docstring,
                # choice 2: the ArduPilot form; the unclipped Nu1 + Nu2
                # sum has a wrong-way equilibrium once |course error|
                # exceeds nu_clip).
                chi_d = wrap_angle(chi_q + nu)

        # Desired ground velocity, NE plane; altitude handled separately.
        v_des = np.array(
            [
                cfg.cruise_speed * math.cos(chi_d),
                cfg.cruise_speed * math.sin(chi_d),
                0.0,
            ],
            dtype=np.float64,
        )

        # Altitude channel (module docstring, choice 6): P/PD on the leg
        # end waypoint's NED z, vz_des = 0, clamped to climb_rate_max.
        z_des = b[2]
        v_des_z = cfg.kp_alt * (z_des - p[2]) + cfg.kd_alt * (0.0 - v[2])
        v_des_z = min(cfg.climb_rate_max, max(-cfg.climb_rate_max, v_des_z))
        v_des[2] = v_des_z

        # Diagnostic lateral acceleration, ArduPilot K_L1 form.
        k_l1 = 4.0 * cfg.zeta * cfg.zeta
        a_lat = k_l1 * ground_speed * ground_speed / l1 * math.sin(nu)

        diagnostics = GuidanceDiagnostics(
            e_ct=e_ct,
            nu1=nu1,
            nu2=nu2,
            nu=nu,
            l1=l1,
            chi_d=chi_d,
            behind_a_recovery=behind_a_recovery,
            a_lat=a_lat,
        )
        return v_des, diagnostics

    def compute_mission(
        self,
        position_ned: ArrayLike,
        velocity_ned: ArrayLike,
        mission: WaypointMission,
    ) -> tuple[NDArray[np.float64], GuidanceDiagnostics]:
        """NLGL velocity command toward the mission's active waypoint.

        The active leg is (mission.previous_point -> active waypoint):
        waypoint i-1 once past the first waypoint, else the recorded
        mission start position (or the current position as a last-resort
        fallback, see WaypointMission.previous_point).
        """
        active = mission.active_waypoint()
        leg_a = mission.previous_point(position_ned)
        return self.compute_leg(position_ned, velocity_ned, leg_a, active.position)


__all__ = [
    "GuidanceDiagnostics",
    "NLGLConfig",
    "NLGLGuidance",
    "cross2",
    "wrap_angle",
]
