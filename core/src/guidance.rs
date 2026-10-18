//! NLGL guidance: Park, Deyst and How (2004, 2007) in the ArduPilot
//! `AP_L1_Control` scalar form, specialized to quadrotor velocity commands.
//!
//! Ported term for term from the previous Python source (`guidance.py`), whose
//! module docstring carries the full derivation and the documented
//! implementation choices. Summary of the law:
//!
//! ```text
//! u_ab  = (wp_b - wp_a)_NE / ||.||                  track unit vector
//! r_ne  = (p - wp_a)_NE
//! e_ct  = cross2(r_ne, u_ab)          (+ = left of the path)
//! s     = dot(r_ne, u_ab) / leg_length              along-track fraction
//! L1    = max(l1_min, zeta * period_t * ground_speed / pi)
//! Nu1   = asin(clip(e_ct / L1, -sin(nu1_clip), +sin(nu1_clip)))
//! Nu2   = atan2(cross2(v_g, u_ab), dot(v_g, u_ab))
//! Nu    = clip(Nu1 + Nu2, -nu_clip, +nu_clip)
//! chi_q = atan2(u_ab.e, u_ab.n)
//! chi_d = wrap(chi_q + Nu)
//! v_des = cruise_speed * [cos(chi_d), sin(chi_d), 0] + altitude channel
//! ```
//!
//! Design decisions preserved deliberately, all from the Python source:
//!
//! 1. `Nu2` uses the EKF NED *ground* velocity. Heading plus airspeed would
//!    hide the crab angle, letting the vehicle drift downwind of the track.
//! 2. `Nu` is clipped to `+/- nu_clip` (90 deg), the ArduPilot form. The
//!    unclipped sum has a wrong-way equilibrium once the course error passes
//!    90 deg.
//! 3. `Nu2` is forced to zero below `min_ground_speed_for_nu2`. A quadrotor at
//!    rest has no course, and the residual velocity is estimator noise.
//! 4. `a_lat` uses the ArduPilot `K_L1 = 4 * zeta^2` form so the diagnostic
//!    matches a tuned ArduPilot vehicle for the same zeta.
//! 5. The first leg originates at the recorded mission start, not `wp[0]`.
//! 6. Behind-A recovery: when `s < -0.7071` and `||p - A||_NE > L1`, steer
//!    straight at A and ignore the path, which prevents a fly-away.
//! 7. The altitude channel is a separate P/PD on the active waypoint's NED z.
//!    NED z is *down*, so a climb is a negative `v_des_z`: `z_des` above the
//!    vehicle (`z_des < z`) yields a negative, that is climbing, command.
//!
//! Guidance is a pure function of its inputs plus config: no internal state and
//! no waypoint advancement, which [`crate::mission::WaypointMission`] owns.

use crate::math;
use crate::mission::WaypointMission;
use crate::types::{Arr3, Waypoint};

/// Horizontal legs shorter than this (m) have no track direction, so the law
/// steers straight at the leg end.
const LEG_LENGTH_EPS: f64 = 1.0e-6;

/// Behind-A recovery trigger, the same corner ArduPilot uses.
const BEHIND_S_THRESHOLD: f64 = -0.7071;

/// NLGL tuning parameters.
///
/// `zeta / period_t` set the look-ahead `L1 = zeta * period_t * V / pi`, floored
/// at `l1_min`. `period_t = 5.0 s` is deliberately much shorter than the
/// ArduPilot fixed-wing default of 17 s: a quadrotor with a cascaded PD velocity
/// inner loop is far more agile than a fixed wing rolling into bank.
#[derive(Debug, Clone, Copy)]
pub struct NlglConfig {
    pub zeta: f64,
    pub period_t: f64,
    pub l1_min: f64,
    pub nu1_clip_deg: f64,
    pub nu_clip_deg: f64,
    pub cruise_speed: f64,
    pub kp_alt: f64,
    pub kd_alt: f64,
    pub climb_rate_max: f64,
    pub min_ground_speed_for_nu2: f64,
}

impl Default for NlglConfig {
    fn default() -> Self {
        Self {
            zeta: 0.75,
            period_t: 5.0,
            l1_min: 3.0,
            nu1_clip_deg: 45.0,
            nu_clip_deg: 90.0,
            cruise_speed: 5.0,
            kp_alt: 1.0,
            kd_alt: 0.5,
            climb_rate_max: 2.0,
            min_ground_speed_for_nu2: 0.3,
        }
    }
}

/// Per-call NLGL diagnostics, SI units and radians.
#[derive(Debug, Clone, Copy, Default)]
pub struct GuidanceDiagnostics {
    /// Signed cross-track error (m, positive is left of the path).
    pub e_ct: f64,
    /// Cross-track angle (rad), clipped at `nu1_clip`.
    pub nu1: f64,
    /// Ground-velocity course angle relative to the track (rad).
    pub nu2: f64,
    /// `nu1 + nu2`, clipped at `nu_clip` (rad).
    pub nu: f64,
    /// Look-ahead distance used (m).
    pub l1: f64,
    /// Commanded course (rad, wrapped to `[-pi, pi)`).
    pub chi_d: f64,
    /// Whether behind-A recovery steering is active this call.
    pub behind_a_recovery: bool,
    /// Diagnostic lateral acceleration, `K_L1` form (m/s^2).
    pub a_lat: f64,
}

/// Stateless NLGL line-of-sight guidance for one leg or a whole mission.
#[derive(Debug, Clone, Copy, Default)]
pub struct NlglGuidance {
    config: NlglConfig,
}

impl NlglGuidance {
    /// Build with the given tuning.
    pub fn new(config: NlglConfig) -> Self {
        Self { config }
    }

    /// The active configuration.
    pub fn config(&self) -> &NlglConfig {
        &self.config
    }

    /// NLGL velocity command for the leg `wp_a -> wp_b`.
    ///
    /// `position_ned` is the current NED position (m) and `velocity_ned` the EKF
    /// NED *ground* velocity (m/s). Returns the desired NED velocity together
    /// with the per-call diagnostics. Pure function of inputs plus config.
    pub fn compute_leg(
        &self,
        position_ned: Arr3,
        velocity_ned: Arr3,
        wp_a: Arr3,
        wp_b: Arr3,
    ) -> (Arr3, GuidanceDiagnostics) {
        let cfg = &self.config;
        let p = position_ned;
        let v = velocity_ned;
        let a = wp_a;
        let b = wp_b;

        let nu1_clip = math::to_radians(cfg.nu1_clip_deg);
        let nu_clip = math::to_radians(cfg.nu_clip_deg);

        let ab_ne = [b[0] - a[0], b[1] - a[1]];
        let leg_length = math::sqrt(ab_ne[0] * ab_ne[0] + ab_ne[1] * ab_ne[1]);
        let r_ne = [p[0] - a[0], p[1] - a[1]];
        let v_g = [v[0], v[1]];
        let ground_speed = math::sqrt(v_g[0] * v_g[0] + v_g[1] * v_g[1]);

        // Look-ahead distance from the true ground speed, floored at l1_min.
        let l1 = f64::max(cfg.l1_min, cfg.zeta * cfg.period_t * ground_speed / math::PI);

        let (e_ct, nu1, nu2, nu, mut chi_d, behind_a_recovery);
        if leg_length < LEG_LENGTH_EPS {
            // Degenerate leg: no track direction, steer straight at wp_b.
            e_ct = 0.0;
            nu1 = 0.0;
            nu2 = 0.0;
            nu = 0.0;
            let to_b = [b[0] - p[0], b[1] - p[1]];
            chi_d = if math::sqrt(to_b[0] * to_b[0] + to_b[1] * to_b[1]) < 1e-12 {
                0.0
            } else {
                math::atan2(to_b[1], to_b[0])
            };
            behind_a_recovery = false;
        } else {
            let u_ab = [ab_ne[0] / leg_length, ab_ne[1] / leg_length];
            e_ct = math::cross2(r_ne, u_ab);
            let s = (r_ne[0] * u_ab[0] + r_ne[1] * u_ab[1]) / leg_length;

            // Nu1: cross-track angle, clipped so a large offset cannot demand
            // more than nu1_clip of capture angle.
            let sin_clip = math::sin(nu1_clip);
            nu1 = math::asin(f64::min(
                sin_clip,
                f64::max(-sin_clip, e_ct / l1),
            ));

            // Nu2 from the EKF ground velocity. Below the speed floor the
            // course is undefined, so Nu2 = 0 and only Nu1 steers.
            if ground_speed >= cfg.min_ground_speed_for_nu2 {
                nu2 = math::atan2(math::cross2(v_g, u_ab), v_g[0] * u_ab[0] + v_g[1] * u_ab[1]);
            } else {
                nu2 = 0.0;
            }
            nu = f64::min(nu_clip, f64::max(-nu_clip, nu1 + nu2));

            let chi_q = math::atan2(u_ab[1], u_ab[0]);

            // Behind-A recovery: well behind the leg start and far from A, so
            // steer directly at A and ignore path steering.
            behind_a_recovery = s < BEHIND_S_THRESHOLD
                && math::sqrt(r_ne[0] * r_ne[0] + r_ne[1] * r_ne[1]) > l1;
            if behind_a_recovery {
                let to_a = [a[0] - p[0], a[1] - p[1]];
                chi_d = if math::sqrt(to_a[0] * to_a[0] + to_a[1] * to_a[1]) < 1e-12 {
                    chi_q
                } else {
                    math::atan2(to_a[1], to_a[0])
                };
            } else {
                chi_d = math::wrap_to_pi(chi_q + nu);
            }
        }

        // Desired ground velocity in the north-east plane.
        let mut v_des = [
            cfg.cruise_speed * math::cos(chi_d),
            cfg.cruise_speed * math::sin(chi_d),
            0.0,
        ];

        // Altitude channel: P/PD on the leg end waypoint's NED z, with a zero
        // vertical velocity setpoint, clamped to climb_rate_max.
        let z_des = b[2];
        let v_des_z = cfg.kp_alt * (z_des - p[2]) + cfg.kd_alt * (0.0 - v[2]);
        let v_des_z = f64::min(cfg.climb_rate_max, f64::max(-cfg.climb_rate_max, v_des_z));
        v_des[2] = v_des_z;

        // Diagnostic lateral acceleration, ArduPilot K_L1 form.
        let k_l1 = 4.0 * cfg.zeta * cfg.zeta;
        let a_lat = k_l1 * ground_speed * ground_speed / l1 * math::sin(nu);

        let diagnostics = GuidanceDiagnostics {
            e_ct,
            nu1,
            nu2,
            nu,
            l1,
            chi_d,
            behind_a_recovery,
            a_lat,
        };
        (v_des, diagnostics)
    }

    /// NLGL velocity command toward the mission's active waypoint.
    ///
    /// The active leg is `mission.previous_point -> active waypoint`.
    pub fn compute_mission(
        &self,
        position_ned: Arr3,
        velocity_ned: Arr3,
        mission: &WaypointMission,
    ) -> (Arr3, GuidanceDiagnostics) {
        let active: &Waypoint = mission.active_waypoint();
        let leg_a = mission.previous_point(position_ned);
        self.compute_leg(position_ned, velocity_ned, leg_a, active.position)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // A 50 m northward leg at 10 m altitude (NED z = -10).
    const A: Arr3 = [0.0, 0.0, -10.0];
    const B: Arr3 = [50.0, 0.0, -10.0];

    fn g() -> NlglGuidance {
        NlglGuidance::new(NlglConfig::default())
    }

    #[test]
    fn on_track_commands_cruise_straight_ahead() {
        let (v, d) = g().compute_leg(A, [0.0; 3], A, B);
        assert!(d.e_ct.abs() < 1e-12, "e_ct {}", d.e_ct);
        assert!(d.nu1.abs() < 1e-12);
        assert!(d.nu2.abs() < 1e-12);
        assert!(d.chi_d.abs() < 1e-12, "should hold north, got {}", d.chi_d);
        assert!((v[0] - 5.0).abs() < 1e-9, "{v:?}");
        assert!(v[1].abs() < 1e-12, "{v:?}");
        assert!(v[2].abs() < 1e-12, "altitude held: {v:?}");
    }

    #[test]
    fn left_of_path_is_positive_and_steers_right() {
        // 5 m west of a northward track is left of it (facing north).
        let (_, d) = g().compute_leg([25.0, -5.0, -10.0], [0.0; 3], A, B);
        assert!((d.e_ct - 5.0).abs() < 1e-12, "e_ct {}", d.e_ct);
        assert!(d.nu1 > 0.0, "nu1 {}", d.nu1);
        assert!(d.chi_d > 0.0, "should aim east of north, got {}", d.chi_d);
    }

    #[test]
    fn right_of_path_is_negative_and_steers_left() {
        let (_, d) = g().compute_leg([25.0, 5.0, -10.0], [0.0; 3], A, B);
        assert!((d.e_ct + 5.0).abs() < 1e-12, "e_ct {}", d.e_ct);
        assert!(d.nu1 < 0.0);
        assert!(d.chi_d < 0.0, "should aim west of north, got {}", d.chi_d);
    }

    #[test]
    fn nu1_clips_at_the_capture_angle() {
        // A 200 m cross-track error at L1 = 3 m would demand asin(66.7), so the
        // clip binds and nu1 is exactly nu1_clip.
        let (_, d) = g().compute_leg([25.0, -200.0, -10.0], [0.0; 3], A, B);
        let expected = crate::math::to_radians(45.0);
        assert!((d.nu1 - expected).abs() < 1e-12, "nu1 {}", d.nu1);
    }

    #[test]
    fn nu_clips_at_ninety_degrees() {
        // Ground velocity pointing due south while the track runs north gives
        // nu2 near 180 deg, so the nu_clip at 90 deg must bind. The cross
        // product is a signed zero here, so only the magnitude is meaningful:
        // either hard-turn direction is equally valid.
        let (_, d) = g().compute_leg([25.0, 0.0, -10.0], [-6.0, 0.0, 0.0], A, B);
        let limit = crate::math::to_radians(90.0);
        assert!((d.nu.abs() - limit).abs() < 1e-12, "nu {}", d.nu);
        assert!(d.nu2.abs() > 3.0, "nu2 should be near pi, got {}", d.nu2);
    }

    #[test]
    fn nu2_compensates_a_crab() {
        // Track runs north; the ground velocity is crabbed to the west, so the
        // law must aim east of the track to cancel the drift.
        let (_, d) = g().compute_leg([25.0, 0.0, -10.0], [6.0, -1.2, 0.0], A, B);
        assert!(d.nu2 > 0.0, "expected an eastward correction, got {}", d.nu2);
        assert!(d.chi_d > 0.0);
    }

    #[test]
    fn nu2_vanishes_below_the_speed_floor() {
        // Residual estimator noise at rest must not create a course command.
        let (_, d) = g().compute_leg([25.0, 0.0, -10.0], [0.2, -0.1, 0.0], A, B);
        assert_eq!(d.nu2, 0.0);
    }

    #[test]
    fn altitude_channel_commands_a_climb_negatively() {
        // Waypoint 15 m above (NED z = -25): a climb is a negative v_des_z.
        let b = [50.0, 0.0, -25.0];
        let (v, _) = g().compute_leg([0.0, 0.0, -10.0], [0.0; 3], A, b);
        assert!(v[2] < 0.0, "climb must be negative in NED, got {}", v[2]);
        assert!((v[2] + 2.0).abs() < 1e-9, "should saturate the climb rate, got {}", v[2]);
    }

    #[test]
    fn altitude_channel_commands_a_descent_positively() {
        // Waypoint 15 m below (NED z = +5).
        let b = [50.0, 0.0, 5.0];
        let (v, _) = g().compute_leg([0.0, 0.0, -10.0], [0.0; 3], A, b);
        assert!((v[2] - 2.0).abs() < 1e-9, "got {}", v[2]);
    }

    #[test]
    fn altitude_channel_respects_the_climb_rate_limit() {
        let b = [50.0, 0.0, -1000.0];
        let (v, _) = g().compute_leg([0.0, 0.0, -10.0], [0.0; 3], A, b);
        assert!((v[2] + 2.0).abs() < 1e-9, "clamped, got {}", v[2]);
    }

    #[test]
    fn degenerate_leg_steers_straight_at_the_end() {
        // A and B coincide horizontally, so there is no track direction.
        let (_, d) = g().compute_leg([0.0, 0.0, -10.0], [0.0; 3], A, [0.0, 0.0, -10.0]);
        assert_eq!(d.e_ct, 0.0);
        assert_eq!(d.nu, 0.0);
        assert!(!d.behind_a_recovery);
        assert!(d.chi_d.abs() < 1e-12, "no offset, so no course command");
    }

    #[test]
    fn degenerate_leg_points_at_a_target_offset_to_the_east() {
        let (_, d) = g().compute_leg([0.0, 0.0, -10.0], [0.0; 3], A, [0.0, 30.0, -10.0]);
        let expected = crate::math::to_radians(90.0);
        assert!((d.chi_d - expected).abs() < 1e-12, "{}", d.chi_d);
    }

    #[test]
    fn behind_a_recovery_steers_back_to_the_leg_origin() {
        // 40 m behind A and 10 m east of the extended track: s is well below
        // -0.7071 and ||p - A||_NE exceeds L1, so recovery triggers and the
        // course points straight at A rather than at the clipped track angle.
        let position = [-40.0, 10.0, -10.0];
        let (_, d) = g().compute_leg(position, [0.0; 3], A, B);
        assert!(d.behind_a_recovery, "recovery should trigger");
        let to_a = [A[0] - position[0], A[1] - position[1]];
        let expected = crate::math::atan2(to_a[1], to_a[0]);
        assert!(
            (d.chi_d - expected).abs() < 1e-12,
            "chi_d {} should point at A ({expected})",
            d.chi_d
        );
        // Without recovery the clipped track angle would be nu1 alone.
        assert!(d.chi_d > -0.5, "recovery should not clip to -45 deg: {}", d.chi_d);
    }

    #[test]
    fn a_lat_is_zero_on_track() {
        let (_, d) = g().compute_leg([25.0, 0.0, -10.0], [0.0; 3], A, B);
        assert!(d.a_lat.abs() < 1e-12);
    }

    #[test]
    fn compute_mission_follows_the_active_leg() {
        use crate::mission::WaypointMission;
        let mission = WaypointMission::new(vec![
            Waypoint::new([0.0, 0.0, -10.0], 5.0, 0.0, 2.0),
            Waypoint::new([0.0, 50.0, -10.0], 5.0, 0.0, 2.0),
        ])
        .unwrap();
        // Sitting on the first waypoint, the active leg is A -> wp0, which is
        // degenerate; guidance falls through to steering at wp0, and the
        // sensible assertion is that it does not command travel toward wp1 yet.
        let (v, _) = g().compute_mission([0.0, 0.0, -10.0], [0.0; 3], &mission);
        assert!(v[1].abs() < 1e-9, "should not head east yet: {v:?}");
    }

    #[test]
    fn compute_mission_steers_at_the_next_waypoint_once_started() {
        use crate::mission::WaypointMission;
        let mut mission = WaypointMission::new(vec![
            Waypoint::new([20.0, 0.0, -10.0], 5.0, 0.0, 2.0),
            Waypoint::new([20.0, 20.0, -10.0], 5.0, 0.0, 2.0),
        ])
        .unwrap();
        mission.set_start([0.0, 0.0, -10.0]);
        // 19 m north of the start, so the first waypoint is 1 m ahead: within
        // radius, the mission advances to waypoint 1 which is due east.
        mission.update([19.0, 0.0, -10.0]);
        let (v, _) = g().compute_mission([19.0, 0.0, -10.0], [0.0; 3], &mission);
        assert!(v[1] > 0.0, "should now head east: {v:?}");
    }
}
