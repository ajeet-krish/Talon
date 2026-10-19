//! Obstacle avoidance: spherical keep-out zones (schema 1.1) and the
//! guidance-side law that steers around them.
//!
//! Two halves share one geometry.
//!
//! * The geometry is a port of `python/talon/avoidance.py` in NED with no
//!   numpy: the nearest approach of a leg to a zone, the penetration test, the
//!   bypass point, the clearance of a position, the minimum clearance over a
//!   trajectory, and the offline planner that inserts detour waypoints. The
//!   Python module delegates here once the PyO3 binding exposes these, so the
//!   arithmetic has one implementation.
//! * [`ReactiveAvoidance`] is the in-the-loop law. It watches the leg the
//!   mission is flying and, when the corridor is threatened within a lookahead
//!   that grows with ground speed, latches a bypass point and hands that to
//!   guidance in place of the waypoint. The latch clears once the vehicle is
//!   near its bypass and the clearance has recovered, so the law can not
//!   chatter, and it never releases while the vehicle is inside the margin.
//!
//! Limits, stated rather than discovered. Decisions are made in the horizontal
//! plane, because the scenarios that use zones cruise at one altitude through
//! zones that span it; clearances are reported in full 3-D. One zone is
//! latched at a time, which means several zones on one leg are handled
//! sequentially rather than in a single pre-planned detour. Vertical escape is
//! not implemented.

use serde::{Deserialize, Serialize};

use crate::math;
use crate::types::{Arr3, Obstacle, Waypoint};

/// Standoff beyond radius + margin for a bypass point (m).
///
/// The bypass itself must clear the zone with room for the acceptance radius
/// and the tracking error, so it sits this far outside `radius + margin`.
pub const DETOUR_BUFFER_M: f64 = 0.5;

/// Shortest lookahead ahead of the vehicle (m), used at low ground speed.
pub const MIN_LOOKAHEAD_M: f64 = 2.0;

/// How close to its bypass the vehicle must be for the latch to be considered
/// complete (m), as a floor; the used radius also scales with the bypass
/// standoff.
pub const MIN_RELEASE_RADIUS_M: f64 = 2.0;

/// Along-track band (m) that separates latching from releasing, so the law
/// cannot flicker between the two at one distance. A zone is only taken over
/// while it is more than this far ahead, and the latch is given up once the
/// vehicle is level with the zone or inside this band at its bypass.
pub const LATCH_HYSTERESIS_M: f64 = 2.0;

/// Leg-length epsilon (m), matching guidance's own.
const LEG_EPS: f64 = 1.0e-6;

/// A negative tuning value.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AvoidanceError {
    /// The clearance margin was negative.
    NegativeClearanceMargin(f64),
    /// The detour buffer was negative.
    NegativeDetourBuffer(f64),
    /// The lookahead time was negative.
    NegativeLookahead(f64),
}

impl std::fmt::Display for AvoidanceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NegativeClearanceMargin(v) => {
                write!(f, "clearance_margin_m must be non-negative, got {v}")
            }
            Self::NegativeDetourBuffer(v) => {
                write!(f, "detour_buffer_m must be non-negative, got {v}")
            }
            Self::NegativeLookahead(v) => write!(f, "lookahead_s must be non-negative, got {v}"),
        }
    }
}

impl std::error::Error for AvoidanceError {}

/// Tuning for the reactive law.
///
/// Serializable, and every field has a default, so a scenario can turn the law
/// on with `"avoidance": {"enabled": true}` and inherit the rest.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AvoidanceConfig {
    /// Master switch. Off (the default) means the law is never consulted, so a
    /// run with zones and the switch off is identical to one without zones.
    pub enabled: bool,
    /// Surface clearance the law plans to keep from a zone (m).
    pub clearance_margin_m: f64,
    /// Lookahead time ahead of the vehicle at its ground speed (s).
    pub lookahead_s: f64,
    /// Standoff beyond radius + margin for a bypass point (m).
    pub detour_buffer_m: f64,
}

impl Default for AvoidanceConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            clearance_margin_m: 2.0,
            lookahead_s: 3.0,
            detour_buffer_m: DETOUR_BUFFER_M,
        }
    }
}

impl AvoidanceConfig {
    /// Reject negative tuning.
    pub fn validate(&self) -> Result<(), AvoidanceError> {
        if self.clearance_margin_m < 0.0 {
            return Err(AvoidanceError::NegativeClearanceMargin(
                self.clearance_margin_m,
            ));
        }
        if self.detour_buffer_m < 0.0 {
            return Err(AvoidanceError::NegativeDetourBuffer(self.detour_buffer_m));
        }
        if self.lookahead_s < 0.0 {
            return Err(AvoidanceError::NegativeLookahead(self.lookahead_s));
        }
        Ok(())
    }

    /// The distance ahead of the vehicle at which a zone counts as a threat
    /// (m). Grows with ground speed so the law commits with the same time to
    /// spare at any cruise speed, and is floored so a slow vehicle still
    /// reacts.
    pub fn lookahead_distance_m(&self, velocity_ned: Arr3) -> f64 {
        let ground_speed = math::hypot(velocity_ned[0], velocity_ned[1]);
        f64::max(MIN_LOOKAHEAD_M, self.lookahead_s * ground_speed)
    }
}

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

/// Minimum horizontal distance from a zone centre to the leg `p0 -> p1` (m).
///
/// The centre is projected onto the leg in the north-east plane and clamped to
/// the segment, so a leg that ends before the closest approach reports the
/// distance to its endpoint.
pub fn leg_obstacle_distance(p0: Arr3, p1: Arr3, obstacle: &Obstacle) -> f64 {
    let (a, b, c) = (planar(p0), planar(p1), planar(obstacle.center_ned));
    let d = [b[0] - a[0], b[1] - a[1]];
    let d_norm_sq = d[0] * d[0] + d[1] * d[1];
    if d_norm_sq < 1e-12 {
        return math::hypot(a[0] - c[0], a[1] - c[1]);
    }
    let t = math::clamp(
        ((c[0] - a[0]) * d[0] + (c[1] - a[1]) * d[1]) / d_norm_sq,
        0.0,
        1.0,
    );
    let closest = [a[0] + t * d[0], a[1] + t * d[1]];
    math::hypot(closest[0] - c[0], closest[1] - c[1])
}

/// True when the horizontal leg passes within `radius + clearance_margin` of
/// the zone centre.
///
/// `clearance_margin` must be non-negative: the callers that take it from
/// outside (the Python binding) reject a negative value rather than treating
/// it as zero, and the law's own config is validated on construction.
pub fn leg_intersects_obstacle(
    p0: Arr3,
    p1: Arr3,
    obstacle: &Obstacle,
    clearance_margin: f64,
) -> bool {
    leg_obstacle_distance(p0, p1, obstacle) < obstacle.radius_m + clearance_margin
}

/// Surface clearance (m) from a position to a zone: positive outside, zero on
/// the surface, negative inside.
pub fn surface_clearance(position_ned: Arr3, obstacle: &Obstacle) -> f64 {
    math::hypot(
        math::hypot(
            position_ned[0] - obstacle.center_ned[0],
            position_ned[1] - obstacle.center_ned[1],
        ),
        position_ned[2] - obstacle.center_ned[2],
    ) - obstacle.radius_m
}

/// Minimum surface clearance (m) from a position to the nearest zone, or
/// `f64::INFINITY` when there are none.
pub fn nearest_surface_clearance(position_ned: Arr3, obstacles: &[Obstacle]) -> f64 {
    let mut worst = f64::INFINITY;
    for obstacle in obstacles {
        let clearance = surface_clearance(position_ned, obstacle);
        if clearance < worst {
            worst = clearance;
        }
    }
    worst
}

/// Minimum surface clearance (m) over a trajectory and a zone list, or
/// `f64::INFINITY` when either is empty.
pub fn min_surface_clearance(positions: &[Arr3], obstacles: &[Obstacle]) -> f64 {
    let mut worst = f64::INFINITY;
    for position in positions {
        let clearance = nearest_surface_clearance(*position, obstacles);
        if clearance < worst {
            worst = clearance;
        }
    }
    worst
}

/// One bypass waypoint that routes a leg around a zone, with the standoff
/// beyond `radius + clearance_margin` given explicitly.
///
/// The closest approach of the centre on the leg is found, then the point is
/// stepped radially outward from the centre until it clears the zone. Altitude
/// is the leg origin's, so the detour is horizontal.
pub fn detour_point_with_buffer(
    p0: Arr3,
    p1: Arr3,
    obstacle: &Obstacle,
    clearance_margin: f64,
    buffer_m: f64,
) -> Arr3 {
    let (a, b, c) = (planar(p0), planar(p1), planar(obstacle.center_ned));
    let reach = obstacle.radius_m + clearance_margin + buffer_m;
    let d = [b[0] - a[0], b[1] - a[1]];
    let d_norm_sq = d[0] * d[0] + d[1] * d[1];
    if d_norm_sq < 1e-12 {
        // Degenerate leg: offset east of the zone as a safe fallback.
        return [c[0] + reach, c[1], p0[2]];
    }
    let t = math::clamp(
        ((c[0] - a[0]) * d[0] + (c[1] - a[1]) * d[1]) / d_norm_sq,
        0.0,
        1.0,
    );
    let closest = [a[0] + t * d[0], a[1] + t * d[1]];
    let away = [closest[0] - c[0], closest[1] - c[1]];
    let away_norm = math::hypot(away[0], away[1]);
    let away_unit = if away_norm < 1e-9 {
        // The centre lies on the leg: step along the leg perpendicular.
        let perp_norm = math::hypot(-d[1], d[0]);
        [-d[1] / perp_norm, d[0] / perp_norm]
    } else {
        [away[0] / away_norm, away[1] / away_norm]
    };
    [
        closest[0] + away_unit[0] * reach,
        closest[1] + away_unit[1] * reach,
        p0[2],
    ]
}

/// One bypass waypoint for a leg and a zone, with the default standoff.
///
/// The signature and the arithmetic match `talon.avoidance.compute_detour_point`
/// exactly, which is what lets the Python module delegate here.
pub fn compute_detour_point(
    p0: Arr3,
    p1: Arr3,
    obstacle: &Obstacle,
    clearance_margin: f64,
) -> Arr3 {
    detour_point_with_buffer(p0, p1, obstacle, clearance_margin, DETOUR_BUFFER_M)
}

/// A waypoint list with one detour waypoint inserted per threatened leg.
///
/// For each consecutive pair, the first zone whose keep-out shell intersects
/// the horizontal leg gets one detour inserted before the leg end. The detour
/// inherits the leg's cruise speed and acceptance radius. At most one detour
/// per leg, matching `talon.avoidance.plan_avoidance_mission`; the reactive
/// law is the path that handles several zones on one leg.
pub fn plan_avoidance_mission(
    waypoints: &[Waypoint],
    obstacles: &[Obstacle],
    clearance_margin: f64,
) -> Vec<Waypoint> {
    if waypoints.len() < 2 {
        return waypoints.to_vec();
    }
    let mut augmented: Vec<Waypoint> = Vec::with_capacity(waypoints.len() * 2);
    augmented.push(waypoints[0].clone());
    for i in 0..waypoints.len() - 1 {
        let p0 = waypoints[i].position;
        let p1 = waypoints[i + 1].position;
        for obstacle in obstacles {
            if leg_intersects_obstacle(p0, p1, obstacle, clearance_margin) {
                let detour = compute_detour_point(p0, p1, obstacle, clearance_margin);
                augmented.push(Waypoint::new(
                    detour,
                    waypoints[i].speed,
                    waypoints[i].heading,
                    waypoints[i].radius,
                ));
                break;
            }
        }
        augmented.push(waypoints[i + 1].clone());
    }
    augmented
}

// ---------------------------------------------------------------------------
// Reactive law
// ---------------------------------------------------------------------------

/// The leg the mission is flying this cycle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MissionLeg {
    /// Leg origin, the previous waypoint or the recorded mission start.
    pub origin: Arr3,
    /// The waypoint being flown toward.
    pub waypoint: Arr3,
    /// The mission's active index, which retires a latch from an older leg.
    pub index: usize,
}

/// What the law wants guidance to fly to this cycle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AvoidanceDirective {
    /// Target for guidance: the waypoint, or a bypass point. `None` when no
    /// leg is being flown, in which case guidance is not asking.
    pub target: Option<Arr3>,
    /// True when `target` is a bypass point rather than the waypoint.
    pub active: bool,
    /// Surface clearance from the current position to the nearest zone (m),
    /// `f64::INFINITY` when there are none.
    pub clearance_m: f64,
}

/// A latched bypass: the zone being avoided and where guidance is being sent.
#[derive(Debug, Clone)]
struct Latch {
    /// Active waypoint index the latch belongs to.
    leg_index: usize,
    /// The bypass point guidance is flying to.
    bypass: Arr3,
    /// Along-track coordinate of the zone's closest approach on the leg (m).
    abreast_m: f64,
    /// Standoff of the bypass from the centre (m), used for the release radius.
    reach_m: f64,
}

/// The in-the-loop avoidance law.
///
/// Stateful only in what it is currently avoiding: the directive itself is a
/// pure function of the current state, the leg, and the zones, so two runs
/// with the same inputs latch and release at the same step.
#[derive(Debug, Clone, Default)]
pub struct ReactiveAvoidance {
    config: AvoidanceConfig,
    latch: Option<Latch>,
    activations: u64,
}

impl ReactiveAvoidance {
    /// Build with the given tuning.
    pub fn new(config: AvoidanceConfig) -> Self {
        Self {
            config,
            latch: None,
            activations: 0,
        }
    }

    /// The active configuration.
    pub fn config(&self) -> AvoidanceConfig {
        self.config
    }

    /// How many times the law has taken over guidance. One per bypass taken.
    pub fn activations(&self) -> u64 {
        self.activations
    }

    /// True while a bypass is latched.
    pub fn is_active(&self) -> bool {
        self.latch.is_some()
    }

    /// One control cycle.
    ///
    /// `leg` is `Some` only while the mission is flying a leg; in every other
    /// phase the law tracks clearance and stays out of the way, and it clears
    /// any latch so a later leg starts from a clean state.
    pub fn update(
        &mut self,
        position_ned: Arr3,
        velocity_ned: Arr3,
        leg: Option<MissionLeg>,
        obstacles: &[Obstacle],
    ) -> AvoidanceDirective {
        let clearance_m = nearest_surface_clearance(position_ned, obstacles);

        let Some(leg) = leg else {
            self.latch = None;
            return AvoidanceDirective {
                target: None,
                active: false,
                clearance_m,
            };
        };

        let idle = AvoidanceDirective {
            target: Some(leg.waypoint),
            active: false,
            clearance_m,
        };

        if !self.config.enabled || obstacles.is_empty() {
            self.latch = None;
            return idle;
        }

        // A latch belongs to one leg: a new waypoint retires it.
        if self
            .latch
            .as_ref()
            .is_some_and(|latched| latched.leg_index != leg.index)
        {
            self.latch = None;
        }

        let Some(direction) = leg_direction(leg.origin, leg.waypoint) else {
            self.latch = None;
            return idle;
        };

        if let Some(latched) = self.latch.clone() {
            let progress_m = along_track(position_ned, leg.origin, direction);
            let remaining_m = latched.abreast_m - progress_m;
            let release_radius = f64::max(MIN_RELEASE_RADIUS_M, 0.5 * latched.reach_m);
            let near = distance3(position_ned, latched.bypass) <= release_radius;
            let clear = clearance_m >= self.config.clearance_margin_m;
            // Released once the vehicle has drawn level with the zone, or once
            // it has arrived at the bypass it was sent to, and the clearance
            // has recovered. Never while inside the margin. Both arms sit
            // inside the band the latch arms on, so a release cannot flicker
            // straight back into a latch.
            let level = remaining_m <= 0.0;
            let at_bypass = near && remaining_m <= LATCH_HYSTERESIS_M;
            if clear && (level || at_bypass) {
                self.latch = None;
                return idle;
            }
            return AvoidanceDirective {
                target: Some(latched.bypass),
                active: true,
                clearance_m,
            };
        }

        // Nothing latched: is any zone a threat within the lookahead?
        let lookahead_m = self.config.lookahead_distance_m(velocity_ned);
        let leg_length = math::hypot(
            leg.waypoint[0] - leg.origin[0],
            leg.waypoint[1] - leg.origin[1],
        );
        let mut threat: Option<(usize, f64)> = None;
        for (index, obstacle) in obstacles.iter().enumerate() {
            let lateral_m = leg_obstacle_distance(leg.origin, leg.waypoint, obstacle);
            if lateral_m >= obstacle.radius_m + self.config.clearance_margin_m {
                continue;
            }
            let abreast_m = along_track(obstacle.center_ned, leg.origin, direction);
            // A zone abreast of the leg origin or past the leg end is not on
            // this leg; the segment endpoints already cover those cases.
            if abreast_m <= 0.0 || abreast_m >= leg_length {
                continue;
            }
            let remaining_m = abreast_m - along_track(position_ned, leg.origin, direction);
            // Only a zone still meaningfully ahead is taken over: the same band
            // the release uses, which is what keeps the law from oscillating.
            if remaining_m <= LATCH_HYSTERESIS_M || remaining_m > lookahead_m {
                continue;
            }
            if threat.is_none_or(|(_, best)| remaining_m < best) {
                threat = Some((index, remaining_m));
            }
        }

        let Some((index, _)) = threat else {
            return idle;
        };
        let obstacle = &obstacles[index];
        let bypass = detour_point_with_buffer(
            leg.origin,
            leg.waypoint,
            obstacle,
            self.config.clearance_margin_m,
            self.config.detour_buffer_m,
        );
        self.latch = Some(Latch {
            leg_index: leg.index,
            bypass,
            abreast_m: along_track(obstacle.center_ned, leg.origin, direction),
            reach_m: obstacle.radius_m
                + self.config.clearance_margin_m
                + self.config.detour_buffer_m,
        });
        self.activations += 1;
        AvoidanceDirective {
            target: Some(bypass),
            active: true,
            clearance_m,
        }
    }
}

/// North-east pair of a NED triple.
fn planar(v: Arr3) -> [f64; 2] {
    [v[0], v[1]]
}

/// Unit north-east direction of a leg, or `None` when the leg is degenerate.
fn leg_direction(origin: Arr3, waypoint: Arr3) -> Option<[f64; 2]> {
    let d = [waypoint[0] - origin[0], waypoint[1] - origin[1]];
    let length = math::hypot(d[0], d[1]);
    if length < LEG_EPS {
        return None;
    }
    Some([d[0] / length, d[1] / length])
}

/// Along-track coordinate of a point measured from a leg origin (m).
fn along_track(point: Arr3, origin: Arr3, direction: [f64; 2]) -> f64 {
    (point[0] - origin[0]) * direction[0] + (point[1] - origin[1]) * direction[1]
}

/// Full 3-D distance between two NED positions (m).
fn distance3(a: Arr3, b: Arr3) -> f64 {
    math::hypot(math::hypot(a[0] - b[0], a[1] - b[1]), a[2] - b[2])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn obstacle(center: Arr3, radius: f64) -> Obstacle {
        Obstacle {
            id: "zone".to_string(),
            center_ned: center,
            radius_m: radius,
        }
    }

    fn wp(n: f64, e: f64, d: f64) -> Waypoint {
        Waypoint::new([n, e, d], 5.0, 0.0, 2.0)
    }

    // Geometry, mirroring python/tests/test_avoidance.py case for case so the
    // port is pinned by the same expectations the Python module is.
    const A: Arr3 = [0.0, 0.0, 0.0];
    const B: Arr3 = [10.0, 0.0, 0.0];

    #[test]
    fn leg_distance_center_on_leg_is_zero() {
        assert!((leg_obstacle_distance(A, B, &obstacle([5.0, 0.0, 0.0], 2.0))).abs() < 1e-12);
    }

    #[test]
    fn leg_distance_off_leg_is_the_lateral_gap() {
        assert!((leg_obstacle_distance(A, B, &obstacle([5.0, 3.0, 0.0], 2.0)) - 3.0).abs() < 1e-12);
    }

    #[test]
    fn leg_distance_clamps_to_the_segment() {
        // Beyond the start and beyond the end, so both clamp to an endpoint.
        assert!(
            (leg_obstacle_distance(A, B, &obstacle([-5.0, 0.0, 0.0], 1.0)) - 5.0).abs() < 1e-12
        );
        assert!(
            (leg_obstacle_distance(A, B, &obstacle([15.0, 0.0, 0.0], 1.0)) - 5.0).abs() < 1e-12
        );
    }

    #[test]
    fn leg_distance_of_a_degenerate_leg_is_the_point_distance() {
        assert!((leg_obstacle_distance(A, A, &obstacle([3.0, 4.0, 0.0], 1.0)) - 5.0).abs() < 1e-12);
    }

    #[test]
    fn intersects_when_clear_and_when_penetrating() {
        assert!(!leg_intersects_obstacle(
            A,
            B,
            &obstacle([5.0, 5.0, 0.0], 2.0),
            0.0
        ));
        assert!(leg_intersects_obstacle(
            A,
            B,
            &obstacle([5.0, 1.0, 0.0], 2.0),
            0.0
        ));
    }

    #[test]
    fn a_margin_turns_a_near_miss_into_a_threat() {
        // The same case as python/tests/test_avoidance.py: 1.5 m of lateral gap
        // against a 1 m sphere.
        let zone = obstacle([5.0, 1.5, 0.0], 1.0);
        assert!(!leg_intersects_obstacle(A, B, &zone, 0.0));
        assert!(leg_intersects_obstacle(A, B, &zone, 1.0));
    }

    #[test]
    fn detour_clears_the_zone_and_holds_the_leg_altitude() {
        let zone = obstacle([5.0, 0.0, -10.0], 2.0);
        let detour = compute_detour_point([0.0, 0.0, -10.0], [10.0, 0.0, -10.0], &zone, 1.0);
        let gap = math::hypot(
            math::hypot(
                detour[0] - zone.center_ned[0],
                detour[1] - zone.center_ned[1],
            ),
            detour[2] - zone.center_ned[2],
        );
        // radius + margin + the standoff buffer.
        assert!(
            (gap - (2.0 + 1.0 + DETOUR_BUFFER_M)).abs() < 1e-9,
            "gap {gap}"
        );
        assert!((detour[2] + 10.0).abs() < 1e-12, "altitude {detour:?}");
    }

    #[test]
    fn a_center_on_the_leg_steps_off_perpendicular() {
        let zone = obstacle([5.0, 0.0, 0.0], 2.0);
        let detour = compute_detour_point(A, B, &zone, 0.5);
        // Perpendicular to a northward leg stays at the same north coordinate.
        assert!((detour[0] - 5.0).abs() < 1e-12, "{detour:?}");
        assert!(
            (detour[1].abs() - (2.0 + 0.5 + DETOUR_BUFFER_M)).abs() < 1e-9,
            "{detour:?}"
        );
    }

    #[test]
    fn plan_inserts_one_detour_and_leaves_clear_legs_alone() {
        let route = vec![wp(0.0, 0.0, -10.0), wp(20.0, 0.0, -10.0)];
        assert_eq!(plan_avoidance_mission(&route, &[], 2.0).len(), 2);
        let clear = plan_avoidance_mission(&route, &[obstacle([10.0, 30.0, -10.0], 2.0)], 2.0);
        assert_eq!(clear.len(), 2);
        let hit = plan_avoidance_mission(&route, &[obstacle([10.0, 3.0, -10.0], 4.0)], 2.0);
        assert_eq!(hit.len(), 3);
        // The detour inherits the leg's speed and radius.
        assert!((hit[1].speed - 5.0).abs() < 1e-12);
        let single = plan_avoidance_mission(&route[..1], &[obstacle([5.0, 0.0, -10.0], 2.0)], 1.0);
        assert_eq!(single.len(), 1);
    }

    #[test]
    fn min_surface_clearance_is_the_worst_sample() {
        assert_eq!(
            min_surface_clearance(&[[0.0, 0.0, 0.0]], &[]),
            f64::INFINITY
        );
        assert_eq!(
            min_surface_clearance(&[], &[obstacle([0.0, 0.0, 0.0], 1.0)]),
            f64::INFINITY
        );
        let zones = [obstacle([0.0, 0.0, 0.0], 2.0)];
        let positions = [[5.0, 0.0, 0.0], [3.0, 0.0, 0.0], [9.0, 0.0, 0.0]];
        // Closest sample is 3 m out, so 1 m of surface clearance.
        assert!((min_surface_clearance(&positions, &zones) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn surface_clearance_signs_the_inside_and_the_outside() {
        let zone = obstacle([0.0, 0.0, -10.0], 4.0);
        assert!((surface_clearance([0.0, 0.0, -20.0], &zone) - 6.0).abs() < 1e-12);
        assert!((surface_clearance([0.0, 0.0, -10.0], &zone) + 4.0).abs() < 1e-12);
    }

    #[test]
    fn config_validation_rejects_negative_tuning() {
        let mut config = AvoidanceConfig {
            enabled: true,
            ..Default::default()
        };
        assert!(config.validate().is_ok());
        config.clearance_margin_m = -1.0;
        assert!(matches!(
            config.validate(),
            Err(AvoidanceError::NegativeClearanceMargin(_))
        ));
        config.clearance_margin_m = 2.0;
        config.detour_buffer_m = -0.5;
        assert!(matches!(
            config.validate(),
            Err(AvoidanceError::NegativeDetourBuffer(_))
        ));
        config.detour_buffer_m = 0.5;
        config.lookahead_s = -1.0;
        assert!(matches!(
            config.validate(),
            Err(AvoidanceError::NegativeLookahead(_))
        ));
    }

    #[test]
    fn lookahead_grows_with_ground_speed_and_is_floored() {
        let config = AvoidanceConfig {
            enabled: true,
            lookahead_s: 3.0,
            ..Default::default()
        };
        assert!((config.lookahead_distance_m([0.0, 0.0, 0.0]) - MIN_LOOKAHEAD_M).abs() < 1e-12);
        assert!((config.lookahead_distance_m([5.0, 0.0, 0.0]) - 15.0).abs() < 1e-12);
        assert!((config.lookahead_distance_m([4.0, 3.0, 0.0]) - 15.0).abs() < 1e-12);
    }

    // The law itself.

    fn enabled() -> ReactiveAvoidance {
        ReactiveAvoidance::new(AvoidanceConfig {
            enabled: true,
            clearance_margin_m: 2.0,
            lookahead_s: 3.0,
            detour_buffer_m: DETOUR_BUFFER_M,
        })
    }

    /// The obstacle_field's first leg: 40 m north at 10 m, zone 3 m east.
    fn leg() -> MissionLeg {
        MissionLeg {
            origin: [0.0, 0.0, -10.0],
            waypoint: [40.0, 0.0, -10.0],
            index: 0,
        }
    }

    fn north_zone() -> Obstacle {
        obstacle([20.0, 3.0, -10.0], 4.0)
    }

    #[test]
    fn disabled_law_never_takes_over() {
        let mut law = ReactiveAvoidance::new(AvoidanceConfig::default());
        let zones = [north_zone()];
        // Inside the lookahead and inside the margin, but the switch is off.
        let directive = law.update([18.0, 0.0, -10.0], [5.0, 0.0, 0.0], Some(leg()), &zones);
        assert_eq!(directive.target, Some(leg().waypoint));
        assert!(!directive.active);
        assert_eq!(law.activations(), 0);
    }

    #[test]
    fn no_zones_means_no_law() {
        let mut law = enabled();
        let directive = law.update([0.0, 0.0, -10.0], [5.0, 0.0, 0.0], Some(leg()), &[]);
        assert_eq!(directive.target, Some(leg().waypoint));
        assert!(!directive.active);
        assert_eq!(directive.clearance_m, f64::INFINITY);
    }

    #[test]
    fn a_threat_beyond_the_lookahead_is_left_alone_until_it_is_close() {
        let mut law = enabled();
        let zones = [north_zone()];
        // 20 m to the abreast point at 5 m/s is 4 s, past the 3 s lookahead.
        let far = law.update([0.0, 0.0, -10.0], [5.0, 0.0, 0.0], Some(leg()), &zones);
        assert!(!far.active);
        // 12 m out is inside it.
        let near = law.update([8.0, 0.0, -10.0], [5.0, 0.0, 0.0], Some(leg()), &zones);
        assert!(near.active);
        assert_eq!(law.activations(), 1);
        // The bypass sits radius + margin + buffer from the centre, on the far
        // side of the leg from it.
        assert!((near.target.unwrap()[0] - 20.0).abs() < 1e-9);
        assert!((near.target.unwrap()[1] + 6.5).abs() < 1e-9);
        assert!((near.target.unwrap()[2] + 10.0).abs() < 1e-12);
    }

    #[test]
    fn a_zone_abreast_of_the_leg_origin_or_past_its_end_is_not_on_the_leg() {
        let mut law = enabled();
        // North of the leg end: the leg ends before its closest approach.
        let past = law.update(
            [0.0, 0.0, -10.0],
            [5.0, 0.0, 0.0],
            Some(leg()),
            &[obstacle([60.0, 0.0, -10.0], 4.0)],
        );
        assert!(!past.active);
        // Behind the leg origin.
        let behind = law.update(
            [0.0, 0.0, -10.0],
            [5.0, 0.0, 0.0],
            Some(leg()),
            &[obstacle([-20.0, 0.0, -10.0], 4.0)],
        );
        assert!(!behind.active);
    }

    #[test]
    fn a_threat_behind_the_vehicle_is_not_re_engaged() {
        let mut law = enabled();
        let zones = [north_zone()];
        let directive = law.update([25.0, 0.0, -10.0], [5.0, 0.0, 0.0], Some(leg()), &zones);
        assert!(!directive.active, "the zone is abreast, not ahead");
        assert_eq!(law.activations(), 0);
    }

    #[test]
    fn the_latch_holds_until_the_vehicle_is_beside_the_zone_and_clear() {
        let mut law = enabled();
        let zones = [north_zone()];
        law.update([8.0, 0.0, -10.0], [5.0, 0.0, 0.0], Some(leg()), &zones);
        assert!(law.is_active());
        // Approaching the bypass: still latched, still aimed at the bypass.
        let approach = law.update([17.0, -4.0, -10.0], [5.0, -2.0, 0.0], Some(leg()), &zones);
        assert!(approach.active);
        assert_eq!(law.activations(), 1, "one bypass, not one per cycle");
        // Beside the bypass and clear: released, back onto the waypoint.
        let released = law.update([20.0, -6.5, -10.0], [5.0, 0.0, 0.0], Some(leg()), &zones);
        assert!(!released.active);
        assert_eq!(released.target, Some(leg().waypoint));
        assert!(!law.is_active());
    }

    #[test]
    fn the_latch_does_not_release_inside_the_margin() {
        let mut law = enabled();
        let zones = [north_zone()];
        law.update([8.0, 0.0, -10.0], [5.0, 0.0, 0.0], Some(leg()), &zones);
        // Inside the zone: 2 m from the centre of a 4 m sphere.
        let inside = law.update([20.0, 1.0, -10.0], [5.0, 0.0, 0.0], Some(leg()), &zones);
        assert!(
            inside.active,
            "the law never lets go while inside the margin"
        );
        assert!(inside.clearance_m < 0.0);
    }

    #[test]
    fn a_new_waypoint_retires_the_latch() {
        let mut law = enabled();
        let zones = [north_zone()];
        law.update([8.0, 0.0, -10.0], [5.0, 0.0, 0.0], Some(leg()), &zones);
        assert!(law.is_active());
        let next = MissionLeg {
            origin: [40.0, 0.0, -10.0],
            waypoint: [40.0, 30.0, -10.0],
            index: 1,
        };
        let directive = law.update([40.0, 0.0, -10.0], [5.0, 0.0, 0.0], Some(next), &zones);
        assert!(!directive.active);
        assert!(!law.is_active());
    }

    #[test]
    fn leaving_the_mission_phase_clears_the_latch() {
        let mut law = enabled();
        let zones = [north_zone()];
        law.update([8.0, 0.0, -10.0], [5.0, 0.0, 0.0], Some(leg()), &zones);
        let directive = law.update([8.0, 0.0, -10.0], [5.0, 0.0, 0.0], None, &zones);
        assert_eq!(directive.target, None);
        assert!(!directive.active);
        assert!(!law.is_active());
    }

    #[test]
    fn several_zones_on_one_leg_are_taken_one_after_another() {
        // Two zones on an 80 m leg, 40 m apart. The law latches one bypass at a
        // time, which is how it handles more than the offline planner's one.
        let zones = [
            obstacle([20.0, 3.0, -10.0], 4.0),
            obstacle([60.0, 3.0, -10.0], 4.0),
        ];
        let leg = MissionLeg {
            origin: [0.0, 0.0, -10.0],
            waypoint: [80.0, 0.0, -10.0],
            index: 0,
        };
        let mut law = enabled();
        // Crude closed loop: step 1 m toward whatever the law asks for. The
        // real controller flies this in the SIL test.
        let mut position = leg.origin;
        for _ in 0..400 {
            let directive = law.update(position, [5.0, 0.0, 0.0], Some(leg), &zones);
            let target = directive.target.expect("a leg is being flown");
            let delta = [target[0] - position[0], target[1] - position[1]];
            let distance = math::hypot(delta[0], delta[1]);
            if distance < 0.4 {
                break;
            }
            position = [
                position[0] + delta[0] / distance,
                position[1] + delta[1] / distance,
                -10.0,
            ];
        }
        let finished = math::hypot(position[0] - leg.waypoint[0], position[1] - leg.waypoint[1]);
        assert!(
            finished < 1.0,
            "walk should reach the waypoint, ended at {position:?}"
        );
        assert_eq!(law.activations(), 2, "one bypass per zone");
    }
}
