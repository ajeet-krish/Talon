//! Waypoint mission manager.
//!
//! Owns the waypoint list, the active-index bookkeeping, and the acceptance
//! radius logic. Guidance ([`crate::guidance`]) is a pure function of
//! `(position, velocity, leg)` and never advances waypoints itself.
//!
//! Positions are NED metres, and acceptance and distance arithmetic is
//! horizontal-plane only (north-east), per `docs/conventions.md`. The mission
//! start position is recorded on the first [`WaypointMission::update`] call or
//! explicitly via [`WaypointMission::set_start`]; it is the origin of the first
//! leg, so a mission begun off the first waypoint still has a defined path.

use std::fmt;

use crate::math::horizontal_distance;
use crate::types::{Arr3, Waypoint};

/// Why a mission could not be constructed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissionError {
    /// The waypoint list was empty.
    NoWaypoints,
}

impl fmt::Display for MissionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoWaypoints => write!(f, "waypoints must be non-empty"),
        }
    }
}

impl std::error::Error for MissionError {}

/// Sequential waypoint list with an active index and an acceptance radius.
///
/// [`WaypointMission::update`] advances the active index while the vehicle is
/// within the *active* waypoint's horizontal radius. Advancing past the last
/// waypoint latches `complete`, and the active index clamps to the last
/// waypoint so guidance can still hold station on it.
#[derive(Debug, Clone)]
pub struct WaypointMission {
    waypoints: Vec<Waypoint>,
    active_index: usize,
    complete: bool,
    start_position: Option<Arr3>,
}

impl WaypointMission {
    /// Build a mission from a non-empty waypoint list.
    pub fn new(waypoints: Vec<Waypoint>) -> Result<Self, MissionError> {
        if waypoints.is_empty() {
            return Err(MissionError::NoWaypoints);
        }
        Ok(Self {
            waypoints,
            active_index: 0,
            complete: false,
            start_position: None,
        })
    }

    /// Index of the waypoint currently being flown toward.
    pub fn active_index(&self) -> usize {
        self.active_index
    }

    /// True once the vehicle has been within radius of the last waypoint.
    pub fn complete(&self) -> bool {
        self.complete
    }

    /// The mission waypoint list.
    pub fn waypoints(&self) -> &[Waypoint] {
        &self.waypoints
    }

    /// The waypoint currently being flown toward.
    pub fn active_waypoint(&self) -> &Waypoint {
        &self.waypoints[self.active_index]
    }

    /// Record the mission start position, the origin of the first leg.
    pub fn set_start(&mut self, position_ned: Arr3) {
        self.start_position = Some(position_ned);
    }

    /// Origin of the active leg (NED, m).
    ///
    /// Returns waypoint `i-1` once past the first waypoint, otherwise the
    /// recorded mission start, otherwise the current position, so guidance
    /// always has a valid leg origin.
    pub fn previous_point(&self, current_position: Arr3) -> Arr3 {
        if self.active_index > 0 {
            return self.waypoints[self.active_index - 1].position;
        }
        self.start_position.unwrap_or(current_position)
    }

    /// Advance the active waypoint if the vehicle is within its radius.
    ///
    /// Records the mission start position on the first call. Returns `true`
    /// once the vehicle has been within radius of the *last* waypoint; complete
    /// stays latched afterwards.
    ///
    /// A non-finite position does not advance the mission. The comparison is the
    /// reason: `NaN >= radius` is false in Rust exactly as in the Python source,
    /// so without this guard a NaN position would fall straight through every
    /// waypoint and complete the mission. Returning early keeps the control loop
    /// panic-free, which a flight core must be.
    pub fn update(&mut self, position_ned: Arr3) -> bool {
        if !position_ned.iter().all(|c| c.is_finite()) {
            return self.complete;
        }
        if self.start_position.is_none() {
            self.start_position = Some(position_ned);
        }

        while !self.complete {
            let active = &self.waypoints[self.active_index];
            if horizontal_distance(position_ned, active.position) >= active.radius {
                break;
            }
            if self.active_index + 1 >= self.waypoints.len() {
                self.complete = true;
            } else {
                self.active_index += 1;
            }
        }
        self.complete
    }

    /// Horizontal distance still to fly (m).
    ///
    /// Distance to the active waypoint plus the horizontal lengths of all
    /// remaining legs. Once complete this is the horizontal distance to the
    /// final waypoint, i.e. how far the vehicle has drifted from the end point.
    pub fn distance_remaining(&self, position_ned: Arr3) -> f64 {
        let mut total = horizontal_distance(position_ned, self.active_waypoint().position);
        for i in self.active_index..self.waypoints.len() - 1 {
            total += horizontal_distance(self.waypoints[i].position, self.waypoints[i + 1].position);
        }
        total
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wp(n: f64, e: f64, d: f64, radius: f64) -> Waypoint {
        Waypoint::new([n, e, d], 5.0, 0.0, radius)
    }

    fn three_waypoints() -> WaypointMission {
        WaypointMission::new(vec![
            wp(0.0, 0.0, -10.0, 2.0),
            wp(20.0, 0.0, -10.0, 2.0),
            wp(20.0, 20.0, -10.0, 2.0),
        ])
        .unwrap()
    }

    #[test]
    fn empty_mission_is_rejected() {
        assert!(matches!(
            WaypointMission::new(vec![]),
            Err(MissionError::NoWaypoints)
        ));
    }

    #[test]
    fn starts_on_the_first_waypoint() {
        let m = three_waypoints();
        assert_eq!(m.active_index(), 0);
        assert!(!m.complete());
        assert_eq!(m.active_waypoint().position, [0.0, 0.0, -10.0]);
    }

    #[test]
    fn advances_when_within_radius() {
        let mut m = three_waypoints();
        // Exactly at waypoint 0: within radius 2, so it should advance.
        m.update([0.0, 0.0, -10.0]);
        assert_eq!(m.active_index(), 1);
        assert!(!m.complete());
    }

    #[test]
    fn does_not_advance_when_outside_radius() {
        let mut m = three_waypoints();
        m.update([10.0, 0.0, -10.0]);
        assert_eq!(m.active_index(), 0);
    }

    #[test]
    fn altitude_does_not_affect_acceptance() {
        let mut m = three_waypoints();
        // Horizontally on the waypoint but 500 m below it: still accepted.
        m.update([0.0, 0.0, 500.0]);
        assert_eq!(m.active_index(), 1);
    }

    #[test]
    fn completes_at_the_last_waypoint_and_latches() {
        let mut m = three_waypoints();
        m.update([0.0, 0.0, -10.0]);
        m.update([20.0, 0.0, -10.0]);
        assert!(!m.complete(), "two of three done");
        m.update([20.0, 20.0, -10.0]);
        assert!(m.complete());
        assert_eq!(m.active_index(), 2, "index clamps to the last waypoint");
        // Flying away does not un-complete the mission.
        m.update([500.0, 500.0, -10.0]);
        assert!(m.complete());
    }

    #[test]
    fn a_nan_position_does_not_complete_the_mission() {
        let mut m = three_waypoints();
        m.update([f64::NAN, 0.0, -10.0]);
        assert_eq!(m.active_index(), 0);
        assert!(!m.complete());
    }

    #[test]
    fn previous_point_chains_waypoints() {
        let mut m = three_waypoints();
        // Before any update, and with no recorded start, the current position
        // is the fallback leg origin.
        assert_eq!(m.previous_point([7.0, 7.0, 0.0]), [7.0, 7.0, 0.0]);
        m.set_start([1.0, 2.0, 0.0]);
        assert_eq!(m.previous_point([7.0, 7.0, 0.0]), [1.0, 2.0, 0.0]);
        m.update([0.0, 0.0, -10.0]);
        assert_eq!(
            m.previous_point([7.0, 7.0, 0.0]),
            [0.0, 0.0, -10.0],
            "second leg starts at waypoint 0"
        );
    }

    #[test]
    fn start_position_is_recorded_on_first_update() {
        let mut m = three_waypoints();
        m.update([100.0, 0.0, -10.0]);
        assert_eq!(m.previous_point([0.0, 0.0, 0.0]), [100.0, 0.0, -10.0]);
    }

    #[test]
    fn distance_remaining_sums_the_legs() {
        let m = three_waypoints();
        // At the origin: 0 to wp0 (0 m) + wp0->wp1 (20 m) + wp1->wp2 (20 m).
        assert!((m.distance_remaining([0.0, 0.0, -10.0]) - 40.0).abs() < 1e-9);
    }

    #[test]
    fn distance_remaining_shrinks_as_the_mission_progresses() {
        // Walk the path in order. Note the advance is deliberately local:
        // being at waypoint 1 does nothing until waypoint 0 is satisfied,
        // because acceptance is measured against the ACTIVE waypoint.
        let mut m = three_waypoints();
        assert!((m.distance_remaining([0.0, 0.0, -10.0]) - 40.0).abs() < 1e-9);
        m.update([0.0, 0.0, -10.0]);
        assert_eq!(m.active_index(), 1);
        // Sitting at waypoint 1 without having satisfied it does not advance.
        m.update([20.0, 0.0, -10.0]);
        assert_eq!(m.active_index(), 2);
        let after = m.distance_remaining([20.0, 0.0, -10.0]);
        assert!((after - 20.0).abs() < 1e-9, "only the last leg remains: {after}");
        assert!(after < 40.0);
    }

    #[test]
    fn an_unreached_waypoint_blocks_progress() {
        let mut m = three_waypoints();
        // Jump straight to the last waypoint without satisfying wp0 or wp1.
        m.update([20.0, 20.0, -10.0]);
        assert_eq!(m.active_index(), 0, "missions are strictly sequential");
        assert!(!m.complete());
    }

    #[test]
    fn distance_remaining_after_completion_is_the_drift_from_the_end() {
        let mut m = three_waypoints();
        m.update([0.0, 0.0, -10.0]);
        m.update([20.0, 0.0, -10.0]);
        m.update([20.0, 20.0, -10.0]);
        assert!(m.complete());
        assert!((m.distance_remaining([30.0, 20.0, -10.0]) - 10.0).abs() < 1e-9);
    }
}
