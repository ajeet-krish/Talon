//! Physical, aerodynamic and timing constants.
//!
//! Ported verbatim from the previous C++ core (`core/include/talon/constants.hpp`)
//! so the numbers are unchanged: this is a language move, not a re-tune.

use serde::{Deserialize, Serialize};

// Vehicle
pub const MASS: f64 = 1.5; // kg
pub const GRAVITY: f64 = 9.81; // m/s^2
pub const ARM_LENGTH: f64 = 0.15; // m

// Inertia (diagonal, body frame, kg m^2)
pub const IXX: f64 = 2.32e-3;
pub const IYY: f64 = 2.32e-3;
pub const IZZ: f64 = 4.05e-3;

// Motors
pub const MOTOR_KF: f64 = 1.28e-6; // thrust coeff: N / (rad/s)^2
pub const MOTOR_KM: f64 = 1.92e-8; // yaw drag coeff: N m / (rad/s)^2
pub const MOTOR_TIME_CONST: f64 = 0.02; // first-order lag time constant, s

/// Readability alias at motor-modelling call sites (matches the Python alias).
pub const MOTOR_LAG: f64 = MOTOR_TIME_CONST;

// Aerodynamics
pub const DRAG_COEFF: f64 = 0.3; // dimensionless Cd
pub const AIR_DENSITY: f64 = 1.225; // kg/m^3
pub const VEHICLE_AREA: f64 = 0.04; // m^2

// Timing (s)
pub const DT: f64 = 0.005; // dynamics / RK4 step (200 Hz)
pub const DT_CONTROL: f64 = 0.01; // controller update (100 Hz)
pub const DT_GPS: f64 = 0.1; // GPS update (10 Hz)
pub const DT_IMU: f64 = 0.005; // IMU update (200 Hz)

// Motor limits
pub const M_MAX: f64 = 20000.0; // max motor RPM

// ---------------------------------------------------------------------------
// Input validation caps
//
// Scenario values arrive as JSON, and JSON `1e999` parses to `inf`, which a
// bare `> 0.0` check waves through: the run duration would then saturate the
// step count to `usize::MAX`, and a NaN would leave targets undefined because
// every comparison against it is false. Scalars with a physical limit get a
// documented ceiling below; validation rejects (never clamps) anything
// non-finite or above its cap and names the offending field in the error.
// Fields with no natural ceiling (waypoint heading, an angle that guidance
// wraps) are still required to be finite, just not bounded.
// ---------------------------------------------------------------------------

/// Longest accepted run duration (s): 15 min of sim time.
pub const MAX_DURATION_S: f64 = 900.0;
/// Highest accepted takeoff altitude above home (m).
pub const MAX_TAKEOFF_ALTITUDE_M: f64 = 1000.0;
/// Largest accepted wind component, turbulence sigma or steady (m/s).
pub const MAX_WIND_COMPONENT_MPS: f64 = 100.0;
/// Largest accepted GPS position noise, horizontal or vertical (m).
pub const MAX_GPS_POSITION_NOISE_M: f64 = 100.0;
/// Largest accepted GPS velocity noise (m/s).
pub const MAX_GPS_VELOCITY_NOISE_MPS: f64 = 50.0;
/// Longest accepted CRUISE hold before the mission releases (s).
pub const MAX_CRUISE_HOLD_S: f64 = 60.0;
/// Most accepted mission waypoints.
pub const MAX_WAYPOINTS: usize = 512;
/// Most accepted keep-out zones.
pub const MAX_OBSTACLES: usize = 512;
/// Largest accepted coordinate magnitude in any NED position (m).
pub const MAX_COORDINATE_M: f64 = 1.0e6;
/// Largest accepted waypoint leg speed (m/s).
///
/// Missions cruise at single-digit m/s; 200 m/s is orders of magnitude past
/// anything this vehicle can fly, so a larger value is a typo (or a JSON
/// exponent run wild), not a real mission, and is rejected rather than flown.
pub const MAX_WAYPOINT_SPEED_MPS: f64 = 200.0;
/// Largest accepted radius magnitude: waypoint acceptance radius or keep-out
/// zone radius (m).
///
/// A radius is a non-negative extent, not a position, so it gets its own cap
/// at the coordinate scale: big enough for any plausible zone or acceptance
/// circle, small enough that `1e308`-style values never reach the geometry.
pub const MAX_RADIUS_M: f64 = 1.0e6;
/// Largest accepted avoidance clearance margin (m).
pub const MAX_AVOIDANCE_MARGIN_M: f64 = 1000.0;
/// Largest accepted avoidance detour buffer (m).
pub const MAX_AVOIDANCE_BUFFER_M: f64 = 1000.0;
/// Longest accepted avoidance lookahead time (s).
pub const MAX_AVOIDANCE_LOOKAHEAD_S: f64 = 60.0;
/// Most authored faults one scenario may schedule.
///
/// A cap keeps a mistyped or hostile document from queueing an unbounded list
/// of one-shot actions; sixteen is far more than any FDIR scenario needs and
/// small enough that a runaway editor loop is caught at the first write.
pub const MAX_FAULTS: usize = 16;
/// Largest accepted battery state of charge forced by a fault (fraction).
pub const MAX_BATTERY_SOC: f64 = 1.0;

// ---------------------------------------------------------------------------
// The caps as one value
//
// The mission editor validates scenarios in TypeScript, so the caps need a
// mirror the web app can import instead of a hand-copied list that drifts.
// [`Limits`] carries every cap above as fields, is serialized by ts-rs into
// `src/types/generated/talon_core.ts`, and is pinned by a vitest drift guard,
// so a change to a Rust constant shows up as a failed test until the mirror
// is regenerated.
// ---------------------------------------------------------------------------

/// The input-validation caps as one serializable value.
///
/// Every field is the named constant the core's own validation rejects on,
/// so [`crate::scenario::ScenarioConfig`] and the editor's validator agree by
/// construction: [`Limits::current`] is the single place the caps are read.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "ts-export", derive(ts_rs::TS))]
pub struct Limits {
    /// Dynamics / RK4 step (s): also the grid the fault window is cut on.
    pub dt: f64,
    /// Longest accepted run duration (s).
    pub max_duration_s: f64,
    /// Highest accepted takeoff altitude above home (m).
    pub max_takeoff_altitude_m: f64,
    /// Largest accepted wind component, turbulence sigma or steady (m/s).
    pub max_wind_component_mps: f64,
    /// Largest accepted GPS position noise, horizontal or vertical (m).
    pub max_gps_position_noise_m: f64,
    /// Largest accepted GPS velocity noise (m/s).
    pub max_gps_velocity_noise_mps: f64,
    /// Longest accepted CRUISE hold before the mission releases (s).
    pub max_cruise_hold_s: f64,
    /// Most accepted mission waypoints.
    #[cfg_attr(feature = "ts-export", ts(type = "number"))]
    pub max_waypoints: usize,
    /// Most accepted keep-out zones.
    #[cfg_attr(feature = "ts-export", ts(type = "number"))]
    pub max_obstacles: usize,
    /// Largest accepted coordinate magnitude in any NED position (m).
    pub max_coordinate_m: f64,
    /// Largest accepted waypoint leg speed (m/s).
    pub max_waypoint_speed_mps: f64,
    /// Largest accepted radius magnitude: waypoint acceptance radius or
    /// keep-out zone radius (m).
    pub max_radius_m: f64,
    /// Largest accepted avoidance clearance margin (m).
    pub max_avoidance_margin_m: f64,
    /// Largest accepted avoidance detour buffer (m).
    pub max_avoidance_buffer_m: f64,
    /// Longest accepted avoidance lookahead time (s).
    pub max_avoidance_lookahead_s: f64,
    /// Most authored faults one scenario may schedule.
    #[cfg_attr(feature = "ts-export", ts(type = "number"))]
    pub max_faults: usize,
    /// Largest accepted battery state of charge forced by a fault.
    pub max_battery_soc: f64,
}

impl Limits {
    /// The caps as they stand in this build of the core.
    ///
    /// Every field reads its own named constant, so `Limits` cannot drift
    /// from the values the validation code checks against.
    pub fn current() -> Self {
        Self {
            dt: DT,
            max_duration_s: MAX_DURATION_S,
            max_takeoff_altitude_m: MAX_TAKEOFF_ALTITUDE_M,
            max_wind_component_mps: MAX_WIND_COMPONENT_MPS,
            max_gps_position_noise_m: MAX_GPS_POSITION_NOISE_M,
            max_gps_velocity_noise_mps: MAX_GPS_VELOCITY_NOISE_MPS,
            max_cruise_hold_s: MAX_CRUISE_HOLD_S,
            max_waypoints: MAX_WAYPOINTS,
            max_obstacles: MAX_OBSTACLES,
            max_coordinate_m: MAX_COORDINATE_M,
            max_waypoint_speed_mps: MAX_WAYPOINT_SPEED_MPS,
            max_radius_m: MAX_RADIUS_M,
            max_avoidance_margin_m: MAX_AVOIDANCE_MARGIN_M,
            max_avoidance_buffer_m: MAX_AVOIDANCE_BUFFER_M,
            max_avoidance_lookahead_s: MAX_AVOIDANCE_LOOKAHEAD_S,
            max_faults: MAX_FAULTS,
            max_battery_soc: MAX_BATTERY_SOC,
        }
    }
}

impl Default for Limits {
    fn default() -> Self {
        Self::current()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `Limits::current()` is the named constants field for field, every cap
    /// is a finite positive magnitude, and the two sentinel values hold: the
    /// pack is never forced above full, and the fault list caps at sixteen.
    #[test]
    fn limits_mirror_the_named_constants_and_every_cap_is_finite_and_positive() {
        let limits = Limits::current();
        assert_eq!(limits.dt, DT);
        assert_eq!(limits.max_duration_s, MAX_DURATION_S);
        assert_eq!(limits.max_takeoff_altitude_m, MAX_TAKEOFF_ALTITUDE_M);
        assert_eq!(limits.max_wind_component_mps, MAX_WIND_COMPONENT_MPS);
        assert_eq!(limits.max_gps_position_noise_m, MAX_GPS_POSITION_NOISE_M);
        assert_eq!(
            limits.max_gps_velocity_noise_mps,
            MAX_GPS_VELOCITY_NOISE_MPS
        );
        assert_eq!(limits.max_cruise_hold_s, MAX_CRUISE_HOLD_S);
        assert_eq!(limits.max_waypoints, MAX_WAYPOINTS);
        assert_eq!(limits.max_obstacles, MAX_OBSTACLES);
        assert_eq!(limits.max_coordinate_m, MAX_COORDINATE_M);
        assert_eq!(limits.max_waypoint_speed_mps, MAX_WAYPOINT_SPEED_MPS);
        assert_eq!(limits.max_radius_m, MAX_RADIUS_M);
        assert_eq!(limits.max_avoidance_margin_m, MAX_AVOIDANCE_MARGIN_M);
        assert_eq!(limits.max_avoidance_buffer_m, MAX_AVOIDANCE_BUFFER_M);
        assert_eq!(limits.max_avoidance_lookahead_s, MAX_AVOIDANCE_LOOKAHEAD_S);
        assert_eq!(limits.max_faults, MAX_FAULTS);
        assert_eq!(limits.max_battery_soc, MAX_BATTERY_SOC);

        // The sentinels the editor relies on: full charge, sixteen faults.
        assert_eq!(limits.max_battery_soc, 1.0);
        assert_eq!(limits.max_faults, 16);

        // Every float cap is finite and positive, so a bad value can never
        // be explained away by a cap of zero or NaN.
        let floats = [
            limits.dt,
            limits.max_duration_s,
            limits.max_takeoff_altitude_m,
            limits.max_wind_component_mps,
            limits.max_gps_position_noise_m,
            limits.max_gps_velocity_noise_mps,
            limits.max_cruise_hold_s,
            limits.max_coordinate_m,
            limits.max_waypoint_speed_mps,
            limits.max_radius_m,
            limits.max_avoidance_margin_m,
            limits.max_avoidance_buffer_m,
            limits.max_avoidance_lookahead_s,
            limits.max_battery_soc,
        ];
        for value in floats {
            assert!(
                value.is_finite() && value > 0.0,
                "cap {value} must be finite and positive"
            );
        }
        // The count caps too.
        assert!(limits.max_waypoints > 0);
        assert!(limits.max_obstacles > 0);
        assert!(limits.max_faults > 0);

        // Default is the same value, so the two entry points agree.
        assert_eq!(Limits::default(), limits);
    }
}
