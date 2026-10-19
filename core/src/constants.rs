//! Physical, aerodynamic and timing constants.
//!
//! Ported verbatim from the previous C++ core (`core/include/talon/constants.hpp`)
//! so the numbers are unchanged: this is a language move, not a re-tune.

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
