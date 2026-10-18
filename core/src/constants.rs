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
