/**
 * Kernel and SIL constants, mirrored from the Python sources so the browser
 * run uses the same numbers: talon/constants.py (vehicle, motors, timing),
 * talon/sensors.py (M_PER_DEG), and talon/mavlink_sil.py (derived-estimate
 * thresholds, pre-roll cap).
 */

// Vehicle (talon.constants)
export const MASS = 1.5
export const GRAVITY = 9.81
export const ARM_LENGTH = 0.15
export const IXX = 2.32e-3
export const IYY = 2.32e-3
export const IZZ = 4.05e-3

// Motor limit
export const M_MAX = 20000.0

// Motors
export const MOTOR_KF = 1.28e-6
export const MOTOR_KM = 1.92e-8
export const MOTOR_TIME_CONST = 0.02
export const MOTOR_FORCE_MIN = 0.0
export const MOTOR_FORCE_MAX = MOTOR_KF * M_MAX ** 2

// Aerodynamics
export const DRAG_COEFF = 0.3
export const AIR_DENSITY = 1.225
export const VEHICLE_AREA = 0.04

// Timing (s)
export const DT = 0.005
export const DT_CONTROL = 0.01
export const DT_GPS = 0.1
export const DT_IMU = 0.005

// Flat-Earth mapping (talon.sensors)
export const M_PER_DEG = 111320.0

// Derived-estimate thresholds (talon.mavlink_sil)
export const CONVERGENCE_MIN_GPS_UPDATES = 20
export const CONVERGENCE_POS_TRACE_M2 = 4.0
export const DIVERGENCE_MIN_TIME_S = 5.0
export const DIVERGENCE_POS_ERROR_M = 20.0
export const DIVERGENCE_GATED_STREAK = 5
export const PREROLL_MAX_S = 5.0

// Hover collective thrust (N): mass * gravity.
export const HOVER_THRUST = MASS * GRAVITY
