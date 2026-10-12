#pragma once

namespace talon {

// Vehicle
inline constexpr double MASS = 1.5;            // kg
inline constexpr double GRAVITY = 9.81;        // m/s^2
inline constexpr double ARM_LENGTH = 0.15;     // m

// Inertia (diagonal, body frame, kg m^2)
inline constexpr double IXX = 2.32e-3;
inline constexpr double IYY = 2.32e-3;
inline constexpr double IZZ = 4.05e-3;

// Motors
inline constexpr double MOTOR_KF = 1.28e-6;      // thrust coeff: N / (rad/s)^2
inline constexpr double MOTOR_KM = 5.96e-3;      // yaw moment coeff: N*m / (rad/s)^2
inline constexpr double MOTOR_TIME_CONST = 0.02; // first-order lag time constant, s

// Aerodynamics
inline constexpr double DRAG_COEFF = 0.3;      // dimensionless Cd
inline constexpr double AIR_DENSITY = 1.225;   // kg/m^3
inline constexpr double VEHICLE_AREA = 0.04;   // m^2

// Timing (s)
inline constexpr double DT = 0.005;         // dynamics / RK4 step (200 Hz)
inline constexpr double DT_CONTROL = 0.01;  // controller update (100 Hz)
inline constexpr double DT_GPS = 0.1;       // GPS update (10 Hz)
inline constexpr double DT_IMU = 0.005;     // IMU update (200 Hz)

// Motor limits
inline constexpr double M_MAX = 20000.0;    // max motor RPM

}  // namespace talon
