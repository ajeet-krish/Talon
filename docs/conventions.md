# Talon Frame and Model Conventions

This document is the single source of truth for frame, sign, and unit
conventions. Every C++ header, Python module, and (in later phases) controller,
estimator, and guidance law must match what is written here. The classic
quadrotor bug factory is a frame mismatch, so when in doubt, re-read this file.

## Coordinate frames

- **NED (North-East-Down)**: the world/inertial frame. x points North, y points
  East, z points Down. All positions, linear velocities, and accelerations
  reported by `State` are in NED unless a name says otherwise.
- **Body frame**: aerospace standard, x forward, y right, z down. Body angular
  rates (`State.angular_rate`) are in the body frame.

## Gravity

In NED, gravity is

```
g_ned = [0, 0, +g]     with g = 9.81 m/s^2
```

An object in free fall accelerates along +z (downward). NED z increases with
altitude loss.

## Quaternion

- Hamilton convention, scalar-first ordering `[w, x, y, z]`.
- The quaternion represents the rotation **from body to NED**:
  `v_ned = R(q) * v_body`, where `R(q)` is the body-to-NED DCM built by
  `Quaternion::to_dcm()` / `rotation_matrix(q)`.
- Kinematics: `q_dot = 0.5 * q * [0, omega_body]` (Hamilton product, body
  rates).
- `propagate` renormalizes the quaternion at the end of every RK4 step.
- Note: a quaternion `q` and `-q` are the same rotation; tests compare with
  that in mind.

## Thrust convention

Collective thrust `T >= 0` acts along **-body-z**:

```
f_thrust_body = [0, 0, -T]
```

When the vehicle is level (identity orientation), -body-z points along -NED-z,
i.e. upward. `hover_thrust() = mass * gravity` is the collective thrust that
exactly balances gravity for a level vehicle.

## Specific force / accelerometer (for the Phase 3 IMU model)

The accelerometer measures specific force, not kinematic acceleration:

```
accel_reading_body = R(q)^T * (a_ned - g_ned)
```

with `g_ned = [0, 0, +g]`. In a perfect hover (`a_ned = 0`) this reads
`[0, 0, -g]` in the body frame: 1 g pointing "up" in body coordinates (the
reaction that supports the vehicle). Any Phase 3 IMU model must reproduce this,
or the EKF will diverge at the first hover check.

## Drag

Quadratic aerodynamic drag, applied to the air-relative ground velocity:

```
v_rel = v_ground_ned - v_wind_ned
F_drag_ned = -0.5 * rho * Cd * A * ||v_rel|| * v_rel
```

with `rho = 1.225 kg/m^3`, `Cd = DRAG_COEFF`, `A = VEHICLE_AREA`. The force is
divided by mass inside the translational dynamics.

## Wind model (Phase 1)

`DrydenWind` is a per-axis first-order Gauss-Markov process (discretized
MIL-F-8785C / Beard & McLain shaping filter):

```
w[k+1] = exp(-Va*dt/L) * w[k] + sigma * sqrt(1 - exp(-2*Va*dt/L)) * N(0, 1)
```

Phase 1 simplification: the turbulence axes are aligned with NED (u along North,
v along East, w along Down). The generator is `std::mt19937` with a fixed seed,
so sequences are reproducible on a given platform.

## Motor model

- X-configuration mixing, with `l = arm_length / sqrt(2)` and
  `c = MOTOR_KM / MOTOR_KF = 0.015 m` (the prop-drag yaw moment arm,
  realistic for a 15 cm quadrotor):

```
[T ]   [  1   1   1   1 ] [f1]
[Mx] = [ -l  -l   l   l ] [f2]
[My]   [ -l   l  -l   l ] [f3]
[Mz]   [ -c   c   c  -c ] [f4]
```

- Motor lag is first order with time constant `tau = MOTOR_TIME_CONST = 0.02 s`,
  integrated with the exact exponential update over `dt`:

```
f_actual += (f_cmd - f_actual) * (1 - exp(-dt / tau))
```

- Actual forces are clamped to `[0, MOTOR_KF * M_MAX^2]`: motors cannot
  produce negative thrust or exceed max RPM. The clamp accepts the
  resulting moment error rather than redistributing it (real-actuator
  behavior).

## Integration

- `propagate` performs one RK4 step of the 13-state vector
  `[position(3), quaternion(4), velocity(3), angular_rate(3)]` at `DT = 0.005 s`
  (200 Hz).
- The quaternion is renormalized at the end of every step.
- Control runs at `DT_CONTROL = 0.01 s`, GPS at `DT_GPS = 0.1 s`, IMU at
  `DT_IMU = 0.005 s`.

## Units

SI everywhere: m, m/s, rad, rad/s, N, N*m, kg, kg*m^2, s. Angles are radians;
GPS lat/lon (later phases) are degrees. Motor RPM (`M_MAX`) appears only at the
motor-layer boundary; dynamics use forces in newtons.
