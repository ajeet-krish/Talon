# Talon Research

Last updated: 2026-10-17

Domain notes behind the kernel and the analysis. These are reference findings,
not project decisions (those live in docs/decisions.md).

## Estimation: multiplicative error-state Kalman filter

The 15-state MEKF (`core/include/talon/ekf.hpp`) follows Trawny and Roumeliotis
(2005) and Sola (2017): the attitude error is a 3-vector in the local tangent
space, injected multiplicatively (global) or additively (local), so the
covariance stays minimal and singular-free. A chi-squared gate on the
normalized innovation squared rejects outlier 10 Hz GPS fixes; divergence is
detected from the trace of the position covariance.

## Guidance: nonlinear line-of-sight (NLGL)

The path follower (`python/talon/guidance.py`) implements the NLGL law of Park,
Deyst and How (2004, 2007) in the scalar ArduPilot "clipped" form. The course
error Nu2 is measured from the estimated ground-velocity vector, so the wind
crab angle cancels exactly and the law is wind-immune. Clipping the total
correction to +/-90 deg removes the wrong-way equilibrium that the raw Nu1 +
Nu2 sum has at path corners.

## Wind: Dryden turbulence spectrum

`core/include/talon/wind.hpp` generates turbulence from the Dryden spectral
form (MIL-F-8785C), shaped by first-order filters driven with unit-variance
noise. Severity and scale lengths follow the low-altitude military
specification used in Beard and McLain (2012).

## Control: cascaded PD with mixer and motor lag

The autopilot (`core/include/talon/controller.hpp`) is a 100 Hz cascaded PD:
an outer altitude and velocity loop produces attitude and thrust setpoints, an
inner attitude and rate loop tracks them, and the mixer saturates thrust and
torque given the X-frame geometry. The truth plant adds a first-order motor
lag so the closed loop sees a realistic actuator bandwidth.

## Avoidance: leg-sphere detour planning

`python/talon/avoidance.py` plans a deterministic one-detour-per-leg path
around spherical keep-out zones: it intersects the leg with each sphere,
offsets the closest approach to the configured lateral margin plus radius, and
inserts the detour point. `min_clearance` returns the minimum 3-D surface
clearance, which the validation driver asserts is positive.

## References

1. Trawny, N. and Roumeliotis, S. I. "Indirect Kalman Filter for 3D Attitude
   Estimation." University of Minnesota, GRASP Laboratory, 2005.
2. Sola, J. "Quaternion kinematics for the error-state Kalman filter."
   arXiv:1711.02508, 2017.
3. Park, S., Deyst, J. and How, J. P. "Performance and Lyapunov stability of a
   nonlinear path-following guidance law." AIAA Journal of Guidance, Control,
   and Dynamics, 2007 (2004 conference precursor).
4. Beard, R. W. and McLain, T. W. Small Unmanned Aircraft: Theory and
   Practice. Princeton University Press, 2012.
5. MIL-F-8785C. Military Specification: Flying Qualities of Piloted Airplanes.
6. Quan, Q. Introduction to Multicopter Design and Control. Springer, 2017.
