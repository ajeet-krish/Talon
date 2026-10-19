//! Motor mixing and first-order motor response.
//!
//! X-configuration mixing, ported from the previous Python source
//! (`motors.py`):
//!
//! ```text
//! [T ]   [  1   1   1   1 ] [f1]
//! [Mx] = [ -l  -l   l   l ] [f2]
//! [My]   [ -l   l  -l   l ] [f3]
//! [Mz]   [ -c   c   c  -c ] [f4]
//! ```
//!
//! with `l = arm_length / sqrt(2)` and `c = MOTOR_KM / MOTOR_KF = 0.015 m`, the
//! prop-drag yaw moment arm implied by the motor coefficients and realistic for
//! a 15 cm quadrotor.
//!
//! The inverse of this matrix is taken once with nalgebra rather than written
//! out by hand, because an algebra slip in four hand-derived entries would be
//! silently wrong. A test pins `MIX * MIX_INVERSE = I` and a round-trip test
//! pins `forward_mix(motor_mixing(w)) = w`.

use std::sync::OnceLock;

use nalgebra::{SMatrix, SVector};

use crate::constants::{ARM_LENGTH, MOTOR_KF, MOTOR_KM, MOTOR_TIME_CONST, M_MAX};
use crate::math;

/// Per-motor force floor (N): motors cannot produce negative thrust.
pub const MOTOR_FORCE_MIN: f64 = 0.0;
/// Per-motor force ceiling (N): `kf * M_MAX^2`.
pub const MOTOR_FORCE_MAX: f64 = MOTOR_KF * M_MAX * M_MAX;

/// Roll/pitch moment arm, `arm_length / sqrt(2)`.
pub fn mix_l() -> f64 {
    ARM_LENGTH / math::sqrt(2.0)
}

/// Yaw moment arm, `MOTOR_KM / MOTOR_KF`.
pub fn mix_c() -> f64 {
    MOTOR_KM / MOTOR_KF
}

/// The forward mix matrix, `wrench = MIX * forces`.
pub fn mix_matrix() -> SMatrix<f64, 4, 4> {
    let l = mix_l();
    let c = mix_c();
    SMatrix::<f64, 4, 4>::from_row_slice(&[
        1.0, 1.0, 1.0, 1.0, //
        -l, -l, l, l, //
        -l, l, -l, l, //
        -c, c, c, -c,
    ])
}

/// The inverse mix matrix, computed once.
fn mix_inverse() -> &'static SMatrix<f64, 4, 4> {
    static INV: OnceLock<SMatrix<f64, 4, 4>> = OnceLock::new();
    INV.get_or_init(|| {
        mix_matrix()
            .try_inverse()
            .expect("the X-configuration mix matrix is invertible")
    })
}

/// Inverse mix: `(collective thrust, body moment) -> 4 motor forces`.
///
/// May return forces outside `[MOTOR_FORCE_MIN, MOTOR_FORCE_MAX]`.
/// [`MotorModel`] clamps to that range, so motors cannot produce negative or
/// unlimited thrust, and accepts the resulting moment error like a real
/// actuator.
pub fn motor_mixing(thrust: f64, moment: [f64; 3]) -> [f64; 4] {
    let wrench = SVector::<f64, 4>::new(thrust, moment[0], moment[1], moment[2]);
    let f = mix_inverse() * wrench;
    [f[0], f[1], f[2], f[3]]
}

/// Forward mix: `4 motor forces -> (collective thrust, body moment)`.
pub fn forward_mix(motor_forces: [f64; 4]) -> (f64, [f64; 3]) {
    let f = SVector::<f64, 4>::new(
        motor_forces[0],
        motor_forces[1],
        motor_forces[2],
        motor_forces[3],
    );
    let w = mix_matrix() * f;
    (w[0], [w[1], w[2], w[3]])
}

/// Why a motor model could not be constructed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotorError {
    /// The lag time constant must be strictly positive.
    NonPositiveTimeConstant(f64),
}

impl std::fmt::Display for MotorError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NonPositiveTimeConstant(v) => {
                write!(f, "time_const must be positive, got {v}")
            }
        }
    }
}

impl std::error::Error for MotorError {}

/// Four first-order lag motor responses.
///
/// Each motor force relaxes toward its commanded value with the exact
/// exponential update over `dt`, then clamps to
/// `[MOTOR_FORCE_MIN, MOTOR_FORCE_MAX]`:
///
/// ```text
/// f += (f_cmd - f) * (1 - exp(-dt / tau))
/// ```
#[derive(Debug, Clone)]
pub struct MotorModel {
    time_const: f64,
    forces: [f64; 4],
}

impl MotorModel {
    /// Build a motor model with the given lag time constant (s).
    pub fn new(time_const: f64) -> Result<Self, MotorError> {
        // NaN is rejected on purpose: !(NaN > 0.0) was true, and
        // `time_const <= 0.0` alone would accept a NaN lag.
        if time_const <= 0.0 || time_const.is_nan() {
            return Err(MotorError::NonPositiveTimeConstant(time_const));
        }
        Ok(Self {
            time_const,
            forces: [0.0; 4],
        })
    }

    /// Build with the default `MOTOR_TIME_CONST`.
    pub fn with_defaults() -> Self {
        Self::new(MOTOR_TIME_CONST).expect("MOTOR_TIME_CONST is positive")
    }

    /// Return all motor forces to zero.
    pub fn reset(&mut self) {
        self.forces = [0.0; 4];
    }

    /// Advance one step and return the actual, lagged motor forces.
    ///
    /// A negative `dt` is treated as zero rather than panicking: the control
    /// loop must not be able to take the vehicle down by arithmetic.
    pub fn step(&mut self, thrust: f64, moment: [f64; 3], dt: f64) -> [f64; 4] {
        let dt = if dt > 0.0 { dt } else { 0.0 };
        let commanded = motor_mixing(thrust, moment);
        let alpha = 1.0 - math::exp(-dt / self.time_const);
        for (force, cmd) in self.forces.iter_mut().zip(commanded) {
            *force += (cmd - *force) * alpha;
            *force = math::clamp(*force, MOTOR_FORCE_MIN, MOTOR_FORCE_MAX);
        }
        self.forces
    }

    /// Current actual motor forces (N).
    pub fn forces(&self) -> [f64; 4] {
        self.forces
    }

    /// The lag time constant (s).
    pub fn time_const(&self) -> f64 {
        self.time_const
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::GRAVITY;
    use crate::dynamics::hover_thrust;

    #[test]
    fn the_inverse_mix_actually_inverts_the_mix() {
        let identity = mix_matrix() * mix_inverse();
        for i in 0..4 {
            for j in 0..4 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (identity[(i, j)] - expected).abs() < 1e-9,
                    "MIX * MIX_INV[{i},{j}] = {}",
                    identity[(i, j)]
                );
            }
        }
    }

    #[test]
    fn mixing_round_trips_through_the_forward_mix() {
        let cases = [
            (hover_thrust(), [0.0, 0.0, 0.0]),
            (12.0, [0.05, -0.02, 0.01]),
            (16.0, [-0.1, 0.1, -0.02]),
        ];
        for (t, m) in cases {
            let forces = motor_mixing(t, m);
            let (t2, m2) = forward_mix(forces);
            assert!((t2 - t).abs() < 1e-9, "{t} -> {t2}");
            for i in 0..3 {
                assert!((m2[i] - m[i]).abs() < 1e-9, "{m:?} -> {m2:?}");
            }
        }
    }

    #[test]
    fn hover_gives_four_equal_forces_that_sum_to_the_thrust() {
        let t = hover_thrust();
        let forces = motor_mixing(t, [0.0, 0.0, 0.0]);
        for f in forces {
            assert!((f - t / 4.0).abs() < 1e-9, "{forces:?}");
        }
        let (back, _) = forward_mix(forces);
        assert!((back - t).abs() < 1e-9);
    }

    #[test]
    fn a_roll_command_differentiates_the_left_and_right_motors() {
        // Positive roll moment: the Mx row is (-l, -l, +l, +l), so motors 2
        // and 3 must carry more force than motors 0 and 1.
        let forces = motor_mixing(0.0, [0.1, 0.0, 0.0]);
        let left = forces[0] + forces[1];
        let right = forces[2] + forces[3];
        assert!(right > left, "right side should lift: {forces:?}");
    }

    #[test]
    fn a_yaw_command_differentiates_by_the_drag_arm() {
        // Positive yaw moment: the Mz row is (-c, +c, +c, -c).
        let forces = motor_mixing(0.0, [0.0, 0.0, 0.1]);
        assert!(forces[0] < forces[1], "{forces:?}");
        assert!(forces[2] > forces[3], "{forces:?}");
    }

    #[test]
    fn the_force_ceiling_matches_the_motor_limit() {
        // kf * M_MAX^2, which is the physical ceiling quoted in the source.
        let expected = MOTOR_KF * M_MAX * M_MAX;
        assert!((MOTOR_FORCE_MAX - expected).abs() < 1e-9);
        // And it is far above the hover thrust per motor, so it is not binding
        // in normal flight.
        assert!(MOTOR_FORCE_MAX > 10.0 * hover_thrust() / 4.0);
    }

    #[test]
    fn forces_never_go_negative_even_for_a_negative_command() {
        let mut m = MotorModel::with_defaults();
        for _ in 0..1000 {
            m.step(-50.0, [0.0, 0.0, 0.0], 0.005);
        }
        for f in m.forces() {
            assert!(f >= MOTOR_FORCE_MIN, "negative thrust: {f}");
        }
    }

    #[test]
    fn forces_never_exceed_the_ceiling() {
        let mut m = MotorModel::with_defaults();
        for _ in 0..1000 {
            m.step(100_000.0, [0.0, 0.0, 0.0], 0.005);
        }
        for f in m.forces() {
            assert!(f <= MOTOR_FORCE_MAX, "over the ceiling: {f}");
        }
    }

    #[test]
    fn the_lag_step_matches_the_exponential_closed_form() {
        // One step of length tau reaches 1 - 1/e of the command.
        let mut m = MotorModel::new(0.02).unwrap();
        let t = 40.0; // 10 N per motor, inside the clamp
        let forces = m.step(t, [0.0, 0.0, 0.0], 0.02);
        let expected = (t / 4.0) * (1.0 - 1.0 / std::f64::consts::E);
        for f in forces {
            assert!((f - expected).abs() < 1e-9, "{f} vs {expected}");
        }
    }

    #[test]
    fn the_lag_converges_to_the_command() {
        let mut m = MotorModel::with_defaults();
        let t = hover_thrust();
        for _ in 0..2000 {
            m.step(t, [0.0, 0.0, 0.0], 0.005);
        }
        for f in m.forces() {
            assert!((f - t / 4.0).abs() < 1e-6, "{f}");
        }
    }

    #[test]
    fn a_zero_length_step_does_not_move_the_forces() {
        let mut m = MotorModel::new(0.02).unwrap();
        m.step(hover_thrust(), [0.0, 0.0, 0.0], 0.02);
        let before = m.forces();
        let after = m.step(0.0, [0.0, 0.0, 0.0], 0.0);
        assert_eq!(before, after);
    }

    #[test]
    fn a_negative_step_is_treated_as_zero() {
        let mut m = MotorModel::new(0.02).unwrap();
        m.step(hover_thrust(), [0.0, 0.0, 0.0], 0.02);
        let before = m.forces();
        let after = m.step(0.0, [0.0, 0.0, 0.0], -1.0);
        assert_eq!(before, after);
        assert!(after.iter().all(|f| f.is_finite()));
    }

    #[test]
    fn reset_zeroes_the_forces() {
        let mut m = MotorModel::with_defaults();
        m.step(hover_thrust(), [0.0, 0.0, 0.0], 0.5);
        assert!(m.forces().iter().any(|f| *f > 0.0));
        m.reset();
        assert_eq!(m.forces(), [0.0; 4]);
    }

    #[test]
    fn a_non_positive_time_constant_is_rejected() {
        assert!(matches!(
            MotorModel::new(0.0),
            Err(MotorError::NonPositiveTimeConstant(_))
        ));
        assert!(MotorModel::new(-0.02).is_err());
    }

    #[test]
    fn a_nan_time_constant_is_rejected_but_a_normal_one_is_accepted() {
        // Regression: `time_const <= 0.0` alone would accept a NaN lag, so
        // the guard carries an explicit `is_nan()` branch. These cases stay
        // valid when the guard later tightens to `!time_const.is_finite()`.
        for bad in [f64::NAN, 0.0, -0.02] {
            assert!(
                matches!(
                    MotorModel::new(bad),
                    Err(MotorError::NonPositiveTimeConstant(_))
                ),
                "time_const {bad} should be rejected"
            );
        }
        let m = MotorModel::new(0.02).expect("a normal positive time constant is accepted");
        assert_eq!(m.time_const(), 0.02);
    }

    #[test]
    fn hovering_motors_reproduce_the_hover_thrust() {
        // End-to-end sanity: drive the motors to steady state at hover and
        // check the forward mix gives back mass * g.
        let mut m = MotorModel::with_defaults();
        let t = hover_thrust();
        for _ in 0..2000 {
            m.step(t, [0.0, 0.0, 0.0], 0.005);
        }
        let (thrust, moment) = forward_mix(m.forces());
        assert!(
            (thrust - crate::constants::MASS * GRAVITY).abs() < 1e-4,
            "{thrust}"
        );
        assert!(moment.iter().all(|c| c.abs() < 1e-6), "{moment:?}");
    }
}
