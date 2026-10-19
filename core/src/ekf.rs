//! 15-state multiplicative error-state Kalman filter (MEKF) for INS/GPS
//! navigation.
//!
//! Ported term for term from the previous C++ core (`ekf.hpp`), which follows
//! Trawny and Roumeliotis (2005) and Sola (2017), "Quaternion kinematics for
//! the error-state Kalman filter". Eigen's fixed-size matrices are replaced by
//! nalgebra `SMatrix` with the same compile-time dimensions, so a dimension
//! mistake is a compile error rather than a runtime surprise.
//!
//! Error-state ordering, fixed and not to be reordered without updating `H`,
//! the injection, and the tests:
//!
//! ```text
//! dx = [dp(3), dv(3), dtheta(3), dbg(3), dba(3)]
//!   dp      position error, NED (m)
//!   dv      velocity error, NED (m/s)
//!   dtheta  attitude error, body-frame right-multiplicative (rad)
//!   dbg     gyro bias error (rad/s, body)
//!   dba     accel bias error (m/s^2, body)
//! ```
//!
//! Attitude error convention: `q_true = q_est (x) dq` with `dq` the small
//! rotation, so the attitude error is expressed in the estimated body frame.
//! Nominal attitude increments are therefore right-multiplied, matching the
//! controller's body-rate convention.
//!
//! Gravity: the accelerometer measures specific force, so `g` is added exactly
//! once in [`Ekf::predict`] as `a_ned = R * f_body + g_ned`.

use nalgebra::{SMatrix, SVector};

use crate::constants::GRAVITY;
use crate::dynamics::{quat_multiply, quat_normalize, rotation_matrix};
use crate::math;
use crate::types::{Mat3, Quaternion, Vec3};

/// Error-state and covariance dimensions.
pub type Matrix15 = SMatrix<f64, 15, 15>;
pub type Matrix6x15 = SMatrix<f64, 6, 15>;
/// Kalman gain shape.
pub type Matrix15x6 = SMatrix<f64, 15, 6>;
pub type Vector15 = SVector<f64, 15>;
pub type Vector6 = SVector<f64, 6>;
/// 6x6 GPS measurement covariance.
pub type Mat6 = SMatrix<f64, 6, 6>;

/// Noise densities, initial covariances and the innovation gate.
#[derive(Debug, Clone, Copy)]
pub struct EkfConfig {
    /// Accelerometer noise density, m/s^2/sqrt(Hz).
    pub accel_noise_density: f64,
    /// Gyro noise density, rad/s/sqrt(Hz).
    pub gyro_noise_density: f64,
    /// Accelerometer bias random walk, m/s^2/sqrt(s).
    pub accel_bias_walk: f64,
    /// Gyro bias random walk, rad/s/sqrt(s).
    pub gyro_bias_walk: f64,
    /// GPS horizontal position 1-sigma, m.
    pub gps_pos_noise: f64,
    /// GPS vertical position 1-sigma, m.
    pub gps_pos_noise_vertical: f64,
    /// GPS velocity 1-sigma, m/s.
    pub gps_vel_noise: f64,
    /// Innovation gate, roughly 5-sigma over 6 degrees of freedom.
    pub innovation_gate_chi2: f64,
    /// Initial position standard deviation, m.
    pub init_pos_std: f64,
    /// Initial velocity standard deviation, m/s.
    pub init_vel_std: f64,
    /// Initial attitude standard deviation, rad.
    pub init_att_std: f64,
    /// Initial gyro bias standard deviation, rad/s.
    pub init_gyro_bias_std: f64,
    /// Initial accel bias standard deviation, m/s^2.
    pub init_accel_bias_std: f64,
}

impl Default for EkfConfig {
    fn default() -> Self {
        Self {
            accel_noise_density: 0.004,
            gyro_noise_density: 0.003,
            accel_bias_walk: 5e-4,
            gyro_bias_walk: 1e-5,
            gps_pos_noise: 1.5,
            gps_pos_noise_vertical: 3.0,
            gps_vel_noise: 0.1,
            innovation_gate_chi2: 25.0,
            init_pos_std: 5.0,
            init_vel_std: 1.0,
            init_att_std: 0.1,
            init_gyro_bias_std: 0.01,
            init_accel_bias_std: 0.1,
        }
    }
}

/// Nominal (error-free) MEKF state. Positions and velocities in NED, attitude
/// as a Hamilton body-to-NED quaternion, biases in the body frame.
#[derive(Debug, Clone, Copy)]
pub struct EkfNominalState {
    pub position: Vec3,
    pub velocity: Vec3,
    pub attitude: Quaternion,
    pub gyro_bias: Vec3,
    pub accel_bias: Vec3,
}

impl Default for EkfNominalState {
    fn default() -> Self {
        Self {
            position: Vec3::zeros(),
            velocity: Vec3::zeros(),
            attitude: Quaternion::default(),
            gyro_bias: Vec3::zeros(),
            accel_bias: Vec3::zeros(),
        }
    }
}

/// Skew-symmetric cross-product matrix, the inverse of [`crate::controller::vee`].
pub fn skew(v: &Vec3) -> Mat3 {
    Mat3::from_row_slice(&[0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0])
}

/// Rotation vector to unit quaternion, `exp_quat(phi)` with `phi` the full
/// rotation vector (`angle * axis`). Falls back to the series `[1, phi/2]`
/// for near-zero angles.
pub fn exp_quat(phi: &Vec3) -> Quaternion {
    let angle = phi.norm();
    if angle < 1e-8 {
        return Quaternion::new(1.0, 0.5 * phi[0], 0.5 * phi[1], 0.5 * phi[2]).normalized();
    }
    let half = 0.5 * angle;
    let s = math::sin(half) / angle;
    Quaternion::new(math::cos(half), s * phi[0], s * phi[1], s * phi[2])
}

/// 15-state MEKF.
///
/// Call [`Ekf::predict`] every `DT_IMU` (0.005 s) with raw IMU samples and
/// [`Ekf::update`] every `DT_GPS` (0.1 s) with a GPS position and velocity in
/// NED.
#[derive(Debug, Clone)]
pub struct Ekf {
    config: EkfConfig,
    nominal: EkfNominalState,
    p: Matrix15,
    last_innovation: Vector6,
    last_innovation_chi2: f64,
    last_update_accepted: bool,
}

impl Ekf {
    /// Build a filter with the given configuration. Initialize before use.
    pub fn new(config: EkfConfig) -> Self {
        Self {
            config,
            nominal: EkfNominalState::default(),
            p: Matrix15::zeros(),
            last_innovation: Vector6::zeros(),
            last_innovation_chi2: 0.0,
            last_update_accepted: true,
        }
    }

    /// Build a filter with the default (ADIS16448-class) configuration.
    pub fn with_defaults() -> Self {
        Self::new(EkfConfig::default())
    }

    /// Set the nominal state and the diagonal initial covariance.
    pub fn initialize(&mut self, initial: EkfNominalState) {
        self.nominal = EkfNominalState {
            attitude: initial.attitude.normalized(),
            ..initial
        };
        self.p = Matrix15::zeros();
        let d = &self.config;
        self.p
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&(Mat3::identity() * (d.init_pos_std * d.init_pos_std)));
        self.p
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(Mat3::identity() * (d.init_vel_std * d.init_vel_std)));
        self.p
            .fixed_view_mut::<3, 3>(6, 6)
            .copy_from(&(Mat3::identity() * (d.init_att_std * d.init_att_std)));
        self.p
            .fixed_view_mut::<3, 3>(9, 9)
            .copy_from(&(Mat3::identity() * (d.init_gyro_bias_std * d.init_gyro_bias_std)));
        self.p
            .fixed_view_mut::<3, 3>(12, 12)
            .copy_from(&(Mat3::identity() * (d.init_accel_bias_std * d.init_accel_bias_std)));
        self.reset_telemetry();
    }

    /// Initialize level (zero roll and pitch) from the assumed stationary
    /// specific force `[0, 0, -g]`, with the given yaw.
    pub fn initialize_level(&mut self, position: Vec3, velocity: Vec3, yaw: f64) {
        self.initialize_from_accel(position, velocity, Vec3::new(0.0, 0.0, -GRAVITY), yaw);
    }

    /// Initialize, leveling roll and pitch from a measured stationary specific
    /// force. A level vehicle reads `accel_body ~ [0, 0, -g]`, and
    /// `pitch = asin(ax / g)`, `roll = atan2(-ay, -az)`.
    pub fn initialize_from_accel(
        &mut self,
        position: Vec3,
        velocity: Vec3,
        accel_body: Vec3,
        yaw: f64,
    ) {
        let ax_g = math::clamp(accel_body[0] / GRAVITY, -1.0, 1.0);
        let pitch = math::asin(ax_g);
        let roll = math::atan2(-accel_body[1], -accel_body[2]);
        let qx = Quaternion::new(math::cos(0.5 * roll), math::sin(0.5 * roll), 0.0, 0.0);
        let qy = Quaternion::new(math::cos(0.5 * pitch), 0.0, math::sin(0.5 * pitch), 0.0);
        let qz = Quaternion::new(math::cos(0.5 * yaw), 0.0, 0.0, math::sin(0.5 * yaw));
        let attitude = quat_normalize(&quat_multiply(&quat_multiply(&qz, &qy), &qx));
        self.initialize(EkfNominalState {
            position,
            velocity,
            attitude,
            ..Default::default()
        });
    }

    /// IMU propagation. `accel_meas` is the specific force in body frame
    /// (a hover reads `[0, 0, -g]`); `gyro_meas` is the body angular rate.
    pub fn predict(&mut self, accel_meas: &Vec3, gyro_meas: &Vec3, dt: f64) {
        let omega = gyro_meas - self.nominal.gyro_bias;
        let f_body = accel_meas - self.nominal.accel_bias;
        let r = rotation_matrix(&self.nominal.attitude);

        // Nominal translation: specific force plus gravity, added once.
        let a_ned = r * f_body + Vec3::new(0.0, 0.0, GRAVITY);
        self.nominal.position += self.nominal.velocity * dt + 0.5 * a_ned * dt * dt;
        self.nominal.velocity += a_ned * dt;

        // Nominal attitude: body-frame increment, right-multiplied.
        self.nominal.attitude = quat_normalize(&quat_multiply(
            &self.nominal.attitude,
            &exp_quat(&(omega * dt)),
        ));

        // Error-state Jacobian F = I + A*dt, first order.
        let mut f = Matrix15::identity();
        f.fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&(Mat3::identity() * dt));
        f.fixed_view_mut::<3, 3>(3, 6)
            .copy_from(&(-r * skew(&f_body) * dt));
        f.fixed_view_mut::<3, 3>(3, 12).copy_from(&(-r * dt));
        f.fixed_view_mut::<3, 3>(6, 6)
            .copy_from(&(Mat3::identity() - skew(&omega) * dt));
        f.fixed_view_mut::<3, 3>(6, 9)
            .copy_from(&(-Mat3::identity() * dt));

        // Discrete process noise.
        let sa2 = self.config.accel_noise_density * self.config.accel_noise_density;
        let sg2 = self.config.gyro_noise_density * self.config.gyro_noise_density;
        let sbg2 = self.config.gyro_bias_walk * self.config.gyro_bias_walk;
        let sba2 = self.config.accel_bias_walk * self.config.accel_bias_walk;

        let mut qd = Matrix15::zeros();
        let cross = Mat3::identity() * (0.5 * sa2 * dt * dt);
        qd.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&(Mat3::identity() * (0.25 * sa2 * dt * dt * dt)));
        qd.fixed_view_mut::<3, 3>(0, 3).copy_from(&cross);
        qd.fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&cross.transpose());
        qd.fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(Mat3::identity() * (sa2 * dt)));
        qd.fixed_view_mut::<3, 3>(6, 6)
            .copy_from(&(Mat3::identity() * (sg2 * dt)));
        qd.fixed_view_mut::<3, 3>(9, 9)
            .copy_from(&(Mat3::identity() * (sbg2 * dt)));
        qd.fixed_view_mut::<3, 3>(12, 12)
            .copy_from(&(Mat3::identity() * (sba2 * dt)));

        self.p = f * self.p * f.transpose() + qd;
        self.p = 0.5 * (self.p + self.p.transpose());
    }

    /// GPS update. Returns `true` when accepted and `false` when gated out, in
    /// which case neither the state nor the covariance changes.
    pub fn update(&mut self, gps_position: &Vec3, gps_velocity: &Vec3) -> bool {
        let mut y = Vector6::zeros();
        y.fixed_rows_mut::<3>(0)
            .copy_from(&(gps_position - self.nominal.position));
        y.fixed_rows_mut::<3>(3)
            .copy_from(&(gps_velocity - self.nominal.velocity));

        let mut h = Matrix6x15::zeros();
        h.fixed_view_mut::<6, 6>(0, 0).copy_from(&Mat6::identity());

        let c = &self.config;
        let mut r_meas = Mat6::zeros();
        r_meas[(0, 0)] = c.gps_pos_noise * c.gps_pos_noise;
        r_meas[(1, 1)] = c.gps_pos_noise * c.gps_pos_noise;
        r_meas[(2, 2)] = c.gps_pos_noise_vertical * c.gps_pos_noise_vertical;
        r_meas[(3, 3)] = c.gps_vel_noise * c.gps_vel_noise;
        r_meas[(4, 4)] = c.gps_vel_noise * c.gps_vel_noise;
        r_meas[(5, 5)] = c.gps_vel_noise * c.gps_vel_noise;

        let s = h * self.p * h.transpose() + r_meas;
        // R sits on the diagonal of S, so S is symmetric positive definite and
        // well conditioned. nalgebra 0.33 exposes `try_inverse` rather than a
        // panicking `inverse`; the failure case cannot occur here.
        let s_inv = s
            .try_inverse()
            .expect("S = H P H^T + R is positive definite because R is diagonal positive");
        let chi2 = y.dot(&(s_inv * y));
        self.last_innovation = y;
        self.last_innovation_chi2 = chi2;

        // Gate before applying: a rejected fix leaves the estimate untouched.
        if chi2 > c.innovation_gate_chi2 {
            self.last_update_accepted = false;
            return false;
        }

        let k: Matrix15x6 = self.p * h.transpose() * s_inv;
        let dx: Vector15 = k * y;

        let dp = dx.fixed_rows::<3>(0).into_owned();
        let dv = dx.fixed_rows::<3>(3).into_owned();
        let dtheta = dx.fixed_rows::<3>(6).into_owned();
        let dbg = dx.fixed_rows::<3>(9).into_owned();
        let dba = dx.fixed_rows::<3>(12).into_owned();

        self.nominal.position += dp;
        self.nominal.velocity += dv;
        self.nominal.attitude =
            quat_normalize(&quat_multiply(&self.nominal.attitude, &exp_quat(&dtheta)));
        self.nominal.gyro_bias += dbg;
        self.nominal.accel_bias += dba;

        // Joseph form keeps the covariance symmetric and positive definite.
        let g = Matrix15::identity() - k * h;
        self.p = g * self.p * g.transpose() + k * r_meas * k.transpose();
        self.p = 0.5 * (self.p + self.p.transpose());

        self.last_update_accepted = true;
        true
    }

    /// Nominal state.
    pub fn nominal(&self) -> &EkfNominalState {
        &self.nominal
    }

    /// Current covariance.
    pub fn covariance(&self) -> Matrix15 {
        self.p
    }

    /// Last innovation vector.
    pub fn last_innovation(&self) -> Vector6 {
        self.last_innovation
    }

    /// Last innovation chi-square statistic.
    pub fn last_innovation_chi2(&self) -> f64 {
        self.last_innovation_chi2
    }

    /// Whether the last update was accepted.
    pub fn last_update_accepted(&self) -> bool {
        self.last_update_accepted
    }

    /// Mutable nominal state, the fault-injection hook.
    pub fn mutable_nominal(&mut self) -> &mut EkfNominalState {
        &mut self.nominal
    }

    /// Mutable covariance, the fault-injection hook.
    pub fn mutable_covariance(&mut self) -> &mut Matrix15 {
        &mut self.p
    }

    /// Estimated body rates for the last gyro sample: `gyro_meas - gyro_bias`.
    pub fn estimated_body_rates(&self, gyro_meas: &Vec3) -> Vec3 {
        gyro_meas - self.nominal.gyro_bias
    }

    fn reset_telemetry(&mut self) {
        self.last_innovation = Vector6::zeros();
        self.last_innovation_chi2 = 0.0;
        self.last_update_accepted = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::to_radians;

    fn level_ekf() -> Ekf {
        let mut ekf = Ekf::with_defaults();
        ekf.initialize_level(Vec3::zeros(), Vec3::zeros(), 0.0);
        ekf
    }

    const HOVER_ACCEL: Vec3 = Vec3::new(0.0, 0.0, -GRAVITY);

    #[test]
    fn initial_covariance_is_diagonal_and_sized() {
        let ekf = level_ekf();
        let p = ekf.covariance();
        assert!((p[(0, 0)] - 25.0).abs() < 1e-12);
        assert!((p[(3, 3)] - 1.0).abs() < 1e-12);
        assert!((p[(6, 6)] - 0.01).abs() < 1e-12);
        assert!(p[(0, 1)].abs() < 1e-15, "expected a diagonal covariance");
    }

    #[test]
    fn hover_specific_force_holds_the_estimate() {
        let mut ekf = level_ekf();
        for _ in 0..100 {
            ekf.predict(&HOVER_ACCEL, &Vec3::zeros(), 0.005);
        }
        let n = ekf.nominal();
        assert!(n.position.norm() < 1e-12, "drifted {}", n.position);
        assert!(n.velocity.norm() < 1e-12, "gained {}", n.velocity);
    }

    #[test]
    fn free_fall_integrates_gravity() {
        let mut ekf = level_ekf();
        let zero = Vec3::zeros();
        ekf.predict(&zero, &zero, 0.005);
        let n = ekf.nominal();
        // a_ned = R * 0 + g_ned = +g downward (NED down is positive z).
        assert!(
            (n.velocity[2] - GRAVITY * 0.005).abs() < 1e-12,
            "{}",
            n.velocity
        );
        assert!(n.position[2] > 0.0, "should sink");
    }

    #[test]
    fn predict_grows_the_covariance() {
        let mut ekf = level_ekf();
        let before = ekf.covariance()[(0, 0)];
        ekf.predict(&HOVER_ACCEL, &Vec3::zeros(), 0.005);
        assert!(ekf.covariance()[(0, 0)] > before);
    }

    #[test]
    fn gps_update_pulls_the_estimate_toward_the_fix() {
        let mut ekf = level_ekf();
        let fix = Vec3::new(10.0, 0.0, 0.0);
        assert!(ekf.update(&fix, &Vec3::zeros()));
        let p = ekf.nominal().position;
        assert!(p[0] > 5.0, "estimate should chase the fix, got {p}");
        assert!(
            p[0] < 10.0,
            "estimate should not overshoot the fix, got {p}"
        );
    }

    #[test]
    fn gps_update_shrinks_the_position_covariance() {
        let mut ekf = level_ekf();
        let before = ekf.covariance()[(0, 0)];
        ekf.update(&Vec3::new(10.0, 0.0, 0.0), &Vec3::zeros());
        let after = ekf.covariance()[(0, 0)];
        assert!(after < before, "P_pos grew: {before} -> {after}");
        assert!(after > 0.0, "P_pos must stay positive");
    }

    #[test]
    fn innovation_gate_rejects_an_outlier_without_touching_the_state() {
        let mut ekf = level_ekf();
        let before = ekf.covariance();
        // A 10 km jump is far outside the 5-sigma gate.
        let accepted = ekf.update(&Vec3::new(10_000.0, 0.0, 0.0), &Vec3::zeros());
        assert!(!accepted, "outlier should be gated out");
        assert!(!ekf.last_update_accepted());
        assert!(ekf.last_innovation_chi2() > 25.0);
        assert_eq!(ekf.nominal().position, Vec3::zeros());
        assert_eq!(ekf.covariance(), before, "a gated fix must not move P");
    }

    #[test]
    fn covariance_stays_symmetric_under_a_long_sequence() {
        let mut ekf = level_ekf();
        for i in 0..2000 {
            ekf.predict(&HOVER_ACCEL, &Vec3::zeros(), 0.005);
            if i % 20 == 0 {
                ekf.update(&Vec3::new(0.5, -0.3, 0.1), &Vec3::new(0.05, 0.0, 0.0));
            }
        }
        let p = ekf.covariance();
        for i in 0..15 {
            for j in 0..15 {
                assert!(
                    (p[(i, j)] - p[(j, i)]).abs() < 1e-9,
                    "asymmetry at ({i},{j})"
                );
            }
            assert!(p[(i, i)] > 0.0, "non-positive variance at {i}");
        }
    }

    #[test]
    fn attitude_stays_normalized_over_many_predicts() {
        let mut ekf = level_ekf();
        let gyro = Vec3::new(0.3, -0.2, 0.1);
        for _ in 0..5000 {
            ekf.predict(&HOVER_ACCEL, &gyro, 0.005);
        }
        assert!((ekf.nominal().attitude.norm() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn initialize_from_accel_recovers_level() {
        let mut ekf = Ekf::with_defaults();
        ekf.initialize_from_accel(Vec3::zeros(), Vec3::zeros(), HOVER_ACCEL, 0.0);
        let q = ekf.nominal().attitude;
        assert!((q.w - 1.0).abs() < 1e-12, "{q:?}");
        assert!(q.x.abs() < 1e-12 && q.y.abs() < 1e-12, "{q:?}");
    }

    #[test]
    fn initialize_from_accel_recovers_a_thirty_degree_roll() {
        // A vehicle rolled +30 deg reads specific force
        // [g sin(pitch), -g sin(roll) cos(pitch), -g cos(roll) cos(pitch)].
        let roll = to_radians(30.0);
        let accel = Vec3::new(
            GRAVITY * math::sin(0.0),
            -GRAVITY * math::sin(roll) * math::cos(0.0),
            -GRAVITY * math::cos(roll) * math::cos(0.0),
        );
        let mut ekf = Ekf::with_defaults();
        ekf.initialize_from_accel(Vec3::zeros(), Vec3::zeros(), accel, 0.0);
        let q = ekf.nominal().attitude;
        let half = 0.5 * roll;
        assert!((q.w - math::cos(half)).abs() < 1e-9, "{q:?}");
        assert!((q.x - math::sin(half)).abs() < 1e-9, "{q:?}");
        assert!(q.y.abs() < 1e-9 && q.z.abs() < 1e-9, "{q:?}");
    }

    #[test]
    fn exp_quat_matches_the_small_angle_series() {
        let phi = Vec3::new(1e-10, 0.0, 0.0);
        let q = exp_quat(&phi);
        assert!((q.w - 1.0).abs() < 1e-12);
        assert!((q.x - 0.5e-10).abs() < 1e-18, "{q:?}");
    }

    #[test]
    fn exp_quat_of_a_known_rotation_is_exact() {
        let q = exp_quat(&Vec3::new(0.0, 0.0, to_radians(90.0)));
        let half = to_radians(45.0);
        assert!((q.w - math::cos(half)).abs() < 1e-12, "{q:?}");
        assert!((q.z - math::sin(half)).abs() < 1e-12, "{q:?}");
    }

    #[test]
    fn accel_bias_fault_injection_shifts_the_estimate() {
        // A spurious +0.1 m/s^2 body-x bias must corrupt the propagated
        // estimate: with the bias present the hover measurement no longer
        // cancels gravity exactly, so the estimate accelerates.
        let mut ekf = level_ekf();
        ekf.mutable_nominal().accel_bias = Vec3::new(0.1, 0.0, 0.0);
        for _ in 0..200 {
            ekf.predict(&HOVER_ACCEL, &Vec3::zeros(), 0.005);
        }
        assert!(
            ekf.nominal().velocity[0] < -0.05,
            "bias should push the estimate, got {}",
            ekf.nominal().velocity
        );
    }

    #[test]
    fn estimated_body_rates_subtract_the_bias() {
        let mut ekf = level_ekf();
        ekf.mutable_nominal().gyro_bias = Vec3::new(0.01, 0.0, 0.0);
        let rates = ekf.estimated_body_rates(&Vec3::new(0.11, 0.0, 0.0));
        assert!((rates[0] - 0.1).abs() < 1e-12);
    }

    #[test]
    fn compensating_bias_before_prediction_restores_hover() {
        // The fault path and the healthy path must agree when the bias is
        // removed, which is the property the sim relies on.
        let mut biased = level_ekf();
        biased.mutable_nominal().accel_bias = Vec3::new(0.1, 0.0, 0.0);
        let mut healthy = level_ekf();
        for _ in 0..100 {
            biased.predict(&HOVER_ACCEL, &Vec3::zeros(), 0.005);
            healthy.predict(&HOVER_ACCEL, &Vec3::zeros(), 0.005);
        }
        assert_ne!(biased.nominal().velocity, healthy.nominal().velocity);
        biased.mutable_nominal().accel_bias = Vec3::zeros();
        biased.reset_telemetry();
        for _ in 0..100 {
            biased.predict(&HOVER_ACCEL, &Vec3::zeros(), 0.005);
        }
        assert!(biased.nominal().position.norm() > 0.0);
    }
}
