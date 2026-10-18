//! Truth-side IMU and GPS measurement models.
//!
//! Ported from the previous Python source (`sensors.py`). These generate
//! measurements *from* a true vehicle state, so their sign conventions must
//! match `docs/conventions.md` exactly:
//!
//! * IMU accelerometer is the specific force in body frame. At hover it reads
//!   `[0, 0, -g]`, the reaction against NED gravity `[0, 0, +g]`. The caller
//!   supplies the true specific force, either from the thrust model or, robustly
//!   with drag and any acceleration, by finite-differencing the true NED
//!   velocity through [`specific_force_from_velocity`].
//! * IMU gyro is the true body rate plus a true simulated bias plus white
//!   noise. The true bias itself random-walks each IMU step, so the filter's
//!   bias-estimation path is exercised against a realistic target.
//! * Noise converts from continuous densities the standard way:
//!   `sigma_sample = density / sqrt(dt)`.
//!
//! GPS reuses the flat-Earth mapping with the origin at latitude and longitude
//! zero: north (m) is `lat * M_PER_DEG`, east is `lon * M_PER_DEG`, down is
//! `-alt`. Noise is applied in *metres* in the local NED frame before the
//! degenerate degree conversion, so the horizontal and vertical sigmas are
//! directly in metres.
//!
//! Noise draw order is load bearing for reproducibility and matches the Python
//! source: accelerometer noise, gyro noise, accelerometer bias walk, gyro bias
//! walk. The generator itself is [`crate::rng::Rng`], so the numerical sequence
//! differs from numpy's ziggurat stream while remaining statistically
//! equivalent, which is the same relationship the previous Python and
//! TypeScript implementations had.

use crate::constants::GRAVITY;
use crate::math;
use crate::rng::Rng;
use crate::types::{Arr3, Mat3, Vec3};

/// Flat-Earth metres per degree at the degenerate origin latitude.
///
/// Only the ratio matters: noise is applied in metres, converted to degrees for
/// the GPS fields, and converted straight back by [`gps_to_ned`].
pub const M_PER_DEG: f64 = 111_320.0;

/// IMU noise and bias parameters.
///
/// The `*_initial` values are the *true* biases of the simulated sensor at
/// `t = 0`; the filter's own estimates start at zero. The `*_walk` values drive
/// the per-axis random walk of the true bias, in units per `sqrt(s)`.
#[derive(Debug, Clone, Copy)]
pub struct ImuConfig {
    pub accel_noise_density: f64,
    pub gyro_noise_density: f64,
    pub accel_bias_initial: Arr3,
    pub gyro_bias_initial: Arr3,
    pub accel_bias_walk: f64,
    pub gyro_bias_walk: f64,
}

impl Default for ImuConfig {
    fn default() -> Self {
        Self {
            accel_noise_density: 0.004,
            gyro_noise_density: 0.003,
            accel_bias_initial: [0.05, -0.03, 0.02],
            gyro_bias_initial: [0.001, -0.002, 0.0015],
            accel_bias_walk: 5e-4,
            gyro_bias_walk: 1e-5,
        }
    }
}

/// GPS noise parameters, 1-sigma in metres and m/s in local NED.
#[derive(Debug, Clone, Copy)]
pub struct GpsConfig {
    pub position_noise: f64,
    pub position_noise_vertical: f64,
    pub velocity_noise: f64,
    pub update_rate: f64,
}

impl Default for GpsConfig {
    fn default() -> Self {
        Self {
            position_noise: 1.5,
            position_noise_vertical: 3.0,
            velocity_noise: 0.1,
            update_rate: 10.0,
        }
    }
}

/// One IMU sample.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuMeasurement {
    pub timestamp: f64,
    /// Specific force in body frame, m/s^2.
    pub accel: Arr3,
    /// Body angular rate, rad/s.
    pub gyro: Arr3,
}

/// One GPS fix.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GpsMeasurement {
    pub timestamp: f64,
    /// Latitude in degrees.
    pub lat: f64,
    /// Longitude in degrees.
    pub lon: f64,
    /// Altitude in metres, positive up.
    pub alt: f64,
    pub vn: f64,
    pub ve: f64,
    pub vd: f64,
}

/// True body-frame specific force from the finite difference of true velocity.
///
/// `a_ned = (v_curr - v_prev) / dt`, then `f_body = R^T (a_ned - g_ned)` with
/// `R` the body-to-NED DCM. At hover this returns `[0, 0, -g]` up to the finite
/// difference error. A non-positive `dt` yields zero force rather than a panic.
pub fn specific_force_from_velocity(
    v_prev_ned: &Vec3,
    v_curr_ned: &Vec3,
    rotation_body_to_ned: &Mat3,
    dt: f64,
) -> Vec3 {
    if dt <= 0.0 {
        return Vec3::zeros();
    }
    let a_ned = (v_curr_ned - v_prev_ned) / dt;
    let g_ned = Vec3::new(0.0, 0.0, GRAVITY);
    rotation_body_to_ned.transpose() * (a_ned - g_ned)
}

/// GPS fix to local NED position `[n, e, d]` in metres, flat-Earth mapping.
pub fn gps_to_ned(gps: &GpsMeasurement) -> Arr3 {
    [gps.lat * M_PER_DEG, gps.lon * M_PER_DEG, -gps.alt]
}

/// GPS fix to NED velocity `[vn, ve, vd]` in m/s.
pub fn gps_velocity_ned(gps: &GpsMeasurement) -> Arr3 {
    [gps.vn, gps.ve, gps.vd]
}

/// Seeded IMU and GPS truth-side measurement generator.
///
/// Holds the *true* bias states, initialized from [`ImuConfig`] and
/// random-walked on every [`SensorSuite::generate_imu`] call. One instance is
/// one sensor unit, reproducible given the same seed and call sequence.
#[derive(Debug, Clone)]
pub struct SensorSuite {
    imu_config: ImuConfig,
    gps_config: GpsConfig,
    rng: Rng,
    accel_bias: Vec3,
    gyro_bias: Vec3,
    time: f64,
}

impl SensorSuite {
    /// Build a suite with the given configuration and seed.
    pub fn new(imu_config: ImuConfig, gps_config: GpsConfig, seed: u64) -> Self {
        Self {
            imu_config,
            gps_config,
            rng: Rng::new(seed),
            accel_bias: Vec3::new(
                imu_config.accel_bias_initial[0],
                imu_config.accel_bias_initial[1],
                imu_config.accel_bias_initial[2],
            ),
            gyro_bias: Vec3::new(
                imu_config.gyro_bias_initial[0],
                imu_config.gyro_bias_initial[1],
                imu_config.gyro_bias_initial[2],
            ),
            time: 0.0,
        }
    }

    /// Build a suite with the default IMU and GPS configurations.
    pub fn with_defaults(seed: u64) -> Self {
        Self::new(ImuConfig::default(), GpsConfig::default(), seed)
    }

    /// Current true accelerometer bias (m/s^2, body).
    pub fn accel_bias(&self) -> Vec3 {
        self.accel_bias
    }

    /// Current true gyro bias (rad/s, body).
    pub fn gyro_bias(&self) -> Vec3 {
        self.gyro_bias
    }

    /// Internal sensor clock (s), advanced by [`SensorSuite::generate_imu`].
    pub fn time(&self) -> f64 {
        self.time
    }

    /// One IMU sample from the true body rate and true body specific force.
    ///
    /// `accel = f_true + true_bias + (density / sqrt(dt)) * N(0, 1)`, and the
    /// same for gyro. The true biases then random-walk by
    /// `walk * sqrt(dt) * N(0, 1)`. A non-positive `dt` returns a zero sample
    /// without advancing the clock.
    pub fn generate_imu(
        &mut self,
        true_angular_rate: &Vec3,
        true_specific_force_body: Arr3,
        dt: f64,
    ) -> ImuMeasurement {
        if dt <= 0.0 {
            return ImuMeasurement {
                timestamp: self.time,
                accel: [0.0; 3],
                gyro: [0.0; 3],
            };
        }
        let f_true = Vec3::new(
            true_specific_force_body[0],
            true_specific_force_body[1],
            true_specific_force_body[2],
        );
        let accel_sigma = self.imu_config.accel_noise_density / math::sqrt(dt);
        let gyro_sigma = self.imu_config.gyro_noise_density / math::sqrt(dt);

        // Draw order is load bearing: accel noise, gyro noise, accel walk, gyro
        // walk. Changing it changes every downstream realisation.
        let accel_noise = self.rng.normal3();
        let gyro_noise = self.rng.normal3();
        let accel = f_true
            + self.accel_bias
            + Vec3::new(
                accel_sigma * accel_noise[0],
                accel_sigma * accel_noise[1],
                accel_sigma * accel_noise[2],
            );
        let gyro = true_angular_rate
            + self.gyro_bias
            + Vec3::new(
                gyro_sigma * gyro_noise[0],
                gyro_sigma * gyro_noise[1],
                gyro_sigma * gyro_noise[2],
            );

        let walk_dt = math::sqrt(dt);
        let accel_walk = self.rng.normal3();
        let gyro_walk = self.rng.normal3();
        self.accel_bias += Vec3::new(
            self.imu_config.accel_bias_walk * walk_dt * accel_walk[0],
            self.imu_config.accel_bias_walk * walk_dt * accel_walk[1],
            self.imu_config.accel_bias_walk * walk_dt * accel_walk[2],
        );
        self.gyro_bias += Vec3::new(
            self.imu_config.gyro_bias_walk * walk_dt * gyro_walk[0],
            self.imu_config.gyro_bias_walk * walk_dt * gyro_walk[1],
            self.imu_config.gyro_bias_walk * walk_dt * gyro_walk[2],
        );

        let measurement = ImuMeasurement {
            timestamp: self.time,
            accel: [accel[0], accel[1], accel[2]],
            gyro: [gyro[0], gyro[1], gyro[2]],
        };
        self.time += dt;
        measurement
    }

    /// One GPS fix from the true vehicle state, noise applied in metres.
    pub fn generate_gps(&mut self, true_position: &Vec3, true_velocity: &Vec3) -> GpsMeasurement {
        let pos_sigma = [
            self.gps_config.position_noise,
            self.gps_config.position_noise,
            self.gps_config.position_noise_vertical,
        ];
        let accel_free = self.rng.normal3();
        let vel_noise = self.rng.normal3();
        let noisy_position = [
            true_position[0] + pos_sigma[0] * accel_free[0],
            true_position[1] + pos_sigma[1] * accel_free[1],
            true_position[2] + pos_sigma[2] * accel_free[2],
        ];
        let noisy_velocity = [
            true_velocity[0] + self.gps_config.velocity_noise * vel_noise[0],
            true_velocity[1] + self.gps_config.velocity_noise * vel_noise[1],
            true_velocity[2] + self.gps_config.velocity_noise * vel_noise[2],
        ];
        GpsMeasurement {
            timestamp: self.time,
            lat: noisy_position[0] / M_PER_DEG,
            lon: noisy_position[1] / M_PER_DEG,
            alt: -noisy_position[2],
            vn: noisy_velocity[0],
            ve: noisy_velocity[1],
            vd: noisy_velocity[2],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hover_specific_force_matches_the_documented_convention() {
        // Level, not accelerating: f_body should be [0, 0, -g].
        let zero = Vec3::zeros();
        let f = specific_force_from_velocity(&zero, &zero, &Mat3::identity(), 0.005);
        assert!(f[0].abs() < 1e-12, "{f}");
        assert!(f[1].abs() < 1e-12, "{f}");
        assert!((f[2] + GRAVITY).abs() < 1e-12, "{f}");
    }

    #[test]
    fn an_accelerating_vehicle_reads_specific_force_beyond_gravity() {
        // Level, accelerating north at 2 m/s^2: f_body = [2, 0, -g].
        let v_prev = Vec3::zeros();
        let v_curr = Vec3::new(0.01, 0.0, 0.0); // 2 m/s^2 over 5 ms
        let f = specific_force_from_velocity(&v_prev, &v_curr, &Mat3::identity(), 0.005);
        assert!((f[0] - 2.0).abs() < 1e-9, "{f}");
        assert!((f[2] + GRAVITY).abs() < 1e-9, "{f}");
    }

    #[test]
    fn a_non_positive_dt_yields_zero_specific_force() {
        let v_curr = Vec3::new(1.0, 0.0, 0.0);
        let zero = Vec3::zeros();
        let f = specific_force_from_velocity(&zero, &v_curr, &Mat3::identity(), 0.0);
        assert_eq!(f, Vec3::zeros());
        let f = specific_force_from_velocity(&zero, &v_curr, &Mat3::identity(), -0.1);
        assert_eq!(f, Vec3::zeros());
    }

    #[test]
    fn gps_position_and_velocity_map_to_ned() {
        let gps = GpsMeasurement {
            timestamp: 1.0,
            lat: 0.001,
            lon: -0.002,
            alt: 12.0,
            vn: 3.0,
            ve: -1.0,
            vd: 0.5,
        };
        let ned = gps_to_ned(&gps);
        assert!((ned[0] - 0.001 * M_PER_DEG).abs() < 1e-9);
        assert!((ned[1] + 0.002 * M_PER_DEG).abs() < 1e-9);
        assert!((ned[2] + 12.0).abs() < 1e-12, "altitude is positive up");
        assert_eq!(gps_velocity_ned(&gps), [3.0, -1.0, 0.5]);
    }

    #[test]
    fn the_suite_starts_with_the_configured_true_biases() {
        let s = SensorSuite::with_defaults(0);
        assert_eq!(s.accel_bias(), Vec3::new(0.05, -0.03, 0.02));
        assert_eq!(s.gyro_bias(), Vec3::new(0.001, -0.002, 0.0015));
        assert_eq!(s.time(), 0.0);
    }

    #[test]
    fn the_same_seed_reproduces_the_same_sequence() {
        let mut a = SensorSuite::with_defaults(2026);
        let mut b = SensorSuite::with_defaults(2026);
        let rate = Vec3::new(0.1, 0.2, 0.3);
        for _ in 0..200 {
            assert_eq!(
                a.generate_imu(&rate, [0.0, 0.0, -GRAVITY], 0.005),
                b.generate_imu(&rate, [0.0, 0.0, -GRAVITY], 0.005)
            );
        }
    }

    #[test]
    fn different_seeds_diverge() {
        let mut a = SensorSuite::with_defaults(1);
        let mut b = SensorSuite::with_defaults(2);
        let rate = Vec3::zeros();
        let mut differing = false;
        for _ in 0..50 {
            let x = a.generate_imu(&rate, [0.0, 0.0, -GRAVITY], 0.005);
            let y = b.generate_imu(&rate, [0.0, 0.0, -GRAVITY], 0.005);
            if x != y {
                differing = true;
                break;
            }
        }
        assert!(differing);
    }

    #[test]
    fn the_true_biases_random_walk() {
        let mut s = SensorSuite::with_defaults(7);
        let before = s.accel_bias();
        let rate = Vec3::zeros();
        for _ in 0..500 {
            s.generate_imu(&rate, [0.0, 0.0, -GRAVITY], 0.005);
        }
        assert_ne!(s.accel_bias(), before, "the true bias should drift");
        // But only slightly: the walk over 2.5 s is tiny.
        assert!((s.accel_bias() - before).norm() < 0.05);
    }

    #[test]
    fn the_clock_advances_by_dt() {
        let mut s = SensorSuite::with_defaults(3);
        let rate = Vec3::zeros();
        for _ in 0..10 {
            s.generate_imu(&rate, [0.0, 0.0, -GRAVITY], 0.005);
        }
        assert!((s.time() - 0.05).abs() < 1e-12, "{}", s.time());
    }

    #[test]
    fn imu_noise_matches_the_density_over_sqrt_dt() {
        // Zero the true biases so the mean is zero and the sample standard
        // deviation is directly comparable to the analytic sigma.
        let cfg = ImuConfig {
            accel_bias_initial: [0.0; 3],
            gyro_bias_initial: [0.0; 3],
            ..Default::default()
        };
        let dt = 0.005;
        let mut s = SensorSuite::new(cfg, GpsConfig::default(), 11);
        let rate = Vec3::zeros();
        let n = 20_000;
        let mut sum = [0.0; 3];
        let mut sum_sq = [0.0; 3];
        for _ in 0..n {
            let m = s.generate_imu(&rate, [0.0, 0.0, -GRAVITY], dt);
            // Specific force at rest is [0, 0, -g]; remove it before measuring.
            let accel = [m.accel[0], m.accel[1], m.accel[2] + GRAVITY];
            for i in 0..3 {
                sum[i] += accel[i];
                sum_sq[i] += accel[i] * accel[i];
            }
        }
        let expected = cfg.accel_noise_density / math::sqrt(dt);
        for i in 0..3 {
            let mean = sum[i] / n as f64;
            let var = sum_sq[i] / n as f64 - mean * mean;
            let sd = math::sqrt(var);
            assert!(
                (sd - expected).abs() < 0.1 * expected,
                "axis {i}: sd {sd} vs sigma {expected}"
            );
        }
    }

    #[test]
    fn gps_noise_matches_the_configured_sigmas_in_metres() {
        let mut s = SensorSuite::with_defaults(13);
        let p = Vec3::new(10.0, -4.0, -12.0);
        let v = Vec3::new(2.0, 1.0, 0.0);
        let n = 20_000;
        let mut sum = [0.0; 3];
        let mut sum_sq = [0.0; 3];
        for _ in 0..n {
            let ned = gps_to_ned(&s.generate_gps(&p, &v));
            for i in 0..3 {
                sum[i] += ned[i];
                sum_sq[i] += ned[i] * ned[i];
            }
        }
        let expected = [
            1.5, 1.5, // horizontal
            3.0, // vertical
        ];
        for i in 0..3 {
            let mean = sum[i] / n as f64;
            let var = sum_sq[i] / n as f64 - mean * mean;
            let sd = math::sqrt(var);
            assert!(
                (sd - expected[i]).abs() < 0.12 * expected[i],
                "axis {i}: sd {sd} vs sigma {}",
                expected[i]
            );
        }
    }

    #[test]
    fn gps_noise_is_centred_on_the_true_value() {
        let mut s = SensorSuite::with_defaults(17);
        let p = Vec3::new(100.0, 50.0, -30.0);
        let v = Vec3::zeros();
        let n = 20_000;
        let mut mean = [0.0; 3];
        for _ in 0..n {
            let ned = gps_to_ned(&s.generate_gps(&p, &v));
            for i in 0..3 {
                mean[i] += ned[i] / n as f64;
            }
        }
        assert!((mean[0] - 100.0).abs() < 0.1, "{mean:?}");
        assert!((mean[1] - 50.0).abs() < 0.1, "{mean:?}");
        assert!((mean[2] + 30.0).abs() < 0.2, "{mean:?}");
    }

    #[test]
    fn a_non_positive_dt_does_not_advance_the_clock() {
        let mut s = SensorSuite::with_defaults(5);
        let rate = Vec3::zeros();
        s.generate_imu(&rate, [0.0, 0.0, -GRAVITY], 0.0);
        assert_eq!(s.time(), 0.0);
        s.generate_imu(&rate, [0.0, 0.0, -GRAVITY], -1.0);
        assert_eq!(s.time(), 0.0);
    }
}
