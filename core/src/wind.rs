//! Simplified Dryden wind turbulence.
//!
//! Per-axis first-order Gauss-Markov process, the continuous-time shaping filter
//! from MIL-F-8785C (Beard and McLain), discretized exactly over one step:
//!
//! ```text
//! w[k+1] = exp(-Va*dt/L) * w[k] + sigma * sqrt(1 - exp(-2*Va*dt/L)) * N(0, 1)
//! ```
//!
//! The three axes are independent, each with its own sigma and integral scale
//! `L`. Turbulence axes are treated as aligned with NED, a documented
//! simplification. At `Va = 0` the process freezes, matching the physical
//! intuition that turbulence is swept past a moving vehicle.

use crate::math;
use crate::rng::Rng;
use crate::types::{Arr3, Vec3};

/// Dryden turbulence generator.
#[derive(Debug, Clone)]
pub struct DrydenWind {
    sigma: [f64; 3],
    integral_scale: [f64; 3],
    dt: f64,
    seed: u64,
    wind: Vec3,
    rng: Rng,
}

impl DrydenWind {
    /// Configure the process. `sigma` is the per-axis turbulence intensity
    /// (m/s) and `integral_scale` the per-axis integral length scale (m).
    pub fn new(sigma: [f64; 3], integral_scale: [f64; 3], dt: f64, seed: u64) -> Self {
        let mut wind = Self {
            sigma,
            integral_scale,
            dt,
            seed,
            wind: Vec3::zeros(),
            rng: Rng::new(seed),
        };
        wind.reset();
        wind
    }

    /// The documented defaults: sigma (2, 2, 1) m/s, scales (200, 200, 50) m.
    pub fn with_defaults(dt: f64, seed: u64) -> Self {
        Self::new([2.0, 2.0, 1.0], [200.0, 200.0, 50.0], dt, seed)
    }

    /// Frozen turbulence (zero sigma): the process never evolves.
    pub fn frozen(dt: f64, seed: u64) -> Self {
        Self::new([0.0, 0.0, 0.0], [200.0, 200.0, 50.0], dt, seed)
    }

    /// Advance one step and return the current wind in NED (m/s).
    pub fn step(&mut self, airspeed: f64) -> Vec3 {
        let va = if airspeed > 0.0 { airspeed } else { 0.0 };
        for i in 0..3 {
            let alpha = math::exp(-va * self.dt / self.integral_scale[i]);
            let noise = self.sigma[i] * math::sqrt(1.0 - alpha * alpha);
            self.wind[i] = alpha * self.wind[i] + noise * self.rng.standard_normal();
        }
        self.wind
    }

    /// Return to the initial condition (zero wind) and reseed, so the exact
    /// same sequence is produced again.
    pub fn reset(&mut self) {
        self.rng = Rng::new(self.seed);
        self.wind = Vec3::zeros();
    }

    /// Current wind in NED (m/s).
    pub fn wind(&self) -> Vec3 {
        self.wind
    }

    /// Current wind as an `Arr3`, for the export document.
    pub fn wind_arr(&self) -> Arr3 {
        [self.wind[0], self.wind[1], self.wind[2]]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn starts_and_resets_to_zero() {
        let mut w = DrydenWind::with_defaults(0.005, 42);
        assert_eq!(w.wind_arr(), [0.0, 0.0, 0.0]);
        for _ in 0..100 {
            w.step(5.0);
        }
        assert_ne!(w.wind_arr(), [0.0, 0.0, 0.0]);
        w.reset();
        assert_eq!(w.wind_arr(), [0.0, 0.0, 0.0]);
    }

    #[test]
    fn reset_reproduces_the_same_sequence() {
        let mut a = DrydenWind::with_defaults(0.005, 7);
        let mut first = Vec::new();
        for _ in 0..200 {
            first.push(a.step(5.0));
        }
        a.reset();
        assert_eq!(a.wind_arr(), [0.0, 0.0, 0.0]);
        for expected in &first {
            assert_eq!(a.step(5.0), *expected);
        }
    }

    #[test]
    fn zero_airspeed_freezes_the_process() {
        let mut w = DrydenWind::with_defaults(0.005, 3);
        for _ in 0..50 {
            w.step(0.0);
        }
        assert_eq!(w.wind_arr(), [0.0, 0.0, 0.0], "frozen process must stay at zero");
    }

    #[test]
    fn zero_sigma_never_evolves() {
        let mut w = DrydenWind::frozen(0.005, 11);
        for _ in 0..500 {
            w.step(8.0);
        }
        assert_eq!(w.wind_arr(), [0.0, 0.0, 0.0]);
    }

    #[test]
    fn variance_tracks_sigma() {
        // The stationary variance of the shaping filter is sigma^2, so the
        // sample standard deviation must approach sigma. Sampling is only
        // meaningful over many correlation times, and the correlation time is
        // tau = L / Va, so this uses a short scale and a high airspeed to fit
        // roughly 400 tau into the run (at the default L = 200 m and 5 m/s,
        // tau is 40 s and a short run estimates nothing).
        let sigma = 2.0;
        let mut w = DrydenWind::new([sigma, sigma, 1.0], [10.0, 10.0, 10.0], 0.005, 2026);
        let n = 80_000;
        let mut sum = 0.0;
        let mut sum_sq = 0.0;
        for _ in 0..n {
            let v = w.step(10.0)[0];
            sum += v;
            sum_sq += v * v;
        }
        let mean = sum / n as f64;
        let var = sum_sq / n as f64 - mean * mean;
        let sd = var.sqrt();
        assert!((sd - sigma).abs() < 0.15 * sigma, "sd {sd} vs sigma {sigma}");
    }

    #[test]
    fn turbulence_is_correlated_not_white() {
        // Consecutive samples must be strongly correlated (alpha close to 1 at
        // 5 m/s with L = 200 m), which distinguishes the shaping filter from
        // white noise.
        let mut w = DrydenWind::with_defaults(0.005, 5);
        for _ in 0..1000 {
            w.step(5.0);
        }
        let mut prev = w.wind()[0];
        let mut same_sign = 0;
        for _ in 0..2000 {
            let cur = w.step(5.0)[0];
            if (cur - prev).abs() < 0.5 {
                same_sign += 1;
            }
            prev = cur;
        }
        assert!(same_sign > 1500, "process looks white: {same_sign}/2000");
    }
}
