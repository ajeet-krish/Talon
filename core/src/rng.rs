//! Seeded, portable normal-noise stream.
//!
//! The generator is splitmix64 (Steele, Lea and Flood, 2014; the same mixing
//! function xoshiro uses for seeding), which is small, fast, has a full 2^64
//! period, and passes the standard statistical test batteries. It is not
//! cryptographic and is not used for anything security relevant.
//!
//! Normals come from Box-Muller with the second deviate cached, and every
//! transcendental is taken from [`crate::math`], so the whole sequence is
//! bit-identical on native and wasm32. Reproducibility is therefore guaranteed
//! by the algorithm rather than by a dependency version.

use crate::math::{self, TAU};

/// Deterministic standard-normal stream.
#[derive(Debug, Clone)]
pub struct Rng {
    state: u64,
    has_spare: bool,
    spare: f64,
}

impl Rng {
    /// Create a stream from a 64-bit seed.
    pub fn new(seed: u64) -> Self {
        Self {
            state: seed,
            has_spare: false,
            spare: 0.0,
        }
    }

    /// Next raw 64-bit draw (splitmix64).
    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in `[0, 1)`.
    pub fn uniform(&mut self) -> f64 {
        // Top 53 bits give exactly the representable grid of [0, 1).
        (self.next_u64() >> 11) as f64 * (1.0 / 9_007_199_254_740_992.0)
    }

    /// Standard normal deviate, mean 0 and variance 1.
    pub fn standard_normal(&mut self) -> f64 {
        if self.has_spare {
            self.has_spare = false;
            return self.spare;
        }
        let mut u = 0.0;
        while u == 0.0 {
            u = self.uniform();
        }
        let v = self.uniform();
        let magnitude = math::sqrt(-2.0 * math::log(u));
        self.spare = magnitude * math::sin(TAU * v);
        self.has_spare = true;
        magnitude * math::cos(TAU * v)
    }

    /// Three independent standard normals.
    pub fn normal3(&mut self) -> [f64; 3] {
        [
            self.standard_normal(),
            self.standard_normal(),
            self.standard_normal(),
        ]
    }

    /// `n` independent standard normals.
    pub fn normal_vec(&mut self, n: usize) -> Vec<f64> {
        (0..n).map(|_| self.standard_normal()).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uniform_in_unit_interval() {
        let mut rng = Rng::new(12345);
        for _ in 0..10_000 {
            let u = rng.uniform();
            assert!((0.0..1.0).contains(&u), "uniform out of range: {u}");
        }
    }

    #[test]
    fn normal_moments_are_close() {
        let mut rng = Rng::new(20261012);
        let n = 200_000;
        let mut sum = 0.0;
        let mut sum_sq = 0.0;
        for _ in 0..n {
            let x = rng.standard_normal();
            sum += x;
            sum_sq += x * x;
        }
        let mean = sum / n as f64;
        let variance = sum_sq / n as f64 - mean * mean;
        assert!(mean.abs() < 0.02, "mean drifted: {mean}");
        assert!(
            (variance - 1.0).abs() < 0.03,
            "variance drifted: {variance}"
        );
    }

    #[test]
    fn same_seed_same_sequence() {
        let mut a = Rng::new(7);
        let mut b = Rng::new(7);
        for _ in 0..64 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
        // A different seed must diverge from the first draw.
        let mut c = Rng::new(8);
        let mut d = Rng::new(7);
        assert_ne!(c.next_u64(), d.next_u64());
    }

    #[test]
    fn normal3_matches_three_scalar_draws() {
        let mut a = Rng::new(99);
        let mut b = Rng::new(99);
        let v = a.normal3();
        assert_eq!(v[0], b.standard_normal());
        assert_eq!(v[1], b.standard_normal());
        assert_eq!(v[2], b.standard_normal());
    }
}
