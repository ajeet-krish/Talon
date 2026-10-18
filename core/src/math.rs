//! Deterministic scalar math.
//!
//! Every transcendental call in the core goes through this module, which binds
//! the pure-Rust `libm` implementations rather than the platform libm. Native
//! (desktop, CI) and wasm32 (browser) therefore run the identical algorithm and
//! produce bit-identical results, so a live browser run reproduces a recorded
//! desktop run exactly.
//!
//! Addition, multiplication and `sqrt` are IEEE-754 correctly rounded on both
//! targets and need no wrapper. `sin`, `cos`, `exp`, `ln`, `atan2`, `asin`,
//! `acos` and `pow` are not correctly rounded and differ by ULPs between musl
//! (wasm), Apple libm and glibc, so those must come from one shared source.

pub const PI: f64 = std::f64::consts::PI;
pub const TAU: f64 = std::f64::consts::TAU;

pub use libm::{acos, asin, atan, atan2, cos, exp, fmod, hypot, log, pow, sin, sqrt, tan};

/// Degrees to radians.
#[inline]
pub fn to_radians(degrees: f64) -> f64 {
    degrees * PI / 180.0
}

/// Radians to degrees.
#[inline]
pub fn to_degrees(radians: f64) -> f64 {
    radians * 180.0 / PI
}

/// Clamp a scalar to `[lo, hi]`.
#[inline]
pub fn clamp(value: f64, lo: f64, hi: f64) -> f64 {
    if value < lo {
        lo
    } else if value > hi {
        hi
    } else {
        value
    }
}

/// Wrap an angle to `[-pi, pi)`.
///
/// Matches the Python and TypeScript helpers one-to-one, including the
/// half-open upper bound, so guidance output is identical across the three.
#[inline]
pub fn wrap_to_pi(theta: f64) -> f64 {
    fmod(theta + PI, TAU) - PI
}

/// 2-D cross product z component: `a[0] * b[1] - a[1] * b[0]`.
#[inline]
pub fn cross2(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

/// Euclidean norm of a 3-vector.
#[inline]
pub fn norm3(v: [f64; 3]) -> f64 {
    sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
}

/// Horizontal (north-east) distance between two NED positions.
#[inline]
pub fn horizontal_distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dn = a[0] - b[0];
    let de = a[1] - b[1];
    sqrt(dn * dn + de * de)
}
