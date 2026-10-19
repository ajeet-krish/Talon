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
/// Matches the Python `wrap_angle` helper (`python/talon/guidance.py`)
/// one-to-one, including the half-open upper bound, so guidance output is
/// identical in Rust and Python. NaN input yields NaN.
#[inline]
pub fn wrap_to_pi(theta: f64) -> f64 {
    (theta + PI).rem_euclid(TAU) - PI
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

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-12;

    fn close(actual: f64, expected: f64) -> bool {
        (actual - expected).abs() <= EPS
    }

    #[test]
    fn wrap_matches_python_pinned_cases() {
        // Pinned by python/tests/test_guidance.py:19-22.
        assert!(
            close(wrap_to_pi(PI + 0.1), -PI + 0.1),
            "{}",
            wrap_to_pi(PI + 0.1)
        );
        assert!(
            close(wrap_to_pi(-PI - 0.1), PI - 0.1),
            "{}",
            wrap_to_pi(-PI - 0.1)
        );
        assert!(close(wrap_to_pi(0.3), 0.3), "{}", wrap_to_pi(0.3));
        assert!(
            close(wrap_to_pi(-2.0 * PI + 0.3), 0.3),
            "{}",
            wrap_to_pi(-2.0 * PI + 0.3)
        );
    }

    #[test]
    fn wrap_recovers_large_negative_input() {
        // regression: fmod kept the numerator sign, returning -4.0 here.
        assert!(
            close(wrap_to_pi(-4.0), 2.283185307179586),
            "{}",
            wrap_to_pi(-4.0)
        );
    }

    #[test]
    fn sweep_stays_in_half_open_interval() {
        let mut theta = -4.0 * PI;
        while theta <= 4.0 * PI {
            let w = wrap_to_pi(theta);
            assert!(
                w >= -PI && w < PI,
                "wrap_to_pi({theta}) = {w} outside [-pi, pi)"
            );
            theta += 0.01;
        }
    }

    #[test]
    fn exact_boundaries_wrap_to_negative_pi() {
        assert_eq!(wrap_to_pi(PI), -PI, "pi folds to the open upper bound");
        assert_eq!(wrap_to_pi(-PI), -PI, "-pi is already in range");
    }

    #[test]
    fn nan_input_propagates_nan() {
        assert!(wrap_to_pi(f64::NAN).is_nan());
    }

    #[test]
    fn clamp_bounds_value() {
        assert_eq!(clamp(-1.0, 0.0, 1.0), 0.0, "below range lifts to lo");
        assert_eq!(clamp(0.5, 0.0, 1.0), 0.5, "inside range passes through");
        assert_eq!(clamp(2.0, 0.0, 1.0), 1.0, "above range clips to hi");
    }

    #[test]
    fn cross2_signs_follow_right_hand_rule() {
        assert_eq!(cross2([1.0, 0.0], [0.0, 1.0]), 1.0);
        assert_eq!(cross2([0.0, 1.0], [1.0, 0.0]), -1.0);
    }

    #[test]
    fn norm3_lengths() {
        assert_eq!(norm3([0.0, 0.0, 0.0]), 0.0, "zero vector has zero norm");
        assert!(
            close(norm3([3.0, 4.0, 0.0]), 5.0),
            "{}",
            norm3([3.0, 4.0, 0.0])
        );
        assert!(
            close(norm3([1.0, 2.0, 2.0]), 3.0),
            "{}",
            norm3([1.0, 2.0, 2.0])
        );
    }

    #[test]
    fn horizontal_distance_ignores_down() {
        let a = [10.0, 0.0, -20.0];
        let b = [0.0, 0.0, -5.0];
        assert!(
            close(horizontal_distance(a, b), 10.0),
            "{}",
            horizontal_distance(a, b)
        );
        assert_eq!(
            horizontal_distance([0.0, 0.0, -30.0], [0.0, 0.0, -3.0]),
            0.0,
            "same north-east position has zero horizontal distance"
        );
    }

    #[test]
    fn degree_round_trips() {
        for degrees in [0.0, 45.0, 90.0, 180.0] {
            let back = to_degrees(to_radians(degrees));
            assert!(close(back, degrees), "{degrees} deg round trip gave {back}");
        }
        assert!(close(to_radians(180.0), PI), "{}", to_radians(180.0));
        assert!(close(to_degrees(PI), 180.0), "{}", to_degrees(PI));
    }
}
