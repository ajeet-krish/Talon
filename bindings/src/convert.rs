//! Marshalling between Python sequences and the core's nalgebra vectors.
//!
//! The Python harness passes positions, velocities, moments and wind as numpy
//! arrays (or plain lists); the kernel wants [`Vec3`]. Getters hand back a plain
//! `list[float]`, which `numpy.asarray` absorbs for free. Keeping the boundary
//! list-based means the extension has no third-party Python dependency beyond
//! PyO3 itself, and the conversions stay in one place.
//!
//! The core's own type aliases ([`Vec3`], [`Mat3`], [`Matrix15`], [`Vector6`])
//! are used directly, so this crate never names `nalgebra` and can never drift
//! from the core's linear-algebra version.

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use talon_core::ekf::{Matrix15, Vector6};
use talon_core::types::{to_arr3, Arr3, Mat3, Vec3};

/// Read exactly three floats from any Python iterable of numbers.
///
/// Accepts numpy arrays, lists, tuples and generators alike. Extra elements
/// past the third are ignored; fewer than three is an error, mirroring how the
/// old pybind11/Eigen boundary behaved for shape-(3,) inputs.
pub fn vec3_from(value: &Bound<'_, PyAny>, what: &str) -> PyResult<Vec3> {
    let mut out = [0.0_f64; 3];
    let mut seen = 0_usize;
    for item in value.try_iter()? {
        if seen == 3 {
            break;
        }
        out[seen] = item?.extract::<f64>()?;
        seen += 1;
    }
    if seen != 3 {
        return Err(PyValueError::new_err(format!(
            "{what} must have 3 elements, got {seen}"
        )));
    }
    Ok(Vec3::new(out[0], out[1], out[2]))
}

/// Read a 3-vector as the core's plain `[f64; 3]` alias.
pub fn arr3_from(value: &Bound<'_, PyAny>, what: &str) -> PyResult<Arr3> {
    Ok(to_arr3(&vec3_from(value, what)?))
}

/// Read exactly four floats from any Python iterable of numbers.
///
/// Same contract as [`vec3_from`]: extra elements past the fourth are
/// ignored, fewer than four is an error.
pub fn vec4_from(value: &Bound<'_, PyAny>, what: &str) -> PyResult<[f64; 4]> {
    let mut out = [0.0_f64; 4];
    let mut seen = 0_usize;
    for item in value.try_iter()? {
        if seen == 4 {
            break;
        }
        out[seen] = item?.extract::<f64>()?;
        seen += 1;
    }
    if seen != 4 {
        return Err(PyValueError::new_err(format!(
            "{what} must have 4 elements, got {seen}"
        )));
    }
    Ok(out)
}

/// Read a 3x3 matrix from any Python iterable of three rows of three floats.
pub fn mat3_from(value: &Bound<'_, PyAny>, what: &str) -> PyResult<Mat3> {
    let mut out = [0.0_f64; 9];
    let mut row = 0_usize;
    for items in value.try_iter()? {
        if row == 3 {
            break;
        }
        let mut col = 0_usize;
        for item in items?.try_iter()? {
            if col == 3 {
                break;
            }
            out[row * 3 + col] = item?.extract::<f64>()?;
            col += 1;
        }
        if col != 3 {
            return Err(PyValueError::new_err(format!(
                "{what} row {row} must have 3 elements, got {col}"
            )));
        }
        row += 1;
    }
    if row != 3 {
        return Err(PyValueError::new_err(format!(
            "{what} must have 3 rows, got {row}"
        )));
    }
    Ok(Mat3::from_row_slice(&out))
}

/// Present a 3-vector as a Python list.
pub fn vec3_to(value: &Vec3) -> Vec<f64> {
    vec![value[0], value[1], value[2]]
}

/// Present a 3x3 matrix as a list of three row lists.
pub fn mat3_to(value: &Mat3) -> Vec<Vec<f64>> {
    (0..3)
        .map(|row| (0..3).map(|col| value[(row, col)]).collect())
        .collect()
}

/// Present the 15x15 EKF covariance as a list of fifteen row lists.
pub fn mat15_to(value: &Matrix15) -> Vec<Vec<f64>> {
    (0..15)
        .map(|row| (0..15).map(|col| value[(row, col)]).collect())
        .collect()
}

/// Present a 6-vector as a Python list.
pub fn vec6_to(value: &Vector6) -> Vec<f64> {
    (0..6).map(|i| value[i]).collect()
}

#[cfg(test)]
mod tests {
    use super::{mat3_to, vec3_to};
    use talon_core::types::{Mat3, Vec3};

    #[test]
    fn vec3_round_trip_is_index_ordered() {
        let v = Vec3::new(1.0, -2.0, 3.5);
        assert_eq!(vec3_to(&v), vec![1.0, -2.0, 3.5]);
    }

    #[test]
    fn mat3_round_trip_is_row_major() {
        let m = Mat3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let rows = mat3_to(&m);
        assert_eq!(rows.len(), 3);
        assert_eq!(rows[0], vec![1.0, 2.0, 3.0]);
        assert_eq!(rows[1], vec![4.0, 5.0, 6.0]);
        assert_eq!(rows[2], vec![7.0, 8.0, 9.0]);
    }
}
