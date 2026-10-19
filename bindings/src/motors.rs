//! Python boundary for motor mixing and the first-order motor response.
//!
//! The core's `motors` module ([`talon_core::motors`]) is the single
//! implementation: the X-configuration mix, its inverse, and the four
//! first-order lag responses are the same numbers the autopilot and the
//! browser run. This file only marshals.
//!
//! A non-positive (or NaN) lag time constant is rejected with the same
//! message text the retired Python module raised, reproduced with Rust's
//! `{:?}` float formatting so `0.0` prints as `0.0` exactly like Python's
//! `str()` does. A negative `dt` follows the Rust core: it is treated as
//! zero and the forces do not move, while the Python module raised there;
//! the parity suite pins that difference as a registered divergence
//! (`motors.step_negative_dt`), as it does the opposite NaN-lag edge
//! (`motors.time_const_nan`: Python accepts a NaN lag, Rust rejects it).

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use talon_core::constants::{DT, MOTOR_TIME_CONST};
use talon_core::motors;

use crate::convert::{arr3_from, vec4_from};

/// The message for a non-positive or NaN lag, or `None` when it is usable.
///
/// Kept pure so the message the Python tests match on is testable without an
/// initialized interpreter. The predicate mirrors `MotorModel::new` in
/// `core/src/motors.rs`, which also rejects NaN explicitly.
fn time_const_error(time_const: f64) -> Option<String> {
    (time_const <= 0.0 || time_const.is_nan())
        .then(|| format!("time_const must be positive, got {time_const:?}"))
}

/// Inverse mix: `(collective thrust, body moment) -> 4 motor forces`.
#[pyfunction]
fn motor_mixing(thrust: f64, moment: &Bound<'_, PyAny>) -> PyResult<Vec<f64>> {
    Ok(motors::motor_mixing(thrust, arr3_from(moment, "moment")?).to_vec())
}

/// Forward mix: `4 motor forces -> (collective thrust, body moment)`.
#[pyfunction]
fn forward_mix(motor_forces: &Bound<'_, PyAny>) -> PyResult<(f64, Vec<f64>)> {
    let (thrust, moment) = motors::forward_mix(vec4_from(motor_forces, "motor_forces")?);
    Ok((thrust, moment.to_vec()))
}

/// The forward mix matrix as four row lists, `wrench = MIX_MATRIX @ forces`.
#[pyfunction]
fn mix_matrix() -> Vec<Vec<f64>> {
    let matrix = motors::mix_matrix();
    (0..4)
        .map(|row| (0..4).map(|col| matrix[(row, col)]).collect())
        .collect()
}

/// Four first-order lag motor responses with per-motor saturation.
#[pyclass(name = "MotorModel")]
pub struct PyMotorModel {
    inner: motors::MotorModel,
}

#[pymethods]
impl PyMotorModel {
    #[new]
    #[pyo3(signature = (time_const = None))]
    fn new(time_const: Option<f64>) -> PyResult<Self> {
        let time_const = time_const.unwrap_or(MOTOR_TIME_CONST);
        match time_const_error(time_const) {
            Some(message) => Err(PyValueError::new_err(message)),
            None => Ok(Self {
                inner: motors::MotorModel::new(time_const)
                    .map_err(|error| PyValueError::new_err(error.to_string()))?,
            }),
        }
    }

    /// Return all motor forces to zero.
    fn reset(&mut self) {
        self.inner.reset();
    }

    /// Advance one step and return the actual, lagged motor forces.
    ///
    /// A negative `dt` is treated as zero, exactly as the core does.
    #[pyo3(signature = (thrust, moment, dt = None))]
    fn step(
        &mut self,
        thrust: f64,
        moment: &Bound<'_, PyAny>,
        dt: Option<f64>,
    ) -> PyResult<Vec<f64>> {
        let dt = dt.unwrap_or(DT);
        Ok(self
            .inner
            .step(thrust, arr3_from(moment, "moment")?, dt)
            .to_vec())
    }

    /// Current actual motor forces (N).
    #[getter]
    fn forces(&self) -> Vec<f64> {
        self.inner.forces().to_vec()
    }

    /// The lag time constant (s).
    #[getter]
    fn time_const(&self) -> f64 {
        self.inner.time_const()
    }
}

/// Register the motors boundary on the extension module.
pub fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(motor_mixing, module)?)?;
    module.add_function(wrap_pyfunction!(forward_mix, module)?)?;
    module.add_function(wrap_pyfunction!(mix_matrix, module)?)?;
    module.add_class::<PyMotorModel>()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::time_const_error;

    #[test]
    fn a_non_positive_time_constant_is_rejected_with_the_python_message() {
        assert!(time_const_error(0.02).is_none());
        let message = time_const_error(0.0).expect("a zero lag is an error");
        assert_eq!(message, "time_const must be positive, got 0.0");
        let message = time_const_error(-0.02).expect("a negative lag is an error");
        assert_eq!(message, "time_const must be positive, got -0.02");
        let message = time_const_error(-0.0).expect("a negative-zero lag is an error");
        assert_eq!(message, "time_const must be positive, got -0.0");
    }

    #[test]
    fn a_nan_time_constant_is_rejected() {
        let message = time_const_error(f64::NAN).expect("a NaN lag is an error");
        assert!(message.contains("NaN"), "{message}");
    }
}
