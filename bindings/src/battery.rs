//! Python boundary for the Coulomb-counting battery model.
//!
//! The core's `battery` module ([`talon_core::battery`]) is the single
//! implementation: the thrust-proportional load with its idle floor, the
//! Coulomb counting and the IR voltage drop are the same numbers the SIL and
//! the browser run. This file only marshals.
//!
//! Config validation happens in `BatteryModel`'s constructor, where the
//! Rust core performs it (`BatteryConfig::validate`), with the message text
//! the retired Python dataclass raised in `__post_init__`, reproduced with
//! Rust's `{:?}` float formatting so `0.0` prints as `0.0` exactly like
//! Python's `str()` does. The Python `BatteryConfig` validates eagerly in
//! its own constructor instead, so the parity suite compares Python
//! `BatteryConfig(...)` errors against `_core.BatteryModel(...)` errors.
//!
//! Two policies follow the Rust core rather than the retired Python module,
//! pinned by the parity suite as registered divergences: a negative `dt` in
//! `update` is treated as zero instead of raising
//! (`battery.update_negative_dt`), and a NaN `capacity_mah` or
//! `hover_thrust_n` is rejected instead of silently accepted
//! (`battery.config_nan_rejected`). The config argument is copied at
//! construction, the same snapshot policy as `NLGLConfig` in the guidance
//! boundary (`battery.config_is_a_snapshot`).

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use talon_core::battery::{BatteryConfig, BatteryModel, BatteryState};

/// The message for a non-positive or NaN capacity, or `None` when it is
/// usable.
///
/// Kept pure so the message the Python tests match on is testable without an
/// initialized interpreter. The predicate mirrors `BatteryConfig::validate`
/// in `core/src/battery.rs`, which also rejects NaN explicitly.
fn capacity_error(capacity_mah: f64) -> Option<String> {
    (capacity_mah <= 0.0 || capacity_mah.is_nan())
        .then(|| format!("capacity_mah must be positive, got {capacity_mah:?}"))
}

/// The message for a non-positive or NaN hover thrust, or `None` when it is
/// usable.
fn hover_thrust_error(hover_thrust_n: f64) -> Option<String> {
    (hover_thrust_n <= 0.0 || hover_thrust_n.is_nan())
        .then(|| format!("hover_thrust_n must be positive, got {hover_thrust_n:?}"))
}

/// Battery electrical and load parameters (SI, mAh for capacity), with the
/// same defaults as the Python dataclass.
#[pyclass(name = "BatteryConfig")]
pub struct PyBatteryConfig {
    pub(crate) inner: BatteryConfig,
}

#[pymethods]
impl PyBatteryConfig {
    #[new]
    #[pyo3(signature = (
        capacity_mah = None,
        nominal_voltage = None,
        hover_current_a = None,
        hover_thrust_n = None,
        internal_resistance = None,
        idle_current_a = None,
    ))]
    #[allow(clippy::too_many_arguments)] // one optional argument per config field
    fn new(
        capacity_mah: Option<f64>,
        nominal_voltage: Option<f64>,
        hover_current_a: Option<f64>,
        hover_thrust_n: Option<f64>,
        internal_resistance: Option<f64>,
        idle_current_a: Option<f64>,
    ) -> Self {
        let defaults = BatteryConfig::default();
        Self {
            inner: BatteryConfig {
                capacity_mah: capacity_mah.unwrap_or(defaults.capacity_mah),
                nominal_voltage: nominal_voltage.unwrap_or(defaults.nominal_voltage),
                hover_current_a: hover_current_a.unwrap_or(defaults.hover_current_a),
                hover_thrust_n: hover_thrust_n.unwrap_or(defaults.hover_thrust_n),
                internal_resistance: internal_resistance.unwrap_or(defaults.internal_resistance),
                idle_current_a: idle_current_a.unwrap_or(defaults.idle_current_a),
            },
        }
    }

    /// Pack capacity (mAh).
    #[getter]
    fn capacity_mah(&self) -> f64 {
        self.inner.capacity_mah
    }

    #[setter]
    fn set_capacity_mah(&mut self, value: f64) {
        self.inner.capacity_mah = value;
    }

    /// Nominal pack voltage (V).
    #[getter]
    fn nominal_voltage(&self) -> f64 {
        self.inner.nominal_voltage
    }

    #[setter]
    fn set_nominal_voltage(&mut self, value: f64) {
        self.inner.nominal_voltage = value;
    }

    /// Current drawn at hover thrust (A).
    #[getter]
    fn hover_current_a(&self) -> f64 {
        self.inner.hover_current_a
    }

    #[setter]
    fn set_hover_current_a(&mut self, value: f64) {
        self.inner.hover_current_a = value;
    }

    /// Collective thrust at hover (N).
    #[getter]
    fn hover_thrust_n(&self) -> f64 {
        self.inner.hover_thrust_n
    }

    #[setter]
    fn set_hover_thrust_n(&mut self, value: f64) {
        self.inner.hover_thrust_n = value;
    }

    /// Pack internal resistance (ohm).
    #[getter]
    fn internal_resistance(&self) -> f64 {
        self.inner.internal_resistance
    }

    #[setter]
    fn set_internal_resistance(&mut self, value: f64) {
        self.inner.internal_resistance = value;
    }

    /// Avionics and idle ESC draw (A).
    #[getter]
    fn idle_current_a(&self) -> f64 {
        self.inner.idle_current_a
    }

    #[setter]
    fn set_idle_current_a(&mut self, value: f64) {
        self.inner.idle_current_a = value;
    }
}

/// One battery sample: state of charge, voltage and current. Read-only.
#[pyclass(name = "BatteryState")]
pub struct PyBatteryState {
    inner: BatteryState,
}

#[pymethods]
impl PyBatteryState {
    /// State of charge (0 to 1).
    #[getter]
    fn soc(&self) -> f64 {
        self.inner.soc
    }

    /// Terminal voltage (V).
    #[getter]
    fn voltage(&self) -> f64 {
        self.inner.voltage
    }

    /// Load current (A).
    #[getter]
    fn current_a(&self) -> f64 {
        self.inner.current_a
    }
}

/// Coulomb-counting battery with a thrust-proportional load and IR drop.
#[pyclass(name = "BatteryModel")]
pub struct PyBatteryModel {
    inner: BatteryModel,
}

#[pymethods]
impl PyBatteryModel {
    #[new]
    #[pyo3(signature = (config = None, initial_soc = 1.0))]
    fn new(config: Option<PyRef<'_, PyBatteryConfig>>, initial_soc: f64) -> PyResult<Self> {
        let config = config.map(|c| c.inner).unwrap_or_default();
        // Validate with the Python dataclass's message text before the core
        // does, so both sides raise identically (the core's own Display
        // prints `0` where Python prints `0.0`).
        if let Some(message) = capacity_error(config.capacity_mah) {
            return Err(PyValueError::new_err(message));
        }
        if let Some(message) = hover_thrust_error(config.hover_thrust_n) {
            return Err(PyValueError::new_err(message));
        }
        Ok(Self {
            inner: BatteryModel::new(config, initial_soc)
                .map_err(|error| PyValueError::new_err(error.to_string()))?,
        })
    }

    /// Current state of charge (0 to 1).
    #[getter]
    fn soc(&self) -> f64 {
        self.inner.soc()
    }

    /// True below 20 percent, the RTL trigger threshold.
    #[getter]
    fn is_low(&self) -> bool {
        self.inner.is_low()
    }

    /// True below 10 percent.
    #[getter]
    fn is_critical(&self) -> bool {
        self.inner.is_critical()
    }

    /// Current load (A).
    #[getter]
    fn current_a(&self) -> f64 {
        self.inner.current_a()
    }

    /// Set the state of charge directly, clamped to `[0, 1]`.
    fn force_soc(&mut self, soc: f64) {
        self.inner.force_soc(soc);
    }

    /// Advance one step under the given collective thrust (N) and return the
    /// state after the update.
    ///
    /// A negative `dt` is treated as zero, exactly as the core does.
    #[pyo3(signature = (thrust_n, dt))]
    fn update(&mut self, thrust_n: f64, dt: f64) -> PyBatteryState {
        PyBatteryState {
            inner: self.inner.update(thrust_n, dt),
        }
    }
}

/// Register the battery boundary on the extension module.
pub fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PyBatteryConfig>()?;
    module.add_class::<PyBatteryState>()?;
    module.add_class::<PyBatteryModel>()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{capacity_error, hover_thrust_error};

    #[test]
    fn a_non_positive_capacity_is_rejected_with_the_python_message() {
        assert!(capacity_error(1500.0).is_none());
        let message = capacity_error(0.0).expect("a zero capacity is an error");
        assert_eq!(message, "capacity_mah must be positive, got 0.0");
        let message = capacity_error(-1.0).expect("a negative capacity is an error");
        assert_eq!(message, "capacity_mah must be positive, got -1.0");
    }

    #[test]
    fn a_non_positive_hover_thrust_is_rejected_with_the_python_message() {
        assert!(hover_thrust_error(14.715).is_none());
        let message = hover_thrust_error(0.0).expect("a zero thrust is an error");
        assert_eq!(message, "hover_thrust_n must be positive, got 0.0");
        let message = hover_thrust_error(-2.0).expect("a negative thrust is an error");
        assert_eq!(message, "hover_thrust_n must be positive, got -2.0");
    }

    #[test]
    fn nan_capacity_and_hover_thrust_are_rejected() {
        // Regression: `x <= 0.0` alone would accept NaN, so both guards
        // carry an explicit `is_nan()` branch, like the core.
        let message = capacity_error(f64::NAN).expect("a NaN capacity is an error");
        assert!(message.contains("NaN"), "{message}");
        let message = hover_thrust_error(f64::NAN).expect("a NaN thrust is an error");
        assert!(message.contains("NaN"), "{message}");
    }
}
