//! Python bindings for the Talon GNC core.
//!
//! This crate is a pure boundary: every behaviour lives in `talon-core`, and
//! the browser reaches the same code through the `talon-wasm` crate. It exposes
//! the `talon._core` extension module, reproducing the surface the retired
//! pybind11 module (`bindings/pytalon.cpp`) published so the Python analysis
//! harness imports the Rust core unchanged.
//!
//! Two deliberate divergences from the pybind11 surface, both documented in
//! `docs/decisions.md`:
//!
//! * `Ekf.nominal()` returns a snapshot rather than a live reference, and
//!   `Ekf.set_nominal(state)` writes a nominal state back into the filter. The
//!   old `mutable_nominal()` aliasing hook is gone.
//! * `Ekf.mutable_covariance()` is not exposed (it had no callers);
//!   `covariance()` and the cheap `position_variance()` /
//!   `position_covariance_trace()` projections cover the use cases.

use pyo3::prelude::*;

mod avoidance;
mod battery;
mod control;
mod convert;
mod estimation;
mod guidance;
mod mission;
mod motors;
mod sensors;
mod state_machine;
mod types;

use control::{PyCascadedController, PyControlCommand, PyControlGains};
use estimation::{PyEkf, PyEkfConfig, PyEkfNominalState};
use talon_core::constants;
use types::{
    hover_thrust, propagate, rotation_matrix, PyDrydenWind, PyQuaternion, PyState, PyVehicleParams,
};

/// Talon Rust computational kernel: rigid-body dynamics, wind, cascaded PD
/// autopilot, MEKF state estimator, constants.
///
/// The surface mirrors the retired C++ `_core` module name for name; the
/// values and equations come from `talon-core`, the same core the browser
/// runs as WebAssembly.
#[pymodule]
fn _core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Constants. Single source of truth: `talon_core::constants`.
    m.add("MASS", constants::MASS)?;
    m.add("GRAVITY", constants::GRAVITY)?;
    m.add("ARM_LENGTH", constants::ARM_LENGTH)?;
    m.add("IXX", constants::IXX)?;
    m.add("IYY", constants::IYY)?;
    m.add("IZZ", constants::IZZ)?;
    m.add("MOTOR_KF", constants::MOTOR_KF)?;
    m.add("MOTOR_KM", constants::MOTOR_KM)?;
    m.add("MOTOR_TIME_CONST", constants::MOTOR_TIME_CONST)?;
    m.add("DRAG_COEFF", constants::DRAG_COEFF)?;
    m.add("AIR_DENSITY", constants::AIR_DENSITY)?;
    m.add("VEHICLE_AREA", constants::VEHICLE_AREA)?;
    m.add("DT", constants::DT)?;
    m.add("DT_CONTROL", constants::DT_CONTROL)?;
    m.add("DT_GPS", constants::DT_GPS)?;
    m.add("DT_IMU", constants::DT_IMU)?;
    m.add("M_MAX", constants::M_MAX)?;

    // Dynamics and vehicle.
    m.add_class::<PyQuaternion>()?;
    m.add_class::<PyState>()?;
    m.add_class::<PyVehicleParams>()?;
    m.add_class::<PyDrydenWind>()?;
    m.add_function(wrap_pyfunction!(propagate, m)?)?;
    m.add_function(wrap_pyfunction!(rotation_matrix, m)?)?;
    m.add_function(wrap_pyfunction!(hover_thrust, m)?)?;

    // Autopilot.
    m.add_class::<PyControlGains>()?;
    m.add_class::<PyControlCommand>()?;
    m.add_class::<PyCascadedController>()?;

    // State estimator.
    m.add_class::<PyEkfConfig>()?;
    m.add_class::<PyEkfNominalState>()?;
    m.add_class::<PyEkf>()?;

    // Keep-out-zone geometry, the offline detour planner, and clearance.
    avoidance::register(m)?;

    // NLGL guidance, the waypoint mission manager, and motor mixing.
    guidance::register(m)?;
    mission::register(m)?;
    motors::register(m)?;

    // Flight-phase state machine, truth-side sensor models, battery model.
    state_machine::register(m)?;
    sensors::register(m)?;
    battery::register(m)?;

    Ok(())
}
